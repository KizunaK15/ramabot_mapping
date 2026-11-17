#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class WslSerialMock(Node):
    def __init__(self):
        super().__init__('wsl_serial_mock')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = -2.0
        self.y = -1.5
        self.th = 0.0
        self.t_elapsed = 0.0

        self.odom_timer = self.create_timer(0.05, self.update_odom)
        self.scan_timer = self.create_timer(0.10, self.publish_scan)
        self.get_logger().info('WSL Mapping Mock aktif: menyiarkan /odom, TF, & /scan virtual...')

    def update_odom(self):
        dt = 0.05
        self.t_elapsed += dt

        vx = 0.25
        vth = 0.14 * math.sin(self.t_elapsed * 0.25) + 0.12

        self.x += vx * math.cos(self.th + 0.5 * vth * dt) * dt
        self.y += vx * math.sin(self.th + 0.5 * vth * dt) * dt
        self.th = math.atan2(math.sin(self.th + vth * dt), math.cos(self.th + vth * dt))

        self.x = max(-3.2, min(3.2, self.x))
        self.y = max(-2.2, min(2.2, self.y))

        now = self.get_clock().now().to_msg()
        qz = math.sin(self.th * 0.5)
        qw = math.cos(self.th * 0.5)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

    def _raycast(self, ox, oy, angle):
        dx = math.cos(angle)
        dy = math.sin(angle)

        tx = ((4.0 - ox) / dx) if dx > 1e-6 else (((-4.0 - ox) / dx) if dx < -1e-6 else 1e9)
        ty = ((3.0 - oy) / dy) if dy > 1e-6 else (((-3.0 - oy) / dy) if dy < -1e-6 else 1e9)
        dist = min(tx, ty)

        bx0, bx1, by0, by1 = 0.6, 1.6, 0.4, 1.4
        if abs(dx) > 1e-6 and abs(dy) > 1e-6:
            t1, t2 = (bx0 - ox) / dx, (bx1 - ox) / dx
            t3, t4 = (by0 - oy) / dy, (by1 - oy) / dy
            tmin = max(min(t1, t2), min(t3, t4))
            tmax = min(max(t1, t2), max(t3, t4))
            if tmax >= max(0.0, tmin) and tmin < dist:
                dist = tmin

        return max(0.15, min(12.0, dist))

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        num_beams = 360
        scan.angle_increment = (2.0 * math.pi) / num_beams
        scan.time_increment = 0.1 / num_beams
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 12.0

        lx = self.x + 0.10 * math.cos(self.th)
        ly = self.y + 0.10 * math.sin(self.th)

        ranges = []
        for i in range(num_beams):
            rel_angle = scan.angle_min + i * scan.angle_increment
            ranges.append(self._raycast(lx, ly, self.th + rel_angle))
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = WslSerialMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
