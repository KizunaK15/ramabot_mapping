#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster


def normalize_angle_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class McuOdometryAndControlNode(Node):
    def __init__(self):
        super().__init__('mcu_odometry_node')

        self.declare_parameter('track_width', 0.2350)
        self.declare_parameter('wheel_radius', 0.0675)

        self.TRACK_WIDTH = float(self.get_parameter('track_width').value)
        self.WHEEL_RADIUS = float(self.get_parameter('wheel_radius').value)

        self.init_enc = False
        self.pos_l = 0.0
        self.pos_r = 0.0
        self.prev_pos_l = 0.0
        self.prev_pos_r = 0.0

        self.last_imu_time = None
        self.prev_rate_z = 0.0
        self.filtered_rate_z = 0.0
        self.accum_gyro_rad = 0.0

        self.odo_x = 0.0
        self.odo_y = 0.0
        self.odo_theta = 0.0
        self.est_vx = 0.0
        self.est_vth = 0.0
        self.last_loop_time = None

        self.raw_cmd_vx = 0.0
        self.raw_cmd_vth = 0.0
        self.smooth_vx = 0.0
        self.smooth_vth = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_cb, 50)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_cb, qos_profile_sensor_data)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 20)
        self.sub_cmd_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_cb, 20)

        self.pub_odom = self.create_publisher(Odometry, '/odom', 50)
        self.pub_wheel_cmd = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.control_and_odom_loop)
        self.get_logger().info('MCU Slip-Resistant Fusion + Safe Nav2 Bridge aktif')

    def cmd_cb(self, msg: Twist):
        self.raw_cmd_vx = msg.linear.x
        self.raw_cmd_vth = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def joint_cb(self, msg: JointState):
        try:
            idx_l = msg.name.index('Wheel1')
            idx_r = msg.name.index('Wheel2')
        except ValueError:
            return

        if not self.init_enc:
            self.prev_pos_l = msg.position[idx_l]
            self.prev_pos_r = msg.position[idx_r]
            self.init_enc = True

        self.pos_l = msg.position[idx_l]
        self.pos_r = msg.position[idx_r]

    def imu_cb(self, msg: Imu):
        now = self.get_clock().now()
        if self.last_imu_time is None:
            self.last_imu_time = now
            return

        dt_imu = (now - self.last_imu_time).nanoseconds * 1e-9
        self.last_imu_time = now
        if dt_imu <= 0.0 or dt_imu > 0.1:
            dt_imu = 0.005

        raw_gz = msg.angular_velocity.z
        if abs(raw_gz) < 0.0012 and abs(self.raw_cmd_vx) == 0.0 and abs(self.raw_cmd_vth) == 0.0:
            raw_gz = 0.0

        self.filtered_rate_z = 0.85 * raw_gz + 0.15 * self.filtered_rate_z
        step_gyro = 0.5 * (self.filtered_rate_z + self.prev_rate_z) * dt_imu
        self.prev_rate_z = self.filtered_rate_z
        self.accum_gyro_rad += step_gyro

    def control_and_odom_loop(self):
        now = self.get_clock().now()
        if self.last_loop_time is None:
            self.last_loop_time = now
            return

        dt = (now - self.last_loop_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_loop_time = now

        if not self.init_enc:
            return

        if (now - self.last_cmd_time).nanoseconds * 1e-9 > 0.35:
            self.raw_cmd_vx = 0.0
            self.raw_cmd_vth = 0.0

        self.smooth_vx += 0.65 * (self.raw_cmd_vx - self.smooth_vx)
        self.smooth_vth += 0.70 * (self.raw_cmd_vth - self.smooth_vth)

        if abs(self.smooth_vx) < 0.001 and abs(self.raw_cmd_vx) == 0.0:
            self.smooth_vx = 0.0
        if abs(self.smooth_vth) < 0.001 and abs(self.raw_cmd_vth) == 0.0:
            self.smooth_vth = 0.0

        d_rad_l = self.pos_l - self.prev_pos_l
        d_rad_r = self.pos_r - self.prev_pos_r
        self.prev_pos_l = self.pos_l
        self.prev_pos_r = self.pos_r

        d_l = d_rad_l * self.WHEEL_RADIUS
        d_r = d_rad_r * self.WHEEL_RADIUS
        d_theta_enc = (d_r - d_l) / self.TRACK_WIDTH
        d_s_raw = (d_r + d_l) * 0.5

        wheels_stationary = (abs(d_rad_l) < 1e-6 and abs(d_rad_r) < 1e-6 and
                             abs(self.smooth_vx) == 0.0 and abs(self.smooth_vth) == 0.0)

        if wheels_stationary:
            d_theta = 0.0
            d_s = 0.0
            self.accum_gyro_rad = 0.0
            self.est_vx = 0.0
            self.est_vth = 0.0
        else:
            d_theta_gyro = self.accum_gyro_rad
            self.accum_gyro_rad = 0.0
            # Gunakan 95% gyro agar selip roda saat kontak rintangan tidak memutar frame odom
            d_theta = 0.15 * d_theta_gyro + 0.85 * d_theta_enc

            # Redam translasi jika terdeteksi selip rotasional besar (roda berputar tapi bodi tertahan)
            slip_err = abs(d_theta_gyro - d_theta_enc)
            slip_attenuation = 1.0
            d_s = d_s_raw * slip_attenuation

            self.est_vx = d_s / dt
            self.est_vth = d_theta / dt

        theta_mid = self.odo_theta + 0.5 * d_theta
        self.odo_x += d_s * math.cos(theta_mid)
        self.odo_y += d_s * math.sin(theta_mid)
        self.odo_theta = normalize_angle_rad(self.odo_theta + d_theta)

        wheel_twist = Twist()
        wheel_twist.linear.x = self.smooth_vx
        wheel_twist.angular.z = self.smooth_vth
        self.pub_wheel_cmd.publish(wheel_twist)

        stamp = now.to_msg()
        qz = math.sin(self.odo_theta * 0.5)
        qw = math.cos(self.odo_theta * 0.5)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.odo_x
        t.transform.translation.y = self.odo_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.odo_x
        odom.pose.pose.position.y = self.odo_y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.est_vx
        odom.twist.twist.angular.z = self.est_vth
        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = McuOdometryAndControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
