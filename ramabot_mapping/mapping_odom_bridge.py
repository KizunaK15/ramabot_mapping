#!/usr/bin/env python3
import math
import re
import time
import serial
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class MappingOdomBridge(Node):
    def __init__(self):
        super().__init__('mapping_odom_bridge')

        self.declare_parameter('serial_port', '/dev/esp32_map')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('wheel_radius', 0.042)
        self.declare_parameter('ticks_per_rev', 360.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.port = self.get_parameter('serial_port').value
        self.baud = int(self.get_parameter('baud_rate').value)
        self.L = float(self.get_parameter('wheel_separation').value)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.ppr = float(self.get_parameter('ticks_per_rev').value)
        self.inv_l = -1.0 if self.get_parameter('invert_left').value else 1.0
        self.inv_r = -1.0 if self.get_parameter('invert_right').value else 1.0
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.meters_per_tick = (2.0 * math.pi * self.r) / self.ppr

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_enc_l = None
        self.prev_enc_r = None
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.ser = None
        self._open_serial()

        self.pattern = re.compile(r'ENCL:(-?\d+),ENCR:(-?\d+)')
        self.timer = self.create_timer(0.02, self.read_serial)

    def _open_serial(self):
        while rclpy.ok():
            try:
                ser = serial.Serial()
                ser.port = self.port
                ser.baudrate = self.baud
                ser.timeout = 0.05
                ser.dtr = False
                ser.rts = False
                ser.open()
                self.ser = ser
                self.get_logger().info(f'Terhubung ke ESP32 mapping di {self.port} @ {self.baud} bps')
                return
            except serial.SerialException as e:
                self.get_logger().warn(f'Menunggu port {self.port} ({e})... coba lagi 2 detik')
                time.sleep(2.0)

    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            self._open_serial()
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                match = self.pattern.search(line)
                if not match:
                    continue

                enc_l = int(match.group(1))
                enc_r = int(match.group(2))
                now = self.get_clock().now()

                if self.prev_enc_l is None:
                    self.prev_enc_l = enc_l
                    self.prev_enc_r = enc_r
                    self.last_time = now
                    continue

                dt = (now - self.last_time).nanoseconds / 1e9
                if dt <= 0.0:
                    continue

                d_l = (enc_l - self.prev_enc_l) * self.meters_per_tick * self.inv_l
                d_r = (enc_r - self.prev_enc_r) * self.meters_per_tick * self.inv_r

                self.prev_enc_l = enc_l
                self.prev_enc_r = enc_r
                self.last_time = now

                d_center = 0.5 * (d_r + d_l)
                d_theta = (d_r - d_l) / self.L

                self.x += d_center * math.cos(self.th + 0.5 * d_theta)
                self.y += d_center * math.sin(self.th + 0.5 * d_theta)
                self.th = math.atan2(math.sin(self.th + d_theta), math.cos(self.th + d_theta))

                vx = d_center / dt
                vth = d_theta / dt

                qz = math.sin(self.th * 0.5)
                qw = math.cos(self.th * 0.5)
                stamp = now.to_msg()

                t = TransformStamped()
                t.header.stamp = stamp
                t.header.frame_id = self.odom_frame
                t.child_frame_id = self.base_frame
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(t)

                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = self.odom_frame
                odom.child_frame_id = self.base_frame
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw
                odom.twist.twist.linear.x = vx
                odom.twist.twist.angular.z = vth
                self.odom_pub.publish(odom)

        except serial.SerialException as e:
            self.get_logger().error(f'Koneksi serial terputus: {e}. Menyambung ulang...')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.prev_enc_l = None
            self.prev_enc_r = None


def main(args=None):
    rclpy.init(args=args)
    node = MappingOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()