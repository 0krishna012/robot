#!/usr/bin/env python3
"""Bridges ROS 2 cmd_vel/odom to the ESP32 hub-motor controller over serial."""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.0835)      # m — measure your wheel, this is a placeholder
        self.declare_parameter('wheel_separation', 0.42)    # m — track width, measure and set
        self.declare_parameter('ticks_per_rev', 60)          # pole_pairs(10) * 6
        self.declare_parameter('max_speed_mps', 1.0)         # real speed at frac == 1.0, calibrate by testing
        self.declare_parameter('publish_tf', True)           # set False if robot_localization publishes odom->base_link
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.max_speed = self.get_parameter('max_speed_mps').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(2.0)  # let the ESP32 finish its reset after the port opens

        self.odom_pub = self.create_publisher(Odometry, 'odom', QoSProfile(depth=10))
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        self._stop = False
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info(f'serial_bridge connected on {self.port} @ {self.baud}')

    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        v_l = v - w * self.wheel_sep / 2.0
        v_r = v + w * self.wheel_sep / 2.0

        frac_l = max(-1.0, min(1.0, v_l / self.max_speed))
        frac_r = max(-1.0, min(1.0, v_r / self.max_speed))

        try:
            self.ser.write(f'C {frac_l:.3f} {frac_r:.3f}\n'.encode())
        except serial.SerialException as e:
            self.get_logger().warn(f'serial write failed: {e}')

    def read_loop(self):
        buf = b''
        while not self._stop and rclpy.ok():
            try:
                buf += self.ser.read(256)
            except serial.SerialException as e:
                self.get_logger().warn(f'serial read failed: {e}')
                time.sleep(0.5)
                continue

            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self.handle_line(line.decode(errors='ignore').strip())

    def handle_line(self, line: str):
        parts = line.split()
        if len(parts) != 4 or parts[0] != 'E':
            return
        try:
            dl = int(parts[1])
            dr = int(parts[2])
            dt_ms = int(parts[3])
        except ValueError:
            return
        if dt_ms <= 0:
            return

        dt = dt_ms / 1000.0
        dist_l = (dl / self.ticks_per_rev) * 2.0 * math.pi * self.wheel_radius
        dist_r = (dr / self.ticks_per_rev) * 2.0 * math.pi * self.wheel_radius

        d = (dist_l + dist_r) / 2.0
        dth = (dist_r - dist_l) / self.wheel_sep

        self.th += dth
        self.x += d * math.cos(self.th)
        self.y += d * math.sin(self.th)

        v = d / dt
        w = dth / dt

        self.publish_odom(self.get_clock().now(), v, w)

    def publish_odom(self, stamp, v, w):
        q = quaternion_from_yaw(self.th)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self._stop = True
        try:
            self.ser.write(b'C 0 0\n')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
