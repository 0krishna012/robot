#!/usr/bin/env python3
"""Reads the MPU6050 over I2C on the Jetson and publishes sensor_msgs/Imu."""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus

MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

ACCEL_SCALE = 16384.0   # LSB/g at +/-2g full scale
GYRO_SCALE = 131.0      # LSB/(deg/s) at +/-250 dps full scale
G = 9.80665


def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) | low
    if val >= 0x8000:
        val -= 0x10000
    return val


class Mpu6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.declare_parameter('i2c_bus', 1)   # Jetson Nano 40-pin header I2C bus
        self.declare_parameter('i2c_addr', MPU_ADDR)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 50.0)

        self.addr = self.get_parameter('i2c_addr').value
        self.frame_id = self.get_parameter('frame_id').value
        bus_num = self.get_parameter('i2c_bus').value
        rate = self.get_parameter('rate_hz').value

        self.bus = SMBus(bus_num)
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0)  # wake the sensor up
        time.sleep(0.1)

        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        try:
            ax = read_word(self.bus, self.addr, ACCEL_XOUT_H) / ACCEL_SCALE * G
            ay = read_word(self.bus, self.addr, ACCEL_XOUT_H + 2) / ACCEL_SCALE * G
            az = read_word(self.bus, self.addr, ACCEL_XOUT_H + 4) / ACCEL_SCALE * G
            gx = math.radians(read_word(self.bus, self.addr, GYRO_XOUT_H) / GYRO_SCALE)
            gy = math.radians(read_word(self.bus, self.addr, GYRO_XOUT_H + 2) / GYRO_SCALE)
            gz = math.radians(read_word(self.bus, self.addr, GYRO_XOUT_H + 4) / GYRO_SCALE)
        except OSError as e:
            self.get_logger().warn(f'MPU6050 read failed: {e}')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.orientation_covariance[0] = -1.0  # no orientation estimate provided by this driver
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Mpu6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
