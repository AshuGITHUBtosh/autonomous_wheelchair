#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math


class CmdVelToSerial(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        # ---------- SERIAL CONFIG ----------
        self.ser = serial.Serial(
            port='/dev/ttyACM0',   # change if needed
            baudrate=115200,
            timeout=1
        )

        # ---------- ROBOT PARAMETERS ----------
        self.wheel_radius = 0.05       # meters
        self.wheel_separation = 0.30   # meters

        # ---------- ROS SUBSCRIBER ----------
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("cmd_vel_to_serial node started")

    def cmd_vel_callback(self, msg):
        # Linear and angular velocity
        v = msg.linear.x      # m/s
        w = msg.angular.z     # rad/s

        # Differential drive kinematics
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        # Convert to RPM
        rpm_left = (v_left / (2 * math.pi * self.wheel_radius)) * 60.0
        rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60.0

        # Send to Arduino
        serial_msg = f"{rpm_left:.2f},{rpm_right:.2f}\n"
        self.ser.write(serial_msg.encode())


def main():
    rclpy.init()
    node = CmdVelToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
