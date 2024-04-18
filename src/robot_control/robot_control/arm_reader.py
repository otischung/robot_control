"""
PROS Arm Commercial License
Copyright 2023 帕亞科技 (PAIA-Tech) . All rights reserved.
Primary Author: 陳麒麟(Kylin Chen)
Contributor(s): 蘇文鈺(Alvin Su), 鍾博丞(Otis Chung)
This software and associated documentation files are proprietary to 帕亞科技 (PAIA-Tech),
and are not to be copied, reproduced, or disclosed to any third party, in whole or in part, without
express prior written permission. Use and modification of this software is permitted for licensed
users only, and must always remain proprietary to 帕亞科技 (PAIA-Tech).

This code is a node to read data from specific arm.
It will publish arm state.
"""
import copy
import math
import orjson
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from serial import Serial
from .env import ARM_SERIAL_PORT_LEFT, ARM_SERIAL_PORT_RIGHT


class ArmSerialReader(Node):
    def __init__(self):
        super().__init__('arm_serial_reader')

        # Set up the serial connection
        serial_port_left = self.declare_parameter('serial_port_left', ARM_SERIAL_PORT_LEFT).value
        serial_port_right = self.declare_parameter('serial_port_right', ARM_SERIAL_PORT_RIGHT).value
        self._serial_left = Serial(serial_port_left, 115200, timeout=0)
        self._serial_right = Serial(serial_port_right, 115200, timeout=0)

        # Create a publisher for the serial data
        # TODO dynamic to adjust arm
        self._left_publisher = self.create_publisher(JointState, 'left_joint_states', 10)
        self._right_publisher = self.create_publisher(JointState, 'right_joint_states', 10)
        self.timer_period = 0.5  # seconds
        self._timer = self.create_timer(0.1, self.reader_callback)
        self._joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        self._position = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.log_interval = Duration(seconds=self.timer_period)
        current_time = self.get_clock().now()
        self.last_log_time = current_time

    def reader_callback(self):
        # Read data from the serial device
        data_left = self._serial_left.readline()
        data_right = self._serial_right.readline()
        # LEFT
        try:
            degree_data_left = orjson.loads(data_left)
            degree_positions_left = degree_data_left["servo_current_angles"]  # Assuming this is the key in your JSON

        except orjson.JSONDecodeError as error:
            self.get_logger().error(f"LEFT: Json decode error when recv {data_left}: {error}")
            return
        except KeyError as error:
            self.get_logger().error(f"LEFT: KeyError when recv {degree_data_left}: {error}")
            return
        except UnicodeDecodeError as error:
            self.get_logger().error(f"LEFT: UnicodeDecodeError when recv {data_left}: {error}")
            return
        # RIGHT
        try:
            degree_data_right = orjson.loads(data_right)
            degree_positions_right = degree_data_right["servo_current_angles"]  # Assuming this is the key in your JSON

        except orjson.JSONDecodeError as error:
            self.get_logger().error(f"RIGHT: Json decode error when recv {data_right}: {error}")
            return
        except KeyError as error:
            self.get_logger().error(f"RIGHT: KeyError when recv {degree_data_right}: {error}")
            return
        except UnicodeDecodeError as error:
            self.get_logger().error(f"RIGHT: UnicodeDecodeError when recv {data_right}: {error}")
            return

        # Convert degree positions to radians
        radian_positions_left = [math.radians(deg) for deg in degree_positions_left]
        radian_positions_right = [math.radians(deg) for deg in degree_positions_right]

        # Publish the data to the serial_data topic
        # LEFT
        msg_left = JointState()
        msg_left.header.stamp = self.get_clock().now().to_msg()
        msg_left.name = self._joint_names
        msg_left.position = radian_positions_left
        msg_left.velocity = []
        msg_left.effort = []
        # RIGHT
        msg_right = copy.deepcopy(msg_left)
        msg_right.position = radian_positions_right

        self._left_publisher.publish(msg_left)
        self._right_publisher.publish(msg_right)
        current_time = self.get_clock().now()
        if current_time - self.last_log_time >= self.log_interval:
            self.get_logger().info(f'LEFT: Receive from arm esp32: {degree_data_left}')
            self.get_logger().info(f'RIGHT Receive from arm esp32: {degree_data_right}')
            self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)

    serial_reader = ArmSerialReader()
    rclpy.spin(serial_reader)

    serial_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
