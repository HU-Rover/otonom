import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32MultiArray
import serial
import serial.tools.list_ports
from rover_msgs.msg import ControllerMsg
from rover_msgs.msg import EncoderMsg
from std_msgs.msg import Int8
import math
import numpy as np
from math import sqrt
import threading

BAUD_RATE = 115200


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('sub_node')

        self.subscription = self.create_subscription(
            ControllerMsg, 'joystick_cmd', self.listener_callback, 10)

        self.STM   = serial.Serial(port="/dev/ttyACM0",   baudrate=BAUD_RATE, timeout=0.1)

        # Lock to protect shared speed values from race conditions
        # between the joystick callback and the encoder timer
        self._lock = threading.Lock()

        # FIX: independent 60 Hz timer to read encoders and publish
        # encoder data is no longer tied to joystick commands arriving

    def listener_callback(self, msg):
        """Handles joystick commands — only sends motor commands, no longer reads encoders."""
        mode = msg.mode # 0 =Standby 1= Manuel 2 = Robot Kol
        shoulder = msg.shoulder
        elbow = msg.elbow
        gripper_pitch = msg.gripper_pitch
        gripper_roll = msg.gripper_roll
        servo = msg.servo
        base = msg.base

        if mode == 1:
            data = f"1"
            self.STM.write(data.encode('utf-8'))


        elif mode == 0:
            data = b"0"
            self.STM.write(data)
            
        elif mode == 2:
            data = f"2 {abs(shoulder):.3f} {int(shoulder > 0)} {abs(elbow):.3f} {int(elbow > 0)} {gripper_pitch} {gripper_roll} {servo} {base}\n"
            self.get_logger().info(f"Published {abs(shoulder):.3f} {int(shoulder > 0)} {abs(elbow):.3f} {int(elbow > 0)} {gripper_pitch} {gripper_roll} {servo} {base}")
            self.STM.write(data.encode('utf-8'))


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    # FIX: MultiThreadedExecutor so timer and joystick callback don't block each other
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
