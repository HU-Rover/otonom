import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import serial
import math
import numpy as np
from math import sqrt



class SerialJoystickPublisher(Node):
    def __init__(self):
        super().__init__('serial_joystick_publisher')

        # Publisher for joystick commands
        self.yurur_sistem_publisher_ = self.create_publisher(JointState, 'joint_command', 10)
          
        self.joy_msg = JointState()
        self.joy_msg.name = ['rkb_sag_joint', 'rkb_sol_joint', 
        'wd_sag_arka_joint','wd_sag_on_joint','wd_sol_arka_joint','wd_sol_on_joint',
        'sag_arka_teker_joint','sag_on_teker_joint','sol_arka_teker_joint','sol_on_teker_joint' ]
	
        self.steering_mode = 0

        # Open serial port
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.get_logger().info('Opened /dev/ttyUSB0')

        # Timer to read serial input periodically
        self.create_timer(0.1, self.read_serial)
    
        

     
    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            try:
                numbers = list(map(int, line.split()))
                if len(numbers) < 6:
                    self.get_logger().warn(f'Incomplete data: {line}')
                    return
                print(numbers)

                angular_velocity = (numbers[0] - 1500)/500   # 1st value
                linear_velocity = (numbers[1] - 1500)/200

                if(numbers[5] == 1000):
                    self.steering_mode = 0
                else:
                    self.steering_mode = 1


		        ##-------------------  MODE 1 --------------------------------------
                if(self.steering_mode):
                    
                    self.joy_msg.header.stamp = self.get_clock().now().to_msg()

                    self.joy_msg.velocity = [0.0,0.0,
                    angular_velocity,angular_velocity,angular_velocity,angular_velocity,
                    linear_velocity,linear_velocity,linear_velocity,linear_velocity]
                    
                    self.yurur_sistem_publisher_.publish(self.joy_msg)

                    
                # Rearranged indexing
                ##------------------- MODE 2  ----------------------------------------
                else:
                    self.joy_msg.header.stamp = self.get_clock().now().to_msg()

                    self.joy_msg.velocity = [0.0,0.0,
                    0.0, 0.0, 0.0, 0.0,
                    linear_velocity,linear_velocity,-linear_velocity,-linear_velocity]

                    self.yurur_sistem_publisher_.publish(self.joy_msg)           
        
             
                
            except ValueError:
                self.get_logger().warn(f'Invalid data format: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialJoystickPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

