import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import serial.tools.list_ports
from rover_msgs.msg import ControllerMsg
from std_msgs.msg import Int8  # Add at top  # Make sure this custom message is built correctly
import math
import numpy as np
from math import sqrt

BAUD_RATE = 115200  
stm_serials = []

class SerialJoystickPublisher(Node):
    def __init__(self):
        super().__init__('serial_joystick_publisher')

        # Publisher for joystick commands
        self.yurur_sistem_publisher_ = self.create_publisher(ControllerMsg, 'joystick_cmd', 10)
          
        self.joy_msg = ControllerMsg()
        self.previous_message = 0
        self.published = 1
        self.mode = 0;
        self.ki = 0.1
        self.pid_published = 0
        
        
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.get_logger().info('Opened /dev/ttyUSB0')
        self.SERIAL_PORT_SAG_ON = '/dev/ttyACM0'
        
        '''
        self.SERIAL_PORT_SAG_ON = '/dev/ttyACM0'
        self.SERIAL_PORT_SOL_ON = '/dev/ttyACM1'
        self.SERIAL_PORT_SOL_ARKA = '/dev/ttyACM2'
        self.SERIAL_PORT_SAG_ARKA = '/dev/ttyACM3'  

        # Open serial port
        
        
        for port in serial.tools.list_ports.comports():
            
            match(port.serial_number):
                case '002400293233510F39363634':
                    self.SERIAL_PORT_SAG_ON = port.device
                case '002400283233511339363634':
                    self.SERIAL_PORT_SOL_ON = port.device
                case '005500373235511238363730':
                    self.SERIAL_PORT_SOL_ARKA = port.device
                case '0049002A3235511138363730':
                    self.SERIAL_PORT_SAG_ARKA = port.device
            
        
        self.stm_sol_on =  serial.Serial(port=self.SERIAL_PORT_SOL_ON,baudrate=BAUD_RATE,timeout=1)
        
        self.stm_sag_on =  serial.Serial(port=self.SERIAL_PORT_SAG_ON,baudrate=BAUD_RATE,timeout=1)
                
        self.stm_sol_arka =  serial.Serial(port=self.SERIAL_PORT_SOL_ARKA,baudrate=BAUD_RATE,timeout=1)
        
        self.stm_sag_arka =  serial.Serial(port=self.SERIAL_PORT_SAG_ARKA,baudrate=BAUD_RATE,timeout=1)
        
        '''
        
        self.stm_sag_on =  serial.Serial(port=self.SERIAL_PORT_SAG_ON,baudrate=BAUD_RATE,timeout=1)
         

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

                # Rearranged indexing
                yonelim = (numbers[0] - 1500)/2500   # 1st value
                throttle = (numbers[1] - 1500)/625
                mode = numbers[5]      
                kp = (numbers[6]-1000)/1000
                kd = (numbers[7]-1000)/1000
                
                if(numbers[3] > 1990):
                    self.ki = self.ki + 0.01
                    
                elif(numbers[3] < 1020):
                    self.ki = self.ki - 0.01 
                
                
                if(mode == 2000):
                    self.mode = 0  # standby mode
                elif(mode == 1000):
                    self.mode = 1 # manuel mode
                    

                # Fill your custom ControllerMsg
                self.joy_msg.solhiz = float(throttle - (yonelim*2) ) / 3
                self.joy_msg.saghiz = float(throttle + (yonelim*2) )  / 3
                self.joy_msg.mode = self.mode;
                self.joy_msg.kp = kp
                self.joy_msg.kd = kd
                self.joy_msg.ki = self.ki
                
                self.yurur_sistem_publisher_.publish(self.joy_msg)
                    #ros2
                self.get_logger().info(f'Published: solhiz={self.joy_msg.solhiz}, saghiz={self.joy_msg.saghiz} kp = {self.joy_msg.kp} kd = {self.joy_msg.kd} ki = {self.joy_msg.ki}')
                 
               
        
                
        
                #self.stm_sag_arka.write(pid_message.encode('utf-8'))
                #self.stm_sag_on.write(pid_message.encode('utf-8'))
                
                #self.stm_sol_arka.write(pid_message.encode('utf-8'))
                
                
                
                # Publish the message
                if (self.mode == 1):
                    
                    saghiz = f"2 {self.joy_msg.saghiz:.4f}"
                    solhiz = f"2 {self.joy_msg.solhiz:.4f} {self.joy_msg.kp:.4f} {self.joy_msg.kd:.4f} {self.joy_msg.ki:.4f}"
                    self.stm_sag_on.write(solhiz.encode('utf-8'))
                    
                    '''
                    self.stm_sag_arka.write(saghiz.encode('utf-8'))
                    self.stm_sag_on.write(saghiz.encode('utf-8'))
                    self.stm_sol_on.write(solhiz.encode('utf-8'))
                    self.stm_sol_arka.write(solhiz.encode('utf-8'))
                    
                    '''
                    
                
                elif(self.mode == 0):
                    data = f"0"
                    
                    '''
                    self.stm_sag_on.write(data.encode('utf-8'))
                    self.stm_sag_arka.write(data.encode('utf-8'))
                    self.stm_sol_arka.write(data.encode('utf-8'))
                    self.stm_sol_on.write(data.encode('utf-8'))
                    '''
                   
                    
                
                
                     
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

