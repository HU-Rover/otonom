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

        self.publisher = self.create_publisher(EncoderMsg, "encoder_data", 10)
        self.encoder_msg = EncoderMsg()

        self.SERIAL_PORT_SAG_ON   = '/dev/ttyACM0'
        self.SERIAL_PORT_SOL_ON   = '/dev/ttyACM1'
        self.SERIAL_PORT_SOL_ARKA = '/dev/ttyACM2'
        self.SERIAL_PORT_SAG_ARKA = '/dev/ttyACM3'
        self.Robot_Kol_Port   = "/dev/ttyACM4"

        for port in serial.tools.list_ports.comports():
            match(port.serial_number):
                case '002800323235511138363730':
                    self.SERIAL_PORT_SOL_ARKA = port.device
                case '002400283233511339363634':
                    self.SERIAL_PORT_SOL_ON = port.device
                case '005500373235511238363730':
                    self.SERIAL_PORT_SAG_ON = port.device
                case '0049002A3235511138363730':
                    self.SERIAL_PORT_SAG_ARKA = port.device
                case '066FFF313358353143091146':
                    self.Robot_Kol_Port = port.device


        self.stm_sol_on   = serial.Serial(port=self.SERIAL_PORT_SOL_ON,   baudrate=BAUD_RATE, timeout=0.1)
        self.stm_sag_on   = serial.Serial(port=self.SERIAL_PORT_SAG_ON,   baudrate=BAUD_RATE, timeout=0.1)
        self.stm_sol_arka = serial.Serial(port=self.SERIAL_PORT_SOL_ARKA, baudrate=BAUD_RATE, timeout=0.1)
        self.stm_sag_arka = serial.Serial(port=self.SERIAL_PORT_SAG_ARKA, baudrate=BAUD_RATE, timeout=0.1)
        #self.robot_kol = serial.Serial(port=self.Robot_Kol_Port, baudrate =BAUD_RATE, timeout = 0.1)
        

        self.sag_on_hiz   = 0.0
        self.sag_arka_hiz = 0.0
        self.sol_arka_hiz = 0.0
        self.sol_on_hiz   = 0.0

        # Lock to protect shared speed values from race conditions
        # between the joystick callback and the encoder timer
        self._lock = threading.Lock()

        # FIX: independent 60 Hz timer to read encoders and publish
        # encoder data is no longer tied to joystick commands arriving
        self.create_timer(0.1, self.publish_encoder)

    def _read_speed(self, stm) -> float:
        """Read one line from a serial port and return float speed. Returns last value on failure."""
        try:
            if stm.in_waiting > 0:
                raw = stm.readline()
                clean = raw.replace(b'\x00', b'')
                text = clean.decode('utf-8', errors='ignore').strip()
                if text:
                    return float(text)
        except (ValueError, serial.SerialException) as e:
            self.get_logger().warn(f'Serial read error: {e}')
        return None  # None means "no new data, keep last value"

    def publish_encoder(self):
        """Runs at 60 Hz — reads all encoders and publishes independently of joystick."""
        with self._lock:
            val = self._read_speed(self.stm_sag_arka)
            if val is not None:
                self.sag_arka_hiz = val

            val = self._read_speed(self.stm_sag_on)
            if val is not None:
                self.sag_on_hiz = val

            val = self._read_speed(self.stm_sol_on)
            if val is not None:
                self.sol_on_hiz = val

            val = self._read_speed(self.stm_sol_arka)
            if val is not None:
                self.sol_arka_hiz = val

            self.encoder_msg.sag_on   = self.sag_on_hiz
            self.encoder_msg.sag_arka = self.sag_arka_hiz
            self.encoder_msg.sol_on   = self.sol_on_hiz
            self.encoder_msg.sol_arka = self.sol_arka_hiz

        self.publisher.publish(self.encoder_msg)

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
            saghiz = f"1 {msg.saghiz:.4f}"
            solhiz = f"1 {msg.solhiz:.4f}"
            self.get_logger().info(f"Published DRİVE | Saghiz:{msg.saghiz} Solhiz:{msg.solhiz}")
            self.stm_sag_arka.write(saghiz.encode('utf-8'))
            self.stm_sag_on.write(saghiz.encode('utf-8'))
            self.stm_sol_on.write(solhiz.encode('utf-8'))
            self.stm_sol_arka.write(solhiz.encode('utf-8'))

        elif mode == 0:
            data = f"0 {msg.kp:.4f} {msg.ki:.4f} {msg.kd:.4f}"
            self.get_logger().info(f"Published STANDBY | KP:{msg.kp:.4f} Kİ:{msg.ki} KD:{msg.kd}")
            self.stm_sag_on.write(data.encode('utf-8'))
            self.stm_sag_arka.write(data.encode('utf-8'))
            self.stm_sol_arka.write(data.encode('utf-8'))
            self.stm_sol_on.write(data.encode('utf-8'))
            
        elif mode == 2:
            data = f"2 {abs(shoulder):.3f} {int(shoulder > 0)} {abs(elbow):.3f} {int(elbow > 0)} {gripper_pitch} {gripper_roll} {servo} {base}\n"
            self.get_logger().info(f"Published {abs(shoulder):.3f} {int(shoulder > 0)} {abs(elbow):.3f} {int(elbow > 0)} {gripper_pitch} {gripper_roll} {servo} {base}")
            #self.robot_kol.write(data.encode('utf-8'))


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
