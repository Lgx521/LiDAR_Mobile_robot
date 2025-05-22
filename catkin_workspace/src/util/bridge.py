#!/usr/bin/env python3
'''
Running on Laptop
multiport wifi communication class
'''

from binascii import crc32
import serial
from serial.tools.list_ports import comports
import socket
import struct
import rospy

LIDAR_PORT = 10001
ENCODER_PORT = 10003
CMD_PORT = 10002

virt_com_LiDAR = '/dev/ttyVIRT0'
virt_com_Encoder = '/dev/ttyVIRT2'

# esp32_ip = '10.0.0.22'
# esp32_ip = '172.20.10.3'
esp32_ip = '192.168.137.83'



class VirtualSerialBridge:
    def __init__(self, tcp_ip, tcp_port, virtual_com=''):
        # self.ser = serial.Serial(virtual_com, baudrate=115200)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((tcp_ip, tcp_port))
        self.virtual_com = virtual_com

    # 开始接收收到的数据并推入虚拟串口
    def run_r(self):
        with serial.Serial(self.virtual_com, baudrate=115200) as ser:
            rospy.loginfo('Communication setup successfully')
            while True:
                data = self.sock.recv(1024)
                # print(data.hex())
                if data:
                    ser.write(data)
                else:
                    rospy.loginfo('no data')

    # 读取数据并发送，应该要接入ros
    def run_t(self, data):
        self.sock.sendall(data)