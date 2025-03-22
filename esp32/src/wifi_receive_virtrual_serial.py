from binascii import crc32
import serial
from serial.tools.list_ports import comports
import socket
import struct

TCP_IP = '192.168.3.109'  # ESP32的IP
TCP_IP = '172.20.10.3'  # ESP32的IP

TCP_PORT = 1234


class VirtualSerialBridge:
    def __init__(self, tcp_ip, tcp_port, virtual_com='/dev/ttyVIRT0'):
        # self.ser = serial.Serial(virtual_com, baudrate=115200)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((tcp_ip, tcp_port))
        self.virtual_com = virtual_com

    def run(self):
        with serial.Serial(self.virtual_com, baudrate=115200) as ser:
            while True:
                data = self.sock.recv(1024)
                # print(data.hex())
                if data:
                    ser.write(data)



if __name__ == '__main__':
    bridge = VirtualSerialBridge(TCP_IP, TCP_PORT)
    bridge.run()