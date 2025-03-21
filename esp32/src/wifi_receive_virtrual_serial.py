from binascii import crc32
import serial
from serial.tools.list_ports import comports
import socket
import struct

TCP_IP = '192.168.3.109'  # ESP32的IP
TCP_PORT = 1234

# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sock.connect((TCP_IP, TCP_PORT))


# class VirtualSerialBridge:
#     def __init__(self, tcp_ip, tcp_port, virtual_com='/dev/ttyVIRT0'):
#         self.ser = serial.serial_for_url(virtual_com, baudrate=115200)
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         self.sock.connect((tcp_ip, tcp_port))

#     def run(self):
#         while True:
#             data = self.sock.recv(1024)
#             if data:
#                 print(data.hex())
#                 print()
#                 self.ser.write(data)

class VirtualSerialBridge:
    def __init__(self, tcp_ip, tcp_port, virtual_com='/dev/pts/5'):
        self.ser = serial.serial_for_url(virtual_com, baudrate=115200)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((tcp_ip, tcp_port))

    def unpack_data(self,data):
        if len(data) < 8:
            return None  # 数据不完整
        header, length = struct.unpack('>2sH', data[:4])
        if header != b'\x54\x2c':
            return None  # 协议头错误
        full_length = 4 + length + 4  # 头4B + 数据 + CRC4B
        if len(data) < full_length:
            return None  # 数据不完整
        frame = data[:full_length]
        data_payload = frame[4:-4]
        crc_received = struct.unpack('>I', frame[-4:])[0]
        if crc32(data_payload) != crc_received:
            return None  # 校验失败
        return data_payload, data[full_length:]  # 返回有效数据和剩余字节

    def run(self):
        # while True:
        #     data = self.sock.recv(2048)
        #     if data:
        #         self.ser.write(data)
                
        buffer = b''
        while True:
            data = self.sock.recv(2048)
            if not data:
                break
            buffer += data
            while True:
                result = self.unpack_data(buffer)
                if not result:
                    break
                payload, buffer = result
                self.ser.write(payload)  # 转发有效数据到虚拟串口
                # print(payload.hex())

            



if __name__ == '__main__':
    bridge = VirtualSerialBridge(TCP_IP, TCP_PORT)
    bridge.run()