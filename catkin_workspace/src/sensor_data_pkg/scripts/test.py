import serial
import struct
import numpy as np


class Stm32SerialNode:
    def __init__(self):        
        self.buffer = bytearray()
        self.packet_length = 24  # 总数据包长度

        self.imu = []
        self.odom = []
        self.battary = []


    def parse_short(self, data):
        """转换short到浮点数（协议规定/1000）"""
        return struct.unpack('>h', data)[0] / 1000.0

    def verify_packet(self, packet):
        """异或校验（包含帧头）"""
        if len(packet) != self.packet_length:
            return False
        if packet[0] != 0x7B or packet[-1] != 0x7D:
            return False
            
        calculated_xor = 0
        for b in packet[:-2]:  # 包含帧头，排除校验位和帧尾
            calculated_xor ^= b
        return calculated_xor == packet[-2]

    def process_packet(self, packet):
        """处理有效数据包"""
        try:
            
            # 加速度（数组号6-8）
            imu_x = self.parse_short(packet[8:10])   # 数组6
            imu_y = self.parse_short(packet[10:12])  # 数组7
            imu_z = self.parse_short(packet[12:14])  # 数组8
            
            # 角速度（数组9-11）
            imu_ax = self.parse_short(packet[14:16])      # 数组9
            imu_ay = self.parse_short(packet[16:18])     # 数组10
            imu_az = self.parse_short(packet[18:20])     # 数组11

            
            # 速度信息
            odom_x = self.parse_short(packet[2:4])    # 数组3
            odom_y = self.parse_short(packet[4:6])    # 数组4
            odom_z = self.parse_short(packet[6:8])    # 数组5
        
            battery = self.parse_short(packet[20:22])            # 数组12

            self.imu = [imu_x, imu_y, imu_z, imu_ax, imu_ay, imu_az]
            self.odom = [odom_x, odom_y, odom_z]
            self.battary = battery

        except Exception as e:
            print("数据处理错误: %s", str(e))
            return None

    def run(self,data):
        if data:
            self.buffer.extend(data)
            
            # 处理完整数据包
            while len(self.buffer) >= self.packet_length:
                start = self.buffer.find(b'\x7b')
                if start < 0:
                    self.buffer.clear()
                    break
                
                end = start + self.packet_length
                if len(self.buffer) < end:
                    break
                
                packet = self.buffer[start:end]
                self.buffer = self.buffer[end:]
                
                if self.verify_packet(packet):
                    self.process_packet(packet)
                    
                    print('IMU data:',end=' ')
                    print(self.imu)
                    print('Odom data:',end=' ')
                    print(self.odom)
                    print('Battary data:',end=' ')
                    print(self.battary)

                else:
                    print("无效数据包，已丢弃")

def create_serial_packet(
    data_str,
    start_byte=0xAA,
    end_byte=0x55,
    checksum_type='xor'
):
    """
    将十六进制字符串转换为串口数据包格式

    参数:
        data_str: 十六进制字符串，如"aa1234ff"
        start_byte: 起始字节（整数，默认为0xAA）
        end_byte: 结束字节（整数，默认为0x55）
        checksum_type: 校验类型，'xor'或'sum'（默认为异或校验）

    返回:
        数据包的字节序列
    """
    # 验证输入有效性
    if not isinstance(data_str, str):
        raise TypeError("输入必须是字符串类型")
    if not data_str:
        raise ValueError("输入字符串不能为空")
    if len(data_str) % 2 != 0:
        raise ValueError("输入字符串长度必须为偶数")
    if not all(c in "0123456789abcdefABCDEF" for c in data_str):
        raise ValueError("包含非法的十六进制字符")

    # 转换数据部分为字节
    data_bytes = bytes.fromhex(data_str)
    data_length = len(data_bytes)
    if data_length > 255:
        raise ValueError("数据长度超过255字节")

    # 计算校验码
    if checksum_type == 'xor':
        checksum = 0
        for byte in data_bytes:
            checksum ^= byte
    elif checksum_type == 'sum':
        checksum = sum(data_bytes) & 0xFF
    else:
        raise ValueError("不支持的校验类型，可选'xor'或'sum'")

    # 组装数据包
    packet = bytes()
    packet += bytes([start_byte])            # 起始字节
    packet += bytes([data_length])           # 数据长度
    packet += data_bytes                      # 数据内容
    packet += bytes([checksum])              # 校验字节
    packet += bytes([end_byte])              # 结束字节

    return packet

# 示例用法
if __name__ == "__main__":
    try:
        # 输入十六进制字符串（示例："aa1234ff"）
        # hex_data = '7B00010100010000FE96FDCE4080FFFB000700015838837D'
        hex_data = '00010100010000FE96FDCE4080FFFB000700015838837D7B00010100010000FE96FDCE4080FFFB000700015838837D7B00010100010000FE96FDCE4080FFFB000700015838837D'
        
        # 生成数据包
        packet = create_serial_packet(hex_data)
        
        # 输出结果
        # print("数据包（十六进制格式）:", packet.hex().upper())
        # print("数据包（字节序列）:", packet)
        
    except Exception as e:
        print(f"错误: {e}")

    try:
        node = Stm32SerialNode()
        node.run(packet)
    except:
        print('error')
