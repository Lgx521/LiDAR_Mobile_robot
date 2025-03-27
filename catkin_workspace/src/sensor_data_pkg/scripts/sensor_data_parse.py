#!/usr/bin/env python3
import rospy
import struct
import serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import Imu

class Stm32OdomNode:
    def __init__(self):
        rospy.init_node('stm32_odom_node')
        
        # 协议参数（根据表格6-2-1定义）
        self.PACKET_CONFIG = {
            'header': 0x7B,
            'footer': 0x7D,
            'total_length': 34,  # 1+1+(17 * 2)+1+1=34
            'scale_factor': 1000.0,  # 根据表格说明的浮点放大倍数
            'fields': [
                ('frame_header', 0, 'B'),    # 数组1 Uint8
                ('flag_store', 1, 'B'),      # 数组2 Uint8
                ('robot_vx', 2, 'h'),        # 数组3 short
                ('robot_vy', 4, 'h'),        # 数组4 short
                ('robot_vz', 6, 'h'),        # 数组5 short
                ('accel_x', 8, 'h'),         # 数组6 short
                ('accel_y', 10, 'h'),        # 数组7 short
                ('accel_z', 12, 'h'),        # 数组8 short
                ('gyro_x', 22, 'h'),         # 数组13 short
                ('gyro_y', 24, 'h'),         # 数组14 short
                ('gyro_z', 26, 'h'),         # 数组15 short
                ('battery_voltage', 28, 'h'),# 数组17 short
                ('checksum', 32, 'B'),       # 数组18 Uint8
                ('frame_end', 33, 'B')       # 数组20 Uint8
            ]
        }

        # 初始化ROS发布器
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.twist_pub = rospy.Publisher('/current_velocity', Twist, queue_size=10)
        self.battery_pub = rospy.Publisher('/battery_voltage', Float32, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)

        # 串口配置（根据实际设备修改）
        self.serial_port = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )

        # 数据缓冲区
        self.raw_buffer = bytearray()

    def convert_value(self, raw, field_type):
        """根据字段类型转换数据"""
        scale = self.PACKET_CONFIG['scale_factor']
        if field_type == 'h':  # short类型
            return struct.unpack('<h', raw)[0] / scale
        return raw[0]  # uint8类型

    def validate_packet(self, packet):
        """执行协议验证"""
        # 帧头帧尾验证
        if packet[0] != self.PACKET_CONFIG['header'] or packet[-1] != self.PACKET_CONFIG['footer']:
            return False
        
        # 校验和验证（数组1到数组17的累加和）
        checksum = sum(packet[1:-2]) & 0xFF
        return checksum == packet[-2]

    def process_packet(self, packet):
        """处理有效数据包"""
        try:
            # 解析数据字段
            data = {}
            for name, pos, dtype in self.PACKET_CONFIG['fields']:
                if dtype == 'B':
                    data[name] = packet[pos]
                elif dtype == 'h':
                    data[name] = struct.unpack('<h', packet[pos:pos+2])[0]
            
            # 转换为实际物理值
            scale = self.PACKET_CONFIG['scale_factor']
            odom_data = {
                'vx': data['robot_vx'] / scale,
                'vy': data['robot_vy'] / scale,
                'vz': data['robot_vz'] / scale,
                'accel': [data['accel_x']/scale, data['accel_y']/scale, data['accel_z']/scale],
                'gyro': [data['gyro_x']/scale, data['gyro_y']/scale, data['gyro_z']/scale],
                'battery': data['battery_voltage'] / scale
            }
            
            # 发布ROS消息
            self.publish_messages(odom_data)

        except Exception as e:
            rospy.logerr(f"Data processing error: {str(e)}")

    def publish_messages(self, data):
        """发布所有ROS话题"""
        timestamp = rospy.Time.now()
        
        # 里程计消息
        odom = Odometry()
        odom.header = Header(frame_id="odom", stamp=timestamp)
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = data['vx']
        odom.twist.twist.linear.y = data['vy']
        odom.twist.twist.angular.z = data['vz']
        self.odom_pub.publish(odom)
        
        # 实时速度
        twist = Twist()
        twist.linear.x = data['vx']
        twist.linear.y = data['vy']
        twist.angular.z = data['vz']
        self.twist_pub.publish(twist)
        
        # IMU数据
        imu = Imu()
        imu.header = Header(frame_id="imu_link", stamp=timestamp)
        imu.linear_acceleration.x = data['accel'][0]
        imu.linear_acceleration.y = data['accel'][1]
        imu.linear_acceleration.z = data['accel'][2]
        imu.angular_velocity.x = data['gyro'][0]
        imu.angular_velocity.y = data['gyro'][1]
        imu.angular_velocity.z = data['gyro'][2]
        self.imu_pub.publish(imu)
        
        # 电池电压
        self.battery_pub.publish(Float32(data['battery']))

    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            # 读取串口数据
            try:
                data = self.serial_port.read(self.serial_port.in_waiting or 1)
                if data:
                    self.raw_buffer.extend(data)
                    
                    # 处理完整数据包
                    while True:
                        # 查找帧头位置
                        start = self.raw_buffer.find(bytes([self.PACKET_CONFIG['header']]))
                        if start == -1:
                            self.raw_buffer.clear()
                            break
                        
                        # 检查最小包长度
                        if len(self.raw_buffer) - start < self.PACKET_CONFIG['total_length']:
                            self.raw_buffer = self.raw_buffer[start:]
                            break
                        
                        # 提取候选数据包
                        end = start + self.PACKET_CONFIG['total_length']
                        candidate = self.raw_buffer[start:end]
                        
                        # 协议验证
                        if self.validate_packet(candidate):
                            rospy.logwarn('true data recieved')
                            self.process_packet(candidate)
                            self.raw_buffer = self.raw_buffer[end:]
                        else:
                            # 无效数据包，跳过当前字节
                            self.raw_buffer = self.raw_buffer[start+1:]
                            rospy.logwarn('wrong data recieved')
            except serial.SerialException as e:
                rospy.logerr(f"Serial communication error: {str(e)}")
                break

        self.serial_port.close()

if __name__ == '__main__':
    node = Stm32OdomNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        node.serial_port.close()