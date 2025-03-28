#!/usr/bin/env python3
import rospy
import serial
import struct
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion
from tf.transformations import quaternion_from_euler

'''
In this file, the Twist data type is not published
Only publish Odometry, tf and batteryState data.
This file has already passed static test.
This file is still under pending, waiting for further validation on the real robot.
'''


class Stm32SerialNode:
    def __init__(self):
        rospy.init_node('stm32_serial_node')
        
        # 初始化发布器
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.battery_pub = rospy.Publisher('/battery', BatteryState, queue_size=10)

        # 串口配置
        self.ser = serial.Serial(
            port='/dev/ttyVIRT3',
            baudrate=115200,
            timeout=0.1
        )
        self.buffer = bytearray()
        self.packet_length = 24  # 总数据包长度

        # Odom初始化
        self.odom_pose = Pose()
        self.odom_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        self.odom_twist = Twist()

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
            # 解析IMU数据
            imu = Imu()
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = "imu_link"
            
            # 加速度（数组号6-8）
            imu.linear_acceleration.x = self.parse_short(packet[8:10])   # 数组6
            imu.linear_acceleration.y = self.parse_short(packet[10:12])  # 数组7
            imu.linear_acceleration.z = self.parse_short(packet[12:14])  # 数组8
            
            # 角速度（数组9-11）
            imu.angular_velocity.x = self.parse_short(packet[14:16])      # 数组9
            imu.angular_velocity.y = self.parse_short(packet[16:18])     # 数组10
            imu.angular_velocity.z = self.parse_short(packet[18:20])     # 数组11

            # 解析里程计数据（数组3-5）
            odom = Odometry()
            odom.header.stamp = imu.header.stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            
            # 速度信息
            odom.twist.twist.linear.x = self.parse_short(packet[2:4])    # 数组3
            odom.twist.twist.linear.y = self.parse_short(packet[4:6])    # 数组4
            odom.twist.twist.linear.z = self.parse_short(packet[6:8])    # 数组5
            
            # 位置信息（需积分实现，此处初始化）
            odom.pose.pose = self.odom_pose

            # 电池电压（数组12）
            battery = BatteryState()
            battery.header.stamp = imu.header.stamp
            battery.voltage = self.parse_short(packet[20:22])            # 数组12

            return imu, odom, battery

        except Exception as e:
            rospy.logerr("数据处理错误: %s", str(e))
            return None

    def run(self):
        rate = rospy.Rate(20)  # 20Hz发布频率
        while not rospy.is_shutdown():
            # 读取串口数据
            data = self.ser.read(self.ser.in_waiting or 1)
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
                        result = self.process_packet(packet)
                        if result:
                            self.imu_pub.publish(result[0])
                            self.odom_pub.publish(result[1])
                            self.battery_pub.publish(result[2])
                    else:
                        rospy.logwarn("无效数据包，已丢弃")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = Stm32SerialNode()
        node.run()
    except rospy.ROSInterruptException:
        node.ser.close()
