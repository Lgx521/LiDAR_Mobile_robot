#!/usr/bin/env python3
import rospy
import serial
import struct
import numpy as np
import math # For math.cos, math.sin

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, TransformStamped # Added TransformStamped
from tf.transformations import quaternion_from_euler
import tf2_ros # Added for TF broadcasting

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

        # 初始化 TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 串口配置
        self.ser = serial.Serial(
            port=rospy.get_param('~serial_port', '/dev/ttyVIRT3'), # Use ROS param for port
            baudrate=rospy.get_param('~baudrate', 115200),      # Use ROS param for baudrate
            timeout=0.1
        )
        self.buffer = bytearray()
        self.packet_length = 24  # 总数据包长度

        # Odom 初始化状态变量 (用于积分)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 时间戳
        self.last_time = rospy.Time.now()
        
        # 从参数服务器获取 frame_id
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.imu_frame_id = rospy.get_param('~imu_frame_id', 'imu_link')


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
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            if dt <= 0: # 防止时间回溯或dt为0
                rospy.logwarn_throttle(1.0, "dt is zero or negative, skipping update.")
                return None

            # 解析IMU数据
            imu = Imu()
            imu.header.stamp = current_time
            imu.header.frame_id = self.imu_frame_id
            
            # 加速度（数组号6-8）
            imu.linear_acceleration.x = self.parse_short(packet[8:10])   # 数组6
            imu.linear_acceleration.y = self.parse_short(packet[10:12])  # 数组7
            imu.linear_acceleration.z = self.parse_short(packet[12:14])  # 数组8
            
            # 角速度（数组9-11）
            imu.angular_velocity.x = self.parse_short(packet[14:16])      # 数组9
            imu.angular_velocity.y = self.parse_short(packet[16:18])     # 数组10
            imu.angular_velocity.z = self.parse_short(packet[18:20])     # 数组11
            # 注意：IMU的 orientation 通常由姿态解算算法（如 Madgwick, Mahony）填充
            # 如果STM32不提供四元数，这里可以留空或设置为单位四元数
            imu.orientation_covariance[0] = -1 # 表示方向未提供

            # 解析里程计数据（数组3-5）
            # 假设 packet[2:4] 是 vx, packet[4:6] 是 vy, packet[6:8] 是 vth (绕z轴角速度)
            # 如果 packet[6:8] 是 vz，你需要从 IMU 获取角速度用于2D里程计
            vx = self.parse_short(packet[2:4])    # 数组3
            vy = self.parse_short(packet[4:6])    # 数组4 (对于差速轮通常为0，但如果全向轮则有值)
            vth = self.parse_short(packet[6:8])   # 数组5 (假设这是角速度 rad/s)

            # 计算里程计位姿 (简单欧拉积分)
            # 假设 vx, vy 是在 base_link 坐标系下的速度
            delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
            delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
            delta_theta = vth * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # 创建四元数
            odom_quat = quaternion_from_euler(0, 0, self.theta)

            # 创建并发布 TF 变换
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0 # 2D 机器人
            t.transform.rotation.x = odom_quat[0]
            t.transform.rotation.y = odom_quat[1]
            t.transform.rotation.z = odom_quat[2]
            t.transform.rotation.w = odom_quat[3]
            self.tf_broadcaster.sendTransform(t)

            # 创建并发布 Odometry 消息
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = self.odom_frame_id
            odom.child_frame_id = self.base_frame_id
            
            # 设置位置
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = odom_quat[0]
            odom.pose.pose.orientation.y = odom_quat[1]
            odom.pose.pose.orientation.z = odom_quat[2]
            odom.pose.pose.orientation.w = odom_quat[3]
            # 设置协方差 (可以根据实际情况调整，这里用较小的值表示比较自信)
            # Px, Py, Pz, Qx, Qy, Qz
            odom.pose.covariance[0] = 0.01  # x
            odom.pose.covariance[7] = 0.01  # y
            odom.pose.covariance[14] = 1e9 # z (2D, 无信息)
            odom.pose.covariance[21] = 1e9 # rot x (2D, 无信息)
            odom.pose.covariance[28] = 1e9 # rot y (2D, 无信息)
            odom.pose.covariance[35] = 0.01 # rot z (theta)

            # 设置速度
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy # 通常为0，除非是全向轮
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = vth
             # 设置速度协方差
            odom.twist.covariance[0] = 0.01  # vx
            odom.twist.covariance[7] = 0.01  # vy
            odom.twist.covariance[14] = 1e9 # vz
            odom.twist.covariance[21] = 1e9 # wx
            odom.twist.covariance[28] = 1e9 # wy
            odom.twist.covariance[35] = 0.01 # wz (vth)

            # 电池电压（数组12）
            battery = BatteryState()
            battery.header.stamp = current_time
            battery.voltage = self.parse_short(packet[20:22])            # 数组12
            battery.present = True # 假设电池总是存在

            self.last_time = current_time # 更新上次时间
            return imu, odom, battery

        except struct.error as e:
            rospy.logerr("数据包解析错误 (struct.error): %s. Packet: %s", str(e), packet.hex())
            return None
        except Exception as e:
            rospy.logerr("数据处理错误: %s", str(e))
            return None

    def run(self):
        rate = rospy.Rate(rospy.get_param('~publish_rate', 20))  # 20Hz发布频率, 可通过参数配置
        
        # 等待串口打开
        if not self.ser.is_open:
            try:
                self.ser.open()
                rospy.loginfo(f"串口 {self.ser.portstr} 打开成功.")
            except serial.SerialException as e:
                rospy.logerr(f"无法打开串口 {self.ser.portstr}: {e}")
                return

        while not rospy.is_shutdown():
            bytes_to_read = self.ser.in_waiting
            if bytes_to_read > 0:
                data = self.ser.read(bytes_to_read)
                if data:
                    self.buffer.extend(data)
                
                # 处理完整数据包
                while len(self.buffer) >= self.packet_length:
                    start_index = self.buffer.find(b'\x7b') # 查找帧头
                    if start_index == -1: # 没有找到帧头，清空缓冲区
                        rospy.logwarn_throttle(5.0, "未找到帧头，清空缓冲区")
                        self.buffer.clear()
                        break
                    
                    # 如果帧头不是缓冲区的第一个字节，丢弃之前的数据
                    if start_index > 0:
                        rospy.logwarn_throttle(5.0, f"丢弃帧头前无效数据: {self.buffer[:start_index].hex()}")
                        self.buffer = self.buffer[start_index:]
                    
                    # 检查是否有足够的数据构成一个完整包
                    if len(self.buffer) < self.packet_length:
                        break # 数据不够，等待更多数据
                    
                    packet = self.buffer[:self.packet_length]
                    
                    if self.verify_packet(packet):
                        result = self.process_packet(packet)
                        if result:
                            self.imu_pub.publish(result[0])
                            self.odom_pub.publish(result[1])
                            self.battery_pub.publish(result[2])
                        # 处理完一个包后，从缓冲区移除这个包
                        self.buffer = self.buffer[self.packet_length:]
                    else:
                        rospy.logwarn_throttle(1.0, f"无效数据包或校验失败，已丢弃: {packet.hex()}")
                        # 丢弃这个无效的包的第一个字节（帧头），然后继续搜索下一个帧头
                        # 这样可以避免如果校验失败但帧头是真的，我们不会跳过太多数据
                        self.buffer = self.buffer[1:] 
            
            rate.sleep()

if __name__ == '__main__':
    node = None # 初始化为 None
    try:
        node = Stm32SerialNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("STM32 Serial Node 关闭中.")
    except serial.SerialException as e:
        rospy.logerr(f"串口操作异常: {e}")
    finally:
        if node and node.ser and node.ser.is_open:
            rospy.loginfo(f"关闭串口 {node.ser.portstr}")
            node.ser.close()