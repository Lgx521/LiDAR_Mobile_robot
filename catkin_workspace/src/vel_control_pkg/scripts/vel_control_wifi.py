#!/usr/bin/env python3

import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
grandparent_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))
sys.path.insert(0, grandparent_dir)

from util import bridge as brg

import rospy
import struct
import time
from geometry_msgs.msg import Twist


class RobotSerialControl:
    def __init__(self):
        # Wifi 桥接对象
        self.bridge = brg.VirtualSerialBridge(brg.esp32_ip, brg.CMD_PORT)
        
        # 获取ROS参数
        self.scale_factor = rospy.get_param('~scale_factor', 1000)  # 速度缩放因子

        # 运动控制参数
        self.last_cmd_time = time.time()
        self.timeout_threshold = 0.1  # 超时时间(秒)
        
        # 订阅cmd_vel话题
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # 超时检测定时器
        rospy.Timer(rospy.Duration(0.1), self.check_timeout)

    def build_velocity_frame(self, vx, vy, vz):
        """构建符合协议的数据帧"""
        # 限制速度范围(short类型范围)
        vx = max(min(int(vx * self.scale_factor), 32767), -32768)
        vy = max(min(int(vy * self.scale_factor), 32767), -32768)
        vz = max(min(int(vz * self.scale_factor), 32767), -32768)

        # 协议帧结构
        frame = bytearray()
        # 帧头 (1 byte)
        frame.append(0x7B)
        # 预留位 (2 bytes)
        frame.extend([0x00, 0x00])
        # 三轴速度 (每个轴2 bytes，大端字节序)
        frame.extend(struct.pack('>h', vx))  # X轴
        frame.extend(struct.pack('>h', vy))  # Y轴
        frame.extend(struct.pack('>h', vz))  # Z轴
        # 校验位 (异或校验)
        checksum = frame[0]
        for b in frame[1:]:  # 从预留位开始计算
            checksum = checksum ^ b
        frame.append(checksum & 0xFF)
        # 帧尾 (1 byte)
        frame.append(0x7D)
        # rospy.loginfo(frame.hex())
        return frame

    def cmd_vel_callback(self, msg):
        """处理速度指令"""
        # 构建协议数据帧
        data_frame = self.build_velocity_frame(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z
        )
        
        # 发送串口数据
        try:
            self.bridge.run_t(data_frame)
            rospy.loginfo(data_frame.hex())
            self.last_cmd_time = time.time()
            rospy.logdebug(f"Sent: {data_frame.hex()}")
        except Exception as e:
            rospy.logerr(f"Wifi send error: {e}")

    def check_timeout(self, event):
        """超时停止机制"""
        if (time.time() - self.last_cmd_time) > self.timeout_threshold:
            stop_frame = self.build_velocity_frame(0, 0, 0)
            try:
                # self.ser.write(stop_frame)
                pass
                # rospy.logwarn("Velocity command timeout, sent stop command")
            except Exception as e:
                rospy.logerr(f"Failed to send stop: {e}")

if __name__ == '__main__':
    rospy.init_node('robot_serial_control')
    node = RobotSerialControl()
    rospy.spin()