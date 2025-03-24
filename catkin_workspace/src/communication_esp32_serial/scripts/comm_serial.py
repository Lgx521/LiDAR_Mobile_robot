#!/usr/bin/env python3

import rospy
import serial
import struct
from sensor_msgs.msg import LaserScan

class LidarROS:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        rospy.init_node('lidar_publisher', anonymous=True)
        self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)
        self.serial_port = port
        self.baudrate = baudrate
        self.ser = None
        self.connect_serial()

    def connect_serial(self):
        """尝试连接串口"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.5)
            rospy.loginfo(f'Connected to {self.serial_port} at {self.baudrate} baud')
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to {self.serial_port}: {e}")
            rospy.signal_shutdown("Serial connection failed")  # 终止ROS节点

    def read_packet(self):
        """读取完整的LIDAR数据包"""
        try:
            header = self.ser.read(2)
            if header == b'\x54\x2C':
                packet = header + self.ser.read(45)  # 总包长47字节
                if len(packet) == 47:
                    return packet
                else:
                    rospy.logwarn("Incomplete packet received")
            return None
        except serial.SerialException as e:
            rospy.logerr(f"Serial error: {e}")
            self.reconnect_serial()
            return None

    def reconnect_serial(self):
        """串口异常时尝试重新连接"""
        rospy.loginfo("Reconnecting serial...")
        if self.ser:
            self.ser.close()
        rospy.sleep(1)
        self.connect_serial()

    @staticmethod
    def parse_lidar_packet(data):
        """解析LIDAR数据包"""
        if len(data) != 47 or data[0] != 0x54 or data[1] != 0x2C:
            rospy.logwarn('data distorted')
            raise ValueError("Invalid packet format")

        speed_rpm = struct.unpack('<H', data[2:4])[0]
        start_angle = struct.unpack('<H', data[4:6])[0] * 0.01 * 3.14159265 / 180.0
        end_angle = struct.unpack('<H', data[-6:-4])[0] * 0.01 * 3.14159265 / 180.0

        ranges, intensities = [], []
        for i in range(6, 41, 3):  # 读取12组点数据
            dist = struct.unpack('<H', data[i:i+2])[0] / 1000.0  # mm → m
            intensity = data[i+2]
            ranges.append(dist)
            intensities.append(intensity)

        angle_increment = (end_angle - start_angle) / len(ranges) if ranges else 0.0
        timestamp = struct.unpack('<H', data[-4:-2])[0]

        return {
            "speed_rpm": speed_rpm,
            "start_angle_rad": start_angle,
            "end_angle_rad": end_angle,
            "ranges": ranges,
            "intensities": intensities,
            "angle_increment": angle_increment,
            "timestamp": timestamp
        }

    def publish_scan(self):
        """读取数据并发布ROS消息"""
        scan_msg = LaserScan()
        scan_msg.header.frame_id = "map"
        scan_msg.range_min = 0.1  # 最小有效测距
        scan_msg.range_max = 15   # 最大测距

        rate = rospy.Rate(15)  # 发布频率
        while not rospy.is_shutdown():
            packet = self.read_packet()
            if packet:
                try:
                    data = self.parse_lidar_packet(packet)

                    scan_msg.header.stamp = rospy.Time.now()
                    scan_msg.angle_min = data["start_angle_rad"]
                    scan_msg.angle_max = data["end_angle_rad"]
                    scan_msg.angle_increment = data["angle_increment"]
                    scan_msg.time_increment = 1.0 / (data["speed_rpm"] / 60.0 * len(data["ranges"]))
                    scan_msg.scan_time = 1.0 / (data["speed_rpm"] / 60.0)
                    scan_msg.ranges = data["ranges"]
                    scan_msg.intensities = data["intensities"]

                    self.scan_pub.publish(scan_msg)
                except Exception as e:
                    rospy.logerr(f"Error processing packet: {e}")
            rate.sleep()

if __name__ == '__main__':
    try:
        lidar_ros = LidarROS()
        lidar_ros.publish_scan()
    except rospy.ROSInterruptException:
        if lidar_ros.ser and lidar_ros.ser.is_open:
            lidar_ros.ser.close()
