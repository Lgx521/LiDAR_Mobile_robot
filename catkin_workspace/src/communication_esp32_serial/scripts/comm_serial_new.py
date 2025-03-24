#!/usr/bin/env python3

import rospy
import serial
import struct
from sensor_msgs.msg import LaserScan

def parse_lidar_packet(data):
    """
    解析雷达数据包（示例为常见 2D LiDAR 协议）
    假设数据包格式：
    - 包头: 0x54 0x2C
    - 转速 (2字节, 低位在前)
    - 起始角度 (2字节, 单位 0.01度)
    - 距离数据 (每组3字节: 2字节距离 + 1字节强度)
    - 结束角度 (2字节)
    - 时间戳 (2字节)
    """
    if len(data) != 47 or data[0] != 0x54 or data[1] != 0x2C:
        raise ValueError("Invalid packet format")

    # 解析转速 (RPM)
    speed = struct.unpack('<H', data[2:4])[0]  # 小端序无符号短整型

    # 解析起始角度和结束角度 (单位：0.01度 → 弧度)
    start_angle = struct.unpack('<H', data[4:6])[0] * 0.01 * 3.14159265 / 180.0
    end_angle = struct.unpack('<H', data[-6:-4])[0] * 0.01 * 3.14159265 / 180.0

    # 解析距离和强度数据
    ranges = []
    intensities = []
    for i in range(6, len(data)-6, 3):
        dist = struct.unpack('<H', data[i:i+2])[0]  # 距离 (单位：毫米 → 米)
        intensity = data[i+2]
        ranges.append(dist / 1000.0)
        intensities.append(intensity)

    # 计算角度增量
    num_points = len(ranges)
    angle_increment = (end_angle - start_angle) / num_points if num_points > 0 else 0.0

    return {
        "speed_rpm": speed,
        "start_angle_rad": start_angle,
        "end_angle_rad": end_angle,
        "ranges": ranges,
        "intensities": intensities,
        "angle_increment": angle_increment,
        "timestamp": struct.unpack('<H', data[-4:-2])[0]
    }

def main():
    rospy.init_node('lidar_publisher')
    scan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)

    # 串口配置
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=115200,
        timeout=0.5
    )

    rospy.loginfo('Connected')

    scan_msg = LaserScan()
    scan_msg.header.frame_id = "map"  # 必须设置坐标系
    scan_msg.range_min = 0.1          # 单位：米
    scan_msg.range_max = 15

    while not rospy.is_shutdown():
        # 读取数据包
        try:
            header = ser.read(2)
            if header == b'\x54\x2C':
                packet = header + ser.read(45)  # 总包长47字节
                if len(packet) == 47:
                    data = parse_lidar_packet(packet)
                    
                    # 填充 LaserScan 消息
                    scan_msg.header.stamp = rospy.Time.now()
                    scan_msg.angle_min = data["start_angle_rad"]
                    scan_msg.angle_max = data["end_angle_rad"]
                    scan_msg.angle_increment = data["angle_increment"]
                    scan_msg.time_increment = 1.0 / (data["speed_rpm"] / 60.0 * len(data["ranges"]))
                    scan_msg.scan_time = 1.0 / (data["speed_rpm"] / 60.0)
                    scan_msg.ranges = data["ranges"]
                    scan_msg.intensities = data["intensities"]
                    
                    scan_pub.publish(scan_msg)
            else:
                rospy.logwarn('data distorted')
        except serial.SerialException as e:
            rospy.logerr(f"Serial error: {e}")
            # 重连逻辑
            ser.close()
            try:
                ser.open()
            except:
                pass
        except Exception as e:
            rospy.logerr(f"Processing error: {e}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        if ser.is_open:
            ser.close()