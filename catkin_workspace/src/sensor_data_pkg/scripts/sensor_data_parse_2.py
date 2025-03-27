#!/usr/bin/env python
import rospy
import serial
import struct
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import BatteryState

class SerialParser:
    def __init__(self):
        rospy.init_node('stm32_serial_parser', anonymous=True)
        
        # 初始化发布器
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.battery_pub = rospy.Publisher('/battery', BatteryState, queue_size=10)

        # 串口配置
        self.ser = serial.Serial(
            port='/dev/ttyVIRT3',
            baudrate=115200,
            timeout=0.1
        )
        self.buffer = bytearray()
        self.packet_length = 24  # 总数据包长度（根据协议计算）

    def convert_short(self, data):
        """转换short类型数据到浮点数"""
        return struct.unpack('>h', data)[0] / 1000.0

    def parse_packet(self, packet):
        """解析完整数据包"""
        try:
            # 校验帧头帧尾
            if packet[0] != 0x7B or packet[-1] != 0x7D:
                return None

            # 校验和检查（异或校验）
            checksum = 0
            for b in packet[0:-2]:
                checksum = checksum ^ b
            if checksum != packet[-2]:
                rospy.logwarn("Checksum error!")
                return None

            # 解析第一部分数据
            twist = Twist()
            twist.linear.x = self.convert_short(packet[2:4])   # x轴速度
            twist.linear.y = self.convert_short(packet[4:6])   # y轴速度
            twist.linear.z = self.convert_short(packet[6:8])   # z轴速度

            # 解析IMU数据
            imu = Imu()
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = "imu_link"
            imu.linear_acceleration.x = self.convert_short(packet[8:10])   # X加速度
            imu.linear_acceleration.y = self.convert_short(packet[10:12])  # Y加速度
            imu.linear_acceleration.z = self.convert_short(packet[12:14])  # Z加速度
            imu.angular_velocity.x = self.convert_short(packet[14:16])     # X角速度
            imu.angular_velocity.y = self.convert_short(packet[16:18])     # Y角速度
            imu.angular_velocity.z = self.convert_short(packet[18:20])     # Z角速度

            # 解析电池数据
            battery = BatteryState()
            battery.header.stamp = rospy.Time.now()
            battery.voltage = self.convert_short(packet[20:22])  # 电池电压

            # 解析里程计数据
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.twist.twist = twist  # 使用速度信息

            return (twist, odom, imu, battery)

        except Exception as e:
            rospy.logerr("Packet parse error: %s", str(e))
            return None

    def run(self):
        rate = rospy.Rate(20)  # 20Hz发布频率
        while not rospy.is_shutdown():
            # 读取串口数据
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                self.buffer.extend(data)
                
                # 搜索完整数据包
                while len(self.buffer) >= self.packet_length:
                    start = self.buffer.find(b'\x7b')
                    if start < 0:
                        self.buffer.clear()
                        break
                    
                    # 检查数据包长度
                    if len(self.buffer[start:]) < self.packet_length:
                        break
                    
                    packet = self.buffer[start:start+self.packet_length]
                    self.buffer = self.buffer[start+self.packet_length:]
                    
                    # 解析并发布数据
                    result = self.parse_packet(packet)
                    if result:
                        self.twist_pub.publish(result[0])
                        self.odom_pub.publish(result[1])
                        self.imu_pub.publish(result[2])
                        self.battery_pub.publish(result[3])
            
            rate.sleep()

if __name__ == '__main__':
    try:
        parser = SerialParser()
        parser.run()
    except rospy.ROSInterruptException:
        parser.ser.close()