#! /usr/bin/env python

import rospy
import socket
from std_msgs.msg import String

def main():
    # === 1. 初始化ROS节点 ===
    rospy.init_node('tcp_ros_bridge')
    
    # === 2. 创建发布者和订阅者 ===
    # 发布来自ESP32的数据到话题 /esp32/sensor
    pub = rospy.Publisher('/esp32/sensor', String, queue_size=10)
    
    # 订阅话题 /esp32/command，接收用户指令
    def callback(msg):
        # 收到指令后通过TCP发送给ESP32
        conn.send(msg.data.encode())
    sub = rospy.Subscriber('/esp32/command', String, callback)
    
    # === 3. 启动TCP服务器 ===
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('0.0.0.0', 8888))  # 监听所有网络接口的8888端口
    sock.listen(1)                # 允许最多1个客户端连接
    rospy.loginfo("等待ESP32连接...")
    
    # 接受ESP32的连接
    conn, addr = sock.accept()
    rospy.loginfo(f"ESP32已连接: {addr}")
    
    # === 4. 主循环：接收数据并转发到ROS ===
    while not rospy.is_shutdown():
        try:
            data = conn.recv(1024).decode()  # 接收数据
            if data:
                pub.publish(String(data))    # 发布到ROS话题
        except:
            break  # 连接中断时退出
    
    # 关闭连接
    conn.close()
    sock.close()

if __name__ == '__main__':
    main()