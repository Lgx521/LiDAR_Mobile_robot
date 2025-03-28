#!/usr/bin/env python3
import sys
import os
import threading

current_dir = os.path.dirname(os.path.abspath(__file__))
grandparent_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))
sys.path.insert(0, grandparent_dir)

from util import bridge as brg


if __name__ == '__main__':
    def lidar_thread():
        LiDAR = brg.VirtualSerialBridge(brg.esp32_ip, brg.LIDAR_PORT, brg.virt_com_LiDAR)
        LiDAR.run_r()

    def encoder_thread():
        Encoder = brg.VirtualSerialBridge(brg.esp32_ip, brg.ENCODER_PORT, brg.virt_com_Encoder)
        Encoder.run_r()
    
    lidar = threading.Thread(target = lidar_thread)
    encoder = threading.Thread(target = encoder_thread)
    # 启动线程
    lidar.start()
    encoder.start()


