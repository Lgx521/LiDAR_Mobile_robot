#!/usr/bin/env python3
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
grandparent_dir = os.path.abspath(os.path.join(current_dir, '..', '..'))
sys.path.insert(0, grandparent_dir)

from util import bridge as brg


if __name__ == '__main__':
    LiDAR = brg.VirtualSerialBridge(brg.esp32_ip, brg.LIDAR_PORT, brg.virt_com_LiDAR)
    LiDAR.run_r()

    Encoder = brg.VirtualSerialBridge(brg.esp32_ip, brg.ENCODER_PORT, brg.virt_com_Encoder)
    Encoder.run_r()



