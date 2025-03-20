from machine import UART, Pin
import time
import sys

# 初始化 UART1（RX 引脚为 GPIO9，但可自定义）
uart = UART(1, baudrate=115200, rx=Pin(16), tx=Pin(17))  # 根据实际引脚调整

while True:
    if uart.any():  # 检查是否有数据可读
        data = uart.read()  # 读取所有字节
        sys.stdout.write(data)