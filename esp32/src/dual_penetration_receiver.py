# 文件名：sta_client.py
import network
import socket
from machine import UART
import sys

# 配置STA连接AP
AP_SSID = "ESP32-AP"
AP_PASSWORD = "123qwe123"
SERVER_IP = "192.168.4.1"  # AP的固定IP
SERVER_PORT = 1234


# 连接AP热点
sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.connect(AP_SSID, AP_PASSWORD)
while not sta.isconnected():
    pass
print("Connected to AP. IP:", sta.ifconfig()[0])

# 连接TCP服务器
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((SERVER_IP, SERVER_PORT))
print("Connected to server")

# 数据透传：串口 ↔ TCP服务器
while True:
    try:
        data = client.recv(1024)
        if data:
            sys.stdout.write(data)
            print("Received from server:", data) # for test purpose
    except:
        pass
