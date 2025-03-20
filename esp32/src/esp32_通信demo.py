import network
import socket
import time

# 连接 Wi-Fi
SSID = "esp32-ap"
PASSWORD = "12345678"

sta = network.WLAN(network.STA_IF)
sta.active(True)
sta.connect(SSID, PASSWORD)

while not sta.isconnected():
    time.sleep(1)

print("success IP:", sta.ifconfig()[0])

# 配置 TCP 客户端
SERVER_IP = "pc IP"  # 例如 "192.168.1.100"
SERVER_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((SERVER_IP, SERVER_PORT))

# 发送数据到 ROS
while True:
    message = "Hello from ESP32"
    sock.send(message.encode())
    # 接收来自 ROS 的指令
    data = sock.recv(1024).decode()
    if data:
        print("received", data)
    time.sleep(1)