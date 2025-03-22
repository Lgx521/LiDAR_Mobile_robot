# 文件名：ap_server.py
import network
import socket
from machine import UART, Pin

# 配置AP热点
AP_SSID = "ESP32-AP"  # 热点名称
AP_PASSWORD = "123qwe123"  # 密码（至少8位）

# 初始化串口
uart = UART(1, baudrate=115200, rx=Pin(16), tx=Pin(17))  # UART1（GPIO9=RX, GPIO10=TX）

# 创建AP热点
ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=AP_SSID, password=AP_PASSWORD)
print("AP IP:", ap.ifconfig()[0])  # 默认IP为 192.168.4.1

# 创建TCP服务器
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(("0.0.0.0", 1234))  # 监听所有IP，端口1234
server.listen(1)
print("Waiting for client connection...")

# 接受客户端连接
client, addr = server.accept()
print("Client connected from:", addr)

# 数据透传：串口 ↔ TCP客户端
while True:
    # 从串口读取数据并发送给客户端
    data = b'0x550x2c'
    client.send(data)
#     if uart.any():
#         data = uart.read(1024)
#         client.send(data)
#         print("Sent to client:", data)
#     
#     # 从客户端接收数据并写入串口（双向透传可选）
#     try:
#         data = client.recv(1024)
#         if data:
#             uart.write(data)
#             print("Received from client:", data)
    # except:
    #     pass
