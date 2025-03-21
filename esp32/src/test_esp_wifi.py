# 文件名：pc_client.py
import socket

# ESP32的AP热点IP和端口
SERVER_IP = "192.168.4.1"  # ESP32默认AP的IP
SERVER_PORT = 1234

# 创建TCP客户端
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((SERVER_IP, SERVER_PORT))
print(f"已连接到ESP32: {SERVER_IP}:{SERVER_PORT}")

# 持续接收数据
try:
    while True:
        data = client.recv(1024)
        if data:
            print(data)  # 根据实际编码调整
except KeyboardInterrupt:
    print("用户中断")
finally:
    client.close()