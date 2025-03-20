import network
import time
from machine import Pin
import usocket as socket

# WiFi配置
SSID = "Your_WiFi_SSID"
PASSWORD = "Your_WiFi_Password"
SERVER_IP = "127.0.0.1"  # ROS主机的IP
SERVER_PORT = 8080           # 自定义端口

# 初始化WiFi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

def connect_wifi():
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            time.sleep(1)
    print("WiFi Connected! IP:", wlan.ifconfig()[0])

def send_data():
    # 创建TCP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((SERVER_IP, SERVER_PORT))
        print("Connected to ROS Server")
        while True:
            # 构造要发送的数据（示例：模拟传感器数据）
            data = "Temp:25.5,Humidity:60"
            sock.send(data.encode('utf-8'))
            print("Sent:", data)
            time.sleep(1)
    except Exception as e:
        print("Error:", e)
    finally:
        sock.close()

# 主程序
connect_wifi()
send_data()