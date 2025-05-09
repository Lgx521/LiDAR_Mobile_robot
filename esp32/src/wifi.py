import network
import socket
from machine import UART, Pin
import time

# WiFi 配置
SSID = '甘圣哲的iPhone'
PASSWORD = 'gan123456'

# 网络参数
PORT = 1234
BUFFER_SIZE = 1024

# 初始化 WiFi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

# 初始化串口 (UART1: TX=GPIO4, RX=GPIO5)
uart = UART(1, baudrate=115200, rx=Pin(16), tx=Pin(17))

# 这是接受计算机发来的控制数据并发送给底盘的串口
# 预计板载与PCB上
uart_out = UART()


def connect_wifi():
    print("Connecting to WiFi...")
    if not wlan.isconnected():
        print('Connecting...')
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            pass
#             print('Connection failed')
    print('Connected: ', wlan.ifconfig())

def start_tcp_server():
    # 创建TCP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('0.0.0.0', PORT))
    sock.listen(1)
    print('TCP server started on port', PORT)
    return sock

def main():
    connect_wifi()
    server_sock = start_tcp_server()
    client_sock = None
        
    while True:
        try:
            # 等待客户端连接
            if not client_sock:
                print("Waiting for client...")
                client_sock, addr = server_sock.accept()
                print("Client connected from:", addr)
            
            # 检查客户端连接状态
            if client_sock:
                # 读取LiDAR串口数据并转发
                if uart.any():
                    data = uart.read(BUFFER_SIZE)
                    try:
                        client_sock.sendall(data)
                    except:
                        print('error')
                        client_sock.close()
                        client_sock = None
                        continue
                
                # 读取数据
                try:
                    data = client_sock.recv(BUFFER_SIZE)
                    if data:
                        uart_out.write(data)
                except:
                    pass

            
        except OSError as e:
            print("Connection error:", e)
            if client_sock:
                client_sock.close()
                client_sock = None
            time.sleep(1)
            
        except KeyboardInterrupt:
            print("Server shutdown")
            server_sock.close()
            wlan.disconnect()
            break

if __name__ == '__main__':
    main()
