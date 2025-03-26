import network
import socket
from machine import UART, Pin
import _thread
import time
import gc

# ========== 硬件配置 ==========
SSID = 'jiuri'
PASSWORD = '123qwe123'

# 通信端口
PORTS = {
    'lidar': 10001,
    'cmd': 10002,
    'encoder': 10003
}

# 串口配置（根据实际硬件调整）
UARTS = {
    'lidar': UART(1, baudrate=115200, rx=Pin(16), tx=Pin(17), timeout=50),
    'mcu': UART(2, baudrate=115200, rx=Pin(25), tx=Pin(26), timeout=50)
}

# ========== 资源管理 ==========
class ConnectionManager:
    def __init__(self):
        self.active_connections = {}
        self.lock = _thread.allocate_lock()
    
    def add_connection(self, port, conn):
        with self.lock:
            self.active_connections[port] = conn
    
    def remove_connection(self, port):
        with self.lock:
            if port in self.active_connections:
                try:
                    self.active_connections[port].close()
                except:
                    pass
                del self.active_connections[port]
                gc.collect()

manager = ConnectionManager()

# ========== WiFi连接 ==========
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(SSID, PASSWORD)
        
        for _ in range(20):  # 10秒超时
            if wlan.isconnected():
                break
            time.sleep(0.5)
    
    if wlan.isconnected():
        print("IP:", wlan.ifconfig()[0])
        return True
    return False

# ========== 线程安全操作 ==========
def safe_send(data, conn, max_retry=3):
    for _ in range(max_retry):
        try:
            return conn.send(data)
        except OSError as e:
            if e.args[0] == 23:  # 资源不可用
                time.sleep_ms(50)
                continue
            raise
    return 0

def safe_recv(conn, size, timeout=1):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < timeout*1000:
        try:
            data = conn.recv(size)
            if data:
                return data
            time.sleep_ms(10)
        except OSError as e:
            if e.args[0] == 23:
                time.sleep_ms(50)
                continue
            raise
    return b''

# ========== 服务线程 ==========
def service_thread(port_name, uart_name, mode):
    port = PORTS[port_name]
    
    # 创建TCP服务器
    sock = socket.socket()
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', port))
    sock.listen(1)
    
    print(f"{port_name.upper()} service started on {port}")
    
    while True:
        try:
            # 接受新连接
            client, addr = sock.accept()
            print(f"{port_name} connected: {addr}")
            manager.add_connection(port, client)
            
            # 处理数据流
            while True:
                # 发送模式（LiDAR/Encoder）
                if mode == 'tx':
                    if UARTS[uart_name].any():
                        data = UARTS[uart_name].read(512)
                        if data:
                            safe_send(data, client)
                
                # 接收模式（CMD）
                elif mode == 'rx':
                    data = safe_recv(client, 128)
                    if data:
                        UARTS[uart_name].write(data)
                        
                time.sleep_ms(10)
                
        except OSError as e:
            print(f"{port_name} error({e.args[0]}): {e}")
            manager.remove_connection(port)
            time.sleep(1)
            
        except Exception as e:
            print(f"{port_name} critical: {type(e).__name__}")
            manager.remove_connection(port)
            time.sleep(5)

# ========== 主程序 ==========
def main():
    if not connect_wifi():
        return
    
    # 启动服务线程
    _thread.start_new_thread(service_thread, ('lidar', 'lidar', 'tx'))
    _thread.start_new_thread(service_thread, ('encoder', 'mcu', 'tx'))
    _thread.start_new_thread(service_thread, ('cmd', 'mcu', 'rx'))
    
    # 主线程监控
    while True:
        print(f"Memory free: {gc.mem_free()} bytes")
        time.sleep(10)

# ========== 安全启动 ==========
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down...")
        network.WLAN(network.STA_IF).disconnect()
        for port in PORTS.values():
            manager.remove_connection(port)

