import socket

TCP_IP = '192.168.3.109'  # ESP32的IP
TCP_PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((TCP_IP, TCP_PORT))

while True:
    data = sock.recv(2048)
    if data:
        print("Received:", data.hex())


