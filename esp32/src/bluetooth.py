import ubluetooth
import machine
import time
import struct
from micropython import const
from machine import UART

# BLE 定义
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

# 自定义 UUID
_UART_SERVICE_UUID = ubluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX_UUID = ubluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")  # 通知特性
_UART_RX_UUID = ubluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")  # 写特性

_UART_TX = ( _UART_TX_UUID, ubluetooth.FLAG_NOTIFY,)
_UART_RX = ( _UART_RX_UUID, ubluetooth.FLAG_WRITE,)

_UART_SERVICE = ( _UART_SERVICE_UUID, (_UART_TX, _UART_RX,), )

class BLEUART:
    def __init__(self, ble, name="ESP32_UART"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle_tx, self._handle_rx), ) = self._ble.gatts_register_services((_UART_SERVICE,))
        
        self._connections = set()
        self._payload = self._advertise_payload(name=name)
        self._advertise()

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            print("Connected:", conn_handle)
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            print("Disconnected:", conn_handle)
            self._connections.remove(conn_handle)
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, attr_handle = data
            if attr_handle == self._handle_rx:
                received = self._ble.gatts_read(self._handle_rx)
                print("RX:", received)

    def send(self, data):
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._handle_tx, data)

    def _advertise_payload(self, name=None):
        payload = bytearray()
        payload.extend(struct.pack("BB", len(name) + 1, 0x09))  # 完整本地名称
        payload.extend(name.encode('utf-8'))
        return payload

    def _advertise(self, interval_us=500000):
        print("Starting advertising")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

# 初始化 BLE
ble = ubluetooth.BLE()
uart_ble = BLEUART(ble)

# 初始化硬件串口
uart = UART(1, baudrate=9600, tx=17, rx=16)  # 根据自己的硬件改 GPIO

# 循环读取 UART 数据并发送到 BLE
while True:
    if uart.any():
        data = uart.read()
        uart_ble.send(data)
