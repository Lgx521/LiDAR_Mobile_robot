import asyncio
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "ESP32"
UART_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

async def main():
    print("扫描 BLE 设备中...")
    devices = await BleakScanner.discover()

    target_device = None
    for d in devices:
        print(f"发现设备: {d.name}, 地址: {d.address}")
        if d.name == DEVICE_NAME:
            target_device = d
            break

    if not target_device:
        print(f"未找到设备 {DEVICE_NAME}")
        return

    print(f"连接到设备: {target_device.name}, 地址: {target_device.address}")

    async with BleakClient(target_device) as client:  # ← 改这里
        if not client.is_connected:
            print("连接失败！")
            return

        print("已连接，等待数据...")

        def handle_rx(sender, data):
            print(f"[{sender}] 接收到数据: {data.decode(errors='ignore')}")

        await client.start_notify(UART_TX_UUID, handle_rx)

        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("取消监听...")
            await client.stop_notify(UART_TX_UUID)

asyncio.run(main())