import asyncio
from bleak import BleakClient

# ESP32的BLE MAC地址（格式必须为 XX:XX:XX:XX:XX:XX）
TARGET_MAC = "A0:B7:65:47:EF:96"

async def main():
    async with BleakClient(TARGET_MAC) as client:
        print(f"Connected to {client.address}")
        # 读取服务/特征值
        services = await client.get_services()
        for service in services:
            print(f"Service: {service.uuid}")
            for char in service.characteristics:
                print(f"  Characteristic: {char.uuid}")

asyncio.run(main())