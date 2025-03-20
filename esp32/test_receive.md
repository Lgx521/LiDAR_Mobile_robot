
```python
# 定义一个函数来解析数据包
def parse_data(data):
    speed = data[1] * 256 + data[0]  # 计算转速
    start_angle = (data[3] * 256 + data[2]) / 100.0  # 计算起始角度
    distances_and_intensities = []  # 初始化距离和光强列表

    # 遍历数据包中的距离和光强数据
    for x in range(4, 40, 3):
        distance = data[x+1] * 256 + data[x]  # 计算距离
        #打印某个角度范围内的距离
        # if  start_angle<100 and start_angle>80:
        #     print(distance)
        intensity = data[x + 2]  # 获取光强
        distances_and_intensities.append((distance, intensity))  # 添加到列表

    end_angle = (data[41] * 256 + data[40]) / 100.0  # 计算结束角度
    time = data[43]*256+data[42]
    return speed, start_angle, distances_and_intensities, end_angle ,time

# 定义一个函数来打印数据包的内容
def print_data(speed, start_angle, distances_and_intensities, end_angle, last_angle,time):
    if last_angle - start_angle > 100:
        print("*******************************")

    # 打印转速、起始角度和数据点
    print("转速:", speed, end="\t")
    print("起始角度:", start_angle, end="\t")
    print("数据【距离（mm）|光强】*12个点：", end="\t")

    # 打印每个数据点的距离和光强
    for distance, intensity in distances_and_intensities:
        print(distance, "|", intensity, end="\t")
    print("时间戳:",time,end="\t")
    print("结束角度:", end_angle, end="\n\n")


# 主程序开始
if __name__ == '__main__':
    last_angle = 0  # 初始化上一个角度
    # LD19波特率：230400     LD14波特率：115200
    # ser= serial.Serial('/dev/wheeltec_lidar', 115200)  # ubuntu，如果未修改串口别名，可通过 ll /dev 查看雷达具体端口再进行更改
    ser = serial.Serial("COM3", 115200, timeout=5)  # window系统，需要先通过设备管理器确认串口COM号
    # 循环读取数据
    while True:
        try:
            data = ser.read(1)  # 读取3个字节的数据
            if data[0] == 0x54:
                data = ser.read(1)  # 读取3个字节的数据
                if data[0] == 0x2C:
                    # 检查数据包的头部
                        data = ser.read(45)  # 读取剩余的55个字节的数据
                        # 解析和打印数据包
                        speed, start_angle, distances_and_intensities, end_angle,time = parse_data(data)
                        print_data(speed, start_angle, distances_and_intensities, end_angle, last_angle,time)
                        last_angle = start_angle  # 更新上一个角度
        except Exception as e:
            pass  # 忽略异常
```