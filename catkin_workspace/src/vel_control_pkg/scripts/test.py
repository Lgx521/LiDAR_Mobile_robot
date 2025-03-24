import struct


def build_velocity_frame(vx, vy, vz):
        """构建符合协议的数据帧"""
        # 限制速度范围(short类型范围)
        vx = max(min(int(vx * 1000), 32767), -32768)
        vy = max(min(int(vy * 1000), 32767), -32768)
        vz = max(min(int(vz * 1000), 32767), -32768)

        # 协议帧结构
        frame = bytearray()
        # 帧头 (1 byte)
        frame.extend([0x7B])
        # 预留位 (2 bytes)
        frame.extend([0x00])
        frame.extend([0x00])
        # 三轴速度 (每个轴2 bytes，小端字节序)
        frame.extend(struct.pack('<h', vx))  # X轴
        frame.extend(struct.pack('<h', vy))  # Y轴
        frame.extend(struct.pack('<h', vz))  # Z轴
        # 校验位 (异或校验)
        checksum = 0
        for i in range(len(frame)):  # 从预留位开始计算
            # print(i)
            checksum = checksum ^ frame[i]

        frame.append(checksum)
        # 帧尾 (1 byte)
        frame.append(0x7D)
        # rospy.loginfo(frame.hex())
        return frame


data = build_velocity_frame(0.2,0,0.2)
print(len(data))
print(data.hex())