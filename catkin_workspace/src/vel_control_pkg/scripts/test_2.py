def hex_to_binary(hex_str):
    hex_str = hex_str.strip().upper()
    if hex_str.startswith('0X'):
        hex_str = hex_str[2:]
    
    hex_to_bin_map = {
        '0': '0000',
        '1': '0001',
        '2': '0010',
        '3': '0011',
        '4': '0100',
        '5': '0101',
        '6': '0110',
        '7': '0111',
        '8': '1000',
        '9': '1001',
        'A': '1010',
        'B': '1011',
        'C': '1100',
        'D': '1101',
        'E': '1110',
        'F': '1111'
    }
    
    binary_str = []
    for char in hex_str:
        if char not in hex_to_bin_map:
            raise ValueError(f"无效的十六进制字符: {char}")
        binary_str.append(hex_to_bin_map[char])
    
    return ''.join(binary_str)

import struct

vx = 32767

vx = int(vx)

res = struct.pack('>h', vx).hex()

print(res)
print(hex_to_binary(res))
print(2**15)