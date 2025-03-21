# LiDAR Mobile Robot

## Communication between ESP32 and Laptop

**Function of this repo**:  
- Realize the wireless communication between a Mobile robot and a laptop running Ubuntu 20.04

---
### version 1.0
**Limitation**:  
Only achieved the serial penetration, single side communication.

**Pipeline**:
- Sensor send data to ESP32 via uart serial
- ESP32 send these data to WiFi
- Laptop receive these data via WiFi
- Transmit these data via virtual serial
- Resultantly ROS sensor driver can receive these data via the virtual serial

**Get started**:  
Notice:  
- Ensure the laptop and ESP32 are connected to same Wi-Fi.
- `wifi.py` is the MicroPython script running on ESP32.
- `wifi_receive_virtual_serial.py` is the Python script running on laptop.
- Only send data when the ESP32 receiced uart data.

Steps to implement, in `Ubuntu20.04`:  
- Upload `wifi.py` to ESP32, change the wifi condition variable's name of `SSID` and `PASSWARD`.
- Chanege the `uart` initiallization to your preferred tx and rx pin.
- Run this program once, write down the ESP32's IP address.
- Then save file to ESP32, rename as `main.py` to ensure autumatically run after boot.
- Create virtual serial and give them sudo access.
  - Create virtual serial: run in terminal: `sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVIRT0 PTY,raw,echo=0,link=/dev/ttyVIRT1`
  - Implement sudo access: run in another terminal: `sudo chmod 777 /dev/ttyVIRT*`
- Run `wifi_receive_virtual_serial.py` on laptop
  - Remember to change `TCP_IP` variable as the noted IP address of you ESP32.
  - Remember to change `virtual_com` signature in `__init__()` function as your virtual serial name(`'/dev/ttyVIRT0'`)
- You can examine the output by cancel the code comments in `run()` function.
- Read the virtual serial data in the other virtual serial which is created in pair(`'/dev/ttyVIRT1'`)
