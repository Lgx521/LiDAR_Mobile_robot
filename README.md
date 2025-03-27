# LiDAR Mobile Robot

The course project of `EE202-17L` - Digital Circuits Laboratory.

## Communication between ESP32, Mobile robot base and Laptop

**Function of this repo**:  
- Realize the wireless communication between a Mobile robot and a laptop running Ubuntu 20.04
- In the ROS workspace, a basic LiDAR-slam algorithm will be implemented.

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


---
### version 2.0
**Limitation**:  
Still in prograss. The basic communication logic is finished, using the multiple-port strategy.  

**What's new?** & **Pipeline**:  
- Use different port to communicate in three way --- LiDAR tx, CMD rx and Encoder tx.  
- Wrote a python package to utililze the Wi-Fi reciever class, to be easily instantiated both in `vel-cmd` node and LiDAR receiver.  
- Created the ROS node of the `vel_cmd` subscriber  
  1. Directly Use the serial from Laptop linked with the base. (Verified)  
  2. Use Wi-Fi port to send data to esp32 and use UART output to the base. (Verified with serial assistant, but due to the cable issue, the data can't received by the base.)  
- `wifi_receiver.py` is not oop one, `communication.py` is new, and can be run by `rosrun`.  
- `wifi_transmitter.py` is the code runs on the esp32. This used multi-thread. And used 2 pair of UART pins.  

**Get started**:   
Notice:  
- Ensure the laptop and ESP32 are connected to same Wi-Fi.  
- `wifi_transmitter.py` is the MicroPython script running on ESP32.  
- `communication.py` is the Python script running on laptop, can be run by `rosrun communication communication.py`.  
- Only send data when the ESP32 receiced uart data.  

Steps to implement, in `Ubuntu20.04`:   
- Upload `wifi_transmitter.py` to ESP32, change the wifi condition variable's name of `SSID` and `PASSWARD`.  
- Chanege the `uart` initiallization to your preferred tx and rx pin.  
- Run this program once, write down the ESP32's IP address.  
- Then save file to ESP32, rename as `main.py` to ensure autumatically run after boot.   
- Create two pairs of virtual serial and give them sudo access.   
  - Create virtual serial: run in terminal: `sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVIRT0 PTY,raw,echo=0,link=/dev/ttyVIRT1`  
  - Create virtual serial: run in terminal: `sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVIRT2 PTY,raw,echo=0,link=/dev/ttyVIRT3`  
  - Implement sudo access: run in another terminal: `sudo chmod 777 /dev/ttyVIRT*`  
- Run `communication.py` on laptop  
  - Remember to change `TCP_IP` variable as the noted IP address of you ESP32.  
  - Remember to change `virtual_com` signature in `__init__()` function as your virtual serial name(`'/dev/ttyVIRT0'`)  
  - Run in terminal: `rosrun communication communication.py`.   
- You can examine the output by cancel the code comments in `run()` function.  
- Read the virtual serial data in the other virtual serial which is created in pair(`'/dev/ttyVIRT1'`)  
- Theoretically, the encoder data is streamed into the second pair of virtual serial, but this is not verified yet.

Check data with RViz:  
- Run `roscore`    
- After all above finished:  
  - `cd catkin_workspace` Enter the ws, change to your only path.  
  - `source devel/setup.sh`  
  - Run the LiDAR's drive: `roslaunch ldlidar_sl_ros ld14.launch`  
  - Open RViz `rviz`  
  - In RViz, change the base to `Laser_base`  
  - Add `LaserScan`, choose topic `/scan`   
  - Finished.  

