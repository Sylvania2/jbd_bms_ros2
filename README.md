# jbd_bms_ros
### Battery Management System BMS jbd
## Overview
Read battery-related information returned by BMS，You can configure serial port name, baudrate and loop rate from outside,
Information will be displayed dynamically depending on the number of batteries and temperature sensors，\
You can also get the error bits of information and the error content.
## Equipment Type
Jbd Power Lithium Battery\
![](https://github.com/I-Quotient-Robotics/jbd_bms_ros/blob/master/type_pic/144283718.jpg)
![](https://github.com/I-Quotient-Robotics/jbd_bms_ros/blob/master/type_pic/60348685.jpg)

## Environment
Ubuntu22.04\
ROS2-Humble
## Nodes
### jbd_bms_status
jbd_bms_status is a driver for jbd_bms. It reads battery info convert to JbdStatus message.
#### Published Topics
bms(jbd_bms_msg/JbdStatus)\
it publishes bms topic from the jbd_bms.
#### Parameters
port_bms(string, default: /dev/port_link_bms)\
serial port name used in your system.\
\
baudrate_bms(int, default: 9600)\
serial port baud rate.\
\
looprate_bms(int, defaule: 2)\
loop rate.\
\
jbd_id(string, default: jbd_bms)\
frame ID for the device.\
## install
```bash
git clone git@github.com:Futu-reADS/jbd_bms_ros.git
cd battery
rosdep install jbd_bms_driver --ignore-src
rosdep install jbd_bms_msg
```
## rules
```bash
sudo cp jbd_bms_driver/udev/10-jbd-bms.rules /etc/udev/rules.d/ or /lib/udec/rules.d
sudo udevadm control--reload-rules && udevadm trigger
```
## run
```bash
cd workspace
roslaunch jbd_bms_bringup jbd_bms_status.launch
```
## Official website
http://www.jiabaida.com/

