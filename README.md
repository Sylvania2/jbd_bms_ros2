# jbd\_bms\_ros

### Battery Management System BMS by Jiabaida

## Overview
This ROS2 package reads battery-related information returned by BMS and publishes it as topics.
You can configure serial port name, baudrate and loop rate from outside.
Number of battery cells and temperature sensors are not fixed.
You can also get the error bits of information and the error content.

## Equipment Type
Jbd Power Lithium Battery\
![](https://github.com/I-Quotient-Robotics/jbd_bms_ros/blob/master/type_pic/144283718.jpg)
![](https://github.com/I-Quotient-Robotics/jbd_bms_ros/blob/master/type_pic/60348685.jpg)

## Environment
Ubuntu22.04\
ROS2-Humble

## Nodes

### jbd\_bms\_status
jbd\_bms\_status is a driver for jbd\_bms. It reads battery info convert to JbdStatus message.

#### Parameters
port\_bms(string, default: /dev/port\_link\_bms)\
serial port name used in your system.\
\
baudrate\_bms(int, default: 9600)\
serial port baud rate.\
\
looprate\_bms(int, default: 2)\
loop rate.\
\
jbd\_id(string, default: jbd\_bms)\
frame ID for the device.\

## Topics

- `/jbd_bms`
- `/diagnostics`


##  /jbd\_bms

State of battery sent from Jiabaida BMS.

Type: `jbd_bms_msg/JbdStatus`

### Definition of `jbd_bms_msg/JbdStatus`

```
std_msgs/Header header    # Header
float32 voltage           # Pack Voltage [V]
float32 current           # Pack Current [A] (positive:charge negative:discharge)
int32 residual_capacity   # Remaining Capacity [mAh]
int32 design_capacity     # Designed Capacity [mAh]
int16 cycle_index         # Number of Charge Cycles
string date_production    # Date of Production
uint32 status_balance     # bit field representing Balance Status of each cell    (1: balancing under way)
uint16 status_protect     # bit field representing Protection Status of each cell (1: protection trigger detected)
uint16 version            # Version (0x10 --> ver1.0)
int16 rsoc                # State-Of-Charge [%]
int16 status_mos          # MosFET status (0:OFF 1:discharge ON 2:charge ON 3:ON for both direction (discharge/charge)
int16 cell_number         # Number of cells connected to BMS
int16 ntc_number          # Number of thermistors connected
float64[] ntc_tem         # Temperature [C] (degree Celsius) of all thermistors connected
float64[] cell_voltage    # Cell voltage [V] of all cells connected
int64[] error_id          # Array of error id's collected from Protection Status
string[] error_info       # Array of error messages corresponding to error_id
```

###### Error ID

| error_id | description                  |
|----------|------------------------------|
| 0        | Cell Overvoltage             |
| 1        | Cell Undervoltage            |
| 2        | Pack Overvoltage             |
| 3        | Pack Undervoltage            |
| 4        | Over-temp while charging     |
| 5        | Under-temp while charging    |
| 6        | Over-temp while discharging  |
| 7        | Under-temp while discharging |
| 8        | Overcurrent while charging   |
| 9        | Overcurrent while discharging|
| 10       | Short circuit                |
| 11       | Front-end IC error           |
| 12       | MosFET software lock-in      |
| 13       | Communication error          |

Note: Length of `error_id` becomes 0 when no error is detected


## /diagnostics

Diagnostics from Jiabaida BMS. Many human-readable messages on battery state.


## Build & Install

Execute following command to build and install the package.

```bash
cd _workspace_/src
git clone git@github.com:Futu-reADS/jbd_bms_ros.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Udev rule

Udev rule file for USB-UART adaptor connecting to Jiabaida BMS:

```
KERNEL=="ttyUSB[0-9]",ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="ea60",MODE:="0777",SYMLINK+="jbd_bms"
```

To install the udev rule:

```bash
sudo cp jbd_bms_driver/udev/10-jbd-bms.rules /etc/udev/rules.d/ or /lib/udec/rules.d
sudo udevadm control--reload-rules && udevadm trigger
```

## Run


```bash
cd _workspace_
ros2 launch jbd_bms_status jbd_bms_status_launch.xml
```


## Official website
http://www.jiabaida.com/

