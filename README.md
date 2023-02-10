| tmcl_ros |
| --- |
Official ROS Driver for Trinamic Motor Controllers (TMC) that uses Trinamic Motion Control Language (TMCL) protocol. |

# Background
- Supported TMC boards: [TMCM-1636](https://www.trinamic.com/products/modules/details/tmcm-1636/), [TMCM-1617](https://www.trinamic.com/products/modules/details/tmcm-1617/)
- Supported communication interface and interface driver: CAN (SocketCAN)
- Supported ROS and OS distro: Noetic (Ubuntu 20.04)
- Supported platform: Intel x86 64-bit (amd64)

# Hardware

For the tested TMCM-1636 setup, the following are used:
- 1 x [TMCM-1636](https://www.trinamic.com/products/modules/details/tmcm-1636/)
- 1 x [QBL4208-61-04-013 BLDC motor](https://www.trinamic.com/products/drives/bldc-motors-details/qbl4208/)
- 1 x External 24V power supply
- 1 x CAN USB Cable (w/SocketCAN support) - with 120 ohm termination resistors

Also the following:
- PWR/GND from board to external 24V power supply
- 5-pin Motor connector (Hall) (see _Note_ below)
- 5-pin Motor connector (Encoder) (see _Note_ below)
- 40 pin Molex connectors

> :memo: _Note: Check Section 4 of [QBL4208-x-1k Datasheet](https://www.trinamic.com/fileadmin/assets/Products/Motors_Documents/QBL4208-x-1k_datasheet_Rev1.40.pdf) for motor wiring references._

The image below shows the actual setup used (with labels):
![TMCM-1636 Connections](./docs/images/tmcm_1636_setup.png)

# Software

## Software Architecture
![Software Architecture Diagram](./docs/images/tmcl_ros_Software_Architecture_Diagram.PNG)

## Software Dependencies
Assumptions before building this package:
* Installed ROS Noetic. If not, follow these [steps](http://wiki.ros.org/noetic/Installation/Ubuntu).
* Setup catkin workspace (with workspace folder named as "catkin_ws"). If not, follow these [steps](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#:~:text=you%20installed%20ROS.-,Create%20a%20ROS%20Workspace,-catkin).

# Clone

In the website:
1. Make sure that the branch dropdown is set to "noetic".
2. Click the "Clone" or "Code" button, then copy the SSH or HTTPS link (eg, "*.git").

In a terminal, do the following:
```bash
$ cd ~/catkin_ws/src
$ git clone <copied SSH o HTTPS link here>
$ mv <directory created by the above command> tmcl_ros #renames the cloned repo to "tmcl_ros"
```

# Build

Do proper exports first:
```bash
$ source /opt/ros/<ROS version>/setup.bash
```
Where:
- "ROS version" is the user's actual ROS version

Then:
```bash
$ cd ~/catkin_ws/
$ catkin_make clean
$ catkin_make
$ source devel/setup.bash
```

# Pre-Launch (One-time per setup)

If it's the first time to use the set of motors for the TMC that you are using or you are using a different set of motors than the ones specified in the [`Hardware`](#-Hardware) section, you need to calibrate and tune the PID settings of the motors first.

You may be able to do the calibrations/tuning by downloading and using [TMCL-IDE](https://www.trinamic.com/support/software/tmcl-ide/).

> :memo: _Note: After calibration/tuning, if you are setting [Parameter](#parameters) *FollowEepromConfig* to "false", then all the corresponding *_Ext TMC board-specific parameters (eg, CommutationMode_Ext, EncoderSteps_Ext, etc) inside the TMC-XXXX_*Ext.yaml file must be modified to contain the new calibrated values instead._

## Calibrate the motors

For a run-through/tutorial of how the calibration is done in the TMCL-IDE via its `Wizard Pool` feature, check this [link](https://www.youtube.com/watch?v=MASVD_2tNuo).

## Tune the PI settings of the motors

For a run-through/tutorial of how the PI tuning is done in the TMCL-IDE via its `PI Tuning` feature, check this [link](https://www.youtube.com/watch?v=rfZAs-QdYlQ).

# Pre-Launch (One-time per connect/disconnect of TMC to Host PC)

## Initialize CAN

> :memo: _Note: Only do this section when either of the following scenario happens:_
> - _Upon boot-up_
> - _If CAN-USB connects/disconnects/reconnects_
>
> _Also, plug-in CAN-USB first before executing the script._

To proceed, first make the script executable (do this only once as this change persists even after power-off):
```bash
$ cd ~/catkin_ws/src/tmcl_ros/scripts
$ chmod +x CAN_init.sh
```

Execute the script:
```bash
$ sudo ./CAN_init.sh <communication interface> <bitrate>
```
Where:
- communication interface is the interface used between the PC and the TMC _(accepted values: 0 -255)_
- bitrate is the rate of communication interface _(accepted values: 20000, 50000, 100000, 125000, 250000, 500000, 1000000)_

For example, to initialize CAN with can0 as communication interface and 1000KBPS bitrate:
```bash
$ sudo ./CAN_init.sh 0 1000000
```

# Launch

The launch file accepts 2 modes; "_normal_" and "_service_".

By default (or no arguments added), the mode is set to "_normal_".
For example, to launch TMCM-1636 in "_normal_" mode, do:
```bash
$ roslaunch tmcl_ros tmcm_1636.launch
```

On the other hand, in "_service_" mode, all the topics are disabled, and the user is expected to use ros services only.
For example, to launch TMCM-1636 in "_service_" mode, do:
```bash
$ roslaunch tmcl_ros tmcm_1636.launch mode:="service"
```

# Nodes

## tmcl_ros_node

> :memo: _Note: For those with <motor_num> in the topic names, these are ideally the motor number. For example, if there are 2 motors used, there should be two published topics for tmc_info, specifically /tmc_info_0 for motor 0 and then /tmc_info_1 for motor 1._

### Published topics

These are the default topic names, topic names can be modified as a ROS parameter.

+ **/tmc_info_<motor_num>**
    - Data containing (1) board voltage (V); (2) velocity (if [Parameter](#parameters) wheel_diameter is set to 0, the unit for published velocity is rpm, else m/s); (3) position (degree angle); and (4) torque (mA) per motor number

### Subscriber topics

+ **/cmd_vel_<motor_num>** 
    - Velocity command (rpm or m/s)
+ **/cmd_abspos_<motor_num>** 
    - Absolute position command (degree angle)
+ **/cmd_relpos_<motor_num>** 
    - Relative position command (degree angle)
+ **/cmd_trq_<motor_num>** 
    - Torque command (mA)

### Advertised services

+ **/tmcl_custom_cmd** (/tmcl_custom_cmd)
    - Executes a custom SAP, GAP, SGP and GGP commands
    - The output contains raw data (velocity = rpm, position = units) from the board. *Do not expect same unit from the publisher.*

### Parameters

> :memo: _Notes:_
> - _If any of these parameters are not set/declared, default values will be used._
> - _ROS parameters can only cover `rosparam get`. `rosparam set` is prohibited, even when the user run `rosparam set`, only the parameter will change **not the board parameter (will not send SAP command to the board)_

##### _Communication Interface Parameters_

+ **comm_interface** (int, default: 0)
    - Interface used between the PC and the TMC (where 0 = CAN)
+ **comm_interface_name** (string, default: can0)
    - Name of the interface or device as detected by the PC
+ **comm_bit_rate** (int, default: 1000000)
    - Rate of communication interface (bits per sec)
+ **comm_tx_id** (int, default: 1)
    - ID for board TX
+ **comm_rx_id** (int, default: 2)
    - ID for board RX

##### _TMC ROS Node Parameters_

+ **comm_timeout_ms** (int, default: 100)
    - Indicates how long should the node will wait for the rx data
+ **comm_exec_cmd_retries** (int, default: 1)
    - Indicates how many the node will retry sending data before shutting down if no data received
+ **wheel_diameter** (int, default: 0)
    - Wheel diameter that is attached on the motor shaft directly. This is to convert linear values to rpm
    - If wheel diameter is 0, cmd_vel is equal to rpm
+ **en_motors** (int, default: 0)
    - Enables/disables active motors or axes. If disabled, settings for those motors will be ignored or not set.
+ **pub_rate_tmc_info** (int, default: 1)
    - Publish rate (hertz) of TMC information
+ **tmc_info_topic** (string, default: /tmc_info_)
    - tmc_info topics that will contain chosen TMC info that will be published
+ **tmc_cmd_vel_topic** (string, default: /cmd_vel_)
    - Twist topics that will be the source of target velocity to be set on the TMC
+ **tmc_cmd_abspos_topic** (string, default: /cmd_abspos_)
    - Int32 topics that will be the source of target position to be set on the TMC
+ **tmc_cmd_relpos_topic** (string, default: /cmd_relpos_)
    - Int32 topics that will be the source of target position to be set on the TMC
+ **tmc_cmd_trq_topic** (string, default: /cmd_trq_)
    - Int32 topics that will be the source of target torque to be set on the TMC
+ **en_pub_tmc_info** (bool, default: false)
    - Enables/disables publishing of TMC information
+ **pub_actual_vel** (bool, default: false)
    - Enable/Disable actual velocity that the user can optionally publish every publish rate as long as en_pub_tmc_info is true
+ **pub_actual_trq** (bool, default: false)
    - Enable/Disable actual torque that the user can optionally publish every publish rate as long as en_pub_tmc_info is true
+ **pub_actual_pos** (bool, default: false)
    - Enable/Disable actual position that the user can optionally publish every publish rate as long as en_pub_tmc_info is true

##### _TMC Board-specific Parameters_

> :memo: _Note: Defaults for these will be based on the corresponding YAML inside `./config/autogenerated` directory._

+ **FollowEepromConfig** (bool)
    - Flag for the ROS Driver to identify where to get motor configurations (EEPROM vs YAML)
+ **CommutationMode_Ext** (int)
    - Commutation sensor selection
+ **CommutationModeVelocity_Ext** (int)
    - Velocity sensor selection
+ **CommutationModePosition_Ext** (int)
    - Position sensor selection
+ **MaxTorque_Ext** (int)
    - Maximum allowed absolute motor current
+ **OpenLoopCurrent_Ext** (int)
    - Motor current for controlled commutation (used if CommutationMode_Ext = 1)
+ **Acceleration_Ext** (int)
    - Motor acceleration value
+ **MotorPolePairs_Ext** (int)
    - Number of motor poles
+ **PWMFrequency_Ext** (int)
    - Frequency of the motor PWM
+ **HallSensorPolarity_Ext** (int)
    - Hall sensor polarity
+ **HallSensorDirection_Ext** (int)
    - Hall sensor direction 
+ **HallInterpolation_Ext** (int)
    - Hall sensor interpolation 
+ **HallSensorOffset_Ext** (int)
    - Offset for electrical angle hall_phi_e of hall sensor
+ **EncoderDirection_Ext** (int)
    - Encoder direction in a way that ROR increases position counter
+ **EncoderSteps_Ext** (int)
    - Encoder steps per full motor rotation
+ **EncoderInitMode_Ext** (int)
    - Encoder init mode
+ **TorqueP_Ext** (int)
    - P parameter for current PID regulator
+ **TorqueI_Ext** (int)
    - I parameter for current PID regulator
+ **VelocityP_Ext** (int)
    - P parameter for velocity PID regulator
+ **VelocityI_Ext** (int)
    - I parameter for velocity PID regulator
+ **PositionP_Ext** (int)
    - P parameter for position PID regulator
+ **BrakeChopperEnabled_Ext** (int)
    - Enable brake chopper fuctionality
+ **BrakeChopperVoltage_Ext** (int)
    - If the brake chopper is enabled and suppyly voltage exceeds this value, the brake chopper output will be activated
+ **BrakeChopperHysteresis_Ext** (int)
    - An activated brake chopper will be disabled if the actual supply voltage is lower than limit voltage hysteresis
+ **PositionScalerM_Ext** (int)
    - Scale the external position

# Quick Tests

### Test Velocity Mode

To do a quick test of Velocity Mode, there is a fake velocity script that you can run.
Idea is, this script will send Velocity commands (as a ROS topic), then the first motor should be expected to:
1. Increase velocity every 3secs, clockwise (in m/s: 3, 6, 9)
2. Stop for 5secs
3. Increase velocity every 3secs, counter-clockwise (in m/s: -3, -6, -9)
4. Stop for 5secs

To proceed with the test, execute these following commands on three (3) different terminals (in sequence):
> :memo: _Note: Assuming that [Initialize CAN](#initialize-can) is already done on Terminal 1._

| Terminal 1 | Terminal 2  | Terminal 3|
--- | --- | ---|
| <pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ roslaunch tmcm_1636.launch | <pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ rostopic echo /tmcm1/tmc_info_0 | <pre>$ cd ~/catkin_ws/src/tmcl_ros/scripts <br>$ sudo chmod 777 fake_cmd_vel.sh <br>$ ./fake_cmd_vel.sh |

**Monitor the velocity of the first motor (watch out for velocity value at Terminal 2).**

> :memo: _Notes:_
> - _Terminals 2 and 3 are best viewed side-by-side._
> - _You may Ctrl-C the command in Terminal 2 once you're done._
> - _The command in Terminal 3 auto-stops by itself._

### Test Position Mode
To do a quick test of Position Mode, there is a fake position script that you can run.
Idea is, this script will send Position commands (as a ROS topic), then the first motor should be expected to:
1. Rotate 360 degrees (clockwise) every 5 secs, 3 times
2. Stop for 5secs
3. Rotate 360 degrees (counter-clockwise) every 5 secs, 3 times
4. Stop for 5secs

To proceed with the test, execute these following commands on three (3) different terminals (in sequence):
> :memo: _Note: Assuming that [Initialize CAN](#initialize-can) is already done on Terminal 1._

| Terminal 1 | Terminal 2  | Terminal 3|
--- | --- | ---|
|<pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ roslaunch tmcm_1636.launch  | <pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ rostopic echo /tmcm1/tmc_info_0 | <pre>$ cd ~/catkin_ws/src/tmcl_ros/scripts <br>$ sudo chmod 777 fake_cmd_pos.sh <br>$ ./fake_cmd_pos.sh |

**Monitor the position of the first motor (watch out for position value at Terminal 2).**

> :memo: _Notes:_
> - _Terminals 2 and 3 are best viewed side-by-side._
> - _You may Ctrl-C the command in Terminal 2 once you're done._
> - _The command in Terminal 3 auto-stops by itself._

### Test Torque Mode
To do a quick test of Torque Mode, there is a fake torque script that you can run.
Idea is, this script will send Torque commands (as a ROS topic), then the first motor should be expected to:
1. Rotate for 5 secs, with torque = 300
2. Stop for 5secs
3. Rotate for 5 secs again, with torque = 300

To proceed with the test, execute these following commands on three (3) different terminals (in sequence):
> :memo: _Note: Assuming that [Initialize CAN](#initialize-can) is already done on Terminal 1._

| Terminal 1 | Terminal 2  | Terminal 3|
--- | --- | ---|
|<pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ roslaunch tmcm_1636.launch  | <pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ rostopic echo /tmcm1/tmc_info_0 | <pre>$ cd ~/catkin_ws/src/tmcl_ros/scripts <br>$ sudo chmod 777 fake_cmd_trq.sh <br>$ ./fake_cmd_trq.sh |

**Monitor the torque of the first motor (watch out for torque value at Terminal 2).**

> :memo: _Notes:_
> - _Terminals 2 and 3 are best viewed side-by-side._
> - _You may Ctrl-C the command in Terminal 2 once you're done._
> - _The command in Terminal 3 auto-stops by itself._

# Miscellaneous

Before doing any of the steps here, please make sure that you already did [`Pre-Build/Launch`](#-Pre-Build/Launch) and [`Build`](#-Build) sections above.

## Changing CAN interfaces (tx_id, rx_id and bitrate)

> :memo: _Note: Do NOT run this script with sudo._

Inside this package is a script to change board CAN interfaces such as TX ID, RX ID and bitrate in case such change is needed.
To make the script executable:
```bash
$ cd ~/catkin_ws/src/tmcl_ros/scripts
$ chmod +x CAN_interface.sh
```
Then, to execute the script:
```bash
$ ./CAN_interface.sh
```

> :memo: _Note: If bitrate is changed, it is advised to restart the board and initialize the CAN again using the new bitrate this time._

# Limitations
1. No support for multiple/concurrent CAN or TMCL connections yet.
2. No support for interfaces other than CAN yet.

# Support

Please contact the `Maintainers` if you want to use this ROS Driver on Trinamic Motor Controllers without YAML files in this repository.

Any other inquiries are also welcome.
