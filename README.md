# adi_tmcl

**adi_tmcl** (previously *tmcl_ros*) is the official ROS Driver for ADI Trinamic Motor Controllers (TMC) that uses Trinamic Motion Control Language (TMCL) protocol.


# Background
- Supported TMC boards: [TMCM-1636](https://www.analog.com/en/products/tmcm-1636.html), [TMCM-1617](https://www.analog.com/en/products/tmcm-1617.html), [TMCM-1241](https://www.analog.com/en/products/tmcm-1241.html), [TMCM-1260](https://www.analog.com/en/products/tmcm-1260.html), [TMCM-6214](https://www.analog.com/en/products/tmcm-6214.html)
- Supported communication interface and interface driver: CAN (SocketCAN)
- Supported ROS and OS distro: Noetic (Ubuntu 20.04)
- Supported platform: Intel x86 64-bit (amd64)
- Supported setup: Single/Multiple TMC in Single/Multiple CAN channel (namespace-managed)

> :memo: _Note: Although officially supported TMC boards are only the abovementioned, all market-ready TMCs with YAMLs in this repository are also expected to work and can be tried and tested by the users._
> _Contact the Developers for any  issues encountered._

# Hardware

For the tested TMCM-1636 setup, the following are used:
- 1 x [TMCM-1636](https://www.analog.com/en/products/tmcm-1636.html)
- 1 x [QBL4208-61-04-013 BLDC motor](https://www.analog.com/en/products/qbl4208.html)
- 1 x External 24V power supply
- 1 x CAN USB Cable (w/SocketCAN support) - with 120 ohm termination resistors

Also the following:
- PWR/GND from board to external 24V power supply
- 5-pin Motor connector (Hall) (see _Note_ below)
- 5-pin Motor connector (Encoder) (see _Note_ below)
- 40 pin Molex connectors

> :memo: _Note: Check Section 4 of [QBL4208-x-1k Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/QBL4208-x-1k_datasheet_Rev1.40.pdf) for motor wiring references._

The image below shows the connection diagram of the setup (with labels):
![TMCM-1636 Connections](./docs/images/tmcm_1636_setup.png)

The image below shows the actual setup used (for reference):
![TMCM-1636 Actual Setup](./docs/images/tmcm_1636_actual_setup.png)

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
# Clone to directory "adi_tmcl" which is the package name
$ git clone <copied SSH o HTTPS link here> adi_tmcl
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

If it's the first time to use the set of motors for the TMC that you are using, it is required to calibrate and tune the PID settings of the motors first.

Do the calibrations/tuning by downloading and using [TMCL-IDE](https://www.analog.com/en/design-center/evaluation-hardware-and-software/motor-motion-control-software/tmcl-ide.html).

## BLDC Motors
### Calibrate the motors

For a run-through/tutorial of how the calibration is done in the TMCL-IDE via its `Wizard Pool` feature, check this [link](https://www.youtube.com/watch?v=MASVD_2tNuo).

### Tune the PI settings of the motors

For a run-through/tutorial of how the PI tuning is done in the TMCL-IDE via its `PI Tuning` feature, check this [link](https://www.youtube.com/watch?v=rfZAs-QdYlQ).

> :memo: _Note: For all the calibration and tuning done, store all the parameters set from TMCL_IDE on the board's EEPROM. Do this by doing any of the following:_
> - _Clicking the "Store Parameter" under Settings of each axis_
> - _Using STAP (Store Axis Parameters) command from Direct Mode_
> - _Creating and uploading a TMCL Program, and enabling the "auto start mode" under SGP (Set Global Parameter) command from Direct Mode_
> - - :memo: _Note: Some boards don't have "auto start mode", so in such a case, use the other options to store the parameters._

## Stepper Motors

### Calibrate the motors

For a run-through/tutorial of how the calibration is done in the TMCL-IDE via its `Wizard Pool` feature, check this [link](https://www.youtube.com/watch?v=l6r63Q7Yr58o).

For more information about Trinamic features on stepper motors, visit this [link](https://www.analog.com/en/product-category/motor-and-motion-control.html).

> :memo: _Note: For all the calibration and tuning done, store all the parameters set from TMCL_IDE on the board's EEPROM. Do this by:_
> - _Creating and uploading a TMCL Program, and enabling the "auto start mode" under SGP (Set Global Parameter) command from Direct Mode_

# Pre-Launch (One-time)

## Initialize CAN

> :memo: _Note: This script automatically brings-up CAN upon boot. Do this step only once._

To proceed, first make the script executable (do this only once as this change persists even after power-off):
```bash
$ cd ~/catkin_ws/src/adi_tmcl/scripts # or $ cd ~/catkin_ws/src/<repo directory>/scripts
$ chmod +x CAN_init.sh
```

Execute the script:
```bash
$ sudo ./CAN_init.sh <communication interface> <bitrate>
```
Where:
- communication interface is the interface used between the PC and the TMC _(accepted values: 0 -255)_
- bitrate is the rate of communication interface _(accepted values: 20000, 50000, 100000, 125000, 250000, 500000, 1000000)_

> :memo: _Note: The above mentioned accepted values are a range, though the user still needs to check the actual TMC's datasheet on the applicable range._
> _Additionally, the bitrate set here must be the same bitrate set into the the TMC as the Global Parameter "CAN bit rate"._

For example, to initialize CAN with can0 as communication interface and 1000KBPS bitrate:
```bash
$ sudo ./CAN_init.sh 0 1000000
```
## De-initialize CAN

> :memo: _Note: This script disables the automatic bring-up of CAN upon boot._

To proceed, first make the script executable (do this only once as this change persists even after power-off):
```bash
$ cd ~/catkin_ws/src/adi_tmcl/scripts # or $ cd ~/catkin_ws/src/<repo directory>/scripts
$ chmod +x CAN_deinit.sh
```

Execute the script:
```bash
$ sudo ./CAN_deinit.sh <communication interface>
```
Where:
- communication interface is the interface used between the PC and the TMC _(accepted values: 0 -255)_

For example, to de-initialize CAN with can0 as communication interface:
```bash
$ sudo ./CAN_deinit.sh 0
```

# Launch
```bash
$ roslaunch adi_tmcl tmcm_1636.launch
```

# Nodes

## tmcl_ros_node

> :memo: _Note: For those with <motor_number> in the topic names, these are ideally the motor number. For example, if there are 2 motors used, there should be two published topics for tmc_info, specifically /tmc_info_0 for motor 0 and then /tmc_info_1 for motor 1._

### Published topics

These are the default topic names, topic names can be modified as a ROS parameter.

+ **/tmc_info_<motor_number>**
    - Data containing:
      + (1) board voltage (V)
      + (2) statusflag value (only for boards with StatusFlags AP, else, value is set to 0)
      + (3) statusflag notifications (only for boards with StatusFlags AP and StatusFlags Register names). In the case of Stepper motor(s), this publishes driver error flags and extended error flags values. Else, value is set to " ")
      + (4) motor number
      + (5) velocity (if [Parameter](#parameters) wheel_diameter is set to 0, the unit for published velocity is rpm, else m/s)
      + (6) position (degree angle)
      + (7) torque (mA)

### Subscriber topics

+ **/cmd_vel_<motor_number>** 
    - Velocity command (rpm or m/s)
+ **/cmd_abspos_<motor_number>** 
    - Absolute position command (degree angle)
+ **/cmd_relpos_<motor_number>** 
    - Relative position command (degree angle)
+ **/cmd_trq_<motor_number>** 
    - Torque command (mA)

### Advertised services

+ **/tmcl_custom_cmd** (/tmcl_custom_cmd)
    - Executes a custom SAP, GAP, SGP and GGP commands
    - The output contains raw data (velocity = rpm, position = units) from the board. *Do not expect same unit from the publisher.*
+ **/tmcl_gap_all** (/tmcl_gap_all)
    - Get all Axis Parameter values
+ **/tmcl_ggp_all** (/tmcl_ggp_all)
    - Get all Global Parameter values

### Parameters

> :memo: _Notes:_
> - _If any of these parameters are not set/declared, default values will be used._
> - _ROS parameters can only cover `rosparam get`. `rosparam set` is prohibited, even when the user run `rosparam set`, only the parameter from server will change but not in the node itself_

##### _Communication Interface Parameters_

+ **comm_interface** (int, default: 0)
    - Interface used between the PC and the TMC (where 0 = CAN)
+ **comm_interface_name** (string, default: can0)
    - Name of the interface or device as detected by the PC
+ **comm_tx_id** (int, default: 1)
    - ID for board TX
+ **comm_rx_id** (int, default: 2)
    - ID for board RX

##### _TMC ROS Node Parameters_

+ **comm_timeout_ms** (int, default: 100)
    - Indicates how long should the node will wait for the rx data
+ **comm_exec_cmd_retries** (int, default: 1)
    - Indicates how many the node will retry sending data before shutting down if no data received
+ **adhoc_mode** (bool, default: false)
    - This mode expects that the used module is not known. The velocity, position and torque relies on the additional_ratio_* values.
+ **en_motors** (int, default: 0)
    - Enables/disables active motors or axes. If disabled, settings for those motors will be ignored or not set.
+ **pub_rate_tmc_info** (int, default: 10)
    - Publish rate (hertz) of TMC information
+ **auto_start_additional_delay** (int, default: 0)
    - Added delay if auto start mode is enabled. Use this if your TMCL program needs to start running after over 2 seconds.

##### _Motor Configuration Settings_
+ **en_pub_tmc_info** (bool, default: false)
    - Enables/disables publishing of TMC information
+ **pub_actual_vel** (bool, default: false)
    - Enable/Disable actual velocity that the user can optionally publish every publish rate as long as en_pub_tmc_info is true
+ **pub_actual_trq** (bool, default: false)
    - Enable/Disable actual torque that the user can optionally publish every publish rate as long as en_pub_tmc_info is true
+ **pub_actual_pos** (bool, default: false)
    - Enable/Disable actual position that the user can optionally publish every publish rate as long as en_pub_tmc_info is true
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
+ **wheel_diameter** (int, default: 0)
    - Wheel diameter that is attached on the motor shaft directly. This is to convert linear values to rpm
    - If wheel diameter is 0, cmd_vel is equal to rpm
+ **additional_ratio_vel** (int, default: 1)
    - Additional Ratio for velocity for general purposes (adhoc mode, added pulley or gear trains). Default value 1 means disabled
+ **additional_ratio_pos** (int, default: 1)
    - Additional Ratio for position for general purposes (adhoc mode, added pulley or gear trains). Default value 1 means disabled
+ **additional_ratio_trq** (int, default: 1)
    - Additional Ratio for torque for general purposes (adhoc mode). Default value 1 means disabled 

# Quick Tests

### Test Velocity Mode

To do a quick test of Velocity Mode, there is a fake velocity script that the user can run.
Idea is, this script will send Velocity commands (as a ROS topic), then the first motor should be expected to:
1. Increase velocity every 3secs, clockwise (in m/s: 3, 6, 9)
2. Stop for 5secs
3. Increase velocity every 3secs, counter-clockwise (in m/s: -3, -6, -9)
4. Stop for 5secs

To proceed with the test, execute these following commands on three (3) different terminals (in sequence):
> :memo: _Note: Assuming that [Initialize CAN](#initialize-can) is already done on Terminal 1._

| Terminal 1 | Terminal 2  | Terminal 3|
--- | --- | ---|
| <pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ roslaunch adi_tmcl tmcm_1636.launch | <pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ rostopic echo /tmcm1/tmc_info_0 | <pre>$ cd ~/catkin_ws/src/adi_tmcl/scripts <br>$ sudo chmod 777 fake_cmd_vel.sh <br>$ ./fake_cmd_vel.sh |

**Monitor the velocity of the first motor (watch out for velocity value at Terminal 2).**

> :memo: _Notes:_
> - _Terminals 2 and 3 are best viewed side-by-side._
> - _You may Ctrl-C the command in Terminal 2 once you're done._
> - _The command in Terminal 3 auto-stops by itself._

### Test Position Mode
To do a quick test of Position Mode, there is a fake position script that the user can run.
Idea is, this script will send Position commands (as a ROS topic), then the first motor should be expected to:
1. Rotate 360 degrees (clockwise) every 5 secs, 3 times
2. Stop for 5secs
3. Rotate 360 degrees (counter-clockwise) every 5 secs, 3 times
4. Stop for 5secs

To proceed with the test, execute these following commands on three (3) different terminals (in sequence):
> :memo: _Note: Assuming that [Initialize CAN](#initialize-can) is already done on Terminal 1._

| Terminal 1 | Terminal 2  | Terminal 3|
--- | --- | ---|
|<pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ roslaunch adi_tmcl tmcm_1636.launch  | <pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ rostopic echo /tmcm1/tmc_info_0 | <pre>$ cd ~/catkin_ws/src/adi_tmcl/scripts <br>$ sudo chmod 777 fake_cmd_pos.sh <br>$ ./fake_cmd_pos.sh |

**Monitor the position of the first motor (watch out for position value at Terminal 2).**

> :memo: _Notes:_
> - _Terminals 2 and 3 are best viewed side-by-side._
> - _You may Ctrl-C the command in Terminal 2 once you're done._
> - _The command in Terminal 3 auto-stops by itself._

### Test Torque Mode
To do a quick test of Torque Mode, there is a fake torque script that the user can run.
Idea is, this script will send Torque commands (as a ROS topic), then the first motor should be expected to:
1. Rotate for 5 secs, with torque = 300
2. Stop for 5secs
3. Rotate for 5 secs again, with torque = 300

To proceed with the test, execute these following commands on three (3) different terminals (in sequence):
> :memo: _Note: Assuming that [Initialize CAN](#initialize-can) is already done on Terminal 1._

| Terminal 1 | Terminal 2  | Terminal 3|
--- | --- | ---|
|<pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ roslaunch adi_tmcl tmcm_1636.launch  | <pre>$ cd ~/catkin_ws/ <br>$ source /opt/ros/noetic/setup.bash <br>$ source devel/setup.bash <br>$ rostopic echo /tmcm1/tmc_info_0 | <pre>$ cd ~/catkin_ws/src/adi_tmcl/scripts <br>$ sudo chmod 777 fake_cmd_trq.sh <br>$ ./fake_cmd_trq.sh |

**Monitor the torque of the first motor (watch out for torque value at Terminal 2).**

> :memo: _Notes:_
> - _Terminals 2 and 3 are best viewed side-by-side._
> - _You may Ctrl-C the command in Terminal 2 once you're done._
> - _The command in Terminal 3 auto-stops by itself._

# Limitations
1. No support for interfaces other than CAN yet.

# Support

Please contact the `Maintainers` if you want to use this ROS Driver on Trinamic Motor Controllers without YAML files in this repository.

Any other inquiries and support are also welcome.
