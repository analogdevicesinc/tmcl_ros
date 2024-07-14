^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tmcl_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.1 (2024-07-15)
------------------
* Merge branch 'noetic' of https://github.com/analogdevicesinc/tmcl_ros into noetic
* Added support for TMCM-2611
  Co-Authored-By: Christian Joseph Acar <124771470+CAcarADI@users.noreply.github.com>
  Co-Authored-By: Jamila Macagba <124771486+jmacagba@users.noreply.github.com>
* Contributors: mmaralit-adi

4.0.0 (2023-12-13)
------------------
* Updated README.md to mention both tmcl_ros (previous package name) and adi_tmcl
* Updated package name and transforms

  - Updated package name in compliance to official ROS release, and updated transforms for multi-axes TMCs

* Contributors: mmaralit-adi, jmacagba

3.0.2 (2023-10-06)
------------------
* Removed obsolete modules and updated analog.com references in README
* Updated to:

  - Remove obsolete modules
  - Update analog.com pages in README.md
  - Update CMakeLists.txt to make sure other drivers that has same names as socket_can_wrapper will not cause conflicts; and also add install support

* Contributors: mmaralit-adi, jmacagba

v3.0.1-noetic (2023-09-26)
--------------------------
* Remove obsolete modules, update copyrights' years, update package.xml version and cleanup CMakeLists.txt

  - Remove obsolete modules
  - Update copyrights' years
  - Update package.xml version
  - Clean-up comments in CMakeLists.txt

* Contributors: CAcarADI, mmaralit-adi

v3.0.0-noetic (2023-09-18)
--------------------------
* Adaptations on ROS coding standard and synchronization with upcoming ROS2 Driver version of this

  * Adaptations on ROS coding standards
  * Synchronization of handling between this ROS1 Driver and the up-for-release ROS2 Driver version
  * Added handling for SIGTERM and SIGKILL
  * Removed support for TMC variants that are either obsolete, no CAN support, or no Motor Drive

* Contributors: CAcarADI, mmaralit-adi

v2.0.0-noetic (2023-07-26)
--------------------------
* Added config files for other BLDC and Stepper type motors
  Added config files for various BLDC and Stepper type motors, added support for stepper type motors, and also added support for multiple TMC connections
* Contributors: CAcarADI, mmaralit-adi

v1.0.2-noetic (2023-05-11)
--------------------------
* Adjustments for position scaler and status, and added script to get all axis parameter values
  Updated for the following features:

  - Added Position Scaler consideration in position calculation (higher prio than encoder steps)
  - Added Status information in published /tmc_info
  - Changed default /tmc_info publish rate to 10Hz (previously 1Hz)
  - Added GAP_params.sh to query YAML's axis parameters' values
  - Made TMCM-1636 FollowEepromConfig as False and then used calibrated values in YAML
  - Updated Maintainer email

* Added TMCM-1636 actual setup image (for reference).
* Contributors: CAcarADI, mmaralit-adi

v1.0.1-noetic (2023-02-10)
--------------------------
* Added support for TMCM-1617
* Contributors: CAcarADI

v1.0.0-noetic (2023-02-10)
--------------------------
* Adding v1.0.0 of tmcl_ros, ROS1, Noetic

  - Contains official ROS Driver for Trinamic Motor Controllers (TMC) that uses Trinamic Motion Control Language (TMCL) protocol
  - Supported TMC boards: TMCM-1636
  - Supported communication interface and interface driver: CAN (SocketCAN)
  - Supported ROS and OS distro: Noetic (Ubuntu 20.04)
  - Supported platform: Intel x86 64-bit (amd64)

* Contributors: CAcarADI, mmaralit-adi
