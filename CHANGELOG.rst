^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hebi_cpp_api
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.12.3 (2025-02-10)
-------------------
* Fixed CMakeLists.txt to include proper C API files
* Contributors: Hariharan Ravichandran

3.12.2 (2025-02-07)
-------------------
* Update HEBI C++ API to 3.12.2
* Contributors: Hariharan Ravichandran

3.10.1 (2025-02-03)
-------------------
* Fix colcon build with symlink error
* Fix symlink for major version
* Contributors: David Conner, Hariharan Ravichandran

3.10.0 (2024-08-30)
-------------------
* Features/Changes:
  * add dynamics comp + doubled joint plugins
  * add getters/setters for impedance control plugin
  * use doubles instead of floats in arm plugins (minor change in experimental arm class)
  * parse "user data" values as same parameter types as for plugins instead of saving as strings (minor change in experimental arm class)
  * store absolute path for robot config file withing RobotConfig structure
  * try to load/set default gains when creating arm from config
  * update C API to 2.15.0 from 2.13.0
  * provides ubuntu 20.04 compatibility again
  * adds support for debug symbols in win X64 build
  * lookup searches localhost by default to work with private imitation groups from Scope
  * multiple log files per second now supported when using default file name (note -- naming convention changed from hh.mm.ss to hh-mm-ss.ms to match MATLAB API convention)
  * prevent certain large info messages returned from modules from getting dropped
  * added control strategy 5 to ControlStrategy enum
  * update minimum required CMake to 3.5 to reduce cmake warnings from newer versions
  * added helpers to optionally create Arm and MobileIO objects using an existing Lookup object
  * access "Runtime Data" in "Extra" info packets, including number of seconds a module has been on and/or commanded. Note -- requires firmware support, and count will only include time for module since first updating to firmware with this feature.
  * MobileIO class now has "sendLayout" and "sendLayoutBuffer" functions, allowing layout configurations to be sent to MobileIO devices

* Bugfixes:
  * added const attribute for a number of getters
  * small refactoring for readability, minor cleanup and formatting
  * update calls for C IK functions from internally deprecated ones to equivalent new ones
  * address a number of compiler warnings, while making warning checking more strict for GCC and clang
  * ignore warnings given by included Eigen library

* Contributors: Matthew Tesch, Aditya Nair

3.9.0 (2024-07-10)
------------------
* Bumped up the HEBI CPP API Version to 3.9.0 (changelogs for previous version are available at https://github.com/HebiRobotics/HEBI-Core-Cpp/releases)
* Initial release of HEBI API for ROS 2
* Contributors: Chris Bollinger, Hariharan Ravichandran

3.2.0 (2020-4-3)
------------------
* Added experimental high-level "Arm API" to enable easier control of robotic arm systems
* Contributors: Chris Bollinger

3.1.1 (2019-12-16)
------------------
* Fix incorrect behavior when getting and setting IO pin values
* Contributors: Matthew Tesch

3.1.0 (2019-12-02)
------------------
* Reduce conversion needs by adding (deprecated) overloads for:

  * getJ
  * getJacobians
  * getFK
  * getForwardKinematics
  * getFrameCount

* Fix multiple definition error

(from 3.0.0)

* Robot Model:

  * Added "input frame" type for forward kinematics operations
  * Added end effector support (custom and parallel gripper types)
  * Added R-series support (actuator, link, bracket, and end effector)
  * Added options for link input + output type
  * Support import of HRDF format 1.2.0

* Robot Model:

  * removed "combine" functionality for addJoint and addRigidBody
  * now only allows addition of elements which match the physical interface of the previous element
  * changed the behavior of "end effector" frames; by default, none are returned any unless an "end effector" is specifically added
  * Changed usages of HebiJointType, HebiFrameType, and HebiRobotModelElementType C-style enums to C++ scoped enums

* Fixed bug when setting IO pins in commands; commands would sometimes affect other pins.

(from 2.2.0)

* Added ability to set and clear text in the experimental mobile IO API
* Added ability to get raw feedback from experimental mobile IO API
* Contributors: Matthew Tesch

2.1.0 (2019-08-21)
------------------
  * Updated various messages:

  * Info:

  * Added "serial" getter for Info packets

  * Info and Command:

  * Added mstop strategy
  * Added position limit strategies
  * Added velocity limits
  * Added effort limits
  * Added flag for whether or not accelerometer feedback includes gravity (on supporting devices, namely Mobile IO devices

  * Command:

  * Added ability to set strings for and clear the "log" text field in the Mobile IO apps

  * Feedback:

  * Added "pwm command" feedback

  * Add "robot element metadata" that allows for introspection of RobotModel objects.
  * Import/Export safety parameters from/to a file into/from GroupCommand objects
  * Export safety parameters to a file from GroupInfo objects
  * Added "experimental" namespace intended for feature-preview items
  * Added "mobile io wrapper" to experimental namespace that allows for easier interface with Mobile IO devices
  * Update core C API from 1.4.2 to 1.8.0

  * Significantly faster Jacobian computation
  * Full wildcard lookup supported when creating groups
  * Significantly faster trajectory solver implementation
  * Added "subaddress" support in lookup, commands, feedback, and logging; allows for simulator support

  * Cleaned up code style:

  * default destructors and accessibility for deleted copy/move assignment operators
  * const on move operators (src/util.hpp)
  * made several getters inline

  * Added "FunctionCallResult" used when importing safety parameter files to allow error message to be accessed
  * Update core C API from 1.4.2 to 1.8.0

  * Fixed getters for motor position, ar position, ar orientation, ar quality, and battery level in feedback
  * Locale invariant conversion when reading in .xml files, such as gains and HRDF (always expect "1.23" instead of "1,23", regardless of system's locale setting)
  * Use Ethernet header instead of message packet content to discover modules on the network (fixes issue when using multiple interfaces - wired and wireless - on iPad or Android running HEBI Mobile I/O)
* Contributors: Matthew Tesch

2.0.2 (2019-01-29)
------------------
* Make package installable
* Moved the header files into an include directory
* Removed the Eigen folder; use ROS package instead
* Fixed CMake for installable package

  - Addressed Eigen dependency
  - Installed include files and libraries correctly

* NOTE: this does not correspond with an official 2.0.2
  release of the upstream HEBI C++ API, because these
  changes were all local ROS build system changes. This
  mismatch will be resolved in v2.1.0.
* Contributors: Matthew Tesch

2.0.1 (2018-12-19)
------------------
* Initial import of the HEBI C++ API v2.0.1
  - Note: package.xml and CMakeLists.txt have been changed to be catkin
  compliant.

* Addressed i386/armhf/aarch64 ros buildfarm issues.
* Contributors: Matthew Tesch
