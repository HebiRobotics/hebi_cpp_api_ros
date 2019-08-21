^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hebi_cpp_api
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2019-08-21)
-----------
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

2.0.2 (2019-01-29)
-----------
* Make package installable
* Moved the header files into an include directory
* Removed the Eigen folder; use ROS package instead
* Fixed CMake for installable package

  * Addressed Eigen dependency
  * Installed include files and libraries correctly

* NOTE: this does not correspond with an official 2.0.2
  release of the upstream HEBI C++ API, because these
  changes were all local ROS build system changes. This
  mismatch will be resolved in v2.1.0.
* Contributors: Matthew Tesch, iamtesch

2.0.1 (2018-12-19)
------------------
* Initial import of the HEBI C++ API v2.0.1

  * Note: package.xml and CMakeLists.txt have been changed to be catkin
    compliant.

* Addressed i386/armhf/aarch64 ros buildfarm issues.
* Contributors: Matthew Tesch
