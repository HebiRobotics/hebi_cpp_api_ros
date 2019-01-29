^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hebi_cpp_api
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2019-01-29)
-----------
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
* Contributors: Matthew Tesch, iamtesch

2.0.1 (2018-12-19)
------------------
* Initial import of the HEBI C++ API v2.0.1
  - Note: package.xml and CMakeLists.txt have been changed to be catkin
  compliant.
* Addressed i386/armhf/aarch64 ros buildfarm issues.
* Contributors: Matthew Tesch
