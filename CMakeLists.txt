cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(hebi_cpp_api VERSION 3.9.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

# If you want to include the C API from an out of source tree location,
# pass `-DHEBI_C_DIR=<dir>` to cmake when generating this project
if (NOT DEFINED HEBI_C_DIR)
  set(HEBI_C_DIR hebi)
endif()

get_filename_component(HEBI_C_DIR ${HEBI_C_DIR} ABSOLUTE)
add_subdirectory(${HEBI_C_DIR} hebi-c-build-dir)

set(hebi_cpp_src
  include/${PROJECT_NAME}/mac_address.cpp
  include/${PROJECT_NAME}/command.cpp
  include/${PROJECT_NAME}/feedback.cpp
  include/${PROJECT_NAME}/info.cpp
  include/${PROJECT_NAME}/message_helpers.cpp
  include/${PROJECT_NAME}/group_command.cpp
  include/${PROJECT_NAME}/group_feedback.cpp
  include/${PROJECT_NAME}/group_info.cpp
  include/${PROJECT_NAME}/group.cpp
  include/${PROJECT_NAME}/log_file.cpp
  include/${PROJECT_NAME}/lookup.cpp
  include/${PROJECT_NAME}/trajectory.cpp
  include/${PROJECT_NAME}/robot_config.cpp
  include/${PROJECT_NAME}/robot_model.cpp
  include/${PROJECT_NAME}/version.cpp)

set(hebi_cpp_util
  include/${PROJECT_NAME}/util/file.cpp
  include/${PROJECT_NAME}/util/mobile_io.cpp)

set(hebi_cpp_arm
  include/${PROJECT_NAME}/arm/kinematics_helper.cpp
  include/${PROJECT_NAME}/arm/arm.cpp)

set(hebi_cpp_third_party
  include/${PROJECT_NAME}/rapidyaml/rapidyaml-0.5.0.cpp)

add_library(_hebic++-obj OBJECT ${hebi_cpp_src} ${hebi_cpp_util} ${hebi_cpp_arm} ${hebi_cpp_third_party})
add_library(hebic++ SHARED $<TARGET_OBJECTS:_hebic++-obj>)
add_library(hebic++-static STATIC $<TARGET_OBJECTS:_hebic++-obj>)

target_link_libraries(hebic++ PUBLIC hebi)
target_link_libraries(hebic++-static PUBLIC hebi)

target_include_directories(_hebic++-obj
  PUBLIC include
  $<TARGET_PROPERTY:hebi,INTERFACE_INCLUDE_DIRECTORIES>)
target_include_directories(hebic++ INTERFACE $<TARGET_PROPERTY:_hebic++-obj,INTERFACE_INCLUDE_DIRECTORIES>)
target_include_directories(hebic++-static INTERFACE $<TARGET_PROPERTY:_hebic++-obj,INTERFACE_INCLUDE_DIRECTORIES>)

set_target_properties(_hebic++-obj hebic++ hebic++-static PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED YES
  POSITION_INDEPENDENT_CODE ON
  VERSION ${hebi_cpp_api_VERSION})

ament_export_dependencies(EIGEN3)
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_libraries(hebic++ hebi)
ament_package()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

install(DIRECTORY hebi/include/
        DESTINATION include/${PROJECT_NAME})

install(TARGETS hebic++ hebic++-static
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib)

# Cannot install imported library targets with CMake, so we have to manually
# install this here...
# Also, note that we cannot get the "version" of a subproject (this variable has local
# scope), so we hardcode the subproject version here.
get_target_property(HEBI_LOCATION hebi LOCATION)
install(FILES ${HEBI_LOCATION} DESTINATION lib)
install(FILES ${HEBI_LOCATION}.2 DESTINATION lib)
install(FILES ${HEBI_LOCATION}.2.13 DESTINATION lib)
