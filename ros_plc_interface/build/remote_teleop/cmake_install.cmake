# Install script for directory: /home/ankur/Documents/rucha_mowito/mw_seimens_s7_plc_interface/ros_plc_interface/src/remote_teleop

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ankur/Documents/rucha_mowito/mw_seimens_s7_plc_interface/ros_plc_interface/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ankur/Documents/rucha_mowito/mw_seimens_s7_plc_interface/ros_plc_interface/build/remote_teleop/catkin_generated/installspace/remote_teleop.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/remote_teleop/cmake" TYPE FILE FILES
    "/home/ankur/Documents/rucha_mowito/mw_seimens_s7_plc_interface/ros_plc_interface/build/remote_teleop/catkin_generated/installspace/remote_teleopConfig.cmake"
    "/home/ankur/Documents/rucha_mowito/mw_seimens_s7_plc_interface/ros_plc_interface/build/remote_teleop/catkin_generated/installspace/remote_teleopConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/remote_teleop" TYPE FILE FILES "/home/ankur/Documents/rucha_mowito/mw_seimens_s7_plc_interface/ros_plc_interface/src/remote_teleop/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/remote_teleop" TYPE PROGRAM FILES "/home/ankur/Documents/rucha_mowito/mw_seimens_s7_plc_interface/ros_plc_interface/build/remote_teleop/catkin_generated/installspace/teleop_plc.py")
endif()

