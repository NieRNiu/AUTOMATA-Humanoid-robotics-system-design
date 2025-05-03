# Install script for directory: /home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nier/robocup_g1/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/unitree_legged_msgs/msg" TYPE FILE FILES
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/MotorCmd.msg"
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/MotorState.msg"
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/BmsCmd.msg"
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/BmsState.msg"
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/Cartesian.msg"
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/IMU.msg"
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/LED.msg"
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/LowCmd.msg"
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/LowState.msg"
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/HighCmd.msg"
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/HighState.msg"
    "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/msg/MoveCmd.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/unitree_legged_msgs/cmake" TYPE FILE FILES "/home/nier/robocup_g1/build/unitree_ros_to_real/unitree_legged_msgs/catkin_generated/installspace/unitree_legged_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/nier/robocup_g1/devel/include/unitree_legged_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/nier/robocup_g1/devel/share/roseus/ros/unitree_legged_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/nier/robocup_g1/devel/share/common-lisp/ros/unitree_legged_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/nier/robocup_g1/devel/share/gennodejs/ros/unitree_legged_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/nier/robocup_g1/devel/lib/python3/dist-packages/unitree_legged_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/nier/robocup_g1/devel/lib/python3/dist-packages/unitree_legged_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nier/robocup_g1/build/unitree_ros_to_real/unitree_legged_msgs/catkin_generated/installspace/unitree_legged_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/unitree_legged_msgs/cmake" TYPE FILE FILES "/home/nier/robocup_g1/build/unitree_ros_to_real/unitree_legged_msgs/catkin_generated/installspace/unitree_legged_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/unitree_legged_msgs/cmake" TYPE FILE FILES
    "/home/nier/robocup_g1/build/unitree_ros_to_real/unitree_legged_msgs/catkin_generated/installspace/unitree_legged_msgsConfig.cmake"
    "/home/nier/robocup_g1/build/unitree_ros_to_real/unitree_legged_msgs/catkin_generated/installspace/unitree_legged_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/unitree_legged_msgs" TYPE FILE FILES "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/unitree_legged_msgs" TYPE DIRECTORY FILES "/home/nier/robocup_g1/src/unitree_ros_to_real/unitree_legged_msgs/include/unitree_legged_msgs/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

