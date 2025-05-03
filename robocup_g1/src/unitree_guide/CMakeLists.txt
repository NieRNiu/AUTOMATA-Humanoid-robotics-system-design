cmake_minimum_required(VERSION 3.5.1)
project(unitree_guide)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Project configuration options
set(ROBOT_TYPE g1) # g1 
set(PLATFORM amd64) # The platform to compile, support amd64 
set(CATKIN_MAKE ON) # Use CATKIN_MAKE or not, ON or OFF
set(SIMULATION ON) # Use Gazebo or not, ON or OFF

if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()

if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

if(${ROBOT_TYPE} STREQUAL "g1")
    add_definitions(-DROBOT_TYPE_G1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

if(SIMULATION)
    add_definitions(-DRUN_ROS)
    set(CATKIN_MAKE ON)
endif()

find_package(Boost)

if(CATKIN_MAKE)
    add_definitions(-DCOMPILE_WITH_ROS)
    add_definitions(-DCOMPILE_WITH_CATKIN_MAKE)
endif()

if(CATKIN_MAKE)
    if(SIMULATION)
        add_definitions(-DCOMPILE_WITH_SIMULATION)
        find_package(catkin REQUIRED COMPONENTS
            controller_manager
            joint_state_controller
            gazebo_ros
        )
    endif()

    find_package(catkin REQUIRED COMPONENTS
        genmsg
        robot_state_publisher
        roscpp
        std_msgs
        tf
        geometry_msgs
        unitree_legged_msgs
    )
    catkin_package(
        CATKIN_DEPENDS
        unitree_legged_msgs
    )
endif()

add_subdirectory(thirdParty/yaml-cpp)
add_subdirectory(thirdParty/MNN)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    /usr/local/include/eigen3  
    thirdParty/MNN/include/
    thirdParty/yaml/include/
    thirdParty/onnxruntime-linux-x64-1.18.0/include/
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
    "src/*/*/*.cpp"
    "src/*/*/*/*.cpp"
)

add_executable(junior_ctrl src/main.cpp ${SRC_LIST} ${STDLIB_LIST} )
target_link_libraries(junior_ctrl yaml-cpp MNN)

if(CATKIN_MAKE)
    target_link_libraries(junior_ctrl ${catkin_LIBRARIES})
    add_dependencies(junior_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()
