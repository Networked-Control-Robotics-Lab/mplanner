cmake_minimum_required(VERSION 2.8.3)
project(mplanner)

set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++0x  -g -static-libstdc++")

find_package(
    catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    tf
    message_generation
)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

set(INC
    ./ 
    ./lib/mavlink_v2/common
    ./src
    ./src/threads)

set(SRC
    ./src/main.cpp
    ./src/serial.cpp
    ./src/quadshell.cpp
    ./src/threads/shell_thread.cpp
    ./src/threads/ros_thread.cpp)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${INC})
add_executable(mplanner ${SRC})
target_link_libraries (mplanner ${catkin_LIBRARIES})
add_dependencies(mplanner ${PROJECT_NAME}_generate_messages_cpp)
