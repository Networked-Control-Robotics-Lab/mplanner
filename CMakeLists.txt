cmake_minimum_required(VERSION 2.8.3)
project(mplanner)

set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++17 -g -static-libstdc++ -Wno-address-of-packed-member")

find_package(PythonLibs REQUIRED)

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
    ./src/lib/mavlink_v2/common
    ./src/lib/mavlink_v2/ncrl_mavlink
    ./src/lib/matplotlib-cpp
    ./src
    ./src/threads
    ./src/mavlink
    ./src/shell
    ./src/trajectory)

set(SRC
    ./src/main.cpp
    ./src/pose.cpp
    ./src/mavlink/serial.cpp
    ./src/mavlink/publisher.cpp
    ./src/mavlink/receiver.cpp
    ./src/mavlink/parser.cpp
    ./src/shell/quadshell.cpp
    ./src/shell/shell_cmds.cpp
    ./src/threads/shell_thread.cpp
    ./src/threads/ros_thread.cpp
    ./src/threads/mavlink_thread.cpp
    ./src/trajectory/planner.cpp
    ./src/trajectory/trajectory.cpp)

add_executable(mplanner ${SRC})
include_directories(${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIR} ./src/lib/cvxopt ${INC})
target_link_libraries (mplanner ${PYTHON_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(mplanner ${PROJECT_NAME}_generate_messages_cpp)
