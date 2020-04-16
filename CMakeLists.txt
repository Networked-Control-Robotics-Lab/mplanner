cmake_minimum_required(VERSION 2.8.3)
project(mplanner)

set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++0x  -g -static-libstdc++")

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
    ./lib/mavlink_v2/common
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
    ./src/trajectory/traj_optimizer.cpp
    ./src/trajectory/trajectory.cpp)

add_executable(mplanner ${SRC})
include_directories(${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIR} ./lib/cvxopt ${INC})
target_link_libraries (mplanner ${PYTHON_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(mplanner ${PROJECT_NAME}_generate_messages_cpp)
