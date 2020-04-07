#include <math.h>
#include <thread>
#include <ros/ros.h>
#include "ros_thread.hpp"
#include "shell_thread.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mplanner");
	ros::Time::init();

	std::thread thread_ros(ros_thread_entry);
	std::thread thread_shell(shell_thread_entry);

	thread_ros.join();
	thread_shell.join();

	return 0;
}
