#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mplanner");
	ros::Time::init();

	ros::NodeHandle node;
	tf::TransformBroadcaster tf_broadcaster;
	tf::Transform transform;

	ros::Rate ros_timer(120);
	while(ros::ok()) {
		tf::Matrix3x3 R;
		R.setValue(1, 0, 0,
        	           0, 1, 0,
	                   0, 0 ,1);
		tf::Quaternion q;
		R.getRotation(q);

		transform.setOrigin(tf::Vector3(0, 0, 0));
		transform.setRotation(q);

		tf_broadcaster.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(),"origin", "uav"));

		ros_timer.sleep();
	}

	return 0;
}
