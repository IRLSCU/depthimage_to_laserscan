#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <functional>
#include "merage_scan.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_tf_Node");
	ros::NodeHandle scan_node("merage_scan");
	ros::NodeHandle sub_node("sub_scan");
	MerageScan(sub_node, sub_node, scan_node);
	ros::spin();
}
