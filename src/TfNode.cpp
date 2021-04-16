// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/tf.h>
// #include <sensor_msgs/LaserScan.h>
// #include <algorithm>
// #include <functional>
// #include "depthimage_to_laserscan/merage_scan.h"
// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "laser_tf_Node");
// 	ros::NodeHandle scan_node;
// 	ros::NodeHandle sub_node("~");
// 	MerageScan(sub_node, sub_node, scan_node);
// 	ros::spin();
// }

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <depthimage_to_laserscan/merage_scan.h>
#include <depthimage_to_laserscan/DepthImageToLaserScan.h>
#include <iostream>
#include <cmath>
//#define FAST
using namespace std;
using namespace message_filters;

ros::Publisher PointCloudInfo_pub;
ros::Publisher ImageInfo_pub;
ros::Publisher res_pub;

sensor_msgs::LaserScan syn_pointcloud, syn_iamge, res;
/**
 * @brief  雷达点云融合函数，将深度雷达扫描图像，融合到360雷达激光图中
 * 			1. 计算angle_min与angle_max对应的雷达图像的index
 * 			2. 根据index 范围进行点扫描，反向计算雷达扫描点对应的深度扫描index
 * 			3. 比较值，小于当前深度就行更新
 * 			4. 
 * @param  laser_scan_ptr_1 My Param doc
 * @param  laser_scan_ptr_2 My Param doc
 * @param  res              My Param doc
 */
void merageScan(
	const sensor_msgs::LaserScan::ConstPtr &laser_scan_ptr_1,
	const sensor_msgs::LaserScan::ConstPtr &laser_scan_ptr_2,
	sensor_msgs::LaserScan &res)
{
	/* 头部信息赋值 */
	res.header = laser_scan_ptr_1->header;
	res.angle_min = laser_scan_ptr_1->angle_min;
	res.angle_max = laser_scan_ptr_1->angle_max;
	res.range_min = laser_scan_ptr_1->range_min;
	res.range_max = laser_scan_ptr_1->range_max;
	res.angle_increment = laser_scan_ptr_1->angle_increment;
	res.time_increment = laser_scan_ptr_1->time_increment;
	res.scan_time = laser_scan_ptr_1->scan_time;
	res.ranges = laser_scan_ptr_1->ranges;
#ifdef FAST
	/* 数据合并 */
	unsigned long start_index, end_index;

	start_index = (unsigned long)(abs(laser_scan_ptr_2->angle_min - laser_scan_ptr_1->angle_min) / laser_scan_ptr_1->angle_increment);
	end_index = (unsigned long)(abs(laser_scan_ptr_2->angle_max - laser_scan_ptr_1->angle_min) / laser_scan_ptr_1->angle_increment);
	end_index = min(end_index, (unsigned long)laser_scan_ptr_1->ranges.size());
	start_index = max(start_index - 18, (unsigned long)0);
	std::cout << "[" << start_index << "," << end_index << "]" << std::endl;
	if (end_index >= start_index)
	{
		//cout<< "===="<<res.ranges[start_index]<<endl;
		while (end_index >= start_index && start_index < laser_scan_ptr_1->ranges.size())
		{
			// 计算对应index
			// 计算对应的角度
			unsigned long temp_index = (unsigned long)((((start_index * laser_scan_ptr_1->angle_increment + laser_scan_ptr_1->angle_min) - laser_scan_ptr_2->range_min) / laser_scan_ptr_2->angle_increment) + 0.5);
			if (temp_index < laser_scan_ptr_2->ranges.size())
			{
				if (depthimage_to_laserscan::DepthImageToLaserScan::use_point(laser_scan_ptr_2->ranges[temp_index], res.ranges[start_index], res.range_min, res.range_max))
				{
					res.ranges[start_index] = laser_scan_ptr_2->ranges.at(temp_index);
				}
			}
			++start_index;
		}
		cout << "====" << res.ranges[start_index] << endl;
	}
	else
	{
		ROS_INFO("depth sscan is over rplidar's angle range");
	}
#else
	// 遍历数据点进行修正
	for (int i = 0; i < laser_scan_ptr_2->ranges.size(); ++i)
	{
		if (!std::isfinite(laser_scan_ptr_2->ranges.at(i)))
		{
			continue;
		}
		//std::cout  <<":" << laser_scan_ptr_2->ranges.at(i)<< ",";
		//std::cout << "start==" << std::endl;
		// 计算新的索引
		// size_t temp_index = laser_scan_ptr_1->ranges.size();
		// auto temp_key = (i * laser_scan_ptr_2->angle_increment);
		// if (temp_key > 0) {
		// 	temp_index = (size_t)((temp_key- laser_scan_ptr_1->angle_min - M_PI_2)/laser_scan_ptr_1->angle_increment);
		// }else {
		// 	temp_index = (size_t)((temp_key- laser_scan_ptr_1->angle_min + M_PI_2)/laser_scan_ptr_1->angle_increment);
		// }
		auto temp_key_index = (long)((i * laser_scan_ptr_2->angle_increment + laser_scan_ptr_2->angle_min - laser_scan_ptr_1->angle_min - M_PI) / laser_scan_ptr_1->angle_increment);
		//std::cout<<temp_key_index<<std::endl;
		if (temp_key_index < 0) {
			temp_key_index += laser_scan_ptr_1->ranges.size();
		}
		size_t temp_index = (size_t)temp_key_index;
		//size_t temp_index = (size_t)((i * laser_scan_ptr_2->angle_increment + laser_scan_ptr_2->angle_min - laser_scan_ptr_1->angle_min - M_PI) / laser_scan_ptr_1->angle_increment);
		//std::cout << "temp_index1:" << res.ranges[temp_index] <<":" << laser_scan_ptr_2->ranges.at(i)<< std::endl;
		if (temp_index < laser_scan_ptr_1->ranges.size())
		{
			if (depthimage_to_laserscan::DepthImageToLaserScan::use_point(
					laser_scan_ptr_2->ranges[i],
					res.ranges[temp_index],
					res.range_min, 
					res.range_max)
					)
			{
				res.ranges[temp_index] = laser_scan_ptr_2->ranges.at(i);
				//std::cout << "temp_index:" << res.ranges[temp_index] << std::endl;
			}
		}
		else
		{
			ROS_INFO("depth sscan is over rplidar's angle range");
		}
	}
	//std::cout<<std::endl;
#endif
};

void Syncallback(const sensor_msgs::LaserScan::ConstPtr &ori_pointcloud, const sensor_msgs::LaserScan::ConstPtr &ori_image)
{
	cout << "\033[1;32m Syn! \033[0m" << endl;
	syn_pointcloud = *ori_pointcloud;
	syn_iamge = *ori_image;
	cout << "syn pointcloud' timestamp : " << syn_pointcloud.header.stamp << endl;
	cout << "syn image's timestamp : " << syn_iamge.header.stamp << endl;
	sensor_msgs::LaserScan temp_res;
	merageScan(ori_pointcloud, ori_image, temp_res);
	// PointCloudInfo_pub.publish(ori_pointcloud);
	// ImageInfo_pub.publish(ori_image);
	res_pub.publish(temp_res);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "hw1");
	ros::NodeHandle node;

	cout << "\033[1;31m hw1! \033[0m" << endl;

	// 建立需要订阅的消息对应的订阅器
	message_filters::Subscriber<sensor_msgs::LaserScan> PointCloudInfo_sub(node, "/scan", 1);
	message_filters::Subscriber<sensor_msgs::LaserScan> ImageInfo_sub(node, "/depth_scan_points", 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), PointCloudInfo_sub, ImageInfo_sub); //queue size=10
	sync.registerCallback(boost::bind(&Syncallback, _1, _2));

	// PointCloudInfo_pub = node.advertise<sensor_msgs::LaserScan>("/djq_pc", 10);
	// ImageInfo_pub = node.advertise<sensor_msgs::LaserScan>("/djq_image", 10);
	res_pub = node.advertise<sensor_msgs::LaserScan>("/merg_res", 10);

	ros::spin();
	return 0;
}