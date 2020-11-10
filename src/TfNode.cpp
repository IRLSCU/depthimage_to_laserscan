#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <functional>
void merg_scan(sensor_msgs::LaserScanPtr laser_scan_ptr_1,sensor_msgs::LaserScanPtr laser_scan_ptr_2,sensor_msgs::LaserScanPtr res)
{
  unsigned long rang_len = laser_scan_ptr_1->ranges.size()+laser_scan_ptr_2->ranges.size();
  res->header.stamp=ros::Time::now();
  res->time_increment=laser_scan_ptr_1->time_increment;
  res->header.frame_id = "laser_frame";
  res->angle_min = std::min(laser_scan_ptr_1->angle_min,laser_scan_ptr_2->angle_min);
  res->angle_max = std::max(laser_scan_ptr_1->angle_max,laser_scan_ptr_2->angle_max);
  res->range_min = std::min(laser_scan_ptr_1->range_min,laser_scan_ptr_2->range_min);
  res->range_max = std::max(laser_scan_ptr_1->angle_max,laser_scan_ptr_2->angle_max);
  res->ranges.clear();
  res->ranges.resize(rang_len);
  merge(laser_scan_ptr_1->ranges.begin(),laser_scan_ptr_1->ranges.end(),laser_scan_ptr_2->ranges.begin(),laser_scan_ptr_2->ranges.end(),res->ranges.begin());
  res->intensities.clear();
  res->intensities.resize(rang_len);
  merge(laser_scan_ptr_1->intensities.begin(),laser_scan_ptr_1->intensities.end(),laser_scan_ptr_2->intensities.begin(),laser_scan_ptr_2->intensities.end(),res->intensities.begin());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_tf_Node");
  ros::NodeHandle node;
  // 订阅节点
  ros::Subscriber sub = ; 
  // 发布节点
  ros::Publisher merage_pub = node.advertise<sensor_msgs::LaserScan>("merage_scan",1000);
  ros::Rate r(100);
 sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
  tf::TransformBroadcaster broadcaster;
  
  while(ros::ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(7, 0.0, 0.0)),ros::Time::now(),"laser1", "laser2"));
    r.sleep();
    ros::spinOnce();
  }
}
