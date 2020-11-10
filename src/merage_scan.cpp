
#include "merage_scan.h"


MerageScan::MerageScan(
                            ros::NodeHandle& lidar_handle,
                            ros::NodeHandle& depth_scan_handle,
                            ros::NodeHandle& publish_handle
                            )
{
    //使用handle进行参数绑定
    
    //创建发布节点;并设置回调函数
    this->merage_pub_ = publish_handle.advertise<sensor_msgs::LaserScan>("scan", 10,std::bind(&MerageScan::connectCallBack,this,_1),std::bind(&MerageScan::disconnectCallBack,this,_1));
}

void MerageScan::merageScan(
                            sensor_msgs::LaserScanPtr laser_scan_ptr_1,
                            sensor_msgs::LaserScanPtr laser_scan_ptr_2,
                            sensor_msgs::LaserScanPtr res
                            )
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
};