
#include "merage_scan.h"


MerageScan::MerageScan(
                            ros::NodeHandle& lidar_handle,
                            ros::NodeHandle& depth_scan_handle,
                            ros::NodeHandle& publish_handle
                        ):
                        lidar_scan_(lidar_handle,"/rplidar_points",1),
                        depth_scan_(depth_scan_handle,"/depth_scan_points",1),
                        sync_(MySyncPolicy(10),lidar_scan_,depth_scan_),
                        res_scan_ptr_(new sensor_msgs::LaserScan())


{
    //创建发布节点;并设置回调函数
    this->merage_pub_ = publish_handle.advertise<sensor_msgs::LaserScan>("scan", 10,std::bind(&MerageScan::connectCallBack,this,_1),std::bind(&MerageScan::disconnectCallBack,this,_1));
    // 设置注册回调函数
    sync_.registerCallback(std::bind(&MerageScan::synCallBack,_1,_2));
}

void MerageScan::merageScan(
                            const sensor_msgs::LaserScanPtr& laser_scan_ptr_1,
                            const sensor_msgs::LaserScanPtr& laser_scan_ptr_2,
                            sensor_msgs::LaserScanPtr& res
                            )
{
    unsigned long rang_len = laser_scan_ptr_1->ranges.size()+laser_scan_ptr_2->ranges.size();
    {
        res->header.stamp=laser_scan_ptr_2->header.stamp;
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
   
};

void MerageScan::synCallBack( 
                        const sensor_msgs::LaserScanPtr& lidar_scan_point,
                        const sensor_msgs::LaserScanPtr& depth_scan_point
                     )
{
    if(is_connected_) {
        merageScan(lidar_scan_point,depth_scan_point,this->res_scan_ptr_);
        merage_pub_.publish(this->res_scan_ptr_);
    }
}

void MerageScan::connectCallBack(const ros::SingleSubscriberPublisher& pub) 
{
    is_connected_.store(true);
    ROS_INFO("SingleSubscriberPublisher connected");
}

void MerageScan::disconnectCallBack(const ros::SingleSubscriberPublisher& pub) 
{
    if(merage_pub_.getNumSubscribers()==0) {
        ROS_DEBUG("Unsubscribing from merage topic.");
        is_connected_.store(false);
        lidar_scan_.unsubscribe();
        depth_scan_.unsubscribe();
    }
}
