/**
 * @file laser_data_correction.h
 * @brief 深度相机图经过单线激光雷达的修正
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2020-11-10 17:11:28
 * @copyright Copyright (c) 2020  IRLSCU
 * 
 * @par 修改日志:
 */

#ifndef LASER_DATA_CORRECTION_H
#define LASER_DATA_CORRECTION_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
//#include <sensor_msgs/LaserScan.h>
//#include <algorithm>
//#include <functional>
//#include <memory>
//#include <mutex>
//#include <atomic>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

class LaserDataCorrection
{
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;

public:
    LaserDataCorrection(ros::NodeHandle &m_pub, ros::NodeHandle &m_lisn);

    ~LaserDataCorrection();

    /**
     * @brief  ros请求连接建立函数 
     * @param  pub              请求数据关键指针
     */
    void connectCallBack(const ros::SingleSubscriberPublisher &pub);

    /**
     * @brief 融合点订阅取消函数
     * @param  pub              请求数据关键指针
     */
    void disconnectCallBack(const ros::SingleSubscriberPublisher &pub);
    
    /**
     * @brief  同步消息和回调函数，执行两个雷达数据的同步和合并
     * @param  lidar_scan_point    雷达扫描点云
     * @param  depth_scan_point    深度扫描点云
     */
    void synCallBack(
        const sensor_msgs::LaserScan::ConstPtr& lidar_scan_point,
        const sensor_msgs::LaserScan::ConstPtr& depth_scan_point);

private:
    message_filters::Subscriber<sensor_msgs::LaserScan> lidar_scan_; ///< 单线激光雷达扫描数据点
    message_filters::Subscriber<sensor_msgs::LaserScan> depth_scan_; ///< 深度相机 雷达转换节点
    message_filters::Synchronizer<MySyncPolicy> sync_;               ///< 异步绑定处理节点
    ros::Publisher merage_pub_;
    ///< 公共发布节点
    std::atomic_bool is_connected_; ///< 是否连接标志位
    std::mutex laser_scan_lock_;             ///< 输出结果数据保护锁
    sensor_msgs::LaserScanPtr res_scan_ptr_; ///< 公共链接标志指针
};

#endif
