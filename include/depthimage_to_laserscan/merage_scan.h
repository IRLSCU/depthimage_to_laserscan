/**
 * @file merg_scan.h
 * @brief 深度相机图和单线激光雷达的扫描点云合作图像
 * @author wangpengcheng  (wangpengcheng2018@gmail.com)
 * @version 1.0
 * @date 2020-11-10 17:11:28
 * @copyright Copyright (c) 2020  IRLSCU
 * 
 * @par 修改日志:
 * <table>
 * <tr>
 *     <th>Date</th>
 *     <th>Version</th>
 *     <th>Author</th>
 *     <th>Description</th> 
 * </tr>
 * <tr>
 *      <td>2020-11-10 17:11:28</td>
 *      <td>1.0</td>
 *      <td>wangpengcheng</td>
 *      <td>创建文件修改初始内容</td>
 * </tr>
 * </table>
 */

#ifndef MERAGER_SCAN_H
#define MERAGER_SCAN_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <atomic>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

class MerageScan {
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy; 
public:
    /**
     * @brief Construct a new Merage Scan object
     * @param  lidar_handle             雷达订阅控制函数    
     * @param  depth_scan_handleMy      深度相机控制函数
     * @param  publish_handle           发布节点版本
     */
    MerageScan(ros::NodeHandle& lidar_handle,ros::NodeHandle& depth_scan_handle,ros::NodeHandle& publish_handle);
    
    /**
     * @brief  激光雷达数据合并点关键函数，将数据进行合并到res1中
     * @param  laser_scan_ptr_1     雷达激光扫描数据1
     * @param  laser_scan_ptr_2     雷达激光扫描数据2
     * @param  res                  扫描输出结果数据
     */
    static void merageScan( const sensor_msgs::LaserScanPtr& laser_scan_ptr_1,
                            const sensor_msgs::LaserScanPtr& laser_scan_ptr_2,
                            sensor_msgs::LaserScanPtr& res);
    /**
     * @brief  ros请求连接建立函数 
     * @param  pub              请求数据关键指针
     */
    void connectCallBack(const ros::SingleSubscriberPublisher& pub);
    /**
     * @brief 融合点订阅取消函数
     * @param  pub              请求数据关键指针
     */
    void disconnectCallBack(const ros::SingleSubscriberPublisher& pub);
    /**
     * @brief  同步消息和回调函数，执行两个雷达数据的同步和合并
     * @param  lidar_scan_point    雷达扫描点云
     * @param  depth_scan_point    深度扫描点云
     */
    void synCallBack(
                     const sensor_msgs::LaserScanPtr& lidar_scan_point,
                     const sensor_msgs::LaserScanPtr& depth_scan_point
                    );
private:

    message_filters::Subscriber<sensor_msgs::LaserScan>  lidar_scan_;           ///< 单线激光雷达扫描数据点
    message_filters::Subscriber<sensor_msgs::LaserScan> depth_scan_;            ///< 深度相机 雷达转换节点
    message_filters::Synchronizer<MySyncPolicy>      sync_;                     ///< 异步绑定处理节点
    ros::Publisher merage_pub_;
                                                  ///< 公共发布节点
    std::atomic<bool> is_connected_=false;         ///< 是否连接标志位
    std::mutex  laser_scan_lock_;                  ///< 输出结果数据保护锁
    sensor_msgs::LaserScanPtr  res_scan_ptr_;                    ///< 公共链接标志指针
};

#endif
