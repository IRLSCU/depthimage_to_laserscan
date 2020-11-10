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

class MerageScan {
public:
    /**
     * @brief Construct a new Merage Scan object
     * @param  lidar_handle             雷达订阅控制函数    
     * @param  depth_scan_handleMy      深度相机控制函数
     */
    MerageScan(ros::NodeHandle& lidar_handle,ros::NodeHandle& depth_scan_handle);
    
    /**
     * @brief  激光雷达数据合并点关键函数，将数据进行合并到res1中
     * @param  laser_scan_ptr_1     雷达激光扫描数据1
     * @param  laser_scan_ptr_2     雷达激光扫描数据2
     * @param  res                  扫描输出结果数据
     */
    static void merageScan(sensor_msgs::LaserScanPtr laser_scan_ptr_1,sensor_msgs::LaserScanPtr laser_scan_ptr_2,sensor_msgs::LaserScanPtr res);
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

    inline ros::Subscriber getLidarScan() {return lidar_scan_;}
    inline ros::Subscriber getDepthScan() {return depth_scan_;}
    void setLidarScan(ros::Subscriber lidar_scan) {lidar_scan_ = lidar_scan;}
    void SetDepthScan(ros::Subscriber depth_scan) {depth_scan_ = depth_scan_;}
    

private:
    ros::Subscriber lidar_scan_;        ///< 单线激光雷达扫描数据点
    ros::Subscriber depth_scan_;        ///< 深度相机 雷达转换节点
    ros::Publisher merage_pub_;         ///< 公共发布节点
    
};

#endif
