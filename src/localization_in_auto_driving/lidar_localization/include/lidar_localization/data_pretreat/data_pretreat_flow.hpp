/*
 * @Author: jia
 * @Date: 2023-01-27 19:33:59
 * @LastEditors: jia
 * @LastEditTime: 2023-01-31 13:03:32
 * @Description: 数据预处理模块
 * 功能：
 *  1）接收各传感器信息
 *  2）传感器数据时间同步
 *  3）点云运动畸变补偿
 *  4）传感器信息统一坐标系
 * 输入：
 *  1）GNSS组合导航位置、姿态、角速度、线速度等
 *  2）雷达点云信息
 *  3）雷达和IMU相对坐标系
 * 输出：
 *  1）GNSS组合导航位置、姿态
 *  2）畸变补偿后的点云
 * 备注：输出的信息均是经过时间同步的，时间戳已保持一致。
 */
#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
// publisher
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
// models
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"

namespace lidar_localization {
class DataPretreatFlow {
  public:
    DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic);

    bool Run();

  private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    // models
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;

    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    VelocityData current_velocity_data_;
    GNSSData current_gnss_data_;

    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}

#endif