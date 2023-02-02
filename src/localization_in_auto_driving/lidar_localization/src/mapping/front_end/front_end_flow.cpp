/*
 * @Author: jia
 * @Date: 2023-01-27 19:33:59
 * @LastEditors: jia
 * @LastEditTime: 2023-02-02 18:24:30
 * @Description: front end 前端里程计任务管理
 * 输入：
 *  1）去畸变后的点云数据
 * 输出：
 *  2）雷达里程计（frame_id: map, child_frame_id: /lidar）
 */
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, odom_topic, "map", "/lidar", 100);

    // 初始化前端里程计类，即初始化前端里程计需要用到的各类参数
    front_end_ptr_ = std::make_shared<FrontEnd>();
}

/**
 * @description: 前端里程计步骤：
 *  1）获取去畸变后的点云数据
 *  2）根据雷达点云数据获得雷达里程计数据
 *  3）发布里程计数据
 * @return {*}
 */
bool FrontEndFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    return true;
}

bool FrontEndFlow::HasData() {
    return cloud_data_buff_.size() > 0;
}

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();

    return true;
}

/**
 * @description: 得到 雷达里程计
 * @return {*}
 */
bool FrontEndFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;
    if (!odometry_inited) {
        odometry_inited = true;
        front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }

    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

/**
 * @description: 发布雷达里程计数据
 * @return {*}
 */
bool FrontEndFlow::PublishData() {
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);

    return true;
}
}