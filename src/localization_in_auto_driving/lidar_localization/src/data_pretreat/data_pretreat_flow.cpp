/*
 * @Author: jia
 * @Date: 2023-01-27 19:33:59
 * @LastEditors: jia
 * @LastEditTime: 2023-02-01 18:10:19
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
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic) {
    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    // 订阅的是 lidar 坐标系变换到 imu 坐标系的坐标变换
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh,  "/velo_link", "/imu_link");
    // lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "imu_link",  "velo_link");

    // publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "velo_link", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "map", "velo_link", 100);
    // model -- 去点云畸变的模块
    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() {
    if (!ReadData())
        return false;

    if (!InitCalibration()) 
        return false;

    if (!InitGNSS())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        TransformData();
        PublishData();
    }

    return true;
}

/**
 * @description: 获取点云数据，原始 imu数据、原始 线速度和角速度数据、原始 gnss 数据，
 *               并将原始 imu数据、原始 线速度和角速度数据、原始 gnss 数据的时间与点云时间对其。
 * @return {*}
 */
bool DataPretreatFlow::ReadData() {
    // 获取点云数据
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    // 定义临时变量用于临时存储未进行时间同步 的数据
    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_;

    // 获取 原始 imu数据、原始 线速度和角速度数据、原始 gnss 数据
    imu_sub_ptr_->ParseData(unsynced_imu_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if (cloud_data_buff_.size() == 0)
        return false;

    // 使用 点云时间 进行 原始 imu数据、原始 线速度和角速度数据、原始 gnss 数据 的时间同步
    double cloud_time = cloud_data_buff_.front().time;
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

    // 全部数据都时间同步后才算完成该函数，从而进行下一步
    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu || !valid_velocity || !valid_gnss) {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

/**
 * @description: 获取 TF 坐标变换数据
 * @return {*}
 */
bool DataPretreatFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

/**
 * @description: 将 GNSS 数据作为机器人的初始化位姿
 * @return {*}
 */
bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (velocity_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;

    return true;
}

bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

    if (diff_velocity_time > 0.05) {
        velocity_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformData() {
    gnss_pose_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_pose_ *= lidar_to_imu_;

    /**
     * 数据中提供的速度是 IMU 所处位置的速度，而我们要的是激光雷达所处位置的速度，由于
     * 这两者并不重合，即存在杆臂，所以在车旋转时它们的速度并不一致，需要按照这两者之间
     * 的相对坐标，把速度转到雷达对应的位置上去，这个功能放在了sensor_data的velocity_data.cpp，
     * 把它作为VelocityData类的成员函数，只要给他一个相对坐标，它就自动把类内部成员变量转换，
     * 就像下面的 TransformCoordinate()
    */
    current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    return true;
}

/**
 * @description: 发布去畸变后的点云 和 GNSS数据
 * @return {*}
 */
bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);

    return true;
}
}