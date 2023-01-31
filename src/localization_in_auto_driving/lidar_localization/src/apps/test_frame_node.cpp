/*
 * @Author: jia
 * @Date: 2023-01-27 19:33:59
 * @LastEditors: jia
 * @LastEditTime: 2023-01-30 18:19:38
 * @Description: 激光雷达SLAM基本框架
 * 输入：激光雷达点云数据、imu数据、GNSS数据、lidar和imu的tf数据
 * 输出：激光雷达点云数据、激光雷达里程计数据
 */
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "test_frame_node");
    ros::NodeHandle nh;

    // Subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    // tf 设置的是 子坐标系相对于父坐标系的 平移和旋转。
    // 获取父坐标系为 lidar ，子坐标系为 imu 的tf数据，用于将imu坐标系下的数据转换到lidar坐标系下
    std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

    // Publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", "map", 100);
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "/lidar", 100);

    // 用于存储订阅到的数据
    std::deque<CloudData> cloud_data_buff;
    std::deque<IMUData> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;
    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
    // 标志
    bool transform_received = false;
    bool gnss_origin_position_inited = false;

    ros::Rate rate(100); // 100Hz
    while (ros::ok()) {
        ros::spinOnce();

        // 获取订阅到的数据
        cloud_sub_ptr->ParseData(cloud_data_buff);
        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        // 获取tf变换数据，并且只需要获取一次
        if (!transform_received) {
            if (lidar_to_imu_ptr->LookupData(lidar_to_imu)) {
                transform_received = true;
                // LOG(INFO) << "lidar to imu transform matrix is:" << std::endl << lidar_to_imu;
            }
        } else {
            // 如果激光雷达点云数据、imu数据、gnss数据和tf数据都获取到了，并且激光雷达点云数据、imu数据、gnss数据均非空
            while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0) {
                // 每次循环都取 deque 中的第一个数据
                CloudData cloud_data = cloud_data_buff.front();
                IMUData imu_data = imu_data_buff.front();
                GNSSData gnss_data = gnss_data_buff.front();

                // 根据激光雷达点云时间戳与imu时间戳的差值，剔除不是同一时间戳的一对数据
                double d_time = cloud_data.time - imu_data.time;
                if (d_time < -0.05) {
                    cloud_data_buff.pop_front();
                } else if (d_time > 0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                } else {
                    // 去掉当前时间戳后边的一个数据，因为前面已经取了当前时间戳前面的数据
                    cloud_data_buff.pop_front();
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();

                    // 用于存储里程计数据
                    Eigen::Matrix4f odometry_matrix;

                    // gnss 初始点初始化
                    if (!gnss_origin_position_inited) {
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
                    }
                    gnss_data.UpdateXYZ(); // 将球坐标系转为笛卡尔坐标系，获得XYZ
                    odometry_matrix(0,3) = gnss_data.local_E;
                    odometry_matrix(1,3) = gnss_data.local_N;
                    odometry_matrix(2,3) = gnss_data.local_U;
                    // 从 imu 数据中获得机器人旋转矩阵
                    odometry_matrix.block<3,3>(0,0) = imu_data.GetOrientationMatrix();
                    // 下面两中写法得到的结果一样
                    // odometry_matrix *= lidar_to_imu;
                    odometry_matrix = lidar_to_imu * odometry_matrix;


                    pcl::transformPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, odometry_matrix);

                    cloud_pub_ptr->Publish(cloud_data.cloud_ptr);
                    odom_pub_ptr->Publish(odometry_matrix);
                }
            }
        }

        rate.sleep();
    }

    return 0;
}