/*
 * @Description: 通过ros发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "glog/logging.h"

namespace lidar_localization {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR&  cloud_ptr_input, double time) {
    ros::Time ros_time((float)time);
    PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR&  cloud_ptr_input) {
    /**
     * use_sim_time为true时，ros::time::now()输出系统时间；
     * use_sim_time为false时，ros::time::now()输出仿真时间，如果回放bag，则是bag的时间
    */
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);
}

/**
 * @description: 将点云数据发布出去
 * @param {CLOUD_PTR& } cloud_ptr_input：点云数据
 * @param {Time} time：时间戳
 * @return {*}
 */
void CloudPublisher::PublishData(CloudData::CLOUD_PTR&  cloud_ptr_input, ros::Time time) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

    cloud_ptr_output->header.stamp = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} // namespace lidar_localization