/*
 * @Description: 点云畸变补偿
 * @Author: Ren Qian
 * @Date: 2020-02-25 14:39:00
 */
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include "glog/logging.h"

namespace lidar_localization {
/**
 * @description: 设置激光雷达点云去畸变需要的一些参数
 * @param {float} scan_period：激光雷达扫描周期，即激光雷达扫描一圈所用的时间
 * @param {VelocityData} velocity_data：激光雷达的加速度和角速度
 * @return {*}
 */
void DistortionAdjust::SetMotionInfo(float scan_period, VelocityData velocity_data) {
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y, velocity_data.linear_velocity.z;
    angular_rate_ << velocity_data.angular_velocity.x, velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
}

/**
 * 点云畸变的补偿方法：
 * 1）获取载体运动信息：包括角速度、速度，分别用来计算相对角度和相对位移。
 * 2）获取该激光点相对于起始时刻的时间差：由于是顺序扫描，我们可以很容易通过atan2(y, x)来计算出
 *      该激光点相对于第一个激光点旋转过的角度 /phi，我们知道雷达内部旋转 360 度用了100 ms，那么
 *      旋转 /phi 角度用了多长时间，就了然了。
 * 3）转换激光点：其实就是坐标系*向量，坐标系是旋转矩阵，向量是转换之前的激光点坐标，这个转换可以先
 *      旋转再平移，也可以用4*4矩阵一次性计算。
*/
/**
 * 函数功能：点云去畸变
*/
/**
 * @description: 激光雷达点云去畸变
 * @param {CLOUD_PTR&} input_cloud_ptr: 原始点云
 * @param {CLOUD_PTR&} output_cloud_ptr: 去畸变后的点云
 * @return {*}
 */
bool DistortionAdjust::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr) {
    CloudData::CLOUD_PTR origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));
    output_cloud_ptr.reset(new CloudData::CLOUD());

    float orientation_space = 2.0 * M_PI;
    float delete_space = 5.0 * M_PI / 180.0;
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);

    //先对点云数据进行简单的变换，使其起始角度为0度
    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotate_matrix = t_V.matrix();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);

    // 得到在新的方向上，机器人的线速度和角速度
    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index) {
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);
        if (orientation < 0.0)
            orientation += 2.0 * M_PI;
        
        if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
            continue;
        /**
         * 在 kitti 原始数据里，提供了每帧点云的起始采集时刻和终止采集时刻，kitti2bag 这个功能包
         * 把数据转成bag文件的过程中，利用起始时刻和终止时刻取了个平均值，即中间时刻，作为这一帧
         * 点云的采集时刻，下面的方法是把点全部转到起始时刻上去，所以在计算的每个激光点采集时刻上
         * 再减去50ms，这样就相当于把一帧点云的坐标系转到中间时刻对应的坐标系上去了。
        */
        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;

        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);
        Eigen::Vector3f rotated_point = current_matrix * origin_point;
        Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time;
        CloudData::POINT point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);
    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
    return true;
}

/**
 * 功能：根据当前激光雷达点与当前帧起始时间的间隔时间，得到该点对应的旋转矩阵
 * 输入：当前激光雷达点与当前帧起始时间的间隔时间
 * 输出：该激光雷达点对应的旋转矩阵
*/
Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time) {
    Eigen::Vector3f angle = angular_rate_ * real_time;
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;
    return t_V.matrix();
}
} // namespace lidar_localization