/**
 * @brief 生成点云地图
 * @author doudou
 * @version 1.0
 * @date 2021.04.21
 */

#include <iostream>
#include <string>
#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread/thread.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h> //传感器消息类型

#include <my_slam/saveMap.h>

/**
 * @brief args and functions in getMap.cpp
 */
//重定义数据类型
typedef pcl::PointXYZI PointT;

//点云回调函数
void points_cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg_in);

//激光里程计回调函数
void laser_odometry_handler(const nav_msgs::Odometry::ConstPtr &odom_msg);

//发布点云地图
void publish_map_cloud(const ros::Time &stamp, const Eigen::Matrix4f &pose);

//保存地图服务回调函数
static bool do_save_map(my_slam::saveMap::Request &req, my_slam::saveMap::Response &resp);

//转换
static geometry_msgs::TransformStamped
matrix2transform(const ros::Time &stamp, const Eigen::Matrix4f &pose, const std::string &frame_id,
                 const std::string &child_frame_id);


//变量
struct localParam {
    double time_laser_odometry = 0.0f;
    bool get_odometry_flag = false;
    Eigen::Matrix<float,4,4> odom_pose;

    double time_points_cloud = 0.0f;
    bool get_points_cloud_flag = false;

    bool debugSwitch = false;

    std::string map_frame_id = "map";
    std::string child_frame_id = "odom";

    pcl::PointCloud<PointT>::ConstPtr map_cloud(pcl::PointCloud<PointT>()); //点云地图空间
    pcl::PointCloud<PointT>::ConstPtr key_frame = nullptr; //点云地图空间
} param;


/**
 * @brief main func
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "getMap");
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 5, &laser_odometry_handler);
    ros::Subscriber cloud_points_sub = nh.subscribe("/filtered", 5, &points_cloud_handler);
    ros::ServiceServer save_map = nh.advertiseService("saveMap", &do_save_map);

    while (ros::ok()) {
        ros::spinOnce();
        //publish_map_cloud();
    }

    return 0;
}


/**
 * @brief 点云回调函数
 * @param msg_pc
 */
void points_cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg_in) { //固定参数写法
    if (!ros::ok()) {
        return;
    }
    if (param.debugSwitch)
        ROS_INFO("Map Node, Get Odom cloudCallback.");
    param.time_points_cloud = cloud_msg_in->header.stamp.toSec();//接受的点云时间戳
    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>); //申请点云空间
    pcl::fromROSMsg(*cloud_msg_in, *cloud_in); //转换消息格式
    param.key_frame = cloud_in;

    param.get_points_cloud_flag = true;
}


/**
 * @brief 激光雷达回调函数,提取里程计信息。
 * @param msg 里程计信息
 */
void laser_odometry_handler(const nav_msgs::Odometry::ConstPtr &odom_msg) { //固定参数写法
    if (!ros::ok()) {
        return;
    }
    param.time_laser_odometry = odom_msg->header.stamp.toSec(); //转化成秒

    param.odom_pose.setIdentity();
    param.odom_pose(0, 3) = odom_msg->pose.pose.position.x; //提取x
    param.odom_pose(1, 3) = odom_msg->pose.pose.position.y; //提取y
    param.odom_pose(2, 3) = odom_msg->pose.pose.position.z; //提取z
    geometry_msgs::Quaternion orientation = odom_msg->pose.pose.orientation; //提取四元数
    param.odom_pose.block<3, 3>(0, 0) = Eigen::Quaternion<float>(orientation.x, orientation.y, orientation.z,
                                                                  orientation.w).matrix(); //转换类型
    if (param.debugSwitch) {
        cout << "map node odometry pose is : " << endl << param.odom_pose << endl;
    }

    param.get_odometry_flag = true;
}


/**
 * @brief 发布点云地图
 * @param pose
 */
void publish_map_cloud(const ros::Time &stamp, const Eigen::Matrix4f &pose) {
    if(!param.get_odometry_flag || !param.get_points_cloud_flag)
        return;
    //发布
    static tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped map2odom = matrix2transform(stamp, pose, param.map_frame_id, param.child_frame_id);
    tf_broadcaster.sendTransform(map2odom);

    //叠加点云


}


/**
 * @brief 保存地图回调函数
 * @param req
 * @param resp
 * @return
 */
static bool do_save_map(my_slam::saveMap::Request &req, my_slam::saveMap::Response &resp) { //固定参数写法
    return true;
}


/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
static geometry_msgs::TransformStamped
matrix2transform(const ros::Time &stamp, const Eigen::Matrix4f &pose, const std::string &frame_id,
                 const std::string &child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion tf_quat;
    tf_quat.w = quat.w();
    tf_quat.x = quat.x();
    tf_quat.y = quat.y();
    tf_quat.z = quat.z();

    geometry_msgs::TransformStamped tf_trans;
    tf_trans.header.stamp = stamp;
    tf_trans.header.frame_id = frame_id;
    tf_trans.child_frame_id = child_frame_id;

    tf_trans.transform.translation.x = pose(0, 3);
    tf_trans.transform.translation.y = pose(1, 3);
    tf_trans.transform.translation.z = pose(2, 3);
    tf_trans.transform.rotation = tf_quat;

    return tf_trans;
}
