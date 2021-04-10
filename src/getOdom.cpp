/**
 * @brief icp获取里程计
 * @author doudou
 * @version 1.1
 * @date 2020.04.10
 */

#include <iostream>
#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread/thread.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h> //传感器消息类型

//订阅者与发布者
ros::Subscriber points_sub;
ros::Publisher odom_pub;


//重定义数据类型
typedef pcl::PointXYZI PointT;


//变量
struct localParam {
    pcl::PointCloud<PointT>::ConstPtr keyframe = nullptr;
    Eigen::Matrix4f keyframe_pose;
    ros::Time keyframe_stamp;

    pcl::IterativeClosestPoint<PointT, PointT> icp; //icp迭代器
    int max_iterations = 500;                 //最大迭代次数
    double euclidean_fitness_epsilon = 0.001; //收敛条件是均方误差和小于阈值， 停止迭代
    double trans_formation_epsilon = 1e-10;  //两次变化矩阵之间的差值（一般设置为1e-10即可）
    double max_correspondence_dis = 100.0;    //最大对应点距离

    std::string filtered_frame_id = "filtered";            //滤波点云id
    std::string raw_frame_id = "raw";        //原始点云id
    std::string odom_frame_id = "odom";
} param;


//本地静态函数
static void initParams(ros::NodeHandle &nh);                                                                   //参数初始化
static void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg_in);                               //点云回调函数
static Eigen::Matrix4f matching(const ros::Time &stamp, const pcl::PointCloud<PointT>::ConstPtr &cloud_in);    //获取位姿
static void publishOdometry(const ros::Time &stamp, const Eigen::Matrix4f &pose);                              //发布里程计
static geometry_msgs::TransformStamped
matrix2transform(const ros::Time &stamp, const Eigen::Matrix4f &pose, const std::string &frame_id,
                 const std::string &child_frame_id);                                                           //转换消息格式

/**
 * @brief main function
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "getOdom");
    ros::NodeHandle nh;

    initParams(nh);

    points_sub = nh.subscribe("/filtered_points", 5, &cloudCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

    ros::spin();
    return 0;
}

/**
 * @brief 初始化参数
 * @param nh 节点句柄
 */
static void initParams(ros::NodeHandle &nh) {
    param.max_iterations = nh.param<int>("maxIterations", 500);                           //最大迭代次数
    param.euclidean_fitness_epsilon = nh.param<double>("euclideanFitnessEpsilon", 0.001); //收敛条件是均方误差和小于阈值， 停止迭代
    param.trans_formation_epsilon = nh.param<double>("transFormationEpsilon", 1e-10);     //两次变化矩阵之间的差值（一般设置为1e-10即可）
    param.max_correspondence_dis = nh.param<double>("maxCorrespondenceDis", 100.0);       //最大对应点距离
}


/**
 * @brief 点云回调函数
 * @param cloud_msg_in 点云信息
 */
static void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg_in) {
    if (!ros::ok()) {
        return;
    }
    ROS_INFO("Get Odom cloudCallback.");
    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>); //申请点云空间
    pcl::fromROSMsg(*cloud_msg_in, *cloud_in); //转换消息格式
    Eigen::Matrix4f pose = matching(cloud_msg_in->header.stamp, cloud_in);
    publishOdometry(cloud_msg_in->header.stamp, pose);
}

/**
 * @brief 计算位姿变化
 * @param stamp 输入点云时间戳
 * @param cloud_in 输入点云指针
 * @return odom 全局位姿变换
 */
static Eigen::Matrix4f matching(const ros::Time &stamp, const pcl::PointCloud<PointT>::ConstPtr &cloud_in) {
    Eigen::Matrix4f pose_trans;
    Eigen::Matrix4f pose_global;
    pose_trans.setIdentity();
    pose_global.setIdentity();

    if (param.keyframe == nullptr) {
        param.keyframe = cloud_in;
        param.keyframe_pose.setIdentity();
        param.keyframe_stamp = stamp;
        return pose_global;
    }

    pcl::PointCloud<PointT> final;

    param.icp.setInputSource(cloud_in);
    param.icp.setInputTarget(param.keyframe);
    param.icp.setMaximumIterations(param.max_iterations);
    param.icp.setEuclideanFitnessEpsilon(param.euclidean_fitness_epsilon);
    param.icp.setTransformationEpsilon(param.trans_formation_epsilon);
    param.icp.setMaxCorrespondenceDistance(param.max_correspondence_dis);
    param.icp.align(final);
    pose_trans = param.icp.getFinalTransformation();

    pose_global = param.keyframe_pose * pose_trans;
    param.keyframe = cloud_in;
    param.keyframe_pose = pose_global;
    param.keyframe_stamp = stamp;
    return pose_global;
}


/**
 * @brief 发布里程计
 * @param stamp time stamp
 * @param base_frame_id
 * @param pose odometry pose to be published
 */
static void publishOdometry(const ros::Time &stamp, const Eigen::Matrix4f &pose) {
    static tf2_ros::TransformBroadcaster odom_broadcaster;
    // broadcast the transform over tf
    geometry_msgs::TransformStamped filtered_trans = matrix2transform(stamp, pose, param.odom_frame_id,
                                                                      param.filtered_frame_id);   //odom相对于filtered points
    geometry_msgs::TransformStamped raw_points_trans = matrix2transform(stamp, pose, param.odom_frame_id,
                                                                        param.raw_frame_id);   //odom相对于raw_points
    odom_broadcaster.sendTransform(filtered_trans);      //发布到TF tree, odom相对于filtered
    odom_broadcaster.sendTransform(raw_points_trans);    //发布到TF tree, odom相对于raw_points

    // publish the transform
    nav_msgs::Odometry odom;                         //里程计信息
    odom.header.stamp = stamp;                       //设置时间戳
    odom.header.frame_id = param.odom_frame_id;      //节点id odom

    odom.pose.pose.position.x = pose(0, 3);//
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = filtered_trans.transform.rotation;

    odom.child_frame_id = param.filtered_frame_id;   //filtered
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub.publish(odom);
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
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
}
