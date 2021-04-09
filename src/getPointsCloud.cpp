/**
 * @brief 从Bag包中读取点云信息，并通过话题/new_points发布
 * @author doudou
 * @version 1.0
 * @date 2020.04.04
 */


#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> //传感器消息类型

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//订阅者与发布者
ros::Subscriber points_sub;
ros::Publisher points_pub;


//重定义
typedef pcl::PointXYZI PointT;


//回调函数
static void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg_in);
static void initParams(ros::NodeHandle &nh);


//参数
static struct localParam{
    std::string points_topic_sub;
    std::string points_topic_pub;
    int loop_rate_set;
}param;


/**
 * @brief main function
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "getCloud"); //节点初始化
    ros::NodeHandle nh; //节点句柄
    initParams(nh); //节点初始化
    points_sub = nh.subscribe<sensor_msgs::PointCloud2>(param.points_topic_sub, 5, &cloudCallback);//订阅者与发布者
    points_pub = nh.advertise<sensor_msgs::PointCloud2>(param.points_topic_pub, 5);
    ros::Rate loop_rate(param.loop_rate_set); //主循环
    while (ros::ok()) {
        loop_rate.sleep(); //休眠
        ros::spinOnce();
    }
    return 0;
}


/**
 * @brief 初始化参数
 * @param nh 节点句柄
 */
static void initParams(ros::NodeHandle &nh) {
    param.points_topic_sub = nh.param<std::string>("pointsCloudInTopic", "/velodyne_points");
    param.points_topic_pub = nh.param<std::string>("pointsCloudOutTopic", "/raw_points_cloud");
    param.loop_rate_set = nh.param<int>("loopRate", 10);  //
}


/**
 * @brief 点云回调函数
 * @param cloud_msg_in 点云数据指针
 */
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg_in) {
    if (!ros::ok()) { //等待ros准备完毕
        return;
    }

    static int cloud_id = 1;
    ROS_INFO("cloudCallback, frame id = %d.\n", cloud_id);
    if (cloud_id <= 1000)
        cloud_id++;
    else
        cloud_id = 1;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>); //申请点云存储空间,点云智能指针，自动释放
    pcl::fromROSMsg(*cloud_msg_in, *cloud); //转换数据类型，ros下为xml

    sensor_msgs::PointCloud2 cloud_msg_out; //传感器数据类型，点云
    pcl::toROSMsg(*cloud, cloud_msg_out); //从C++数据类型转换为ros下数据类型
    cloud_msg_out.header.stamp = cloud_msg_in->header.stamp;
    cloud_msg_out.header.frame_id = "raw_points";
    points_pub.publish(cloud_msg_out);
}