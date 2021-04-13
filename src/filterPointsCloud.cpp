/**
 * @brief 对点云滤波
 * @author doudou
 * @version 1.1
 * @date 2020.04.10
 */

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> //传感器消息类型

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


//订阅者与发布者
ros::Subscriber points_sub;
ros::Publisher points_pub;


//重定义数据类型
typedef pcl::PointXYZI PointT;


//变量
pcl::PassThrough<PointT> filter_pass;  //直通滤波器
pcl::VoxelGrid<PointT> filter_voxel_grid; //体素网格滤波器
pcl::StatisticalOutlierRemoval<PointT> filter_outlier_removal; //统计滤波器

static struct localParam{
    std::string filter_method;
    float z_limit_min = -0.2f;
    float z_limit_max = 100.0f;
    float leaf_size = 0.1f;
    int mean_k = 50;
    float std_mul = 1.0f;
    std::string filtered_frame_id = "filtered";
    bool debugSwitch = false;
}param;


//本地静态函数
static void initParams(ros::NodeHandle &nh);                                                                   //参数初始化
static void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg_in);                               //点云回调函数
static pcl::PointCloud<PointT>::ConstPtr passThroughFilter(const pcl::PointCloud<PointT>::ConstPtr &cloud_in); //滤波函数
static pcl::PointCloud<PointT>::ConstPtr voxelGridFilter(const pcl::PointCloud<PointT>::ConstPtr &cloud_in);   //滤波函数
static pcl::PointCloud<PointT>::ConstPtr outlierRemovalFilter(const pcl::PointCloud<PointT>::ConstPtr &cloud_in); //滤波器函数


/**
 * @brief main function
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "filterCloud"); //节点初始化
    ros::NodeHandle nh; // 节点句柄
    initParams(nh);
    points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/raw_points", 5, &cloudCallback); //设置订阅者与发布者
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 5);
    ros::spin();
    return 0;
}

/**
 * @brief 初始化参数
 * @param nh 节点句柄
 */
static void initParams(ros::NodeHandle &nh) {
    param.filter_method = nh.param<std::string>("filterMethod", "pass_through");
    //passThrough filter
    param.z_limit_min = nh.param<float>("zLimitMin", -0.2f);
    param.z_limit_max = nh.param<float>("zLimitMax", 100.0f);
    //voxelGrid filter
    param.leaf_size = nh.param<float>("leafSize", 0.1f);
    //outlierRemoval Filter
    param.mean_k = nh.param<int>("meanK", 50);
    param.std_mul = nh.param<float>("stdMul", 1.0f);
    //debugSwitch
    param.debugSwitch = nh.param<bool>("debugSwitch", false);
}


/**
 * @brief 点云回调函数
 * @param cloud_msg_in 传递进回调函数的点云
 */
static void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg_in) {
    if (!ros::ok())
        return;
    if(param.debugSwitch)
        ROS_INFO("Filter Points cloudCallback.");
    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>); // 申请点云空间 进入的点云
    pcl::PointCloud<PointT>::ConstPtr cloud_filtered;                      // 点云指针
    pcl::fromROSMsg(*cloud_msg_in, *cloud_in);                          // 转换数据类型

    // 点云滤波
    cloud_filtered = passThroughFilter(cloud_in);
    //cloud_filtered = voxelGridFilter(cloud_filtered);
    //cloud_filtered = outlierRemovalFilter(cloud_filtered);

    sensor_msgs::PointCloud2 cloud_msg_out;
    pcl::toROSMsg(*cloud_filtered, cloud_msg_out);
    cloud_msg_out.header.frame_id = param.filtered_frame_id;        // frame id
    cloud_msg_out.header.stamp = cloud_msg_in->header.stamp; // 时间戳
    points_pub.publish(cloud_msg_out);                       // 发送
}


/**
 * @brief 直通滤波
 * @param cloud_in 输入点云指针
 * @return 滤波后点云指针
 */
static pcl::PointCloud<PointT>::ConstPtr passThroughFilter(const pcl::PointCloud<PointT>::ConstPtr &cloud_in) {
    filter_pass.setInputCloud(cloud_in);
    filter_pass.setFilterFieldName("z");                    //直通滤波领域
    filter_pass.setFilterLimits(param.z_limit_min, param.z_limit_max); //直通滤波范围
    //pass.setFilterLimitsNegative(true);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>); //申请滤波后的点云存储空间
    filter_pass.filter(*cloud_filtered);

    return cloud_filtered;
}


/**
 * @brief 体素网格滤波
 * @param cloud_in 输入点云指针
 * @return 滤波后点云指针
 */
static pcl::PointCloud<PointT>::ConstPtr voxelGridFilter(const pcl::PointCloud<PointT>::ConstPtr &cloud_in) {
    filter_voxel_grid.setInputCloud(cloud_in);
    filter_voxel_grid.setLeafSize(param.leaf_size, param.leaf_size, param.leaf_size);
    filter_voxel_grid.setDownsampleAllData(true);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    filter_voxel_grid.filter(*cloud_filtered);

    return cloud_filtered;
}


/**
 * @brief 统计滤波器，去除点云的离群点。使用统计分析技术，从一个点云数据中集中移除测量噪声点。对每个点的邻域进行统计分析，剔除不符合一定标准的邻域点。
 * 1.对于每个点，计算它到所有相邻点的平均距离。假设得到的分布是高斯分布，我们可以计算出一个均值 μ 和一个标准差 σ；
 * 2.这个邻域点集中所有点与其邻域距离大于 μ + std_mul * σ 区间之外的点都可以被视为离群点，并可从点云数据中去除。std_mul 是标准差倍数的一个阈值，可以自己指定。
 * @param cloud_in 输入点云指针
 * @return 滤波后点云指针
 */
static pcl::PointCloud<PointT>::ConstPtr outlierRemovalFilter(const pcl::PointCloud<PointT>::ConstPtr &cloud_in) {
    filter_outlier_removal.setMeanK(param.mean_k);                   //设置在进行统计时考虑的临近点个数
    filter_outlier_removal.setStddevMulThresh(param.std_mul);  //设置std_mul
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    filter_outlier_removal.filter(*cloud_filtered);

    return cloud_filtered;
}