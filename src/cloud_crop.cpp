// 第三课：请按 LEARNING_03.md 手写筛选程序。
// 写完后再按教程更新 package.xml 和 CMakeLists.txt。
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
ros::Publisher output_pub;
std::string filter_axis;
double min_;
double max_;
void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{   
    pcl::PCLPointCloud2::Ptr
    input(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*msg, *input);

    pcl::PassThrough<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(input);
    filter.setFilterFieldName(filter_axis);
    filter.setFilterLimits(min_, max_);

    pcl:: PCLPointCloud2 filtered;
    filter.filter(filtered);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(filtered, output);
    output.header= msg->header;
    // output_pub.publish(output);
    output_pub.publish(output);
    ROS_INFO_THROTTLE(1.0,
        "input=%u, output=%u",
        msg->width*msg->height,
        output.width*output.height
    );
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_crop");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    pnh.param<std::string>("axis", filter_axis, "z");
    pnh.param<double>("min", min_, 0.1);
    pnh.param<double>("max", max_, 2.0);

    if (filter_axis !="x"&&filter_axis !="y"&&filter_axis !="z")
    {
        ROS_ERROR("err");
        return 1;
    }

    output_pub=nh.advertise<sensor_msgs::PointCloud2>("/learning/XPoint",1);
    ros::Subscriber Sub= nh.subscribe("/ws_points_raw", 1, CloudCallback);
    ros::spin();
    return 0;

}