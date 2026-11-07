#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <cstddef>
ros::Publisher output_pub;
double leaf_size;

void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr input(new 
    pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *input);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cleaned(new pcl::PointCloud<pcl::PointXYZI>);
    cleaned->header=input->header;
    cleaned->points.reserve(input->size());

    std::size_t invalid_count=0;
    std::size_t zero_count=0;

    for(const auto& point: input->points)
    {
        if(!std::isfinite(point.x)||!std::isfinite(point.y)||!std::isfinite(point.z)||!std::isfinite(point.z)||!std::isfinite(point.intensity))
        {
            ++invalid_count;
            continue;
        }

        if(point.x==0.0f&&point.z==0.0f&&point.y==0.0f)
        {
            ++zero_count;
            continue;
        }
        cleaned->push_back(point);
    }
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(cleaned);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZI> filtered;

    filter.filter(filtered);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(filtered, output);
    output.header=msg->header;
    output_pub.publish(output);
    ROS_INFO_THROTTLE(1.0, "output=%u", output.width*output.height);

}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"radar_voxel");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param<double>("leaf_size", leaf_size ,0.1);
    if(!(std::isfinite(leaf_size)))
    {
        ROS_ERROR("err");
        return 1;
    }
    output_pub=nh.advertise<sensor_msgs::PointCloud2>("/learning/downsampled", 1);
    ros::Subscriber sub= nh.subscribe("/learning/XPoint",1 , CloudCallback);
    ros::spin();
    return 0;
}