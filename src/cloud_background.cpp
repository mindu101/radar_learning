#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <string>
#include <vector>
ros::Publisher background_pub;
ros::Publisher fore_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr background(new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ> back_tree;
int back_frames;
int received_frames=0;
double background_leaf;
double background_distance;
bool background_ready = false;
std::string frame;

void publishCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const sensor_msgs::PointCloud2& source, const ros::Publisher& publisher)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud,output);
    output.header=source.header;
    publisher.publish(output);

}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr&msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*msg, *input);
    input->is_dense = false;
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*input, *input, valid_indices);

    pcl::PointCloud<pcl::PointXYZ> fore;
    fore.height=1;
    fore.is_dense=1;

    if(!background_ready)
    {
        if(!input->empty())
        {
            if(received_frames==0)
            {
                frame=(msg->header).frame_id;
            }
            *background+=*input;
            ++received_frames;

            if(received_frames>=back_frames)
            {
                pcl::VoxelGrid<pcl::PointXYZ> filter;
                filter.setInputCloud(background);
                filter.setLeafSize(background_leaf,background_leaf,background_leaf);
                pcl::PointCloud<pcl::PointXYZ> reduced;
                filter.filter(reduced);

                *background=reduced;
                back_tree.setInputCloud(background);
                background_ready=true;
                ROS_INFO_THROTTLE(1.0, "Start running");

            }
        }
    }
    else
    {
        std::vector<int> nearest_index(1);
        std::vector<float> nearest_distance2(1);

        const double threshold2=background_distance*background_distance;

        for(const auto& point: input->points)
        {
            const int fo=back_tree.nearestKSearch(point,1,nearest_index,nearest_distance2);
            if(fo==0||nearest_distance2[0]>threshold2)
            {
                fore.push_back(point);
            }
        }

    }

    publishCloud(*background, *msg, background_pub);
    publishCloud(fore, *msg, fore_pub);


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_background");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param<int>("background_frames",back_frames,20);
    pnh.param<double>("background_leaf", background_leaf, 0.03);
    pnh.param<double>("background_distance",background_distance, 0.08);

    background_pub=nh.advertise<sensor_msgs::PointCloud2>("/learning/background", 1);
    fore_pub=nh.advertise<sensor_msgs::PointCloud2>("/learning/foreground", 1);
    // ros::Subscriber sub=nh.subscribe("/learning/downsampled", 1, cloudCallback);
    ros::Subscriber sub=nh.subscribe("/ws_points_raw", 1, cloudCallback);
    ros::spin();
    return 0;
}