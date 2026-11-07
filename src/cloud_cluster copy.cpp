#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <algorithm>
double tolerance;
int min_cluster_size;
ros::Publisher cent_pub;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input);
    std::vector<int> valid_index;
    pcl::removeNaNFromPointCloud(*input, *input, valid_index);
    if(input->empty())
    {
        ROS_INFO_THROTTLE(1.0, "bug");
        return;
    }
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
    extractor.setInputCloud(input);
    extractor.setSearchMethod(tree);
    extractor.setClusterTolerance(tolerance);
    extractor.setMinClusterSize(min_cluster_size);
    extractor.setMaxClusterSize(3000);
    std::vector<pcl::PointIndices> groups;
    extractor.extract(groups);
    if(groups.empty())
    {
        ROS_INFO_THROTTLE(1.0, "no cluster");
        return ;
    }
    pcl::PointIndices largest{};
    for (const auto & group: groups)
    {
        if(group.indices.size() > largest.indices.size())
            largest=group;
    }
    const auto & first=input->points[largest.indices[0]];
    float xmin=first.x, xmax=first.x, ymin=first.y, ymax=first.y, zmin=first.z, zmax=first.z;
    for(int index:largest.indices)
    {
        const auto& point = input->points[index];
        xmin = std::min(xmin, point.x);
        xmax = std::max(xmax, point.x);
        ymin = std::min(ymin, point.y);
        ymax = std::max(ymax, point.y);
        zmin = std::min(zmin, point.z);
        zmax = std::max(zmax, point.z);
    }
    Eigen::Vector4f center;
    pcl::compute3DCentroid(*input, largest.indices, center);
    visualization_msgs::Marker marker;
    marker.header =msg->header;
    marker.ns="largest_cluster";
    marker.id=0;
    marker.type=visualization_msgs::Marker::CUBE;
    marker.action=visualization_msgs::Marker::ADD;
    
    marker.pose.position.x=(xmin + xmax) / 2.0;
    marker.pose.position.y=(ymin + ymax) / 2.0;
    marker.pose.position.z=(zmin + zmax) / 2.0;
    marker.pose.orientation.w = 1.0;

marker.scale.x = std::max(xmax - xmin, 0.02f);
marker.scale.y = std::max(ymax - ymin, 0.02f);
marker.scale.z = std::max(zmax - zmin, 0.02f);

    marker.color.r=0.0;
    marker.color.g=1.0;
    marker.color.b=0.0;
    marker.color.a=0.25;

    marker.lifetime = ros::Duration(0.5);
    cent_pub.publish(marker);
    ROS_INFO_THROTTLE(1.0, "center: %.2f, %.2f, %.2f", center[0],center[1],center[2]);



}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "radar_cluster");
    ros::NodeHandle nh;
    cent_pub=nh.advertise<visualization_msgs::Marker>("learning/cluster_center", 1);
    ros::NodeHandle pnh("~");
    pnh.param<double>("tol", tolerance, 0.3);
    pnh.param<int>("min_cluster_size", min_cluster_size, 10);
    // ros::Subscriber sub=nh.subscribe("/ws_points_raw", 1, cloudCallback);
    ros::Subscriber sub=nh.subscribe("/learning/downsampled", 1, cloudCallback);
    ros::spin();
    return 0;
}
