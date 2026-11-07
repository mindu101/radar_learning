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
double marker_lifetime;
ros::Publisher cent_pub;

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    const ros::WallTime t0 = ros::WallTime::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input);
    std::vector<int> valid_index;
    pcl::removeNaNFromPointCloud(*input, *input, valid_index);
    const ros::WallTime t1 = ros::WallTime::now();
    if (input->empty())
    {
        ROS_INFO_THROTTLE(1.0, "points=0 clusters=0 total=%.1f ms",
                          (t1 - t0).toSec() * 1000.0);
        return;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
    extractor.setInputCloud(input);
    extractor.setSearchMethod(tree);
    extractor.setClusterTolerance(tolerance);
    extractor.setMinClusterSize(min_cluster_size);
    extractor.setMaxClusterSize(100000);
    std::vector<pcl::PointIndices> groups;
    const ros::WallTime t2 = ros::WallTime::now();
    extractor.extract(groups);
    const ros::WallTime t3 = ros::WallTime::now();
    if (groups.empty())
    {
        ROS_INFO_THROTTLE(1.0,
            "points=%zu clusters=0 convert=%.1f prepare=%.1f cluster=%.1f markers=0 total=%.1f ms",
            input->size(), (t1 - t0).toSec() * 1000.0,
            (t2 - t1).toSec() * 1000.0, (t3 - t2).toSec() * 1000.0,
            (t3 - t0).toSec() * 1000.0);
        return;
    }

    // 每帧复用显示编号；编号不代表跨帧跟踪身份。
    int mark_id = 0;
    for (const auto& largest : groups)
    {
        const auto& first = input->points[largest.indices[0]];
        float xmin = first.x, xmax = first.x;
        float ymin = first.y, ymax = first.y;
        float zmin = first.z, zmax = first.z;
        for (int index : largest.indices)
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
        marker.header = msg->header;
        marker.ns = "largest_cluster";
        marker.id = mark_id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = (xmin + xmax) / 2.0;
        marker.pose.position.y = (ymin + ymax) / 2.0;
        marker.pose.position.z = (zmin + zmax) / 2.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = std::max(xmax - xmin, 0.02f);
        marker.scale.y = std::max(ymax - ymin, 0.02f);
        marker.scale.z = std::max(zmax - zmin, 0.02f);
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.25;
        marker.lifetime = ros::Duration(marker_lifetime);
        cent_pub.publish(marker);
        ROS_INFO_THROTTLE(1.0, "center: %.2f, %.2f, %.2f",
                          center[0], center[1], center[2]);
    }

    const ros::WallTime t4 = ros::WallTime::now();
    ROS_INFO_THROTTLE(1.0,
        "points=%zu clusters=%zu convert=%.1f prepare=%.1f cluster=%.1f markers=%.1f total=%.1f ms",
        input->size(), groups.size(),
        (t1 - t0).toSec() * 1000.0, (t2 - t1).toSec() * 1000.0,
        (t3 - t2).toSec() * 1000.0, (t4 - t3).toSec() * 1000.0,
        (t4 - t0).toSec() * 1000.0);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "radar_cluster");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param<double>("tol", tolerance, 0.06);
    pnh.param<int>("min_cluster_size", min_cluster_size, 10);
    pnh.param<double>("marker_lifetime", marker_lifetime, 2.0);
    if (!std::isfinite(tolerance) || tolerance <= 0.0 ||
        min_cluster_size < 1 || min_cluster_size > 100000 ||
        !std::isfinite(marker_lifetime) || marker_lifetime <= 0.0)
    {
        ROS_ERROR("tol and marker_lifetime must be finite and > 0; min_cluster_size must be 1..100000");
        return 1;
    }
    ROS_INFO("cluster parameters: tol=%.3f min_cluster_size=%d marker_lifetime=%.2f",
             tolerance, min_cluster_size, marker_lifetime);
    // 一帧可连发多个框，避免发送队列只有一条时丢掉尚未发送的框。
    cent_pub = nh.advertise<visualization_msgs::Marker>("learning/cluster_center", 100);
    // ros::Subscriber sub=nh.subscribe("/ws_points_raw", 1, cloudCallback);
    // ros::Subscriber sub=nh.subscribe("/learning/downsampled", 1, cloudCallback);
    ros::Subscriber sub=nh.subscribe("/learning/foreground", 1, cloudCallback);
    ros::spin();
    return 0;
}
