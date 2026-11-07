// 第二课：请按 LEARNING_02.md 手写点云转发程序。
// 完成后，再按教程把 cloud_relay 目标加入 CMakeLists.txt。
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher output_pub;
void cloudeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    output_pub.publish(msg);
    ROS_INFO_THROTTLE(1.0, "Points = %u, frame=%s", msg->width*msg->height, msg->header.frame_id.c_str());

}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "radar_rely");
    ros::NodeHandle nh;
    output_pub=nh.advertise<sensor_msgs::PointCloud2>("newname",1);
    ros::Subscriber subsc=nh.subscribe("/ws_points_raw", 1, cloudeCallback);
    ros::spin();
    return 0;
}