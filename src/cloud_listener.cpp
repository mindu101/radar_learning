// 第一课：在这里亲手输入 LEARNING.md 中的程序，然后保存。
// 目前只有注释，尚无 main 函数；填写前编译失败是正常的。
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    static unsigned int frame_count=0;
    ++frame_count;
    unsigned int point_count=msg->width * msg->height;

    ROS_INFO_THROTTLE(2.0, "frames=%u Point=%u frame%s w=%u h=%u",frame_count, point_count, msg->header.frame_id.c_str(), msg->width, msg->height );

}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, 
    "radar_listener");
    ros::NodeHandle nh;
    ros::Subscriber subscriber=nh.subscribe("/ws_points_raw", 1, cloudCallback);
    ros::spin();
    return 0;
}