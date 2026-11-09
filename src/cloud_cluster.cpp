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
#include <cstddef>
#include <cstdint>

double tolerance;
int min_cluster_size;
double marker_lifetime;
bool enable_size_filter;
bool print_features;
double min_dx, max_dx;
double min_dy, max_dy;
double min_dz, max_dz;
ros::Publisher cent_pub;
struct Candidate
{
    pcl::PointXYZ position;
    double dx;
    double dy;
    double dz;
};

struct Track
{
    int id;
    pcl::PointXYZ position;//当前状态位置，实际的，滤波后的结果
    pcl::PointXYZ prediction;//预测位置，滤波前预测
    pcl::PointXYZ measurement;//最近一次的候选位置
    double vx;
    double vy;
    double vz;
    double dx;
    double dy;
    double dz;
    int missed_frames;
    int age_frames;
    ros::Time state_stamp;//this time;
    ros::Time last_seen;// former time
};


std::vector<Track> tracks;
int next_track_id=0;
double association_distance;
int max_missed_frames;
double filter_alpha;
double filter_beta;
double lost_timeout;//目标丢失多少秒后删除
double max_predict_gap;//每一帧间隔多大后重置滤波器
ros::Time previous_frame_stamp;
std::string tracking_frame;


void publishTrackMarker(const Track& track, const sensor_msgs::PointCloud2 & source, uint8_t action)
{
    visualization_msgs::Marker marker;
    marker.header = source.header;
    marker.ns = "radar_track";
    marker.id = track.id;
    marker.action = action;

    if(action == visualization_msgs::Marker::DELETE)
    {
        cent_pub.publish(marker);
        return ;
    }

    marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = track.position.x;
    marker.pose.position.y = track.position.y;
    marker.pose.position.z = track.position.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(track.dx, 0.02);
    marker.scale.y = std::max(track.dy, 0.02);
    marker.scale.z = std::max(track.dz, 0.02);
    const bool predict = track.missed_frames>0;
    marker.color.r = predict?1.0:0.1;
    marker.color.g = predict?0.5:0.4;
    marker.color.b = predict?0.0:1.0;
    marker.color.a = 0.35;
    marker.lifetime = ros::Duration(marker_lifetime);
    cent_pub.publish(marker);
}

// void updateTracks(const std::vector<Candidate> & candidates, const sensor_msgs::PointCloud2 &source )
// {
//     const double limit2 = association_distance * association_distance;
//     std::vector<bool> used(candidates.size(), false);

//     for (auto& track : tracks)
//     {
//         int best_index = -1;
//         double best_distance = limit2;
//         for (std::size_t i =0;i<candidates.size();i++)
//         {
//             if(used[i]==true)continue;
            
//             const double dx= static_cast<double>(track.position.x) - candidates[i].position.x;
//             const double dy= static_cast<double>(track.position.y) - candidates[i].position.y;
//             const double dz= static_cast<double>(track.position.z) - candidates[i].position.z;
//             const double distance =dx*dx +dy*dy + dz*dz;
//             if(distance<=best_distance)
//             {
//                 best_distance=distance;
//                 best_index  =static_cast<int>(i);
//             } 

//         }
//         if(best_index>=0)
//         {
//             const Candidate& candidate = candidates[best_index];
//             track.position = candidate.position;
//             track.dx = candidate.dx;
//             track.dy = candidate.dy;
//             track.dz = candidate.dz;
//             track.missed_frames=0;
//             ++track.age_frames;
//             used[best_index]=1;
//         }
//         else
//         {
//             ++track.missed_frames;
//         }
//     }
    


//     std::vector<int> remove_ids;
//     tracks.erase(std::remove_if(
//         tracks.begin(), 
//         tracks.end(),
//         [&remove_ids](const Track& track ){
//         if (track.missed_frames>max_missed_frames)
//         {
//             remove_ids.push_back(track.id);
//             return true;
//         }
//         return false;
//     }),
//     tracks.end());

//     for (int id : remove_ids)
//     {
//         Track removed{};
//         removed.id = id;
//         publishTrackMarker(removed, source, visualization_msgs::Marker::DELETE);
//     }

//     for(std::size_t i=0; i< candidates.size(); i++)
//     {
//         if(used[i])
//         {
//             continue;
//         }
//         const Candidate& candidate = candidates[i];
//         tracks.push_back(
//             Track{
//                 next_track_id++,
//                 candidate.position,
//                 candidate.dx,
//                 candidate.dy,
//                 candidate.dz,
//                 0,1}
//         );
//     }

//     for (const auto& track : tracks)
//     {
//         publishTrackMarker(track, source, visualization_msgs::Marker::ADD);

//     }

// }

void updateTracks(const std::vector<Candidate> & candidates, const sensor_msgs::PointCloud2 & source)
{
    const ros::Time stamp = source.header.stamp;
    if(stamp.is_zero())
    {
        ROS_WARN_THROTTLE(1.0,"stamp");
        return ;
    }

    if(!previous_frame_stamp.is_zero())
    {
        const double frame_dt=(stamp - previous_frame_stamp).toSec();

        if(frame_dt<0.001)
        {
             ROS_WARN_THROTTLE(1.0,"dt is lower than 0.001");
            return ;
        }
        if(source.header.frame_id!=tracking_frame||frame_dt>max_predict_gap){
            ROS_WARN_THROTTLE(1.0,"axis is wrong or break in radar");
            // return ;
        }

    }
    previous_frame_stamp = stamp;
    tracking_frame= source.header.frame_id;
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(), 
    [&](const Track & track){if((stamp-track.last_seen).toSec()> lost_timeout){publishTrackMarker(track , source, visualization_msgs::Marker::DELETE);
        return true;}
        return false;}), tracks.end());

    const double limit = association_distance*association_distance;

    std::vector<bool> used(candidates.size(),false);

    for (auto& track: tracks)
        {
            const double dt = (stamp-track.state_stamp).toSec();
            const double px = track.position.x + track.vx * dt;
            const double py = track.position.y + track.vy * dt;
            const double pz = track.position.z + track.vz * dt;
            track.prediction = pcl::PointXYZ(px, py, pz);
            track.position = track.prediction;

            int best_index = -1;
            double best_distance = limit;
            for(std::size_t i=0; i<candidates.size(); ++i)
            {
                if(used[i])continue;

                const double ex = candidates[i].position.x - px;
                const double ey = candidates[i].position.y - py;
                const double ez = candidates[i].position.z - pz;
                const double distance = ex*ex + ey*ey +ez*ez;

                if(distance <= best_distance)
                {
                    best_distance= distance;
                    best_index = static_cast<int>(i);
                }
            }

            if(best_index>=0)
            {
                const Candidate & candidate = candidates[best_index];
                const double rx = candidate.position.x - px;
                const double ry = candidate.position.y - py;
                const double rz = candidate.position.z - pz;

                track.position = pcl::PointXYZ(px + filter_alpha* rx, py + filter_alpha * ry, pz + filter_alpha * rz);
                track.vx += filter_beta * rx/dt;
                track.vy += filter_beta * ry/dt;
                track.vz += filter_beta * rz/dt;
                track.measurement = candidate.position;
                track.dx = candidate.dx;
                track.dy = candidate.dy;
                track.dz = candidate.dz;
                track.last_seen = stamp;
                track.missed_frames =0;
                ++track.age_frames;
                used[best_index]=true;
            }else
            {
                ++track.missed_frames;
            }
            track.state_stamp=stamp;
        }
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
    [&](const Track & track)
    {
        if (track.missed_frames>max_missed_frames)
        {
            publishTrackMarker(track,source, visualization_msgs::Marker::DELETE);
            return true;
        }
        return false;
    }), tracks.end());

    for(std::size_t i=0; i<candidates.size();i++)
    {
        if(used[i])continue;

        const Candidate& candidate = candidates[i];
        Track track{};
        track.id = next_track_id++;
        track.position = candidate.position;
        track.prediction = candidate.position;
        track.measurement = candidate.position;
        track.vx  =  track.vy =track.vz=0.0;
        track.dx = candidate.dx;
        track.dy = candidate.dy;
        track.dz = candidate.dz;
        track.missed_frames=0;
        track.age_frames=1;
        track.state_stamp=stamp;
        track.last_seen=stamp;
        tracks.push_back(track);
    }
    for(const auto & track: tracks)
    {
        publishTrackMarker(track, source, visualization_msgs::Marker::ADD);
    }
}




void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::vector<Candidate> candidates;
    const ros::WallTime t0 = ros::WallTime::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input);
    if(input->empty())
    {
        updateTracks({}, *msg);
        return ;
    }
    std::vector<int> valid_index;
    input->is_dense = false;
    pcl::removeNaNFromPointCloud(*input, *input, valid_index);
    const ros::WallTime t1 = ros::WallTime::now();
    if (input->empty())
    {
        updateTracks({}, *msg);
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
        updateTracks({}, *msg);
        ROS_INFO_THROTTLE(1.0,
            "points=%zu clusters=0 convert=%.1f prepare=%.1f cluster=%.1f markers=0 total=%.1f ms",
            input->size(), (t1 - t0).toSec() * 1000.0,
            (t2 - t1).toSec() * 1000.0, (t3 - t2).toSec() * 1000.0,
            (t3 - t0).toSec() * 1000.0);
        return;
    }

    // 每帧复用显示编号；编号不代表跨帧跟踪身份。
    int mark_id = 0;
    std::size_t accepted_count=0;
    std::size_t reject_count=0;
    std::ostringstream details;
    if(print_features)
        details<<std::fixed<<std::setprecision(3);
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
        const std::size_t n = largest.indices.size();
        const double dx = static_cast<double>(xmax)-xmin;
        const double dy = static_cast<double>(ymax)-ymin;
        const double dz = static_cast<double>(zmax)-zmin;
        const double bx = (static_cast<double>(xmin)+xmax)/2.0;
        const double by = (static_cast<double>(ymin)+ymax)/2.0;
        const double bz = (static_cast<double>(zmin)+zmax)/2.0;


        Eigen::Vector4f center;
        pcl::compute3DCentroid(*input, largest.indices, center);
        const bool dx_ok = dx >= min_dx && dx <= max_dx;
        const bool dy_ok = dy >= min_dy && dy <= max_dy;
        const bool dz_ok = dz >= min_dz && dz <= max_dz;
        const bool accepted = !enable_size_filter || (dx_ok && dy_ok && dz_ok);
        if(accepted)
        {
            ++accepted_count;
            candidates.push_back(Candidate{
                pcl::PointXYZ(
                    static_cast<float>(bx),
                    static_cast<float>(by),
                    static_cast<float>(bz)),
                    dx,dy,dz
                
            });
        }else
        {
            ++reject_count;
        }
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
        marker.color.r = accepted ? 0.0 : 1.0;
        marker.color.g = accepted ? 1.0 : 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.25;
        marker.lifetime = ros::Duration(marker_lifetime);
        cent_pub.publish(marker);
        ROS_INFO_THROTTLE(1.0, "center: %.2f, %.2f, %.2f",
                          center[0], center[1], center[2]);
    }
    updateTracks(candidates, *msg);
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
    pnh.param<bool>("enable_size_filter", enable_size_filter, true);
    pnh.param<bool>("print_features", print_features, false);
    pnh.param<double>("min_dx", min_dx, 0.1);
    pnh.param<double>("min_dy", min_dy, 0.1);
    pnh.param<double>("min_dz", min_dz, 1.0);
    pnh.param<double>("max_dx", max_dx, 1.0);
    pnh.param<double>("max_dy", max_dy, 1.0);
    pnh.param<double>("max_dz", max_dz, 4.0);
    pnh.param<double>("association_distance", association_distance, 0.5);
    pnh.param<int>("max_missed_frames",max_missed_frames, 4);
    pnh.param<double>("filter_alpha", filter_alpha, 0.60);
    pnh.param<double>("filter_beta", filter_beta, 0.15);
    pnh.param<double>("lost_timeout", lost_timeout, 0.6);
    pnh.param<double>("max_predict_gap", max_predict_gap, 1.0);
    
    const auto valid_range = [] (double lower, double upper)
    {
        return std::isfinite(lower)&&std::isfinite(upper)&&lower>=0.0&&upper>=lower;
    };
    if(!valid_range(min_dx,max_dx)||!valid_range(min_dy,max_dy)||!valid_range(min_dz,max_dz))
    {
        ROS_ERROR("invalid_range");
        return 1;
    }
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
