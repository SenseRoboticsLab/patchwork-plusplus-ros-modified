#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include "patchworkpp/patchworkpp.hpp"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <signal.h>

struct PointXYZILID_patchwork {
    PCL_ADD_POINT4D; // quad-word XYZ
    float intensity; ///< laser intensity reading
    uint16_t label;  ///< point label
    uint16_t id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILID_patchwork,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, label,
                                                                                                       label)(uint16_t,
                                                                                                              id, id))
using PointType = PointXYZILID_patchwork;

using namespace std;

boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

ros::Publisher pub_cloud;
ros::Publisher pub_ground;
ros::Publisher pub_non_ground;

template <typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, const ros::Time &stamp, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.stamp = stamp;
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg) {
    double time_taken;

    pcl::PointCloud<PointType> cloud_in;
    // pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;

    pcl::fromROSMsg(*cloud_msg, cloud_in);
    // for (auto &point : cloud_in.points) {
    // PointType pt;
    // pt.x = point.x;
    // pt.y = point.y;
    // pt.z = point.z;
    // point.intensity = 0;
    // pt.label = point.label;
    // pc_curr.points.push_back(pt);
    // }

    PatchworkppGroundSeg->estimate_ground(cloud_in, pc_ground, pc_non_ground, time_taken);

    ROS_INFO_STREAM("\033[1;32m"
                    <<"Label: " << cloud_in[0].label << " Input PointCloud: " << cloud_in.size() << " -> Ground: " << pc_ground.size()
                    << "/ NonGround: " << pc_non_ground.size() << " (running_time: " << time_taken << " sec)"
                    << "\033[0m");

    pub_cloud.publish(cloud2msg(cloud_in, cloud_msg->header.stamp ));
    pub_ground.publish(cloud2msg(pc_ground, cloud_msg->header.stamp));
    pub_non_ground.publish(cloud2msg(pc_non_ground, cloud_msg->header.stamp));
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "Demo");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string cloud_topic;
    pnh.param<string>("cloud_topic", cloud_topic, "/pointcloud");

    cout << "Operating patchwork++..." << endl;
    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&pnh));

    pub_cloud = pnh.advertise<sensor_msgs::PointCloud2>("cloud", 10, true);
    pub_ground = pnh.advertise<sensor_msgs::PointCloud2>("/patchworkpp/ground_pc", 10, true);
    pub_non_ground = pnh.advertise<sensor_msgs::PointCloud2>("/patchworkpp/nonground_pc", 10, true);

    ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 1000, callbackCloud);

    ros::spin();

    return 0;
}
