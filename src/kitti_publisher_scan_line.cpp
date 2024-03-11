//
// Created by Hyungtae Lim on 6/23/21.
//

// For disable PCL complile lib, to use PointXYZILID
#define PCL_NO_PRECOMPILE
#include "tools/kitti_loader.hpp"
#include "patchworkpp/utils.hpp"
#include <chrono>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>

struct PointXYZILID_patchwork {
  PCL_ADD_POINT4D; // quad-word XYZ
  float intensity; ///< laser intensity reading
  uint16_t label;  ///< point label
  uint16_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZILID_patchwork,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, label,
                                                       label)(uint16_t, id, id))

using PointType = PointXYZILID_patchwork;
using namespace std;

ros::Publisher NodePublisher;

string data_dir;
string seq;

void callbackSignalHandler(int signum) {
  cout << "Caught Ctrl + c " << endl;
  // Terminate program
  exit(signum);
}

template <typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud,
                                   std::string frame_id = "map") {
  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Ros_Kitti_Publisher");
  int kitti_hz;
  int scan_line;
  ros::NodeHandle nh;
  std::string node_topic;
  nh.param<string>("/node_topic", node_topic, "/kitti_cloud");
  nh.param<string>("/data_dir", data_dir,
                   "/mnt/gpuServerFolder/media/pro/Seagate Portable "
                   "Drive/DataSets/kitti/data_odometry_velodyne/sequences");
  nh.param<string>("/seq", seq, "00");
  nh.param<int>("/kitti_hz", kitti_hz, 10);
  nh.param<int>("/scan_line", scan_line, 10);

  cout << "\033[1;32m"
       << "Node topic: " << node_topic << "\033[0m" << endl;
  cout << "\033[1;32m"
       << "KITTI data directory: " << data_dir << "\033[0m" << endl;
  cout << "\033[1;32m"
       << "Sequence: " << seq << "\033[0m" << endl;

  ros::Rate r(kitti_hz);
  ros::Publisher NodePublisher =
      nh.advertise<sensor_msgs::PointCloud2>(node_topic, 100, true);

  std::string data_path = data_dir + "/" + seq;
  KittiLoader loader(data_path);
  int N = loader.size();

  signal(SIGINT, callbackSignalHandler);
  cout << "\033[1;32m[Kitti Publisher] Total " << N
       << " clouds are loaded\033[0m" << endl;
  std::set<int> valid_id;
  int step = std::round(double(64) / double(scan_line));
  for (int i = 0; i < 64; ++i) {
    if (i % step == 0) {
      valid_id.insert(i);
    }
  }
  std::this_thread::sleep_for(std::chrono::seconds(5));
  for (int n = 0; n < N; ++n) {
    //  int n = 0;
    //  while (true) {
    cout << n << "th node is published!" << endl;

    pcl::PointCloud<pcl::PointXYZI> pc_curr;

    pcl::PointCloud<pcl::PointXYZI> pc_original;
    if (scan_line != 64) {
      pc_original = *loader.cloud(n);
      for (const auto &pt : pc_original.points) {
        double hypotxy = (pt.x) * (pt.x) + (pt.y) * (pt.y);
        double elev = atan2(pt.z, hypotxy);
        int rowIdn = round((64 - 1) * (1 - (elev - (-24.8 * M_1_PI / 180)) /
                                               (26.8 * M_1_PI / 180)));
        if (valid_id.find(rowIdn) != valid_id.end()) {
          pc_curr.points.push_back(pt);
        }
      }
    } else {
      pc_curr = *loader.cloud(n);
    }

    std::cout << "Complete load!" << std::endl;
    pcl::PointCloud<PointType> pc_label;
    for (const auto &pt : pc_curr.points) {
      PointType pt_new;
      pt_new.x = pt.x;
      pt_new.y = pt.y;
      pt_new.z = pt.z;
      pt_new.intensity = pt.intensity;
      pt_new.label = n;
      pc_label.push_back(pt_new);
    }
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(pc_label, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";
    NodePublisher.publish(cloud_msg);
    r.sleep();
    //    if (n == 0) {
    //      n = 1;
    //    }
    //    else if (n == 1) {
    //      n = 0;
    //    }
  }
  return 0;
}