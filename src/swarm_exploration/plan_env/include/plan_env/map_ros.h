#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <memory>
#include <random>

using std::shared_ptr;
using std::normal_distribution;
using std::default_random_engine;

namespace fast_planner {
class SDFMap;

class MapROS {

public:
  MapROS();
  ~MapROS();
  void setMap(SDFMap* map);
  void init();

private:
  // Callbacks
  void updateESDFCallback(const ros::TimerEvent& e);
  void pubMapCallback(const ros::TimerEvent& e);
  void depthCallback(const sensor_msgs::ImageConstPtr& img, const geometry_msgs::PoseStampedConstPtr& pose);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, const geometry_msgs::PoseStampedConstPtr& pose);
  // void basecoorCallback(const swarm_msgs::swarm_drone_basecoorConstPtr& msg);

  // Helper
  void publishMapAll();
  void publishMapLocal();
  void publishESDF();
  void publishUpdateRange();
  void publishUnknown();

  void publishDepth();
  void processDepthImage();
  void publishCloud();


  SDFMap* map_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> SyncPolicyCloudPose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

  ros::NodeHandle nh_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerCloudPose sync_cloud_pose_;

  ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_;
  ros::Publisher unknown_pub_, update_range_pub_, depth_pub_, cloud_pub_;
  ros::Timer esdf_timer_, vis_timer_;

  // Params
  double cx_, cy_, fx_, fy_;
  double depth_filter_maxdist_, depth_filter_mindist_;
  double lidar_filter_maxdist_, lidar_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;
  string frame_id_;

  double esdf_slice_height_;
  double visualization_truncate_height_, visualization_truncate_low_, box_max_z_;
  bool show_esdf_time_, show_occ_time_;
  bool show_all_map_, show_local_map_;
  bool show_ground_;

  // Data
  bool local_updated_;
  bool esdf_need_update_;
  Eigen::Vector3d camera_pos_, lidar_pos_;
  Eigen::Quaterniond camera_q_;
  unique_ptr<cv::Mat> depth_image_;

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int fuse_num_, esdf_num_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;

  normal_distribution<double> rand_noise_;
  default_random_engine eng_;

  friend SDFMap;
};
}

#endif