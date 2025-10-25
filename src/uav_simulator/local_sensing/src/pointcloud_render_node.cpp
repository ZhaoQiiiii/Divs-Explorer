#include <fstream>
#include <iostream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <Eigen/Eigen>
#include "eigen3/Eigen/Dense"

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include "backward.hpp"
namespace backward {
backward::SignalHandling sh;
}

using namespace cv;
using namespace std;
using namespace Eigen;
 
bool has_global_map = false;
bool has_odom = false;
bool use_depth = true;
double sensing_rate, estimation_rate;

pcl::PointCloud<pcl::PointXYZ> cloudIn;
nav_msgs::Odometry odom_;
Eigen::Vector3d last_pose_world;
ros::Time last_odom_stamp = ros::TIME_MAX;

double sensing_horizon;

// Depth Camera
int width, height;
double fx, fy, cx, cy;
Matrix4d cam2body, cam2world;
Eigen::Quaterniond cam2world_quat;

// Lidar
double hfov, vfov;
Matrix4d lidar2body, lidar2world;
Eigen::Quaterniond lidar2world_quat;
double lidar_height;

// Ros Utils
ros::Publisher pub_data, pub_pose;
ros::Subscriber odom_sub, global_map_sub;
ros::Timer local_sensing_timer, estimation_timer;



// 
// Callbacks 
//

// Sub "/map_generator/global_cloud"
void pointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map) return;
  ROS_WARN("Global Pointcloud received.");

  // Transform pointcloud map to PCL format
  pcl::fromROSMsg(pointcloud_map, cloudIn);
  ROS_WARN("Global Pointcloud Map has points: %d.\n", cloudIn.points.size());
  has_global_map = true;
}

// Sub "/state_ukf/odom"
void odometryCallbck(const nav_msgs::Odometry& odom) {
  // if(!has_global_map) return;

  has_odom = true;
  odom_ = odom;

  // Pose = Position + Oritention
  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_oritention;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;
  request_oritention.x() = odom.pose.pose.orientation.x;
  request_oritention.y() = odom.pose.pose.orientation.y;
  request_oritention.z() = odom.pose.pose.orientation.z;
  request_oritention.w() = odom.pose.pose.orientation.w;

  // Transform Matrix
  Matrix4d pose_receive = Matrix4d::Identity();
  pose_receive.block<3, 3>(0, 0) = request_oritention.toRotationMatrix();
  pose_receive(0, 3) = request_position(0);
  pose_receive(1, 3) = request_position(1);
  pose_receive(2, 3) = request_position(2);

  // Sensor2World = Body2World * Sensor2Body
  Matrix4d body_pose = pose_receive;
  if (use_depth) { 
    cam2world = body_pose * cam2body; 
    cam2world_quat = cam2world.block<3, 3>(0, 0); 
  }

  else { 
    lidar2world = body_pose * lidar2body; 
    lidar2world_quat = lidar2world.block<3, 3>(0, 0); 
  }

  last_odom_stamp = odom.header.stamp;
  last_pose_world(0) = odom.pose.pose.position.x;
  last_pose_world(1) = odom.pose.pose.position.y;
  last_pose_world(2) = odom.pose.pose.position.z;
}

// Timer 0.1s
void pubSensedPoints(const ros::TimerEvent& event) {
  if (!has_global_map) return;

  // 
  // Pub Depth Image (Depth Camera)
  // 

  if (use_depth) {

    // Get the Depth Matrix from Point Cloud
    cv::Mat depth_mat;
    depth_mat = cv::Mat::zeros(height, width, CV_32FC1);

    Eigen::Matrix4d Tcw = cam2world.inverse(); // World -> Camera
    Eigen::Matrix3d Rcw = Tcw.block<3, 3>(0, 0);
    Eigen::Vector3d tcw = Tcw.block<3, 1>(0, 3);
    Eigen::Vector3d pos = cam2world.block<3, 1>(0, 3); 

    for (const auto& pt : cloudIn.points) {
      Eigen::Vector3d pw(pt.x, pt.y, pt.z);
      if ((pw - pos).norm() > sensing_horizon) continue;
      
      Eigen::Vector3d pc = Rcw * pw + tcw;
      if (pc[2] <= 0.0) continue;

      float projected_x, projected_y;
      projected_x = pc[0] / pc[2] * fx + cx;
      projected_y = pc[1] / pc[2] * fy + cy;
      if (projected_x < 0 || projected_x >= width || projected_y < 0 || projected_y >= height) continue;

      float dist = pc[2];
      int r = 0.0573 * fx / dist + 0.5;
      int min_x = max(int(projected_x - r), 0);
      int max_x = min(int(projected_x + r), width - 1);
      int min_y = max(int(projected_y - r), 0);
      int max_y = min(int(projected_y + r), height - 1);

      for (int to_x = min_x; to_x <= max_x; to_x++) {
        for (int to_y = min_y; to_y <= max_y; to_y++) {
          float value = depth_mat.at<float>(to_y, to_x);
          if (value < 1e-3) depth_mat.at<float>(to_y, to_x) = dist;
          else depth_mat.at<float>(to_y, to_x) = min(value, dist);
        }
      }
    }

    // Pub the Depth Image from Depth Matrix
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = last_odom_stamp;
    out_msg.header.frame_id = "world";
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    out_msg.image = depth_mat.clone();
    pub_data.publish(out_msg.toImageMsg()); // Pub "/pcl_render_node/depth"
  }


  // 
  // Pub Point Cloud (Lidar)
  //
   
  else { 

    // Get the Local Cloud from cloud map
    Eigen::Vector3d pos = lidar2world.block<3, 1>(0, 3); 

    pcl::PointCloud<pcl::PointXYZ> local_cloud;
    for (auto pt : cloudIn.points) {
      Eigen::Vector3d pw(pt.x, pt.y, pt.z);
      if ((pw - pos).norm() > sensing_horizon) continue;
      local_cloud.push_back(pt);
    }
    // cout << "point_cloud1 = " << local_cloud.size() << endl; // 276261


    // Check the Fov and Occlusion
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::PointCloud<pcl::PointXYZ> extra_cloud; 
 
    struct RayInfo {
      double min_dist = sensing_horizon;
      pcl::PointXYZ closest_pt;
      bool save = false;
    };
 
    int h_rays = 720, v_rays = 128;  
    vector<RayInfo> ray_grid(h_rays * v_rays);

    double hfov_rad = M_PI * hfov / 180.0;
    double vfov_rad = M_PI * vfov / 180.0;

    for (auto pt : local_cloud.points) {
      Eigen::Vector3d pw(pt.x, pt.y, pt.z);
      Eigen::Vector3d relative = pw - pos;
      double dist = relative.norm();

      // Get the extra point (For upper frontier)
      if (pw[2] == 0 && dist < 0.5 * sensing_horizon) { 
        pcl::PointXYZ new_pt = pt;
        new_pt.z = 2.5; extra_cloud.push_back(new_pt); 
      }

      // Check the FoV
      double hangle = atan2(relative.y(), relative.x());
      double vangle = atan2(relative.z(), hypot(relative.x(), relative.y()));
      if (fabs(hangle) > hfov_rad / 2 || fabs(vangle) > vfov_rad / 2) continue;

      // Get the closest point 
      int h_index = static_cast<int>((hangle + M_PI) / (2 * M_PI) * h_rays);
      int v_index = static_cast<int>((vangle + vfov_rad / 2) / vfov_rad * v_rays);

      h_index = std::max(0, std::min(h_rays - 1, h_index));
      v_index = std::max(0, std::min(v_rays - 1, v_index));

      int ray_index = h_index * v_rays + v_index;
      if (dist < ray_grid[ray_index].min_dist) {
        ray_grid[ray_index].min_dist = dist;
        ray_grid[ray_index].closest_pt = pt;
        ray_grid[ray_index].save = true;
      }
    }

    for (auto ray : ray_grid) {
      if (!ray.save) continue;

      if (ray.min_dist < sensing_horizon) {
        point_cloud.push_back(ray.closest_pt);
      }

      else {
        pcl::PointXYZ pt = ray.closest_pt;
        Eigen::Vector3d pw(pt.x, pt.y, pt.z);
        pw = pos + (pw - pos).normalized() * 0.5 * sensing_horizon;
        pt.x = pw.x();
        pt.y = pw.y();
        pt.z = pw.z();
        point_cloud.push_back(pt);
      }
    }
    point_cloud += extra_cloud;
    // cout << "point_cloud2 = " << point_cloud.size() << endl;


    // Voxel Grid Filter (Downsampling)
    // pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    // voxel_filter.setInputCloud(point_cloud.makeShared());
    // voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); 
    // voxel_filter.filter(point_cloud);
    // cout << "point_cloud3 = " << point_cloud.size() << endl << endl;


    // Pub the Point Cloud
    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(point_cloud, out_msg);
    out_msg.header.stamp = last_odom_stamp;
    out_msg.header.frame_id = "world";
    pub_data.publish(out_msg); // Pub "/pcl_render_node/cloud"
  }
}

// Timer 0.1s (Pub "/pcl_render_node/sensor_pose")
void pubSensorPose(const ros::TimerEvent& event) {
  geometry_msgs::PoseStamped sensor_pose;

  if (use_depth) {
    sensor_pose.pose.position.x = cam2world(0, 3);
    sensor_pose.pose.position.y = cam2world(1, 3);
    sensor_pose.pose.position.z = cam2world(2, 3);
    sensor_pose.pose.orientation.w = cam2world_quat.w();
    sensor_pose.pose.orientation.x = cam2world_quat.x();
    sensor_pose.pose.orientation.y = cam2world_quat.y();
    sensor_pose.pose.orientation.z = cam2world_quat.z();
  }

  else {
    sensor_pose.pose.position.x = lidar2world(0, 3);
    sensor_pose.pose.position.y = lidar2world(1, 3);
    sensor_pose.pose.position.z = lidar2world(2, 3);

    sensor_pose.pose.orientation.w = lidar2world_quat.w();
    sensor_pose.pose.orientation.x = lidar2world_quat.x();
    sensor_pose.pose.orientation.y = lidar2world_quat.y();
    sensor_pose.pose.orientation.z = lidar2world_quat.z();
  }

  sensor_pose.header = odom_.header;
  sensor_pose.header.frame_id = "/map";
  pub_pose.publish(sensor_pose);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  // Params
  nh.param("use_depth", use_depth, true);
  nh.param("sensing_rate", sensing_rate, 10.0);
  nh.param("estimation_rate", estimation_rate, 10.0);
  nh.param("sensing_horizon", sensing_horizon, 5.0);

  nh.getParam("cam_width", width);
  nh.getParam("cam_height", height);
  nh.getParam("cam_fx", fx);
  nh.getParam("cam_fy", fy);
  nh.getParam("cam_cx", cx);
  nh.getParam("cam_cy", cy);
  nh.getParam("lidar_hfov", hfov);
  nh.getParam("lidar_vfov", vfov);
  nh.getParam("lidar_height", lidar_height);

  cam2body << 0.0, 0.0, 1.0, 0.0,
              -1.0, 0.0, 0.0, 0.0, 
               0.0, -1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0;  
  cam2world = Matrix4d::Identity(); 

  lidar2body << 1.0, 0.0, 0.0, 0.0, 
                 0.0, 1.0, 0.0, 0.0, 
                 0.0, 0.0, 1.0, 0.0, 
                 0.0, 0.0, 0.0, 1.0;  
  lidar2world = Matrix4d::Identity(); 

  // Ros Utils
  global_map_sub = nh.subscribe("global_map", 1, pointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, odometryCallbck);

  pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pcl_render_node/sensor_pose", 1000);
  if (use_depth) pub_data = nh.advertise<sensor_msgs::Image>("/pcl_render_node/depth", 1000);
  else pub_data = nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/cloud", 1000);

  local_sensing_timer = nh.createTimer(ros::Duration(1.0 / sensing_rate), pubSensedPoints);
  estimation_timer = nh.createTimer(ros::Duration(1.0 / estimation_rate), pubSensorPose);

  // Loop 
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}