#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>
#include <plan_env/multi_map_manager.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

namespace fast_planner {

MapROS::MapROS() {
}

MapROS::~MapROS() {
}

void MapROS::setMap(SDFMap* map) {
  this->map_ = map;
}

void MapROS::init() {

  // Params
  nh_.getParam("cam_fx", fx_);
  nh_.getParam("cam_fy", fy_);
  nh_.getParam("cam_cx", cx_);
  nh_.getParam("cam_cy", cy_);

  nh_.param("map_ros/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
  nh_.param("map_ros/depth_filter_mindist", depth_filter_mindist_, -1.0);
  nh_.param("map_ros/depth_filter_margin", depth_filter_margin_, -1);
  nh_.param("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
  nh_.param("map_ros/lidar_filter_maxdist", lidar_filter_maxdist_, -1.0);
  nh_.param("map_ros/lidar_filter_mindist", lidar_filter_mindist_, -1.0);
  nh_.param("map_ros/skip_pixel", skip_pixel_, -1);
  nh_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
  nh_.param("map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
  nh_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
  nh_.param("map_ros/show_occ_time", show_occ_time_, false);
  nh_.param("map_ros/show_esdf_time", show_esdf_time_, false);
  nh_.param("map_ros/show_all_map", show_all_map_, false);
  nh_.param("map_ros/show_local_map", show_local_map_, false);
  nh_.param("map_ros/frame_id", frame_id_, string("world"));
  nh_.param("sdf_map/box_max_z", box_max_z_, 2.0);
  nh_.param("show_ground", show_ground_, true);

  proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  proj_points_cnt = 0;

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  esdf_num_ = 0;
  depth_image_.reset(new cv::Mat);

  rand_noise_ = normal_distribution<double>(0, 0.1);
  random_device rd;
  eng_ = default_random_engine(rd());

  // Ros Utils
  vis_timer_ = nh_.createTimer(ros::Duration(0.2), &MapROS::pubMapCallback, this);
  esdf_timer_ = nh_.createTimer(ros::Duration(0.05), &MapROS::updateESDFCallback, this);

  map_all_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 10);
  map_local_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 10);
  map_local_inflate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
  unknown_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  esdf_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = nh_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sdf_map/local_cloud", 10);

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/map_ros/depth", 50));
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/map_ros/cloud", 50));
  pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/map_ros/pose", 25));

  sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
                         MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
  sync_image_pose_->registerCallback(boost::bind(&MapROS::depthCallback, this, _1, _2));

  sync_cloud_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
                         MapROS::SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
  sync_cloud_pose_->registerCallback(boost::bind(&MapROS::cloudCallback, this, _1, _2));
}


//
// Callbacks
//

// Sub "/map_ros/depth" + "/map_ros/sensor_pose"
void MapROS::depthCallback(const sensor_msgs::ImageConstPtr& img, 
  const geometry_msgs::PoseStampedConstPtr& pose) {
  
  // Get the Camera Pose
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  if (!map_->isInMap(camera_pos_)) return; 
  map_->mm_->drone_pos_ = camera_pos_; // Simulate swarm communication
  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                 pose->pose.orientation.y, pose->pose.orientation.z);

  // Get and Process the Depth Image from Depth Camera
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(*depth_image_);
  processDepthImage();

  // Update the Global Map
  map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);

  // Update the Local Map
  if (local_updated_) {
    map_->InflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
}

// Sub "/map_ros/cloud" + "/map_ros/sensor_pose"
void MapROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg, 
  const geometry_msgs::PoseStampedConstPtr& pose) {
  
  // Get the Lidar Pose
  lidar_pos_(0) = pose->pose.position.x;
  lidar_pos_(1) = pose->pose.position.y;
  lidar_pos_(2) = pose->pose.position.z;
  if (!map_->isInMap(lidar_pos_)) return;

  // Get and Process the Point Cloud from Lidar
  auto t1 = ros::Time::now();
  pcl::fromROSMsg(*msg, point_cloud_);
  publishCloud();

  // Update the Global Map
  map_->inputPointCloud(point_cloud_, point_cloud_.points.size(), lidar_pos_);

  // Update the Local Map
  if (local_updated_) {
    map_->InflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
}


// Timer 0.2s
void MapROS::pubMapCallback(const ros::TimerEvent& e) {
  
  if (show_all_map_) {
    static double t1 = 0.0;
    t1 += (e.current_real - e.last_real).toSec();
    if (t1 > 0.1) {
      publishMapAll();
      t1 = 0.0;
    }
  }

  if (show_local_map_) {
    static double t2 = 0.0; 
    t2 += (e.current_real - e.last_real).toSec();
    if (t2 > 0.1) {
      publishMapLocal();
      t2 = 0.0;
    }
  }

  // publishUnknown();
  // publishESDF();
  // publishUpdateRange();
}

// Timer 0.05s
void MapROS::updateESDFCallback(const ros::TimerEvent& e) {
  
  if (!esdf_need_update_) return;

  // Update ESDF
  auto t1 = ros::Time::now();
  map_->updateESDF3d();
  esdf_need_update_ = false;

  // Record Time
  auto t2 = ros::Time::now();
  esdf_time_ += (t2 - t1).toSec();
  max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
  esdf_num_++;
  if (show_esdf_time_) {
    ROS_WARN("ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), esdf_time_ / esdf_num_, max_esdf_time_);
  }
}



//
// Helper
//

void MapROS::processDepthImage() {
  
  proj_points_cnt = 0;

  uint16_t* row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;

  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {
      depth = (*row_ptr) * inv_factor;
      row_ptr = row_ptr + skip_pixel_;

      // // filter depth
      // if (depth > 0.01)
      //   depth += rand_noise_(eng_);

      // TODO: simplify the logic here
      // if (*row_ptr == 0) continue;

      if (*row_ptr == 0 || depth > depth_filter_maxdist_) depth = depth_filter_maxdist_;
      else if (depth < depth_filter_mindist_) continue;

      pt_cur(0) = (u - cx_) * depth / fx_;
      pt_cur(1) = (v - cy_) * depth / fy_;
      pt_cur(2) = depth;
      pt_world = camera_r * pt_cur + camera_pos_;
      auto& pt = point_cloud_.points[proj_points_cnt++];
      pt.x = pt_world[0];
      pt.y = pt_world[1];
      pt.z = pt_world[2];
    }
  }

  publishCloud();
}

void MapROS::publishCloud() {
  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const auto& pt : point_cloud_) cloud.push_back(pt);

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_pub_.publish(cloud_msg); // "/sdf_map/depth_cloud" or "/sdf_map/lidar_cloud"
}

void MapROS::publishMapAll() {
  
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;

  Eigen::Vector3i min_idx, max_idx;
  Eigen::Vector3d all_min = map_->md_->all_min_;
  Eigen::Vector3d all_max = map_->md_->all_max_;

  // all_min[2] = show_ground_ ? 0.1 : all_min[2];
  all_min[2] = show_ground_ ? 0.1 : 0.1;
  
  map_->posToIndex(all_min, min_idx);
  map_->posToIndex(all_max, max_idx);
  map_->boundIndex(min_idx);
  map_->boundIndex(max_idx);

  for (int x = min_idx[0]; x <= max_idx[0]; ++x) {
    for (int y = min_idx[1]; y <= max_idx[1]; ++y) {
      for (int z = min_idx[2]; z <= max_idx[2]; ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          if (pos(2) > box_max_z_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
      }
    }
  }

  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_all_pub_.publish(cloud_msg);
}

void MapROS::publishMapLocal() {
  
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1;
  pcl::PointCloud<pcl::PointXYZ> cloud2;

  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);
  
  for (int x = min_cut(0); x <= max_cut(0); ++x) {
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        // Occupied Voxel
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
        
        // Inflated Occupied Voxel
        else if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y, z)] == 1) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud2.push_back(pt);
        }
      }
    }
  }

  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;

  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_local_pub_.publish(cloud_msg);

  pcl::toROSMsg(cloud2, cloud_msg);
  map_local_inflate_pub_.publish(cloud_msg);
}

void MapROS::publishUnknown() {
  
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_idx, max_idx;
  map_->posToIndex(map_->md_->all_min_, min_idx);
  map_->posToIndex(map_->md_->all_max_, max_idx);

  map_->boundIndex(min_idx);
  map_->boundIndex(max_idx);

  for (int x = min_idx(0); x <= max_idx(0); ++x) {
    for (int y = min_idx(1); y <= max_idx(1); ++y) {
      for (int z = min_idx(2); z <= max_idx(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] <
            map_->mp_->clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void MapROS::publishESDF() {
  
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i offset = {map_->mp_->local_map_margin_, map_->mp_->local_map_margin_, map_->mp_->local_map_margin_};
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - offset;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + offset;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x) {
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      Eigen::Vector3d pos;
      map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = esdf_slice_height_;
      dist = map_->getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = 0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  esdf_pub_.publish(cloud_msg);
}

void MapROS::publishUpdateRange() {
  
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
  map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

} // namespace fast_planner