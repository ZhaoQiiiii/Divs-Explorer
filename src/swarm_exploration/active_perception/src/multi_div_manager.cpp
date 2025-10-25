#include <active_perception/multi_div_manager.h>
#include <visualization_msgs/Marker.h>
#include <fstream>

namespace fast_planner {

MultiDivManager::MultiDivManager() {
}

MultiDivManager::~MultiDivManager() {
}

void MultiDivManager::init(const shared_ptr<DivisionParam>& params, 
  const unordered_map<int, shared_ptr<Division>>& divs) {
  // Params
  nh_.param("is_drone", is_drone_, true);
  nh_.param("exploration/drone_id", drone_id_, 1);
  nh_.param("exploration/drone_num", drone_num_, 1);

  // Containers
  dp_ = params;
  div_num_ = dp_->div_total_number_;
  last_update_divs_times_.resize(div_num_, 0.0);
  last_recv_msg_time_ = 0.0;
  all_divs_ = divs;

  // Ros Utils
  data_pub_ = nh_.advertise<DivisionPkg>("/multi_div_manager/divison_data_send", 100);
  data_sub_ = nh_.subscribe("/multi_div_manager/divison_data_recv", 50, &MultiDivManager::recvDataCallback, this, 
    ros::TransportHints().tcpNoDelay());
}


//
// Main API
//

// Sync the Divs Data
void MultiDivManager::syncDivData(const unordered_map<int, shared_ptr<Division>>& divs) {

  if (divs.empty()) return;

  DivisionPkg pkg;
  pkg.from_drone_id = drone_id_;
  pkg.send_time = ros::Time::now().toSec();

  for (const auto& [id, div] : divs) {
    DivisionData data;
    convertDivToData(div, data);
    pkg.divs.push_back(data);
  }
  data_pub_.publish(pkg); // Pub "/multi_div_manager/divison_data"
}


//
// Helper
//

// Sub "/multi_div_manager/divison_data"
void MultiDivManager::recvDataCallback(const DivisionPkgPtr& msg) {

  if (msg->from_drone_id == drone_id_) return;
  if (msg->send_time < last_recv_msg_time_ + 0.1) return;

  for (const auto& data : msg->divs) {
    // Check the validness of id
    if (all_divs_.find(data.id) == all_divs_.end()) continue;

    // Check the freshness of data
    if (msg->send_time <= last_update_divs_times_[data.id]) continue;

    // Update own data
    convertDataToDiv(data, all_divs_[data.id]);

    // Record update data time
    last_update_divs_times_[data.id] = msg->send_time;
  }

  last_recv_msg_time_ = msg->send_time;
}

// Div Data -> Ros Data
void MultiDivManager::convertDivToData(const shared_ptr<Division>& div, DivisionData& data) {
  data.id = div->id_;

  // Unknown
  data.unknown_num = div->unknown_number_;
  data.unknown_center.x = div->unknown_center_.x();
  data.unknown_center.y = div->unknown_center_.y();
  data.unknown_center.z = div->unknown_center_.z();
  for (const auto& voxel : div->unknown_) {
    geometry_msgs::Point pt;
    pt.x = voxel.x(), pt.y = voxel.y(), pt.z = voxel.z();
    data.unknown_voxels.push_back(pt);
  }

  // Free
  data.free_num = div->free_number_;
  data.free_center.x = div->free_center_.x();
  data.free_center.y = div->free_center_.y();
  data.free_center.z = div->free_center_.z();
  for (const auto& voxel : div->free_) {
    geometry_msgs::Point pt;
    pt.x = voxel.x(), pt.y = voxel.y(), pt.z = voxel.z();
    data.free_voxels.push_back(pt);
  }

  // Ground
  for (const auto& voxel : div->ground_) {
    geometry_msgs::Point pt;
    pt.x = voxel.x(), pt.y = voxel.y(), pt.z = voxel.z();
    data.ground_voxels.push_back(pt);
  }

  // Coverage Rate
  data.coverage_rate = div->coverage_rate_;
}

// Ros Data -> Div Data
void MultiDivManager::convertDataToDiv(const DivisionData& data, shared_ptr<Division>& div) {
  div->coverage_rate_ = data.coverage_rate;

  // Unknown
  div->unknown_number_ = data.unknown_num;
  div->unknown_center_ = Vector3d(data.unknown_center.x, data.unknown_center.y, data.unknown_center.z);
  div->unknown_.clear();
  for (const auto& pt : data.unknown_voxels) {
    div->unknown_.emplace_back(pt.x, pt.y, pt.z);
  }

  // Free
  div->free_number_ = data.free_num;
  div->free_center_ = Vector3d(data.free_center.x, data.free_center.y, data.free_center.z);
  div->free_.clear();
  for (const auto& pt : data.free_voxels) {
    div->free_.emplace_back(pt.x, pt.y, pt.z);
  }

  // Ground
  div->ground_.clear();
  for (const auto& pt : data.ground_voxels) {
    div->ground_.emplace_back(pt.x, pt.y, pt.z);
  }

  // Coverage Rate
  div->coverage_rate_ = data.coverage_rate;
}

} // namespace fast_planner