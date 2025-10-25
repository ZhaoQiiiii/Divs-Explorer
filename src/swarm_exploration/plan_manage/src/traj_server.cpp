#include <ros/ros.h>
#include <poly_traj/polynomial_traj.h>
#include <active_perception/perception_utils.h>
#include <traj_utils/planning_visualization.h>
// #include <swarmtal_msgs/drone_onboard_command.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <plan_manage/backward.hpp>

#include "bspline/Bspline.h"
#include "bspline/non_uniform_bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"


namespace backward {
backward::SignalHandling sh;
}

using fast_planner::NonUniformBspline;
using fast_planner::PerceptionUtils;
using fast_planner::PlanningVisualization;
using fast_planner::Polynomial;
using fast_planner::PolynomialTraj;

ros::Publisher pos_cmd_pub, sensor_fov_pub, cmd_vis_pub, traj_pub, swarm_pos_cmd_pub;
nav_msgs::Odometry odom;
quadrotor_msgs::PositionCommand cmd_;
shared_ptr<PerceptionUtils> percep_utils_;

// Replan
bool receive_traj_ = false;
double replan_time_;

// Trajectory
vector<NonUniformBspline> traj_;
ros::Time traj_start_time_;
int traj_id_, pub_traj_id_;
double traj_duration_;

vector<Eigen::Vector3d> traj_cmd_, traj_real_;

// Benchmark Data (Traj Server)
ros::Time start_time, end_time, last_time;
double energy;

// Loop correction
Eigen::Matrix3d R_loop;
Eigen::Vector3d T_loop;
bool isLoopCorrection;

bool is_drone_, use_depth_;
int drone_id_, drone_num_;
double max_ray_length_;


//
// Helper
//

double calcPathLength(const vector<Eigen::Vector3d>& path) {
  if (path.empty()) return 0;
  double len = 0.0;
  for (int i = 0; i < path.size() - 1; ++i) len += (path[i + 1] - path[i]).norm();
  return len;
}

void drawDepthFoV(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2) {

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  Eigen::Vector4d color(0, 0, 0, 1);
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = mk.scale.y = mk.scale.z = 0.05;

  // Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  sensor_fov_pub.publish(mk);

  // Pub new marker
  if (list1.size() == 0) return;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }

  mk.action = visualization_msgs::Marker::ADD;
  sensor_fov_pub.publish(mk);
}

void drawLidarFoV(const Vector3d& pos) {

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_STRIP;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  Eigen::Vector4d color(0, 0, 0, 1);
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = mk.scale.y = mk.scale.z = 0.05;

  // Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;

  // Pub new marker
  for (int i = 0; i <= 1000; ++i) {
    double theta = 2.0 * M_PI * i / 1000;
    geometry_msgs::Point point;
    point.x = pos(0) + max_ray_length_ * cos(theta);
    point.y = pos(1) + max_ray_length_ * sin(theta);
    point.z = pos(2); 
    mk.points.push_back(point);
  }

  mk.action = visualization_msgs::Marker::ADD;
  sensor_fov_pub.publish(mk);
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id, const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void drawExecutedTraj(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}


//
// Callbacks
//

// Sub "/planning/bspline"
void bsplineCallback(const bspline::BsplineConstPtr& msg) { 

  // Received traj should have ascending traj_id
  if (msg->traj_id <= traj_id_) {
    ROS_ERROR("out of order bspline.");
    return;
  }

  // 
  // Parse the msg to get the Vector of NonUniform Bspline
  //

  traj_start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  // Pos Ctrl Points
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  // Knots
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];

  // Pos Traj
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // Yaw Traj
  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i) yaw_pts(i, 0) = msg->yaw_pts[i];
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);
  
  // Update the Vector of NonUniform Bspline
  traj_.clear();

  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  traj_.push_back(traj_[2].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
  if (start_time.isZero()) {
    ROS_WARN("[Traj Server] Start Flight...");
    start_time = ros::Time::now();
  }
}

// Sub "/planning/replan"
void replanCallback(std_msgs::Empty msg) {
  // Informed of new replan, end the current traj after some time
  const double time_out = 0.2;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - traj_start_time_).toSec() + time_out + replan_time_;
  traj_duration_ = min(t_stop, traj_duration_);
}

// Sub "/planning/new"
void newCallback(std_msgs::Empty msg) {
  // Clear the data of executed traj
  traj_cmd_.clear();
  traj_real_.clear();
}

// Sub "/odom_world"
void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;
  odom = msg;
  traj_real_.push_back(Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

// Sub "/loop_fusion/pg_T_vio"
void pgTVioCallback(geometry_msgs::Pose msg) {
  // World to odom
  Eigen::Quaterniond q = Eigen::Quaterniond(
      msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  R_loop = q.toRotationMatrix();
  T_loop << msg.position.x, msg.position.y, msg.position.z;

  // cout << "R_loop: " << R_loop << endl;
  // cout << "T_loop: " << T_loop << endl;
}


// Timer 0.01s
void cmdCallback(const ros::TimerEvent& e) {
  if (!receive_traj_) return;

  //
  // 1、Get the received state of B-spline
  //

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - traj_start_time_).toSec();
  Eigen::Vector3d pos, vel, acc, jer;
  double yaw, yawdot;

  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    jer = traj_[5].evaluateDeBoorT(t_cur);
  } 

  else if (t_cur >= traj_duration_) { // Keep static
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = 0.0;
  } 

  else {
    ROS_WARN("[Traj Server] Invalid Time.");
  }

  if (isLoopCorrection) { // Correct state if necessary
    pos = R_loop.transpose() * (pos - T_loop);
    vel = R_loop.transpose() * vel;
    acc = R_loop.transpose() * acc;

    Eigen::Vector3d yaw_dir(cos(yaw), sin(yaw), 0);
    yaw_dir = R_loop.transpose() * yaw_dir;
    yaw = atan2(yaw_dir[1], yaw_dir[0]);
  }


  //
  // 2、Pub Data
  //

  // Pub the Position Command
  cmd_.header.stamp = time_now;
  cmd_.trajectory_id = traj_id_;

  cmd_.position.x = pos(0);
  cmd_.position.y = pos(1);
  cmd_.position.z = pos(2);

  cmd_.velocity.x = vel(0);
  cmd_.velocity.y = vel(1);
  cmd_.velocity.z = vel(2);

  cmd_.acceleration.x = acc(0);
  cmd_.acceleration.y = acc(1);
  cmd_.acceleration.z = acc(2);

  cmd_.yaw = yaw; 
  cmd_.yaw_dot = yawdot;
 
  pos_cmd_pub.publish(cmd_); // "/planning/position_cmd"


  // Draw FoV
  if (is_drone_ || (use_depth_ && false)) {
  // if (use_depth_) {
    percep_utils_->setPose(pos, yaw);
    vector<Eigen::Vector3d> l1, l2;
    percep_utils_->getFOV(l1, l2);
    drawDepthFoV(l1, l2);
  }

  else drawLidarFoV(pos);


  // 
  // 3、Report and Record the Flight
  //

  // Report
  double len = calcPathLength(traj_cmd_);
  double flight_t = (end_time - start_time).toSec();
  ROS_WARN_THROTTLE(2.0, "[Traj Server] Drone %d, time: %lf, length: %lf, vel: %lf, energy: %lf", 
                    drone_id_, flight_t, len, len / flight_t, energy);
  

  // Record
  if (traj_cmd_.size() == 0) {
    traj_cmd_.push_back(pos);
  } 

  else if ((pos - traj_cmd_.back()).norm() > 1e-6) {
    traj_cmd_.push_back(pos);
    double dt = (time_now - last_time).toSec();
    energy += jer.squaredNorm() * dt;
    end_time = ros::Time::now();
  }

  last_time = time_now;
}

// Timer 0.25s
void visCallback(const ros::TimerEvent& e) {
  // Draw the Executed Traj (Desired State)
  drawExecutedTraj(traj_cmd_, 0.05, 
    PlanningVisualization::getColor((drone_id_ - 1) / double(drone_num_)), pub_traj_id_);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  // Params
  nh.param("use_depth", use_depth_, true);
  nh.param("drone_id", drone_id_, 1);
  nh.param("drone_num", drone_num_, 1);
  nh.param("traj_server/pub_traj_id", pub_traj_id_, -1);
  nh.param("fsm/replan_time", replan_time_, 0.1);
  nh.param("loop_correction/isLoopCorrection", isLoopCorrection, false);
  nh.param("is_drone", is_drone_, true);
  nh.param("max_ray_length", max_ray_length_, 5.0);

  R_loop = Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();
  T_loop = Eigen::Vector3d(0, 0, 0);
  percep_utils_.reset(new PerceptionUtils(nh));

  // Ros Utils
  ros::Subscriber bspline_sub = nh.subscribe("/planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = nh.subscribe("/planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = nh.subscribe("/planning/new", 10, newCallback);
  ros::Subscriber odom_sub = nh.subscribe("/odom_world", 50, odomCallbck);
  ros::Subscriber pg_T_vio_sub = nh.subscribe("/loop_fusion/pg_T_vio", 10, pgTVioCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/position_cmd", 50);
  sensor_fov_pub = nh.advertise<visualization_msgs::Marker>("/planning_vis/sensor_fov", 10);
  traj_pub = nh.advertise<visualization_msgs::Marker>("/planning_vis/executed_traj", 10);
  // cmd_vis_pub = nh.advertise<visualization_msgs::Marker>("/planning_vis/position_cmd_vis", 10);
  // swarm_pos_cmd_pub = nh.advertise<swarmtal_msgs::drone_onboard_command>("/drone_commander/onboard_command", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = nh.createTimer(ros::Duration(0.25), visCallback);

  ros::Duration(1.0).sleep();

  // Command Init 
  cmd_.header.stamp = ros::Time::now();
  cmd_.header.frame_id = "world";
  cmd_.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd_.trajectory_id = traj_id_;
  cmd_.position.x = 0.0;
  cmd_.position.y = 0.0;
  cmd_.position.z = 0.0;
  cmd_.velocity.x = 0.0;
  cmd_.velocity.y = 0.0;
  cmd_.velocity.z = 0.0;
  cmd_.acceleration.x = 0.0;
  cmd_.acceleration.y = 0.0;
  cmd_.acceleration.z = 0.0;
  cmd_.yaw = 0.0;
  cmd_.yaw_dot = 0.0;

  ros::spin();
  return 0;
}
