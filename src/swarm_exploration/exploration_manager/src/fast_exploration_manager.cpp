#include <exploration_manager/expl_data.h>
#include <exploration_manager/fast_exploration_manager.h>

#include <fstream>
#include <iostream>
#include <thread>

#include <active_perception/frontier_finder.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/hgrid.h>
#include <active_perception/multi_div_manager.h>
#include <active_perception/perception_utils.h>

#include <plan_env/edt_environment.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>

#include <lkh_mtsp_solver/SolveMTSP.h>
#include <lkh_tsp_solver/SolveTSP.h>
#include <lkh_tsp_solver/lkh_interface.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner {

FastExplorationManager::FastExplorationManager() {}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void FastExplorationManager::initialize(ros::NodeHandle &nh_) {
  // Params
  ep_.reset(new ExplParam);
  ed_.reset(new ExplData);
  dp_.reset(new DivisionParam);
  md_.reset(new MultiDivManager);

  nh_.param("exploration/refine_local", ep_->refine_local_, false);
  nh_.param("exploration/refined_num", ep_->refined_num_, -1);
  nh_.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh_.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh_.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh_.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh_.param("exploration/mtsp_dir", ep_->mtsp_dir_, string("null"));
  nh_.param("exploration/result_dir", ep_->result_dir_, string("null"));
  nh_.param("exploration/relax_time", ep_->relax_time_, 1.0);
  nh_.param("exploration/drone_num", ep_->drone_num_, 1);
  nh_.param("exploration/drone_id", ep_->drone_id_, 1);
  nh_.param("exploration/init_plan_num", ep_->init_motion_plan_num_, 2);

  nh_.param("exploration/vm", ViewNode::vm_, -1.0);
  nh_.param("exploration/am", ViewNode::am_, -1.0);
  nh_.param("exploration/yd", ViewNode::yd_, -1.0);
  nh_.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh_.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  nh_.param("sdf_map/box_min_x", ep_->box_min_x_, -1.0);
  nh_.param("sdf_map/box_min_y", ep_->box_min_y_, -1.0);
  nh_.param("sdf_map/box_min_z", ep_->box_min_z_, -1.0);
  nh_.param("sdf_map/box_max_x", ep_->box_max_x_, -1.0);
  nh_.param("sdf_map/box_max_y", ep_->box_max_y_, -1.0);
  nh_.param("sdf_map/box_max_z", ep_->box_max_z_, -1.0);

  nh_.param("expl_task", ep_->expl_task_, 0);
  nh_.param("expl_method", ep_->expl_method_, 0);
  nh_.param("is_drone", ep_->is_drone_, true);
  nh_.param("is_drone", dp_->is_drone_, true);
  nh_.param("complete_expl", dp_->complete_expl_, true);
  nh_.param("exploration/div_inflation", dp_->div_inflation_, 1.5);
  nh_.param("exploration/divide_size", dp_->div_size_, 5.0);
  nh_.param("exploration/coverage_rate_thr", dp_->coverage_rate_thr_, 1.0);

  ep_->box_min_ = {ep_->box_min_x_, ep_->box_min_y_, ep_->box_min_z_};
  ep_->box_max_ = {ep_->box_max_x_, ep_->box_max_y_, ep_->box_max_z_};

  ed_->begin_ = true;
  ed_->finish_partial_ = false;
  ed_->triggered_ = false;
  ed_->find_coverage_ = false;
  ed_->greedy_ = false;
  ed_->follow_ = false;
  ed_->need_expl_replan_ = true;
  ed_->to_viewpoint_ = true;
  ed_->loop_ = false;
  ed_->reset_ = false;

  ed_->plan_num_ = 1;
  ep_->process_dir_ = ep_->result_dir_ + "/process_" + to_string(ep_->drone_id_) + ".txt";
  ep_->compute_dir_ = ep_->result_dir_ + "/compute_" + to_string(ep_->drone_id_) + ".txt";
  std::ofstream process_clear(ep_->process_dir_, std::ios::trunc);
  std::ofstream compute_clear(ep_->compute_dir_, std::ios::trunc);

  // Planner Manager
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh_);
  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  planner_manager_->path_finder_->max_search_time_ = 1.0;
  planner_manager_->swarm_traj_data_.init(ep_->drone_id_, ep_->drone_num_);
  ed_->motion_plan_num_ = 0;

  // ESDF Map
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;

  // Path Finder
  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh_, edt_environment_);
  ViewNode::map_ = sdf_map_;
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);

  // RayCaster
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(sdf_map_->getResolution(), origin);

  // Frontier Finder
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh_));

  // Hgrid
  hgrid_.reset(new HGrid(edt_environment_, nh_));

  // Swarm
  ed_->swarm_state_.resize(ep_->drone_num_);
  ed_->pair_opt_stamps_.resize(ep_->drone_num_);
  ed_->pair_opt_res_stamps_.resize(ep_->drone_num_);
  for (int i = 0; i < ep_->drone_num_; ++i) {
    ed_->swarm_state_[i].stamp_ = 0.0;
    ed_->pair_opt_stamps_[i] = 0.0;
    ed_->pair_opt_res_stamps_[i] = 0.0;
  }
  for (auto &state : ed_->swarm_state_) {
    state.stamp_ = 0.0;
    state.recent_interact_time_ = 0.0;
    state.recent_attempt_time_ = 0.0;
  }
  ed_->last_grid_ids_ = {};
  ed_->reallocated_ = true;
  ed_->pair_opt_stamp_ = 0.0;
  ed_->wait_response_ = false;

  // ATSP & CVRP Solver
  tsp_client_ = nh_.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_tsp_" + to_string(ep_->drone_id_), true);
  acvrp_client_ = nh_.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_acvrp_" + to_string(ep_->drone_id_), true);

  // Division
  double reduce = 0.25;
  Vector3d box_min = {ep_->box_min_x_ + reduce, ep_->box_min_y_ + reduce, ep_->box_min_z_};
  Vector3d box_max = {ep_->box_max_x_ - reduce, ep_->box_max_y_ - reduce, ep_->box_max_z_};
  DivisionUtils::divideExplorationSpace(box_min, box_max, edt_environment_, dp_, ed_->all_divs_);
  ed_->cur_div_ = make_shared<Division>();
  ed_->ori_div_ = make_shared<Division>();
  ed_->next_div_ = make_shared<Division>();
  ed_->next_corner_div_ = make_shared<Division>();
  if (ep_->is_drone_)
    next_cor_pub_ = nh_.advertise<NextCor>("/exploration/next_cor_1", 10);

  // Multi Division Manager
  if (ep_->expl_method_ == 0) {
    md_->nh_ = nh_;
    md_->init(dp_, ed_->all_divs_);
  }

  // Exploration Process
  ep_->voxel_number = 0;
  double res = sdf_map_->getResolution();
  for (double x = ep_->box_min_[0]; x <= ep_->box_max_[0]; x += res) {
    for (double y = ep_->box_min_[1]; y <= ep_->box_max_[1]; y += res) {
      for (double z = ep_->box_min_[2]; z <= ep_->box_max_[2]; z += res) {
        ep_->voxel_number++;
      }
    }
  }
  ed_->unknown_number_ = ep_->voxel_number;
  ed_->coverage_ = double(ep_->voxel_number - ed_->unknown_number_) / double(ep_->voxel_number);
}

//
// Main Modules
//

int FastExplorationManager::updateFrontierStruct(const Eigen::Vector3d &pos) {

  //
  // Update exploration data
  //

  ros::Time t1 = ros::Time::now();

  //
  // 1、Search & Group Frontiers and Delete old Frontiers
  //

  frontier_finder_->searchFrontiers();

  // double frontier_time = (ros::Time::now() - t1).toSec();
  // t1 = ros::Time::now();

  //
  // 2、Get the Viewpoints by Sampling for New Frontiers
  //

  frontier_finder_->computeFrontiersToVisit();
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  frontier_finder_->getTopViewpointsInfo(pos, ed_->top_points_, ed_->top_yaws_, ed_->averages_);
  if (ed_->frontiers_.empty())
    return 0;

  ed_->views_.clear();
  for (int i = 0; i < ed_->top_points_.size(); ++i) {
    ed_->views_.push_back(ed_->top_points_[i] + 2.0 * Vector3d(cos(ed_->top_yaws_[i]), sin(ed_->top_yaws_[i]), 0));
  }

  ed_->ftrs_.clear();
  for (const auto &ftr : frontier_finder_->frontiers_) {
    ed_->ftrs_.push_back(ftr);
  }

  // double view_time = (ros::Time::now() - t1).toSec();
  // t1 = ros::Time::now();

  //
  // 3、Update the cost matrix between Frontiers
  //

  frontier_finder_->updateFrontierCostMatrix();

  // double mat_time = (ros::Time::now() - t1).toSec();
  // t1 = ros::Time::now();

  //
  // 4、Update & Sync the Divs Data
  //

  // Get the Updated Box
  edt_environment_->sdf_map_->getUpdatedBox(ed_->updated_box_min_, ed_->updated_box_max_, true);
  double offset = (ep_->box_min_[2] + ep_->box_max_[2]) / 2.0;
  DivisionUtils::getBoxDrawing(ed_->updated_box_min_, ed_->updated_box_max_, ed_->updated_box_vis_, offset);

  // Get the overlap Divs with Updated Box
  unordered_set<int> overlap_ids;
  unordered_map<int, shared_ptr<Division>> overlap_divs;
  DivisionUtils::getOverlapDivs(ed_->updated_box_min_, ed_->updated_box_max_, ed_->all_divs_, overlap_divs,
                                overlap_ids);

  // Update & Sync Data for Divs
  DivisionUtils::updateDivData(frontier_finder_, dp_, ed_->all_divs_, ed_->ori_div_->id_); // All Divs
  if (ep_->expl_method_ == 0)
    md_->syncDivData(overlap_divs); // Overlap Divs

  // double divs_time = (ros::Time::now() - t1).toSec();
  // t1 = ros::Time::now();

  //
  // 5、Update the Exploration Process
  //

  updateExplProcess();
  double process_time = (ros::Time::now() - t1).toSec();

  // double total_time = frontier_time + view_time + mat_time + divs_time + process_time;
  // ROS_INFO_THROTTLE(1.0, "[FIS] Drone %d: frontier: %lf, viewpoint: %lf, mat: %lf, division: %lf, process: %lf, Total
  // Time: %lf",
  //                   ep_->drone_id_, frontier_time, view_time, mat_time, divs_time, process_time, total_time);
  return ed_->frontiers_.size();
}

int FastExplorationManager::generateExploreTraj(const Vector3d &cur_pos, const Vector3d &cur_vel,
                                                const Vector3d &cur_acc, const Vector3d &cur_yaw,
                                                const Vector3d &next_pos, const double &next_yaw) {
  //
  // Generate exploration trajectory
  //

  // ROS_INFO_STREAM("[Explore] Drone " << ep_->drone_id_ << ", Next Pos: " << next_pos.transpose() << ", Next Yaw: " <<
  // next_yaw);

  auto t1 = ros::Time::now();

  //
  // 1、Generate the Motion Trajectory of Pos
  //

  // Get the Time Lower Bound
  double diff = fabs(next_yaw - cur_yaw[0]);
  double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;
  if (!ed_->to_viewpoint_)
    time_lb *= dp_->div_inflation_;

  // Get the Initial A* Path
  planner_manager_->path_finder_->reset();
  bool plan_init = ed_->motion_plan_num_ < ep_->init_motion_plan_num_; // For Init
  if (planner_manager_->path_finder_->search(cur_pos, next_pos, plan_init || !ed_->to_viewpoint_) != Astar::REACH_END) {
    ROS_WARN_STREAM("[Motion] Drone " << ep_->drone_id_ << ", No path to next position.");
    ed_->reset_ = true;
    return FAIL;
  }
  vector<Eigen::Vector3d> init_path = planner_manager_->path_finder_->getPath();
  ed_->init_path_ = init_path;

  // Optimize the Initial A* Path
  shortenPath(init_path);
  double len = ViewNode::astar_->pathLength(init_path);
  double radius_far = 7.0;
  double radius_close = 1.5;

  if (plan_init || !ed_->to_viewpoint_ || len < radius_close) { // Waypoints-based
    planner_manager_->planExploreTraj(init_path, cur_vel, cur_acc, time_lb);
    ed_->next_goal_ = next_pos;
    if (ed_->motion_plan_num_ < ep_->init_motion_plan_num_)
      ed_->motion_plan_num_++;
  }

  else if (len > radius_far) { // Waypoints-based + Truncated Path
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = {init_path.front()};
    for (int i = 1; i < init_path.size() && len2 < radius_far; ++i) {
      auto cur_pt = init_path[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    planner_manager_->planExploreTraj(truncated_path, cur_vel, cur_acc, time_lb);
  }

  else { // Kino Replan
    ed_->kino_path_.clear();
    ed_->next_goal_ = next_pos;
    if (!planner_manager_->kinodynamicReplan(cur_pos, cur_vel, cur_acc, ed_->next_goal_, {}, time_lb))
      return FAIL;
    ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  }

  // Check the Time Lower Bound
  // if (planner_manager_->local_traj_.position_traj_.getTimeSum() < time_lb - 0.5) {
  //   ROS_WARN_STREAM("[Motion] Drone " << ep_->drone_id_ << ", Time Lower Bound is not satified!");
  //   return FAIL;
  // }

  double pos_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  //
  // 2、Generate the Motion Trajectory of Yaw
  //

  if (!ed_->to_viewpoint_)
    planner_manager_->planYaw(cur_yaw);
  else
    planner_manager_->planExploreYaw(cur_yaw, next_yaw, true, ep_->relax_time_);
  double yaw_time = (ros::Time::now() - t1).toSec();

  // ROS_INFO("[Motion] Pos Planning: %lf, Yaw Planning: %lf", pos_time, yaw_time);
  return SUCCEED;
}

//
// Proposed
//

int FastExplorationManager::planExploreDrone(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_acc,
                                             const Vector3d &cur_yaw) {

  ros::Time t1 = ros::Time::now();

  if (ed_->frontiers_.empty()) {
    // ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", There is no Frontiers.");
    return NO_FRONTIER;
  }

  //
  // 1、Find the Coverage Path to Get Next Division
  //

  if (!ed_->find_coverage_) {

    ed_->find_coverage_ = true;

    // Get the candidate Divs
    getCandidateDivs(cur_pos);

    // Get the Next Division with activated Divs
    if (ed_->activated_divs_.size() == 1) {
      ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", Just one activated Div.");
      for (auto [id, div] : ed_->activated_divs_)
        ed_->next_div_ = div;
      ed_->coverage_path_ = {ed_->ori_div_->center_, ed_->next_div_->unknown_center_};
    }

    else if (ed_->activated_divs_.size() > 1) {
      // ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", More than one activated Divs.");
      findCoveragePath(cur_pos, cur_vel, cur_yaw, ed_->activated_divs_, ed_->next_div_, ed_->coverage_path_);
    }

    else {
      // ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", There is no activated Divs.");
      if (ed_->frontiers_.empty())
        return NO_FRONTIER;
      if (ep_->expl_task_ == 1) {
        ed_->finish_partial_ = true;
        return NO_DIVISION;
      }

      ed_->greedy_ = true;
      ed_->need_expl_replan_ = false;
      ed_->cur_div_ = make_shared<Division>();
      ed_->ori_div_ = make_shared<Division>();
      ed_->next_div_ = make_shared<Division>();
      ed_->next_corner_div_ = make_shared<Division>();
      ed_->coverage_path_.clear();
      ed_->exploration_path_.clear();
    }

    // Get the Next Corner Division
    if (!ed_->greedy_)
      getNextCornerDiv(ed_->next_corner_div_);

    // double coverage_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO_STREAM("[Explore] Drone " << ep_->drone_id_ << ", Total Time: " << coverage_time);
    // t1 = ros::Time::now();
  }

  //
  // 2、Find the Exploration Path to get the Next Viewpoint
  //

  if (ed_->find_coverage_) {

    ed_->to_viewpoint_ = true;

    // Get the chosen frontier
    getChosenFrontiers(cur_pos, cur_vel, cur_yaw, ed_->chosen_ftrs_);

    // Get the Next Viewpoint with chosen frontier
    if (ed_->greedy_ || ed_->reset_ || (ep_->expl_task_ == 1 && ed_->loop_) || ed_->chosen_ftrs_.empty()) {
      // ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", No chosen frontier.");
      ed_->reset_ = false;
      if (greedyExplore(cur_pos, cur_vel, cur_yaw, ed_->ftrs_, ed_->next_pos_, ed_->next_yaw_) == FAIL)
        return FAIL;
      ed_->exploration_path_ = {cur_pos, ed_->next_pos_};
      if (ed_->next_corner_div_->id_ != -1)
        ed_->exploration_path_.push_back(ed_->next_corner_div_->center_);
    }

    else if (ed_->chosen_ftrs_.size() > 1) {
      // ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", More than one chosen frontiers.");
      if (findExplorationPath(cur_pos, cur_vel, cur_yaw, ed_->chosen_ftrs_, ed_->next_pos_, ed_->next_yaw_,
                              ed_->exploration_path_) == FAIL)
        return FAIL;
    }

    else if (ed_->chosen_ftrs_.size() == 1) {
      // ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", Just one chosen frontier.");
      if (findBestViewpoint(cur_pos, cur_vel, cur_yaw, ed_->chosen_ftrs_.front(), ed_->next_pos_, ed_->next_yaw_) ==
          FAIL)
        return FAIL;
      ed_->exploration_path_ = {cur_pos, ed_->next_pos_, ed_->next_corner_div_->center_};
    }

    // double exploration_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO_STREAM("[Explore] Drone " << ep_->drone_id_ << ", Total Time: " << exploration_time);
  }

  // // Write the Compute Time into txt
  // ofstream file_write(ep_->compute_dir_, std::ios::app);
  // file_write << "Plan Number:" << ed_->plan_num_++ << " Compute Time:" << 1000 * (ros::Time::now() - t1).toSec() <<
  // endl;

  //
  // 3、Generate Trajectory to Next Viewpoint
  //

  return generateExploreTraj(cur_pos, cur_vel, cur_acc, cur_yaw, ed_->next_pos_, ed_->next_yaw_);
}

int FastExplorationManager::planExploreGround(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_acc,
                                              const Vector3d &cur_yaw, const Vector3d &drone_pos,
                                              const double &drone_yaw) {

  ros::Time t1 = ros::Time::now();
  ed_->drone_pos_ = drone_pos;
  ed_->drone_yaw_ = drone_yaw;

  if (ed_->frontiers_.empty()) {
    // ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", There is no Frontiers.");
    return NO_FRONTIER;
  }

  //
  // 1、Find the Coverage Path to Get Next Division
  //

  if (!ed_->find_coverage_) {

    ed_->find_coverage_ = true;

    // Get the candidate Divs
    getCandidateDivs(cur_pos);

    // Get the Next Division with forget Divs
    if (ed_->forget_divs_.size() == 1) {
      // ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", Just one forget Div.");
      for (auto [id, div] : ed_->forget_divs_)
        ed_->next_div_ = div;
      ed_->coverage_path_ = {ed_->ori_div_->center_, ed_->next_div_->unknown_center_, ed_->drone_pos_};
    }

    else if (ed_->forget_divs_.size() > 1) {
      // ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", More than one forget Divs.");
      findCoveragePath(cur_pos, cur_vel, cur_yaw, ed_->forget_divs_, ed_->next_div_, ed_->coverage_path_);
    }

    else {
      // ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", There is no forget Divs.");
      ed_->follow_ = true;
    }

    // double coverage_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO_STREAM("[Explore] Drone " << ep_->drone_id_ << ", Total Time: " << coverage_time);
    // t1 = ros::Time::now();
  }

  //
  // 2、Find the Exploration Path to get the Next Viewpoint
  //

  if (ed_->find_coverage_) {

    ed_->to_viewpoint_ = true;

    // Get the chosen frontier
    if (!ed_->follow_)
      getChosenFrontiers(cur_pos, cur_vel, cur_yaw, ed_->chosen_ftrs_);

    // Get the Next Viewpoint with chosen frontier
    if (ed_->follow_ || ed_->reset_ || ed_->chosen_ftrs_.empty()) {
      ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", No chosen frontier.");
      ed_->follow_ = ed_->reset_ = false;
      ed_->next_pos_ = ed_->drone_pos_;
      ed_->next_yaw_ = ed_->drone_yaw_ + 0.01;
      ed_->exploration_path_ = {cur_pos, ed_->next_pos_};
    }

    else if (ed_->chosen_ftrs_.size() > 1) {
      ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", More than one chosen frontiers.");
      if (findExplorationPath(cur_pos, cur_vel, cur_yaw, ed_->chosen_ftrs_, ed_->next_pos_, ed_->next_yaw_,
                              ed_->exploration_path_) == FAIL)
        return FAIL;
    }

    else if (ed_->chosen_ftrs_.size() == 1) {
      ROS_WARN_STREAM("[Explore] Drone " << ep_->drone_id_ << ", Just one chosen frontier.");
      if (findBestViewpoint(cur_pos, cur_vel, cur_yaw, ed_->chosen_ftrs_.front(), ed_->next_pos_, ed_->next_yaw_) ==
          FAIL)
        return FAIL;
      ed_->exploration_path_ = {cur_pos, ed_->next_pos_, ed_->drone_pos_};
    }

    // double exploration_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO_STREAM("[Explore] Drone " << ep_->drone_id_ << ", Total Time: " << exploration_time);
  }

  // Write the Compute Time into txt
  // ofstream file_write(ep_->compute_dir_, std::ios::app);
  // file_write << "Plan Number:" << ed_->plan_num_++ << " Compute Time:" << 1000 * (ros::Time::now() - t1).toSec() <<
  // endl;

  //
  // 3、Generate Trajectory to Next Viewpoint
  //

  return generateExploreTraj(cur_pos, cur_vel, cur_acc, cur_yaw, ed_->next_pos_, ed_->next_yaw_);
}

// Coverage Path Planning
void FastExplorationManager::getCandidateDivs(const Vector3d &cur_pos) {

  //
  // 1、Get the Original Div
  //

  // Drone
  if (ep_->is_drone_) {
    if (ed_->next_div_->id_ == -1) { // At the beginning
      int cur_id = DivisionUtils::findDivisionID(dp_, cur_pos);
      ed_->ori_div_ = ed_->all_divs_[cur_id];
      ed_->ori_div_->status_ = EXPLORED;
    }

    else {
      ed_->ori_div_ = ed_->next_div_;
      ed_->ori_div_->status_ = EXPLORED;
      ed_->begin_ = false;
    }
  }

  // Ground
  else {
    if (ed_->next_div_->id_ == -1) { // At the beginning
      int cur_id = DivisionUtils::findDivisionID(dp_, cur_pos);
      ed_->ori_div_ = ed_->all_divs_[cur_id];
      ed_->next_div_ = ed_->ori_div_;
    }

    else {
      ed_->ori_div_ = ed_->next_div_;
      ed_->begin_ = false;
    }
  }

  //
  // 2、Get the Candidate Divs
  //

  // Drone
  if (ep_->is_drone_) {
    // Update all the Neighbors Divs of Explored Divs
    for (const auto &[id, div] : ed_->all_divs_) {
      if (div->status_ == EXPLORED) {
        unordered_set<int> ids_nbrs = div->ids_nbrs_eight_;
        for (const int &id : ids_nbrs) {
          shared_ptr<Division> div = ed_->all_divs_[id];
          if (div->status_ == SPARE)
            div->status_ = ACTIVATED;
        }
      }
    }

    // Get the Activated Divs
    ed_->activated_divs_.clear();
    for (const auto &[id, div] : ed_->all_divs_) {
      if (div->status_ == ACTIVATED)
        ed_->activated_divs_.insert({id, div});
    }
  }

  // Ground
  else {
    static int index = 0;
    static unordered_set<int> next_cor_ids = {ed_->next_cor_ids_[index]};

    // Check if finish the 1/4 region
    bool finish = true;
    unordered_map<int, std::shared_ptr<Division>> tmp_divs;
    for (const auto &[id, div] : ed_->all_divs_) {
      if (next_cor_ids.find(div->id_corner_) != next_cor_ids.end()) {
        tmp_divs.insert({id, div});
        if (div->status_ != EXPLORED)
          finish = false;
        if (div->status_ == EXPLORED && !div->ftrs_.empty())
          finish = false;
      }
    }

    // Get the forget Divs
    ed_->forget_divs_.clear();
    if (finish) {
      index = index < 3 ? ++index : index;
      next_cor_ids.insert(ed_->next_cor_ids_[index]);
    } else {
      for (const auto &[id, div] : tmp_divs) {
        if (div->status_ != SPARE && !div->ftrs_.empty())
          ed_->forget_divs_.insert({id, div});
      }
    }
  }
}

void FastExplorationManager::findCoveragePath(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
                                              unordered_map<int, shared_ptr<Division>> divs,
                                              shared_ptr<Division> &next_div, vector<Vector3d> &tour) {

  // The Coverage Path will Travel all candidate Divs

  // 1、Compute Divisions Cost Matrix for ATSP and Solve it with LKH Solver
  MatrixXd cost_mat;
  vector<int> div_ids, ids;
  computeCoverageCostMatrix(cur_pos, cur_vel, cur_yaw, divs, div_ids, cost_mat);
  solveTSP(cost_mat, ids, 0);

  // 2、Get the Next Div and new Div IDs
  vector<int> new_div_ids;
  for (const int &id : ids)
    new_div_ids.push_back(div_ids[id]);
  next_div = divs[new_div_ids[0]];

  // 3、Get the drawing info of Coverage Path
  getCoveragePathVis(divs, new_div_ids, tour);
}

void FastExplorationManager::computeCoverageCostMatrix(const Vector3d &cur_pos, const Vector3d &cur_vel,
                                                       const Vector3d &cur_yaw,
                                                       unordered_map<int, shared_ptr<Division>> divs,
                                                       vector<int> &div_ids, MatrixXd &cost_mat) {

  // Compute the Divisions Cost Matrix for ATSP

  list<shared_ptr<Division>> candidate_divs;
  for (auto &[id, div] : divs) {
    candidate_divs.push_back(div);
    div_ids.push_back(id);
  }

  //
  // 1、Update the Costs between two Candidate Divs
  //

  for (auto &div : candidate_divs)
    div->costs_.clear();
  for (auto it1 = candidate_divs.begin(); it1 != candidate_divs.end(); ++it1) {
    for (auto it2 = it1; it2 != candidate_divs.end(); ++it2) {
      if (it1 == it2)
        (*it1)->costs_.push_back(0);
      else
        updateDivsCost(*it1, *it2);
    }
  }

  //
  // 2、Fill the Cost Matrix of Covearge Path
  //

  // Drone
  if (ep_->is_drone_) {

    int dimen = candidate_divs.size() + 1;
    cost_mat.resize(dimen, dimen);
    cost_mat.setZero();

    // Cost of Ori Div
    int i = 1;
    double w_toward = ed_->begin_ ? 10.0 : 0;
    double div_size = dp_->div_size_;
    for (const auto &div : candidate_divs) { // To Candidate Divs

      // Cost of Toward
      double cost_toward = w_toward * ViewNode::computeCostToward(cur_pos, div->center_, cur_yaw[0]);

      // Cost of Dist
      double cost_dist = 2.0 * div_size * DivisionUtils::getStepBetweenDivs(ed_->ori_div_, div, div_size);

      // Cost of Boundary
      double cost_boundary = div->cost_boundary;

      // Cost of Next Corner Div
      double cost_corner = (ed_->next_corner_div_->id_ != -1)
                               ? div_size * DivisionUtils::getStepBetweenDivs(div, ed_->next_corner_div_, div_size)
                               : 0;

      // Combine
      double cost = cost_toward + cost_dist + cost_boundary + cost_corner;
      // cout << "Coverage Path Cost " << div->id_ << ":" << endl;
      // cout << "next corner div id =  " << ed_->next_corner_div_->id_ << endl;
      // cout << "cost_dist = " << cost_dist << endl;
      // cout << "cost_boundary = " << cost_boundary << endl;
      // cout << "cost_corner = " << cost_corner << endl;
      // cout << "cost = " << cost << endl << endl;

      cost_mat(0, i) = cost;
      cost_mat(i++, 0) = 0; // For ATSP
    }

    // Cost of Candidate Divs
    int j = 1, k = 1;
    for (const auto &div : candidate_divs) { // To other Candidate Divs
      for (const auto &cost : div->costs_) {
        cost_mat(j, k++) = cost;
      }
      ++j;
      k = 1;
    }

    // Cost of Diag
    for (int i = 0; i < dimen; ++i)
      cost_mat(i, i) = 0;
  }

  // Ground
  else {

    int dimen = candidate_divs.size() + 1;
    cost_mat.resize(dimen, dimen);
    cost_mat.setZero();

    // Cost of Ori Div
    int i = 1;
    double div_size = dp_->div_size_;
    for (const auto &div : candidate_divs) { // To Candidate Divs

      // Cost of Dist
      double cost_dist = 2.0 * div_size * DivisionUtils::getStepBetweenDivs(ed_->ori_div_, div, div_size);

      // Cost of Boundary
      double cost_boundary = div->cost_boundary;

      // Combine
      double cost = cost_dist + cost_boundary;
      // cout << "Coverage Path Cost " << div->id_ << ":" << endl;
      // cout << "next corner div id =  " << ed_->next_corner_div_->id_ << endl;
      // cout << "cost_dist = " << cost_dist << endl;
      // cout << "cost_boundary = " << cost_boundary << endl;
      // cout << "cost = " << cost << endl << endl;

      cost_mat(0, i) = cost;
      cost_mat(i++, 0) = 0; // For ATSP
    }

    // Cost of Candidate Divs
    int j = 1, k = 1;
    for (const auto &div : candidate_divs) { // To other Candidate Divs
      for (const auto &cost : div->costs_) {
        cost_mat(j, k++) = cost;
      }
      ++j;
      k = 1;
    }

    // Cost of Diag
    for (int i = 0; i < dimen; ++i)
      cost_mat(i, i) = 0;
  }
}

void FastExplorationManager::updateDivsCost(const shared_ptr<Division> &divi, const shared_ptr<Division> &divj) {

  // Update the cost between Divs

  Vector3d &ceni = divi->unknown_center_;
  Vector3d &cenj = divj->unknown_center_;

  double cost_ij = (ceni - cenj).norm();
  divi->costs_.push_back(cost_ij);
  divj->costs_.push_back(cost_ij);
}

void FastExplorationManager::getCoveragePathVis(unordered_map<int, shared_ptr<Division>> divs, vector<int> new_div_ids,
                                                vector<Vector3d> &tour) {

  // Get the Drawing Info of Coverage Path

  double height = (ep_->box_min_z_ + ep_->box_max_z_) / 2.0;
  Vector3d center = ed_->ori_div_->center_;
  center[2] = height;
  tour = {center};

  for (const int &id : new_div_ids) {
    auto div = divs[id];
    center = div->unknown_center_;
    center[2] = height;
    tour.push_back(center);
  }
}

void FastExplorationManager::getNextCornerDiv(shared_ptr<Division> &next_corner_div) {

  // Get the Next Corner Division for Exploration Path Planning

  const shared_ptr<Division> &next_div = ed_->next_div_;
  const shared_ptr<Division> &corner_div = ed_->all_divs_[next_div->id_corner_];

  static bool first = true, rotation;
  static int first_corner_id, last_corner_id;

  // First time to find the Next Corner Div
  if (first) {

    first = false;

    if (dp_->ids_corner_.find(next_div->id_) == dp_->ids_corner_.end()) { // Next Div is not Corner Div
      next_corner_div = corner_div;
    }

    else { // Next Div is Corner Div
      int corner_id;
      double dist_min = std::numeric_limits<double>::max();
      for (const int &id : dp_->ids_corner_) {
        if (id == next_div->id_)
          continue;
        double dist = (next_div->center_ - ed_->all_divs_[id]->center_).norm();
        if (dist < dist_min) {
          dist = dist_min;
          corner_id = id;
        }
      }
      next_corner_div = ed_->all_divs_[corner_id];
    }

    first_corner_id = last_corner_id = next_corner_div->id_;
    rotation = DivisionUtils::getRotationCornerDiv(ed_->next_div_, next_corner_div, dp_);

    // Pub the Next Corner Div data to Ground
    if (ep_->is_drone_) {
      vector<int> ids_corner(dp_->ids_corner_cw_.begin(), dp_->ids_corner_cw_.end());
      auto it = std::find(ids_corner.begin(), ids_corner.end(), first_corner_id);
      int start_index = std::distance(ids_corner.begin(), it);

      NextCor next_cor_data;
      for (int i = 0; i < 4; i++) {
        int index;
        if (rotation)
          index = (start_index - i + 4) % 4; // Counter Clockwise
        else
          index = (start_index + i) % 4; // Clockwise
        next_cor_data.next_cor_ids.push_back(ids_corner[index]);
      }

      next_cor_pub_.publish(next_cor_data);
    }
  }

  // Continue to find the Next Corner Div
  else if ((next_div->id_ == last_corner_id || next_corner_div->status_ == EXPLORED) && !ed_->loop_) {
    int id = DivisionUtils::getNextCornerDivID(last_corner_id, dp_->ids_corner_cw_, rotation);
    next_corner_div = ed_->all_divs_[id];
    last_corner_id = next_corner_div->id_;
    ed_->loop_ = (first_corner_id == last_corner_id);
  }

  if (ed_->loop_)
    next_corner_div = next_div;
}

// Exploration Path Planning
void FastExplorationManager::getChosenFrontiers(const Vector3d &cur_pos, const Vector3d &cur_vel,
                                                const Vector3d &cur_yaw, vector<Frontier> &ftrs) {

  if (ed_->next_div_->id_ == -1)
    return;

  // Get the Current Div
  int cur_id = DivisionUtils::findDivisionID(dp_, cur_pos);
  ed_->cur_div_ = (cur_id == -1) ? ed_->ori_div_ : ed_->all_divs_[cur_id];

  // Drone
  if (ep_->is_drone_) {

    // Get the basic Divs (Ori Div + Next Div)
    ed_->basic_divs_.clear();
    ed_->basic_divs_.insert({ed_->ori_div_->id_, ed_->ori_div_});
    ed_->basic_divs_.insert({ed_->next_div_->id_, ed_->next_div_});

    // Get the chosen ftrs with basic Divs
    ftrs.clear();
    vector<Vector3d> path;
    double size = dp_->div_size_;

    if (dp_->complete_expl_) { // Complete Explore

      for (const auto &ftr : frontier_finder_->frontiers_) {
        bool pushed = false;
        Viewpoint vp = ftr.vps_.front();
        double cost = ViewNode::searchPath(cur_pos, vp.pos_, path, false) / ViewNode::vm_;

        for (const auto &[id, div] : ed_->basic_divs_) {
          if (ed_->begin_ && id == ed_->ori_div_->id_)
            continue; // No Ori Ftrs at the beginning

          double cost_thr = size * (DivisionUtils::getStepBetweenDivs(ed_->cur_div_, div, size) + 1) / ViewNode::vm_;
          if (ed_->loop_)
            cost_thr = 1000;

          // Ftr average inside || Top Viewpoint inside but Ftr average not inside
          if (!pushed &&
              (DivisionUtils::checkPointInside(ftr.average_, div) || DivisionUtils::checkPointInside(vp.pos_, div))) {
            if (cost < cost_thr) {
              ftrs.push_back(ftr);
              pushed = true;
            }
          }
        }
      }
    }

    else { // Partial Explore

      // Check the Next Div
      for (const auto &[id, div] : ed_->basic_divs_) {
        if (id == ed_->next_div_->id_) {
          double cost_thr =
              std::max(M_PI / ViewNode::yd_,
                       size * (DivisionUtils::getStepBetweenDivs(ed_->cur_div_, div, size) + 1) / ViewNode::vm_);
          if (ed_->loop_)
            cost_thr = 1000;

          // Ftr average inside
          for (const auto &ftr : div->ftrs_) {
            Viewpoint vp = ftr.vps_.front();
            double cost = ViewNode::computeCostLB(cur_pos, vp.pos_, cur_yaw[0], vp.yaw_, cur_vel, 0, path, false);
            if (cost < cost_thr)
              ftrs.push_back(ftr);
          }
        }
      }
    }
  }

  // Ground
  else {
    ed_->basic_divs_.clear();
    ed_->basic_divs_.insert({ed_->ori_div_->id_, ed_->ori_div_});
    ed_->basic_divs_.insert({ed_->next_div_->id_, ed_->next_div_});

    ftrs.clear();
    for (const auto &[id, div] : ed_->basic_divs_) {
      for (const auto &ftr : div->ftrs_) {
        ftrs.push_back(ftr);
      }
    }
  }
}

int FastExplorationManager::findExplorationPath(const Vector3d &cur_pos, const Vector3d &cur_vel,
                                                const Vector3d &cur_yaw, const vector<Frontier> &ftrs,
                                                Vector3d &next_pos, double &next_yaw, vector<Vector3d> &tour) {

  // The Exploration Path will travel all Chosen Viewpoints to reach the Next Div

  // 1、Compute the Exploration Cost Matrix for ATSP and Solve it with LKH Solver
  MatrixXd cost_mat;
  vector<int> ids;
  computeExplCostMatrix(cur_pos, cur_vel, cur_yaw, ftrs, cost_mat);
  solveTSP(cost_mat, ids, 1);

  // 2、Get the best Next Viewpoint from the best Viewpoints Set
  int res = findBestViewpoint(cur_pos, cur_vel, cur_yaw, ftrs[ids[1] - 1], next_pos, next_yaw);

  // 3、Get the drawing info of Exploration Path
  if (res == SUCCEED)
    getExplorationPathVis(cur_pos, ids, ftrs, tour);
  return res;
}

void FastExplorationManager::computeExplCostMatrix(const Vector3d &cur_pos, const Vector3d &cur_vel,
                                                   const Vector3d &cur_yaw, const vector<Frontier> &ftrs,
                                                   MatrixXd &cost_mat) {

  // Compute the Exploration Cost Matrix for ATSP

  if (ftrs.empty())
    return;

  vector<vector<Viewpoint>> vps_sets;
  for (Frontier ftr : ftrs)
    vps_sets.push_back(ftr.vps_);

  vector<Viewpoint> top_vps;
  for (vector<Viewpoint> vps_set : vps_sets)
    top_vps.push_back(vps_set.front());

  //
  // 1、Update the Costs between two Viewpoints
  //

  for (auto &vp : top_vps)
    vp.costs_.clear();
  for (auto it1 = top_vps.begin(); it1 != top_vps.end(); ++it1) {
    for (auto it2 = it1; it2 != top_vps.end(); ++it2) {
      if (it1 == it2)
        it1->costs_.push_back(0);
      else
        updateVpsCost(it1, it2);
    }
  }

  //
  // 2、Fill the Cost Matrix (Virtual Depot, Drone or Ground, Viewpoints, Next Corner Div or Drone)
  //

  // Drone
  if (ep_->is_drone_) {

    int dimen = top_vps.size() + 3;
    cost_mat.resize(dimen, dimen);
    cost_mat.setZero();

    // Cost of Virtual Depot
    cost_mat(0, 1) = -1000;         // To Drone
    cost_mat(dimen - 1, 0) = -1000; // To Next Corner Div

    // Cost of Drone || Ground
    int i = 2;
    for (const auto &vp : top_vps) { // To Viewpoints
      vector<Vector3d> path;
      double cost = ViewNode::computeCostLB(cur_pos, vp.pos_, cur_yaw[0], vp.yaw_, Vector3d(0, 0, 0), 0, path, false);
      cost_mat(1, i) = cost;
      cost_mat(i++, 1) = cost;
    }

    // Cost of Viewpoint
    int j = 2, k = 2;
    for (const auto &vp : top_vps) { // To Viewpoints
      for (const auto &cost : vp.costs_) {
        cost_mat(j, k++) = cost;
      }
      ++j;
      k = 2;
    }

    i = 2;
    for (const auto &vp : top_vps) { // To Next Corner Div
      double cost = (vp.pos_ - ed_->next_corner_div_->center_).norm() / ViewNode::vm_;
      cost_mat(i, dimen - 1) = cost;
      cost_mat(dimen - 1, i++) = cost;
    }

    // Cost of Diag
    for (int i = 0; i < dimen; ++i)
      cost_mat(i, i) = 0;
  }

  // Ground
  else {

    int dimen = top_vps.size() + 3;
    cost_mat.resize(dimen, dimen);
    cost_mat.setZero();

    // Cost of Virtual Depot
    cost_mat(0, 1) = -1000;         // To Ground
    cost_mat(dimen - 1, 0) = -1000; // To Drone

    // Cost of Ground
    int i = 2;
    for (const auto &vp : top_vps) { // To Viewpoints
      vector<Vector3d> path;
      double cost = ViewNode::searchPath(cur_pos, vp.pos_, path, false) / ViewNode::vm_;
      cost_mat(1, i) = cost;
      cost_mat(i++, 1) = cost;
    }

    // Cost of Viewpoint
    int j = 2, k = 2;
    for (const auto &vp : top_vps) { // To Viewpoints
      for (const auto &cost : vp.costs_) {
        cost_mat(j, k++) = cost;
      }
      ++j;
      k = 2;
    }

    i = 2;
    for (const auto &vp : top_vps) { // To Drone
      double cost = (vp.pos_ - ed_->drone_pos_).norm() / ViewNode::vm_;
      cost_mat(i, dimen - 1) = cost;
      cost_mat(dimen - 1, i++) = cost;
    }

    // Cost of Diag
    for (int i = 0; i < dimen; ++i)
      cost_mat(i, i) = 0;
  }
}

void FastExplorationManager::updateVpsCost(const vector<Viewpoint>::iterator &vpi,
                                           const vector<Viewpoint>::iterator &vpj) {

  // Update the cost between Viewpoints

  Vector3d &posi = vpi->pos_;
  Vector3d &posj = vpj->pos_;

  double &yawi = vpi->yaw_;
  double &yawj = vpj->yaw_;

  vector<Vector3d> path;
  double cost_ij;

  if (ep_->is_drone_)
    cost_ij = ViewNode::computeCostLB(posi, posj, yawi, yawj, Vector3d(0, 0, 0), 0, path, false);
  else
    cost_ij = ViewNode::searchPath(posi, posj, path, false) / ViewNode::vm_;

  vpi->costs_.push_back(cost_ij);
  vpj->costs_.push_back(cost_ij);
}

void FastExplorationManager::getExplorationPathVis(const Vector3d &cur_pos, const vector<int> &indices,
                                                   const vector<Frontier> &ftrs, vector<Vector3d> &tour) {
  // Get the Vis of Exploration Path

  if (ftrs.empty())
    return;

  tour = {cur_pos};
  for (int i = 1; i < indices.size() - 1; i++)
    tour.push_back(ftrs[indices[i] - 1].vps_.front().pos_);

  if (ep_->is_drone_)
    tour.push_back(ed_->next_corner_div_->center_);
  else
    tour.push_back(ed_->drone_pos_);
}

// Greedy Explore
int FastExplorationManager::greedyExplore(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
                                          const vector<Frontier> &ftrs, Vector3d &next_pos, double &next_yaw) {

  if (ftrs.empty())
    return FAIL;

  vector<vector<Viewpoint>> vps_sets;
  for (Frontier ftr : ftrs)
    vps_sets.push_back(ftr.vps_);

  vector<Viewpoint> top_vps;
  for (vector<Viewpoint> vps_set : vps_sets)
    top_vps.push_back(vps_set.front());

  // 1、Find the best ftr
  int index = 0;
  vector<Vector3d> path;
  double cost_min = std::numeric_limits<double>::max();
  double w_next = ed_->next_div_->id_ != -1 ? 1 : 0;

  for (int i = 0; i < ftrs.size(); i++) {

    // Cost of Viewpoint
    double cost_vp = ViewNode::searchPath(cur_pos, top_vps[i].pos_, path, false) / ViewNode::vm_;

    // Cost of Next Div
    double cost_next = w_next * (top_vps[i].pos_ - ed_->next_div_->center_).norm() / ViewNode::vm_;

    // Combine
    double cost = cost_vp + cost_next;
    // cout << "Greedy 1th Cost: " << endl;
    // cout << "cost_vp = " << cost_vp << endl;
    // cout << "cost_next = " << cost_next << endl;
    // cout << "cost = " << cost << endl << endl;

    if (cost < cost_min) {
      cost_min = cost;
      index = i;
    }
  }

  // 2、Check if need go to the center of next Div
  if (ed_->next_div_->id_ != -1) {
    double size = dp_->div_size_;
    double cost_thr = dp_->div_inflation_ * size *
                      (DivisionUtils::getStepBetweenDivs(ed_->ori_div_, ed_->next_div_, size) + 1) / ViewNode::vm_;
    if (ed_->loop_)
      cost_thr = 1000;

    if (cost_min > cost_thr) {
      ed_->to_viewpoint_ = false;
      Vector3d next_center;
      DivisionUtils::getValidCenter(ed_->next_div_, next_center);
      next_pos = next_center;
      return SUCCEED;
    }
  }

  // 3、Find the best vp from best ftr
  return findBestViewpoint(cur_pos, cur_vel, cur_yaw, ftrs[index], next_pos, next_yaw);
}

int FastExplorationManager::findBestViewpoint(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
                                              const Frontier &ftr, Vector3d &next_pos, double &next_yaw) {

  // Find the best vp from best ftr

  vector<Viewpoint> vps_set = ftr.vps_;
  if (vps_set.empty())
    return FAIL;

  if (dp_->complete_expl_ || !ep_->is_drone_) { // Complete Explore or Ground
    next_pos = vps_set[0].pos_;
    next_yaw = vps_set[0].yaw_;
  }

  else { // Partial Explore

    int index = 0;
    double cost_min = std::numeric_limits<double>::max();
    Vector3d toward_std = (ftr.average_ - vps_set.front().pos_).normalized();

    for (int i = 0; i < vps_set.size(); i++) {

      // Cost of vel consistency
      double cost_con = ViewNode::computeCostCon(cur_yaw[0], vps_set[i].yaw_, cur_vel);

      // Cost of offset top viewpoint
      double cost_offset = 2.0 * acos(toward_std.dot((ftr.average_ - vps_set[i].pos_).normalized()));

      // Combine
      double cost = cost_con + cost_offset;
      // cout << "Greedy 2nd Cost: " << endl;
      // cout << "cost_con = " << cost_con << endl;
      // cout << "cost_offset = " << cost_offset << endl;
      // cout << "cost = " << cost << endl << endl;
      if (cost < cost_min) {
        cost_min = cost;
        index = i;
      }
    }

    next_pos = vps_set[index].pos_;
    next_yaw = vps_set[index].yaw_;
  }

  return SUCCEED;
}

//
// RACER
//

int FastExplorationManager::racerExplore(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
                                         const Vector3d &yaw) {

  ros::Time t1 = ros::Time::now();
  auto t2 = t1;

  //
  // 1、Coverage Path Planning -> Exploration Path Planning
  //

  ed_->frontier_tour_.clear();
  vector<int> grid_ids, frontier_ids;
  findGridAndFrontierPath(pos, vel, yaw, grid_ids, frontier_ids);

  //
  // 2、Retrieve the exploration path to get the Next Viewpoint
  //

  if (grid_ids.empty()) {
    // No grid is assigned to ego drone, but keep moving if necessary
    ROS_WARN("[Explore] Empty grid");
    return NO_GRID;

    // Move to the closest targets
    // double min_cost = 100000;
    // int min_cost_id = -1;
    // vector<Vector3d> tmp_path;
    // for (int i = 0; i < ed_->averages_.size(); ++i) {
    //   auto tmp_cost =
    //       ViewNode::computeCost(pos, ed_->points_[i], yaw[0], ed_->yaws_[i], vel, yaw[1], tmp_path);
    //   if (tmp_cost < min_cost) {
    //     min_cost = tmp_cost;
    //     min_cost_id = i;
    //   }
    // }
    // ed_->next_pos_ = ed_->points_[min_cost_id];
    // ed_->next_yaw_ = ed_->yaws_[min_cost_id];
  }

  else if (frontier_ids.size() == 0) {
    // The assigned grid contains no frontier, find the one closest to the grid
    ROS_WARN("[Explore] No frontier in grid");

    Eigen::Vector3d grid_center = ed_->grid_tour_[1];

    double min_cost = 100000;
    int min_cost_id = -1;
    for (int i = 0; i < ed_->top_points_.size(); ++i) {
      // double cost = (grid_center - ed_->averages_[i]).norm();
      vector<Eigen::Vector3d> path;
      double cost = ViewNode::computeCostLB(grid_center, ed_->averages_[i], 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
      if (cost < min_cost) {
        min_cost = cost;
        min_cost_id = i;
      }
    }
    ed_->next_pos_ = ed_->top_points_[min_cost_id];
    ed_->next_yaw_ = ed_->top_yaws_[min_cost_id];

    // // Simply go to the center of the unknown grid
    // ed_->next_pos_ = grid_center;
    // Eigen::Vector3d dir = grid_center - pos;
    // ed_->next_yaw_ = atan2(dir[1], dir[0]);

  }

  else if (frontier_ids.size() == 1) {
    // The assigned grid contains no frontier, find the one closest to the grid
    ROS_WARN("[Explore] Single frontier");

    if (ep_->refine_local_) {
      // Single frontier, find the min cost viewpoint for it
      ed_->refined_ids_ = {frontier_ids[0]};
      ed_->unrefined_points_ = {ed_->top_points_[frontier_ids[0]]};
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(pos, {frontier_ids[0]}, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_,
                                          n_yaws);

      if (grid_ids.size() <= 1) {
        // Only one grid is assigned
        double min_cost = 100000;
        int min_cost_id = -1;
        vector<Vector3d> tmp_path;
        for (int i = 0; i < ed_->n_points_[0].size(); ++i) {
          auto tmp_cost =
              ViewNode::computeCostLB(pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
          if (tmp_cost < min_cost) {
            min_cost = tmp_cost;
            min_cost_id = i;
          }
        }
        ed_->next_pos_ = ed_->n_points_[0][min_cost_id];
        ed_->next_yaw_ = n_yaws[0][min_cost_id];
      }

      else {
        // More than one grid, the next grid is considered for path planning
        // vector<Eigen::Vector3d> grid_pos = { ed_->grid_tour_[2] };
        // Eigen::Vector3d dir = ed_->grid_tour_[2] - ed_->grid_tour_[1];
        // vector<double> grid_yaw = { atan2(dir[1], dir[0]) };

        Eigen::Vector3d grid_pos;
        double grid_yaw;
        if (hgrid_->getNextGrid(grid_ids, grid_pos, grid_yaw)) {
          ed_->n_points_.push_back({grid_pos});
          n_yaws.push_back({grid_yaw});
        }

        ed_->refined_points_.clear();
        ed_->refined_views_.clear();
        vector<double> refined_yaws;
        refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
        ed_->next_pos_ = ed_->refined_points_[0];
        ed_->next_yaw_ = refined_yaws[0];
      }
      ed_->refined_points_ = {ed_->next_pos_};
      ed_->refined_views_ = {ed_->next_pos_ + 2.0 * Vector3d(cos(ed_->next_yaw_), sin(ed_->next_yaw_), 0)};
    }
  }

  else {
    // More than two frontiers are assigned
    // Do refinement for the next few viewpoints in the global tour
    ROS_WARN("[Explore] Multiple Frontiers.");

    t1 = ros::Time::now();

    ed_->refined_ids_.clear();
    ed_->unrefined_points_.clear();
    int knum = min(int(frontier_ids.size()), ep_->refined_num_);
    for (int i = 0; i < knum; ++i) {
      auto tmp = ed_->top_points_[frontier_ids[i]];
      ed_->unrefined_points_.push_back(tmp);
      ed_->refined_ids_.push_back(frontier_ids[i]);
      if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2)
        break;
    }

    // Get top N viewpoints for the next K frontiers
    ed_->n_points_.clear();
    vector<vector<double>> n_yaws;
    frontier_finder_->getViewpointsInfo(pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_,
                                        n_yaws);

    ed_->refined_points_.clear();
    ed_->refined_views_.clear();
    vector<double> refined_yaws;
    refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
    ed_->next_pos_ = ed_->refined_points_[0];
    ed_->next_yaw_ = refined_yaws[0];

    // Get marker for view visualization
    for (int i = 0; i < ed_->refined_points_.size(); ++i) {
      Vector3d view = ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
      ed_->refined_views_.push_back(view);
    }
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();
    for (int i = 0; i < ed_->refined_points_.size(); ++i) {
      vector<Vector3d> v1, v2;
      frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
      frontier_finder_->percep_utils_->getFOV(v1, v2);
      ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
      ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
    }

    // double local_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO("Local refine time: %lf", local_time);
  }

  // Write the Compute Time into txt
  ofstream file_write(ep_->compute_dir_, std::ios::app);
  file_write << "Plan Number:" << ed_->plan_num_++ << " Compute Time:" << 1000 * (ros::Time::now() - t1).toSec()
             << endl;

  //
  // 3、Generate Trajectory to the Next Viewpoint
  //

  if (generateExploreTraj(pos, vel, acc, yaw, ed_->next_pos_, ed_->next_yaw_) == FAIL)
    return FAIL;

  double total = (ros::Time::now() - t2).toSec();
  // ROS_INFO("Total time: %lf", total);
  // ROS_ERROR_COND(total > 0.1, "Total time too long!!!");
  return SUCCEED;
}

void FastExplorationManager::findGridAndFrontierPath(const Vector3d &cur_pos, const Vector3d &cur_vel,
                                                     const Vector3d &cur_yaw, vector<int> &grid_ids,
                                                     vector<int> &frontier_ids) {

  //
  // Find the Optimal Coordinated Tour for quadrotor swarm
  //

  auto t1 = ros::Time::now();

  // Select nearby drones according to their states stamp
  vector<Eigen::Vector3d> positions = {cur_pos};
  vector<Eigen::Vector3d> velocities = {cur_vel};
  vector<double> yaws = {cur_yaw[0]};

  //
  // 1、Partitioning-based Coverage Path Planning
  //

  vector<int> ego_ids;           // IDs of Ego Drone HGrid
  vector<vector<int>> other_ids; // IDs of Other Drones HGrid
  if (!findGlobalTourOfGrid(positions, velocities, ego_ids, other_ids)) {
    grid_ids = {};
    return;
  }
  grid_ids = ego_ids; // Order of travel allocated HGrids (Coverage Path)

  double grid_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  //
  // 2、Frontier-based Exploration Path Planning
  //

  // Restrict frontier within the first visited grid
  vector<int> ftr_ids;
  // uniform_grid_->getFrontiersInGrid(ego_ids[0], ftr_ids);
  hgrid_->getFrontiersInGrid(ego_ids, ftr_ids);
  // ROS_INFO("Find frontier tour, %d involved.", ftr_ids.size());

  if (ftr_ids.empty()) {
    frontier_ids = {};
    return;
  }

  // Consider next grid in the exploration path
  Eigen::Vector3d grid_pos;
  double grid_yaw;
  vector<Eigen::Vector3d> grid_pos_vec;
  if (hgrid_->getNextGrid(ego_ids, grid_pos, grid_yaw)) {
    grid_pos_vec = {grid_pos};
  }

  // Do Exploration Path Planning
  findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr_ids, grid_pos_vec, frontier_ids);
  double ftr_time = (ros::Time::now() - t1).toSec();

  // ROS_INFO("Grid tour t: %lf, frontier tour t: %lf.", grid_time, ftr_time);
}

// Coverage Path Planning
bool FastExplorationManager::findGlobalTourOfGrid(const vector<Eigen::Vector3d> &positions,
                                                  const vector<Eigen::Vector3d> &velocities, vector<int> &indices,
                                                  vector<vector<int>> &others, bool init) {

  // Find the Coverage Path to travel allocated HGrids

  auto t1 = ros::Time::now();

  //
  // 1、Update and Allocate the Hgrid for ego-drone
  //

  hgrid_->inputFrontiers(ed_->averages_);
  auto &grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;

  vector<int> first_ids, second_ids;
  hgrid_->updateGridsData(ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_, first_ids, second_ids);
  if (grid_ids.empty()) {
    ROS_WARN("Empty dominance.");
    ed_->grid_tour_.clear();
    return false;
  }

  // Compute the Cost Matrix between Allocated Hgrid
  Eigen::MatrixXd mat;
  if (!init)
    hgrid_->getCostMatrix(positions, velocities, {first_ids}, {second_ids}, grid_ids, mat);
  else
    hgrid_->getCostMatrix(positions, velocities, {{}}, {{}}, grid_ids, mat);
  const int dimension = mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // cout << "Allocated grid: ";
  // for (auto id : grid_ids) cout << id << ", ";
  // cout << endl;

  //
  // 2、Plan the Coverage Path with ATSP or CVRP
  //

  vector<int> ids;
  const int drone_num = 1;

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  file.open(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  // file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) <<
  // "\n"; file << "MTSP_MAX_SIZE = "
  //      << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 2;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return false;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();
  // std::cout << "AmTSP time: " << mtsp_time << std::endl;

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1)
      break;
  }
  fin.close();

  // cout << "Planned grid: ";
  // for (auto id : ids) cout << id << ", ";
  // cout << endl;

  //
  // 3、Parse the m-tour of grid
  //

  vector<int> tour;
  vector<vector<int>> tours;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0)
      tours.push_back(tour);
    else
      tour.push_back(id);
  }

  // for (auto tr : tours) {
  //   std::cout << "tour: ";
  //   for (auto id : tr) std::cout << id << ", ";
  //   std::cout << "" << std::endl;
  // }

  others.resize(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    }

    else {
      others[tours[i][0] - 2].insert(others[tours[i][0] - 2].end(), tours[i].begin(), tours[i].end());
    }
  }

  for (auto &id : indices) {
    id -= 1 + drone_num;
  }

  for (auto &other : others) {
    for (auto &id : other)
      id -= 1 + drone_num;
  }

  // std::cout << "Grid tour: ";
  for (auto &id : indices) {
    id = grid_ids[id];
    // std::cout << id << ", ";
  }
  // std::cout << "" << std::endl;

  // uniform_grid_->getGridTour(indices, ed_->grid_tour_);
  grid_ids = indices;
  hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_);

  ed_->last_grid_ids_ = grid_ids;
  ed_->reallocated_ = false;

  // hgrid_->checkFirstGrid(grid_ids.front());
  return true;
}

// Exploration Path Planning
void FastExplorationManager::findTourOfFrontier(const Vector3d &cur_pos, const Vector3d &cur_vel,
                                                const Vector3d &cur_yaw, const vector<int> &ftr_ids,
                                                const vector<Eigen::Vector3d> &grid_pos, vector<int> &indices) {

  // Find the Exploration Path to travel chosen viewpoints and Reach the next grid

  auto t1 = ros::Time::now();

  //
  // 1、Get the Swarm Cost Matrix to Find optimal allocation through mATSP
  //

  vector<Eigen::Vector3d> positions = {cur_pos};
  vector<Eigen::Vector3d> velocities = {cur_vel};
  vector<double> yaws = {cur_yaw[0]};

  Eigen::MatrixXd mat;
  frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, ftr_ids, grid_pos, mat);
  const int dimension = mat.rows();
  // std::cout << "dim of frontier TSP mat: " << dimension << std::endl;

  double mat_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("mat time: %lf", mat_time);

  t1 = ros::Time::now();

  //
  // 2、Create problem file and par file
  //

  ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  const int drone_num = 1;
  file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
  file << "MTSP_MAX_SIZE = " << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  //
  // 3、Call the LKH to solve ATSP
  //

  auto par_dir = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 1;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("AmTSP time: %lf", mtsp_time);

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1)
      break;
  }
  fin.close();

  //
  // 3、Parse the m-tour
  //

  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  vector<vector<int>> others(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    }
    // else {
    //   others[tours[i][0] - 2].insert(
    //       others[tours[i][0] - 2].end(), tours[i].begin() + 1, tours[i].end());
    // }
  }
  for (auto &id : indices) {
    id -= 1 + drone_num;
  }
  // for (auto& other : others) {
  //   for (auto& id : other)
  //     id -= 1 + drone_num;
  // }

  if (ed_->grid_tour_.size() > 2) { // Remove id for next grid, since it is considered in the TSP
    indices.pop_back();
  }
  // Subset of frontier inside first grid
  for (int i = 0; i < indices.size(); ++i) {
    indices[i] = ftr_ids[indices[i]];
  }

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
  if (!grid_pos.empty()) {
    ed_->frontier_tour_.push_back(grid_pos[0]);
  }

  // ed_->other_tours_.clear();
  // for (int i = 1; i < positions.size(); ++i) {
  //   ed_->other_tours_.push_back({});
  //   frontier_finder_->getPathForTour(positions[i], others[i - 1], ed_->other_tours_[i - 1]);
  // }

  double parse_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("Cost mat: %lf, TSP: %lf, parse: %f, %d frontiers assigned.", mat_time, mtsp_time,
  //     parse_time, indices.size());
}

// Refine Viewpoints
void FastExplorationManager::refineLocalTour(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
                                             const vector<vector<Vector3d>> &n_points,
                                             const vector<vector<double>> &n_yaws, vector<Vector3d> &refined_pts,
                                             vector<double> &refined_yaws) {

  // Refine local tour for next few frontiers, using more diverse viewpoints

  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  // std::cout << "Local refine graph size: 1, ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group)
        g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    // std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  // std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

//
void FastExplorationManager::allocateGrids(const vector<Eigen::Vector3d> &positions,
                                           const vector<Eigen::Vector3d> &velocities,
                                           const vector<vector<int>> &first_ids, const vector<vector<int>> &second_ids,
                                           const vector<int> &grid_ids, vector<int> &ego_ids, vector<int> &other_ids) {

  // ROS_INFO("Allocate grid.");

  auto t1 = ros::Time::now();
  auto t2 = t1;

  if (grid_ids.size() == 1) { // Only one grid, no need to run ACVRP
    auto pt = hgrid_->getCenter(grid_ids.front());
    // double d1 = (positions[0] - pt).norm();
    // double d2 = (positions[1] - pt).norm();
    vector<Eigen::Vector3d> path;
    double d1 = ViewNode::computeCostLB(positions[0], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
    double d2 = ViewNode::computeCostLB(positions[1], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
    if (d1 < d2) {
      ego_ids = grid_ids;
      other_ids = {};
    } else {
      ego_ids = {};
      other_ids = grid_ids;
    }
    return;
  }

  Eigen::MatrixXd mat;
  // uniform_grid_->getCostMatrix(positions, velocities, prev_first_ids, grid_ids, mat);
  hgrid_->getCostMatrix(positions, velocities, first_ids, second_ids, grid_ids, mat);

  // int unknown = hgrid_->getTotalUnknwon();
  int unknown;

  double mat_time = (ros::Time::now() - t1).toSec();

  // Find optimal path through AmTSP
  t1 = ros::Time::now();
  const int dimension = mat.rows();
  const int drone_num = positions.size();

  vector<int> unknown_nums;
  int capacity = 0;
  for (int i = 0; i < grid_ids.size(); ++i) {
    int unum = hgrid_->getUnknownCellsNum(grid_ids[i]);
    unknown_nums.push_back(unum);
    capacity += unum;
    // std::cout << "Grid " << i << ": " << unum << std::endl;
  }
  // std::cout << "Total: " << capacity << std::endl;
  capacity = capacity * 0.75 * 0.1;

  // int prob_type;
  // if (grid_ids.size() >= 3)
  //   prob_type = 2;  // Use ACVRP
  // else
  //   prob_type = 1;  // Use AmTSP

  const int prob_type = 2;

  // Create problem file--------------------------
  ofstream file(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : pairopt\n";

  if (prob_type == 1)
    file << "TYPE : ATSP\n";
  else if (prob_type == 2)
    file << "TYPE : ACVRP\n";

  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";

  if (prob_type == 2) {
    file << "CAPACITY : " + to_string(capacity) + "\n";  // ACVRP
    file << "VEHICLES : " + to_string(drone_num) + "\n"; // ACVRP
  }

  // Cost matrix
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }

  if (prob_type == 2) { // Demand section, ACVRP only
    file << "DEMAND_SECTION\n";
    file << "1 0\n";
    for (int i = 0; i < drone_num; ++i) {
      file << to_string(i + 2) + " 0\n";
    }
    for (int i = 0; i < grid_ids.size(); ++i) {
      int grid_unknown = unknown_nums[i] * 0.1;
      file << to_string(i + 2 + drone_num) + " " + to_string(grid_unknown) + "\n";
    }
    file << "DEPOT_SECTION\n";
    file << "1\n";
    file << "EOF";
  }

  file.close();

  // Create par file------------------------------------------
  int min_size = int(grid_ids.size()) / 2;
  int max_size = ceil(int(grid_ids.size()) / 2.0);
  file.open(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp\n";
  if (prob_type == 1) {
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    // file << "MTSP_OBJECTIVE = MINMAX\n";
    file << "MTSP_MIN_SIZE = " << to_string(min_size) << "\n";
    file << "MTSP_MAX_SIZE = " << to_string(max_size) << "\n";
    file << "TRACE_LEVEL = 0\n";
  } else if (prob_type == 2) {
    file << "TRACE_LEVEL = 1\n"; // ACVRP
    file << "SEED = 0\n";        // ACVRP
  }
  file << "RUNS = 1\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour\n";

  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 3;
  // if (!tsp_client_.call(srv)) {
  if (!acvrp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ACVRP.");
    return;
  }
  // system("/home/boboyu/software/LKH-3.0.6/LKH
  // /home/boboyu/workspaces/hkust_swarm_ws/src/swarm_exploration/utils/lkh_mtsp_solver/resource/amtsp3_1.par");

  double mtsp_time = (ros::Time::now() - t1).toSec();
  std::cout << "Allocation time: " << mtsp_time << std::endl;

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1)
      break;
  }
  fin.close();

  // Parse the m-tour of grid
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }
  // // Print tour ids
  // for (auto tr : tours) {
  //   std::cout << "tour: ";
  //   for (auto id : tr) std::cout << id << ", ";
  //   std::cout << "" << std::endl;
  // }

  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      ego_ids.insert(ego_ids.end(), tours[i].begin() + 1, tours[i].end());
    } else {
      other_ids.insert(other_ids.end(), tours[i].begin() + 1, tours[i].end());
    }
  }
  for (auto &id : ego_ids) {
    id = grid_ids[id - 1 - drone_num];
  }
  for (auto &id : other_ids) {
    id = grid_ids[id - 1 - drone_num];
  }
  // // Remove repeated grid
  // unordered_map<int, int> ego_map, other_map;
  // for (auto id : ego_ids) ego_map[id] = 1;
  // for (auto id : other_ids) other_map[id] = 1;

  // ego_ids.clear();
  // other_ids.clear();
  // for (auto p : ego_map) ego_ids.push_back(p.first);
  // for (auto p : other_map) other_ids.push_back(p.first);

  // sort(ego_ids.begin(), ego_ids.end());
  // sort(other_ids.begin(), other_ids.end());
}

//
double FastExplorationManager::computeGridPathCost(const Eigen::Vector3d &pos, const vector<int> &grid_ids,
                                                   const vector<int> &first, const vector<vector<int>> &firsts,
                                                   const vector<vector<int>> &seconds, const double &w_f) {

  if (grid_ids.empty())
    return 0.0;

  double cost = 0.0;
  vector<Eigen::Vector3d> path;
  cost += hgrid_->getCostDroneToGrid(pos, grid_ids[0], first);
  for (int i = 0; i < grid_ids.size() - 1; ++i) {
    cost += hgrid_->getCostGridToGrid(grid_ids[i], grid_ids[i + 1], firsts, seconds, firsts.size());
  }
  return cost;
}

//
// FUEL
//

int FastExplorationManager::fuelExpore(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_acc,
                                       const Vector3d &cur_yaw) {

  ros::Time t1 = ros::Time::now();

  if (ed_->frontiers_.empty()) {
    ROS_WARN("[Explore] There is no Frontiers.");
    return NO_FRONTIER;
  }

  //
  // 1、Find the Global Tour to get the Next Viewpoint
  //

  ed_->exploration_path_.clear();

  if (ed_->frontiers_.size() > 1) {
    vector<int> index;
    findGlobalTour(cur_pos, cur_vel, cur_yaw, index);

    if (ep_->refine_local_) {
      t1 = ros::Time::now();

      ed_->refined_ids_.clear();
      ed_->unrefined_points_.clear();

      int knum = min(int(index.size()), ep_->refined_num_);

      // Get unrefined top viewpoints set and record their index
      for (int i = 0; i < knum; ++i) {
        auto tmp = ed_->top_points_[index[i]];
        ed_->unrefined_points_.push_back(tmp);
        ed_->refined_ids_.push_back(index[i]);

        // Once the distance more than refined_radius_ and have two more unrefined top vps, exit loop
        if ((tmp - cur_pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2)
          break;
      }

      // Get the positon and yaw set of viewpoints to refine for each clusters
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(cur_pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_,
                                          ed_->n_points_, n_yaws);

      // Start Refine
      ed_->refined_points_.clear();
      ed_->refined_views_.clear();
      vector<double> refined_yaws;
      refineLocalTour(cur_pos, cur_vel, cur_yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);

      // Get next_pos and next_yaw as the target
      ed_->next_pos_ = ed_->refined_points_[0];
      ed_->next_yaw_ = refined_yaws[0];

      // Get marker for view visualization
      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        Vector3d view = ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
        ed_->refined_views_.push_back(view);
      }

      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();

      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        vector<Vector3d> v1, v2;
        frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
        frontier_finder_->percep_utils_->getFOV(v1, v2);
        ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
        ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
      }

      double local_time = (ros::Time::now() - t1).toSec();
      // ROS_WARN("Local refine time: %lf", local_time);
    }

    else {
      ed_->next_pos_ = ed_->top_points_[index[0]];
      ed_->next_yaw_ = ed_->top_yaws_[index[0]];
    }
  }

  else if (ed_->frontiers_.size() == 1) {
    ed_->exploration_path_ = {cur_pos, ed_->top_points_[0]};
    ed_->refined_tour_.clear();
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();

    if (ep_->refine_local_) {
      ed_->refined_ids_ = {0};
      ed_->unrefined_points_ = {ed_->top_points_[0]};
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;

      double min_cost = 100000;
      int min_cost_id = -1;
      vector<Vector3d> tmp_path;

      frontier_finder_->getViewpointsInfo(cur_pos, {0}, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      // Choose the min cost of viewpoint of only frontier cluster
      for (int i = 0; i < ed_->n_points_[0].size(); ++i) {
        // Compute the cost from current position to each vps of only frontier cluster
        auto tmp_cost = ViewNode::computeCostLB(cur_pos, ed_->n_points_[0][i], cur_yaw[0], n_yaws[0][i], cur_vel,
                                                cur_yaw[1], tmp_path);

        if (tmp_cost < min_cost) {
          min_cost = tmp_cost;
          min_cost_id = i;
        }
      }

      // Get the next_pos and next_yaw as target
      ed_->next_pos_ = ed_->n_points_[0][min_cost_id];
      ed_->next_yaw_ = n_yaws[0][min_cost_id];

      ed_->refined_points_ = {ed_->next_pos_};
      ed_->refined_views_ = {ed_->next_pos_ + 2.0 * Vector3d(cos(ed_->next_yaw_), sin(ed_->next_yaw_), 0)};
    }

    else {
      ed_->next_pos_ = ed_->top_points_[0];
      ed_->next_yaw_ = ed_->top_yaws_[0];
    }
  }

  else {
    ROS_ERROR("Empty destination.");
  }

  // Write the Compute Time into txt
  ofstream file_write(ep_->compute_dir_, std::ios::app);
  file_write << "Plan Number:" << ed_->plan_num_++ << " Compute Time:" << 1000 * (ros::Time::now() - t1).toSec()
             << endl;

  //
  // 2、Generate Trajectory to Next Viewpoint
  //

  return generateExploreTraj(cur_pos, cur_vel, cur_acc, cur_yaw, ed_->next_pos_, ed_->next_yaw_);
}

void FastExplorationManager::findGlobalTour(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
                                            vector<int> &indices) {
  //
  // Find the Global Tour
  //

  // 1、Get the cost matrix of ATSP
  auto t1 = ros::Time::now();
  Eigen::MatrixXd cost_mat;

  frontier_finder_->updateFrontierCostMatrix();
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // 2、Call the LKH Solver
  solveTSP(cost_mat, indices, 1);
  double tsp_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // 3、Get the vis of global tour
  getGlobalTourVis(cur_pos, indices, ed_->exploration_path_);
}

void FastExplorationManager::getGlobalTourVis(const Vector3d &cur_pos, const vector<int> &indices,
                                              vector<Vector3d> &tour) {

  //
  // Vis the Exploration Path
  //

  if (indices.empty())
    return;

  // Make an frontier_indexer to access the frontier list easier
  vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = frontier_finder_->frontiers_.begin(); it != frontier_finder_->frontiers_.end(); ++it)
    frontier_indexer.push_back(it);

  // Get the Global tour with straight line
  tour = {cur_pos};
  for (int i = 0; i < indices.size(); i++)
    tour.push_back(frontier_indexer[indices[i]]->vps_.front().pos_);

  // Get the Global tour with A* path
  // vector<Vector3d> segment;
  // ViewNode::searchPath(cur_pos, frontier_indexer[indices[0]]->vps_.front().pos_, segment);
  // tour.insert(tour.end(), segment.begin(), segment.end());

  // for (int i = 0; i < indices.size() - 1; ++i) {
  //   // Move to path to next cluster
  //   auto path_iter = frontier_indexer[indices[i]]->paths_.begin();
  //   int next_idx = indices[i + 1];
  //   for (int j = 0; j < next_idx; ++j) ++path_iter;
  //   tour.insert(tour.end(), path_iter->begin(), path_iter->end());
  // }
}

//
// Greedy (Classic)
//

int FastExplorationManager::classicExpore(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_acc,
                                          const Vector3d &cur_yaw) {

  ros::Time t1 = ros::Time::now();

  if (ed_->frontiers_.empty()) {
    ROS_WARN("[Explore] There is no Frontiers.");
    return NO_FRONTIER;
  }

  //
  // 1、Find the best ftr
  //

  vector<vector<Viewpoint>> vps_sets;
  for (Frontier ftr : ed_->ftrs_)
    vps_sets.push_back(ftr.vps_);

  vector<Viewpoint> top_vps;
  for (vector<Viewpoint> vps_set : vps_sets)
    top_vps.push_back(vps_set.front());

  int index = 0;
  vector<Vector3d> path;
  double cost_min = std::numeric_limits<double>::max();

  for (int i = 0; i < top_vps.size(); i++) {
    // double cost = ViewNode::searchPath(cur_pos, top_vps[i].pos_, path, false);
    double cost =
        ViewNode::computeCostLB(cur_pos, top_vps[i].pos_, cur_yaw[0], top_vps[i].yaw_, cur_vel, 0, path, false);
    // cout << "Classic 1th Cost: " << endl;
    // cout << "cost = " << cost << endl << endl;
    if (cost < cost_min) {
      cost_min = cost;
      index = i;
    }
  }

  Frontier best_ftr = ed_->ftrs_[index];
  // ed_->next_pos_ = best_ftr.vps_.front().pos_;
  // ed_->next_yaw_ = best_ftr.vps_.front().yaw_;

  //
  // 2、Find the best vp from best ftr
  //

  index = 0;
  cost_min = std::numeric_limits<double>::max();
  Vector3d toward_std = (best_ftr.average_ - best_ftr.vps_.front().pos_).normalized();

  for (int i = 0; i < best_ftr.vps_.size(); i++) {

    // Cost of vel consistency
    double cost_con = ViewNode::computeCostCon(cur_yaw[0], best_ftr.vps_[i].yaw_, cur_vel);

    // Cost of offset top viewpoint
    double cost_offset = 2.0 * acos(toward_std.dot((best_ftr.average_ - best_ftr.vps_[i].pos_).normalized()));

    // Combine
    double cost = cost_con + cost_offset;
    // cout << "Classic 2nd Cost: " << endl;
    // cout << "cost_con = " << cost_con << endl << endl;
    // cout << "cost_offset = " << cost_offset << endl << endl;
    // cout << "cost = " << cost << endl << endl;
    if (cost < cost_min) {
      cost_min = cost;
      index = i;
    }
  }

  ed_->next_pos_ = best_ftr.vps_[index].pos_;
  ed_->next_yaw_ = best_ftr.vps_[index].yaw_;

  // Write the Compute Time into txt
  ofstream file_write(ep_->compute_dir_, std::ios::app);
  file_write << "Plan Number:" << ed_->plan_num_++ << " Compute Time:" << 1000 * (ros::Time::now() - t1).toSec()
             << endl;

  //
  // 3、Generate Trajectory to Next Viewpoint
  //

  return generateExploreTraj(cur_pos, cur_vel, cur_acc, cur_yaw, ed_->next_pos_, ed_->next_yaw_);
}

//
// Helper
//

void FastExplorationManager::updateExplProcess() {

  //
  // Update exploration process
  //

  // Update the Coverage of Exploration
  double res = sdf_map_->getResolution();
  ed_->unknown_number_ = 0;
  for (double x = ep_->box_min_[0]; x <= ep_->box_max_[0]; x += res) {
    for (double y = ep_->box_min_[1]; y <= ep_->box_max_[1]; y += res) {
      for (double z = ep_->box_min_[2]; z <= ep_->box_max_[2]; z += res) {
        Eigen::Vector3d pos(x, y, z);
        if (sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN)
          ed_->unknown_number_ += 1;
      }
    }
  }
  ed_->coverage_ = double(ep_->voxel_number - ed_->unknown_number_) / double(ep_->voxel_number);

  // Write the Expl Process into txt
  if (ed_->triggered_ && !ed_->finish_partial_) {
    double time_now = (ros::Time::now() - ep_->trigger_time_).toSec();
    ofstream file_write(ep_->process_dir_, std::ios::app);
    file_write << "Time:" << time_now << " Coverage:" << ed_->coverage_ << endl;
  }

  // ROS_WARN_THROTTLE(1.0, "[FIS] Total Number: %d, Unknown Number: %d, Coverage Rate: %lf",
  //                   ep_->voxel_number, ed_->unknown_number_, ed_->coverage_);
}

void FastExplorationManager::shortenPath(vector<Vector3d> &path) {

  //
  // Shorten the init path of trajectory
  //

  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }

  // Reserve the critical intermediate waypoints of path
  double threshold = 3.0;
  vector<Vector3d> short_tour = {path.front()};
  for (int i = 1; i < path.size() - 1; ++i) {
    // Filter some waypoints which distance lower than threshold
    if ((path[i] - short_tour.back()).norm() > threshold)
      short_tour.push_back(path[i]);

    // Add waypoints to Avoid Collision
    else {
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }

  if ((path.back() - short_tour.back()).norm() > 1e-3)
    short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2) {
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  }
  path = short_tour;
}

bool FastExplorationManager::solveTSP(const Eigen::MatrixXd &cost_mat, vector<int> &indices, int problem_id) {

  //
  // Fomulate ATSP and Call LKH Solver
  //

  ros::Time t1 = ros::Time::now();
  const int dimension = cost_mat.rows();

  string file_name;
  if (problem_id == 0)
    file_name = "/cover_";
  else if (problem_id == 1)
    file_name = "/expl_";

  //
  // 1、Create the Par File (drone_id.par) (Basic Info)
  //

  ofstream par_file(ep_->tsp_dir_ + file_name + to_string(ep_->drone_id_) + ".par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ + file_name + to_string(ep_->drone_id_) + ".tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ + file_name + to_string(ep_->drone_id_) + ".tour\n";
  par_file << "RUNS = 1\n";
  par_file.close();

  //
  // 2、Create the Problem File (drone_id.tsp) (Specification + Cost Matrix)
  //

  ofstream prob_file(ep_->tsp_dir_ + file_name + to_string(ep_->drone_id_) + ".tsp");

  // Write the Specification (Follow the Format of TSPLIB)
  string prob_spec;
  prob_file << "NAME : single\n";
  prob_file << "TYPE : ATSP\n";
  prob_file << "DIMENSION : " + to_string(dimension) + "\n";
  prob_file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  prob_file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  prob_file << "EDGE_WEIGHT_SECTION\n";
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";

  // Write the Cost Matrix
  const int scale = 100;
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }
  prob_file << "EOF";
  prob_file.close();

  //
  // 3、Solve the TSP with LKH Solver
  //

  solveTSPLKH((ep_->tsp_dir_ + file_name + to_string(ep_->drone_id_) + ".par").c_str());

  //
  // 4、Read the Optimal Tour from result file (drone_id.tour)
  //

  string res;
  ifstream res_file(ep_->tsp_dir_ + file_name + to_string(ep_->drone_id_) + ".tour");

  while (getline(res_file, res)) { // Go to the comment of "TOUR_SECTION"
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  while (getline(res_file, res)) { // Read the indices from optimal tour
    int id = stoi(res);
    if (id == 1)
      continue; // Ignore the current state
    if (id == -1)
      break;
    indices.push_back(id - 2);
  }
  res_file.close();

  // double tsp_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("[TSP] Total Time: %lf", tsp_time);

  return true;
}

} // namespace fast_planner