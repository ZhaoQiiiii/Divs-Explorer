#include <fstream>

#include <exploration_manager/fast_exploration_manager.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/GridTour.h>
#include <exploration_manager/HGrid.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/fast_exploration_fsm.h>

#include <active_perception/hgrid.h>
#include <active_perception/perception_utils.h>
#include <plan_env/edt_environment.h>
#include <plan_env/multi_map_manager.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;

namespace fast_planner {

void FastExplorationFSM::initialize(ros::NodeHandle &nh) {
  // Params
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("fsm/attempt_interval", fp_->attempt_interval_, 0.2);
  nh.param("fsm/pair_opt_interval", fp_->pair_opt_interval_, 1.0);
  nh.param("fsm/repeat_send_num", fp_->repeat_send_num_, 10);
  nh.param("show_ground", fp_->show_ground_, true);
  nh.param("show_shape", fp_->show_shape_, false);
  nh.param("is_drone", fp_->is_drone_, true);
  nh.param("expl_method", fp_->expl_method_, 0);

  fd_->have_odom_ = false;
  fd_->static_state_ = true;
  fd_->avoid_collision_ = false;
  fd_->have_next_cor_ = false;
  fd_->have_drone_odom_ = false;

  state_ = EXPL_STATE::INIT;
  fd_->state_str_ = {"INIT", "WAIT_TRIGGER", "WAIT_DRONE", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "IDLE", "FINISH"};

  // Color
  if (fp_->is_drone_) {
    fp_->color_traj_ = {1, 0, 0, 1},        // Red
        fp_->color_cover_ = {0, 1, 0.5, 1}, // Light Green
        fp_->color_expl_ = {1, 0.5, 0, 1};  // Orange
  } else {
    fp_->color_traj_ = {0, 0, 1, 1},        // Blue
        fp_->color_cover_ = {0.5, 0, 1, 1}, // Purple
        fp_->color_expl_ = {1, 1, 0.2, 1};  // Yellow
  }

  // Exploration Utils
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  planner_manager_ = expl_manager_->planner_manager_;
  visualization_.reset(new PlanningVisualization(nh));

  // Exploration Ros Utils
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::terminalCallback, this);

  trigger_sub_ = nh.subscribe("/move_base_simple/goal", 1, &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);

  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);

  if (fp_->is_drone_)
    expl_replan_pub_ = nh.advertise<std_msgs::Empty>("/exploration/expl_replan_" + to_string(getId()), 10);

  else {
    wait_drone_timer_ = nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::waitDroneCallback, this);
    drone_odom_sub_ = nh.subscribe("/state_ukf/odom_1", 1, &FastExplorationFSM::droneOdomCallback, this);
    next_cor_sub_ = nh.subscribe("/exploration/next_cor_1", 1, &FastExplorationFSM::nextCorCallback, this);
    expl_replan_sub_ = nh.subscribe("/exploration/expl_replan_1", 1, &FastExplorationFSM::explReplanCallback, this);
  }

  // Swarm Ros Utils (RACER)
  if (fp_->expl_method_ == 3) {
    drone_state_timer_ = nh.createTimer(ros::Duration(0.04), &FastExplorationFSM::droneStateTimerCallback, this);
    drone_state_pub_ = nh.advertise<exploration_manager::DroneState>("/swarm_expl/drone_state_send", 10);
    drone_state_sub_ =
        nh.subscribe("/swarm_expl/drone_state_recv", 10, &FastExplorationFSM::droneStateMsgCallback, this);

    opt_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::optTimerCallback, this);
    opt_pub_ = nh.advertise<exploration_manager::PairOpt>("/swarm_expl/pair_opt_send", 10);
    opt_sub_ = nh.subscribe("/swarm_expl/pair_opt_recv", 100, &FastExplorationFSM::optMsgCallback, this,
                            ros::TransportHints().tcpNoDelay());

    opt_res_pub_ = nh.advertise<exploration_manager::PairOptResponse>("/swarm_expl/pair_opt_res_send", 10);
    opt_res_sub_ = nh.subscribe("/swarm_expl/pair_opt_res_recv", 100, &FastExplorationFSM::optResMsgCallback, this,
                                ros::TransportHints().tcpNoDelay());

    swarm_traj_pub_ = nh.advertise<bspline::Bspline>("/planning/swarm_traj_send", 100);
    swarm_traj_sub_ = nh.subscribe("/planning/swarm_traj_recv", 100, &FastExplorationFSM::swarmTrajCallback, this);
    swarm_traj_timer_ = nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::swarmTrajTimerCallback, this);

    hgrid_pub_ = nh.advertise<exploration_manager::HGrid>("/swarm_expl/hgrid_send", 10);
    grid_tour_pub_ = nh.advertise<exploration_manager::GridTour>("/swarm_expl/grid_tour_send", 10);
  }
}

int FastExplorationFSM::callExplorationPlanner() {

  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  //
  // 1. Call the Exploration Planner
  //

  int res;
  if (fd_->avoid_collision_) {
    res = expl_manager_->generateExploreTraj(fd_->start_pos_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_,
                                             expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
    fd_->avoid_collision_ = false;
  }

  else {
    // Proposed
    if (fp_->expl_method_ == 0) {
      if (fp_->is_drone_)
        res = expl_manager_->planExploreDrone(fd_->start_pos_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
      else
        res = expl_manager_->planExploreGround(fd_->start_pos_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_,
                                               fd_->drone_pos_, fd_->drone_yaw_);
    }

    // Classic
    else if (fp_->expl_method_ == 1)
      res = expl_manager_->classicExpore(fd_->start_pos_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);

    // FUEL
    else if (fp_->expl_method_ == 2)
      res = expl_manager_->fuelExpore(fd_->start_pos_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);

    // RACER
    else if (fp_->expl_method_ == 3)
      res = expl_manager_->racerExplore(fd_->start_pos_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
  }

  //
  // 2、Generate the B-spline if get the next viewpoint
  //

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_traj_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    // Get the info from Non-Uniform B-spline
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    // Get the Pos Ctrl Points
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    // Get the Pos Knots
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    // Get the Yaw Ctrl Points
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }

    // Get the Yaw Knot
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

    // Update the Newest Trajectory
    fd_->newest_traj_ = bspline;
  }

  return res;
}

//
// Callbacks
//

void FastExplorationFSM::FSMCallback(const ros::TimerEvent &e) {

  // ROS_INFO_STREAM_THROTTLE(3.0, "[FSM] Drone " << getId() << ": State: " << fd_->state_str_[int(state_)]);

  switch (state_) {

  case INIT: {
    if (!fd_->have_odom_) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[FSM] Drone " << getId() << ", No Odom.");
      return;
    }

    if ((ros::Time::now() - fd_->fsm_init_time_).toSec() < 2.0) {
      ROS_WARN_STREAM_THROTTLE(1.0, "[FSM] Drone " << getId() << ", Waiting for Init.");
      return;
    }

    transiteState(WAIT_TRIGGER, "FSM");
    break;
  }

  case WAIT_TRIGGER: {
    ROS_WARN_STREAM_THROTTLE(3.0, "[FSM] Drone " << getId() << ", Waiting for Trigger.");
    break;
  }

  case WAIT_DRONE: {
    ROS_WARN_STREAM_THROTTLE(3.0, "[FSM] Drone " << getId() << ", Waiting for Drone.");
    break;
  }

  case PLAN_TRAJ: {
    // Switch FSM state with the result of Exploration Planner

    // Get the start state
    if (fd_->static_state_) { // Replan from static state (hover)
      fd_->start_pos_ = fd_->odom_pos_;
      fd_->start_vel_ = fd_->odom_vel_;
      fd_->start_acc_.setZero();
      fd_->start_yaw_ << fd_->odom_yaw_, 0, 0;
    }

    else { // Replan from replan_time seconds later with last traj
      LocalTrajData *info = &planner_manager_->local_traj_;
      double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
      fd_->start_pos_ = info->position_traj_.evaluateDeBoorT(t_r);
      fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
      fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
      fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
      fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
      fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
    }

    // Call the Exploration Planner
    replan_pub_.publish(std_msgs::Empty());
    int res = callExplorationPlanner();

    if (res == SUCCEED) {
      // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", Plan Succeed.");
      transiteState(PUB_TRAJ, "FSM");
    }

    else if (res == FAIL) {
      // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", Plan Fail.");
      fd_->static_state_ = true;
    }

    else if (res == NO_DIVISION) {
      // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", No Divs.");
      fd_->static_state_ = true;
      replan_pub_.publish(std_msgs::Empty());
      transiteState(IDLE, "FSM");
    }

    else if (res == NO_FRONTIER) {
      // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", No Frontiers.");
      fd_->static_state_ = true;
      replan_pub_.publish(std_msgs::Empty());
      transiteState(FINISH, "FSM");
    }

    break;
  }

  case PUB_TRAJ: {
    // Visulize the Exploration Data

    double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
    if (dt > 0) {
      bspline_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;

      if (fp_->expl_method_ == 3) {
        fd_->newest_traj_.drone_id = expl_manager_->ep_->drone_id_;
        swarm_traj_pub_.publish(fd_->newest_traj_);
      }

      thread vis_thread(&FastExplorationFSM::visualize, this);
      vis_thread.detach();
      transiteState(EXEC_TRAJ, "FSM");
    }
    break;
  }

  case EXEC_TRAJ: {
    // Replan to approach the next viewpoint

    auto tn = ros::Time::now();
    LocalTrajData *info = &planner_manager_->local_traj_;
    double t_cur = (tn - info->start_time_).toSec();

    bool need_replan = false;
    const auto &ed_ptr = expl_manager_->ed_;
    const auto &dp_ptr = expl_manager_->dp_;

    //
    // Explore Replan: [Coverage Path] -> [Exploration Path / Greedy Explore]
    //

    if (fp_->expl_method_ == 0 && ed_ptr->need_expl_replan_ && checkExploreReplan(ed_ptr, dp_ptr, fp_->is_drone_)) {
      // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", Replan: Next Div is no need to explore.");
      ed_ptr->find_coverage_ = false;
      need_replan = true;
      if (fp_->is_drone_)
        expl_replan_pub_.publish(std_msgs::Empty());
    }

    //
    // Motion Replan: [Exploration Path / Greedy Explore]
    //

    else if (info->duration_ - t_cur < fp_->replan_thresh1_) {
      // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", Replan: Trajectory is fully Executed.");
      need_replan = true;
    }

    else if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
      // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", Replan: Next Frontiers Cluster is fully covered.");
      need_replan = true;
    }

    else if (t_cur > fp_->replan_thresh3_) {
      // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", Replan: Trigger the Periodic Call.");
      need_replan = true;
    }

    // Check if finish the exploration
    if (need_replan) {
      if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
        transiteState(PLAN_TRAJ, "FSM");
      }

      else {
        ROS_WARN("[FSM] There is no Frontiers.");
        fd_->static_state_ = true;
        replan_pub_.publish(std_msgs::Empty());
        transiteState(FINISH, "FSM");
      }
    }
    break;
  }

  case IDLE: {
    ROS_WARN_STREAM_ONCE("[FSM] Drone " << getId() << ", Enter IDLE.");
    if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) == 0) {
      ROS_WARN_STREAM("[FSM] Drone " << getId() << ", There is no Frontiers.");
      transiteState(FINISH, "FSM");
    }
    break;
  }

  case FINISH: {
    ROS_WARN_STREAM_ONCE("[FSM] Drone " << getId() << ", Finish the Exploration.");
    break;
  }
  }
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent &e) {

  // Check safety and trigger replan if necessary

  if (state_ == EXPL_STATE::EXEC_TRAJ) {

    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", Replan: Collision Detected.");
      fd_->avoid_collision_ = true;
      transiteState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void FastExplorationFSM::terminalCallback(const ros::TimerEvent &e) {

  // Visulize in WAIT_TRIGGER and FINISH

  auto ed_ptr = expl_manager_->ed_;
  auto ep_ptr = expl_manager_->ep_;

  double div_height = ep_ptr->box_min_z_;
  if (fp_->show_shape_)
    div_height = (ep_ptr->box_min_z_ + ep_ptr->box_max_z_) / 2.0;

  if (state_ == WAIT_TRIGGER) {
    expl_manager_->updateFrontierStruct(fd_->odom_pos_);

    // Trajectory
    auto local_traj = &planner_manager_->local_traj_;
    visualization_->drawBspline(local_traj->position_traj_, 0, fp_->color_traj_, false, 0.15, fp_->color_traj_, 0);

    // Updated Box
    visualization_->drawLines(ed_ptr->updated_box_vis_.first, ed_ptr->updated_box_vis_.second, 0.1, fp_->color_traj_,
                              "box", ep_ptr->drone_id_, 5);

    // Frontier
    for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                                visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.5), "frontiers", i,
                                6);
    }

    // Viewpoint
    visualization_->drawSpheres(ed_ptr->top_points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 7);
    visualization_->drawLines(ed_ptr->top_points_, ed_ptr->views_, 0.08, Vector4d(0, 0.5, 0, 1), "views", 0, 7);

    // Division
    for (const auto &[id, div] : ed_ptr->all_divs_) {
      if (div->id_ == -1 || div == nullptr)
        continue;
      if (fp_->show_ground_)
        visualization_->drawCubes(div->ground_, 0.1, Vector4d(0, 0.7, 1, 0.7), "division_ground", id, 8);
      if (fp_->expl_method_ != 0)
        continue;
      visualization_->drawLines(div->draw_.first, div->draw_.second, 0.1, Vector4d(0, 0, 0, 0.3), "division", id, 8);
      visualization_->drawText({div->center_[0], div->center_[1], div_height}, to_string(id), 1.0, Vector4d(0, 0, 0, 1),
                               "division_id", id, 8);
    }
  }

  else if (state_ == IDLE || state_ == FINISH) {

    // Trajectory
    auto local_traj = &planner_manager_->local_traj_;
    visualization_->drawBspline(local_traj->position_traj_, 0, Vector4d(0, 0, 0, 0), false, 0, Vector4d(0, 0, 0, 0), 0);

    // Updated Box
    visualization_->drawLines({}, {}, 0.1, Vector4d(0, 0, 0, 0), "box", ep_ptr->drone_id_, 5);

    // Frontier
    if (ed_ptr->frontiers_.empty()) {
      for (int i = 0; i < 100; ++i)
        visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontiers", i, 6);
    }

    else {
      static int last_ftr_num = 0;
      for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
        visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                                  visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.5), "frontiers", i,
                                  6);
      }
      for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
        visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0), "frontiers", i, 6);
      }
      last_ftr_num = ed_ptr->frontiers_.size();
    }

    // Viewpoint
    visualization_->drawSpheres(ed_ptr->top_points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 7);
    visualization_->drawLines(ed_ptr->top_points_, ed_ptr->views_, 0.08, Vector4d(0, 0.5, 0, 1), "views", 0, 7);

    // Path
    visualization_->drawLines({}, 0, Vector4d(0, 0, 0, 0), "coverage_path", 0, 9);
    visualization_->drawLines({}, 0, Vector4d(0, 0, 0, 0), "exploration_path", 0, 9);
    visualization_->drawLines({}, 0, Vector4d(0, 0, 0, 0), "init_path", 0, 9);
    visualization_->drawSphere({}, 0, Vector4d(1, 0, 0, 1), "next_goal", 0, 9);

    // Division
    if (state_ == IDLE) {
      for (const auto &[id, div] : ed_ptr->all_divs_) {
        if (div == nullptr)
          continue;

        if (fp_->show_ground_)
          visualization_->drawCubes(div->ground_, 0.1, Vector4d(0, 0.7, 1, 0.7), "division_ground", id, 8);

        else if (fp_->expl_method_ == 0) {
          // Spare
          if (div->status_ == SPARE) {
            visualization_->drawLines(div->draw_.first, div->draw_.second, 0.1, Vector4d(0, 0, 0, 0.3), "division", id,
                                      8);
          }

          // Activated
          else if (div->status_ == ACTIVATED) {
            visualization_->drawCube({div->center_[0], div->center_[1], div_height},
                                     {div->size_[0], div->size_[1], 0.01}, Vector4d(0.5, 0.5, 0.5, 0.5), "division", id,
                                     8);
          }

          // Explored
          else if (div->status_ == EXPLORED) {
            visualization_->drawCube({div->center_[0], div->center_[1], div_height},
                                     {div->size_[0], div->size_[1], 0.01}, Vector4d(0, 0.5, 1, 0.5), "division", id, 8);
          }
        }

        if (fp_->expl_method_ != 0)
          continue;
        visualization_->drawText({div->center_[0], div->center_[1], div_height}, to_string(id), 1.0,
                                 Vector4d(0, 0, 0, 1), "division_id", id, 8);
      }
    }

    else if (state_ == FINISH) {
      for (const auto &[id, div] : ed_ptr->all_divs_) {
        if (div == nullptr)
          continue;
        if (fp_->show_ground_)
          visualization_->drawCubes(div->ground_, 0.1, Vector4d(0, 0.7, 1, 0.7), "division_ground", id, 8);
        if (fp_->expl_method_ != 0)
          continue;
        visualization_->drawLines({}, 0.1, Vector4d(0, 0, 0, 0), "division", id, 8);
        visualization_->drawText({}, to_string(-1), 1.0, Vector4d(0, 0, 0, 0), "division_id", id, 8);
      }
    }

    // RACER
    if (fp_->expl_method_ == 3) {
      if (expl_manager_->ep_->drone_id_ == 1) {
        visualization_->drawLines({}, 0.1, Vector4d(0, 0, 0, 0), "partition", 1, 10);
        for (int i = 0; i < 1000; i++)
          visualization_->drawText({}, to_string(-1), 1.0, Vector4d(0, 0, 0, 0), "text", i, 10);
      }
      visualization_->drawLines({}, 0.1, Vector4d(0, 0, 0, 0), "grid_tour", 0, 10);
    }
  }
}

void FastExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr &msg) {

  if (state_ != WAIT_TRIGGER)
    return;
  // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", Triggered!");

  fd_->start_pos_ = fd_->odom_pos_;
  expl_manager_->ed_->triggered_ = true;
  fd_->trigger_time_ = expl_manager_->ep_->trigger_time_ = ros::Time::now();

  if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {
    if (fp_->is_drone_)
      transiteState(PLAN_TRAJ, "triggerCallback");
    else
      transiteState(WAIT_DRONE, "triggerCallback");
  } else
    transiteState(FINISH, "triggerCallback");
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {

  // position
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  // velocity
  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  // orientation
  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (!fd_->have_odom_) {
    fd_->have_odom_ = true;
    fd_->fsm_init_time_ = ros::Time::now();
  }
}

void FastExplorationFSM::waitDroneCallback(const ros::TimerEvent &e) {
  if (state_ != WAIT_DRONE)
    return;

  if (checkGroundExplore()) {
    if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0)
      transiteState(PLAN_TRAJ, "waitDroneCallback");
    else
      transiteState(FINISH, "waitDroneCallback");
  }
}

void FastExplorationFSM::droneOdomCallback(const nav_msgs::OdometryConstPtr &msg) {
  // position
  fd_->drone_pos_(0) = msg->pose.pose.position.x;
  fd_->drone_pos_(1) = msg->pose.pose.position.y;
  fd_->drone_pos_(2) = msg->pose.pose.position.z;

  // orientation
  fd_->drone_orient_.w() = msg->pose.pose.orientation.w;
  fd_->drone_orient_.x() = msg->pose.pose.orientation.x;
  fd_->drone_orient_.y() = msg->pose.pose.orientation.y;
  fd_->drone_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->drone_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->drone_yaw_ = atan2(rot_x(1), rot_x(0));

  if (!fd_->have_drone_odom_)
    fd_->have_drone_odom_ = true;
}

void FastExplorationFSM::nextCorCallback(const NextCorPtr &msg) {
  // Sub "/exploration/next_cor_1" (For Ground)

  if (fd_->have_next_cor_)
    return;

  // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", Get the IDs of Next Corner Divs!");
  fd_->have_next_cor_ = true;
  expl_manager_->ed_->next_cor_ids_ = msg->next_cor_ids;
  fd_->get_next_cor_time_ = ros::Time::now();
}

void FastExplorationFSM::explReplanCallback(const std_msgs::Empty &msg) {
  // Sub "/exploration/expl_replan_1" (For Ground)

  if (state_ == WAIT_DRONE)
    return;

  // ROS_WARN_STREAM("[FSM] Drone " << getId() << ", Replan: Explore Replan Detected.");
  const auto &ed_ptr = expl_manager_->ed_;
  ed_ptr->find_coverage_ = false;
  transiteState(PLAN_TRAJ, "explReplanCallback");
}

//
// Helper
//

void FastExplorationFSM::transiteState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;

  // ROS_INFO_STREAM("[" + pos_call + "]: Drone "
  //                 << getId()
  //                 << " from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]);
}

void FastExplorationFSM::visualize() {
  // Visulize in PUB_TRAJ

  auto local_traj = &planner_manager_->local_traj_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;
  auto ep_ptr = expl_manager_->ep_;

  // Trajectory
  visualization_->drawBspline(local_traj->position_traj_, 0.1, fp_->color_traj_, false, 0.15, fp_->color_traj_, 0);

  // Updated Box
  visualization_->drawLines(ed_ptr->updated_box_vis_.first, ed_ptr->updated_box_vis_.second, 0.1, fp_->color_traj_,
                            "box", ep_ptr->drone_id_, 5);

  // Frontier
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                              visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.5), "frontiers", i, 6);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontiers", i, 6);
  }
  last_ftr_num = ed_ptr->frontiers_.size();

  // Viewpoint
  visualization_->drawSpheres(ed_ptr->top_points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 7);
  visualization_->drawLines(ed_ptr->top_points_, ed_ptr->views_, 0.08, Vector4d(0, 0.5, 0, 1), "views", 0, 7);

  // Path
  visualization_->drawLines(ed_ptr->coverage_path_, 0.1, fp_->color_cover_, "coverage_path", 0, 9);
  visualization_->drawLines(ed_ptr->exploration_path_, 0.1, fp_->color_expl_, "exploration_path", 0, 9);
  visualization_->drawLines(ed_ptr->init_path_, 0.1, Vector4d(1, 1, 0.5, 1), "init_path", 0, 9);
  visualization_->drawSphere(ed_ptr->next_goal_, 0.4, fp_->color_traj_, "next_goal", 0, 9);

  // Division
  for (const auto &[id, div] : ed_ptr->all_divs_) {
    if (id == -1 || div == nullptr)
      continue;
    if (fp_->show_ground_)
      visualization_->drawCubes(div->ground_, 0.1, Vector4d(0, 0.7, 1, 0.7), "division_ground", id, 8);
    if (fp_->expl_method_ != 0)
      continue;

    double div_height = div->center_[2] - div->size_[2] / 2.0;
    if (fp_->show_shape_)
      div_height = div->center_[2];

    // Spare
    if (div->status_ == SPARE) {
      if (!fp_->show_ground_) {
        visualization_->drawLines(div->draw_.first, div->draw_.second, 0.1, Vector4d(0, 0, 0, 0.3), "division", id, 8);
      }

      if (id == ed_ptr->next_corner_div_->id_) { // Next Corner Div
        visualization_->drawText({div->center_[0], div->center_[1], div_height}, to_string(id), 1.0,
                                 Vector4d(1, 0.5, 0, 1), "division_id", id, 8);
      }

      else {
        visualization_->drawText({div->center_[0], div->center_[1], div_height}, to_string(id), 1.0,
                                 Vector4d(0, 0, 0, 1), "division_id", id, 8);
      }
    }

    // Activated
    else if (div->status_ == ACTIVATED) {
      if (!fp_->show_ground_) {
        visualization_->drawCube({div->center_[0], div->center_[1], div_height}, {div->size_[0], div->size_[1], 0.01},
                                 Vector4d(0.5, 0.5, 0.5, 0.5), "division", id, 8);
      }

      if (id == ed_ptr->next_div_->id_) { // Next Div
        visualization_->drawText({div->center_[0], div->center_[1], div_height}, to_string(id), 1.0,
                                 Vector4d(1, 0, 0, 1), "division_id", id, 8);
      }

      else if (id == ed_ptr->next_corner_div_->id_) { // Next Corner Div
        visualization_->drawText({div->center_[0], div->center_[1], div_height}, to_string(id), 1.0,
                                 Vector4d(1, 0.5, 0, 1), "division_id", id, 8);
      }

      else {
        visualization_->drawText({div->center_[0], div->center_[1], div_height}, to_string(id), 1.0,
                                 Vector4d(0, 0, 0, 1), "division_id", id, 8);
      }
    }

    // Explored
    else if (div->status_ == EXPLORED) {
      if (!fp_->show_ground_) {
        visualization_->drawCube({div->center_[0], div->center_[1], div_height}, {div->size_[0], div->size_[1], 0.01},
                                 Vector4d(0, 0.5, 1, 0.5), "division", id, 8);
      }

      if (id == ed_ptr->ori_div_->id_) { // Ori Div
        visualization_->drawText({div->center_[0], div->center_[1], div_height}, to_string(id), 1.0,
                                 Vector4d(0, 0.5, 0, 1), "division_id", id, 8);
      }

      else if (id == ed_ptr->next_div_->id_) { // Next Div
        visualization_->drawText({div->center_[0], div->center_[1], div_height}, to_string(id), 1.0,
                                 Vector4d(1, 0, 0, 1), "division_id", id, 8);
      }

      else if (id == ed_ptr->next_corner_div_->id_) { // Next Corner Div
        visualization_->drawText({div->center_[0], div->center_[1], div_height}, to_string(id), 1.0,
                                 Vector4d(1, 0.5, 0, 1), "division_id", id, 8);
      }

      else {
        visualization_->drawText({div->center_[0], div->center_[1], div_height}, to_string(id), 1.0,
                                 Vector4d(0, 0, 0, 1), "division_id", id, 8);
      }
    }
  }

  // RACER
  if (fp_->expl_method_ == 3) {

    // Partition & Text
    if (expl_manager_->ep_->drone_id_ == 1) {
      vector<Eigen::Vector3d> pts1, pts2;
      expl_manager_->hgrid_->getGridMarker(pts1, pts2);
      visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "partition", 1, 10);

      vector<Eigen::Vector3d> pts;
      vector<string> texts;
      expl_manager_->hgrid_->getGridMarker2(pts, texts);
      static int last_text_num = 0;
      for (int i = 0; i < pts.size(); ++i) {
        visualization_->drawText(pts[i], texts[i], 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 10);
      }
      for (int i = pts.size(); i < last_text_num; ++i) {
        visualization_->drawText(Eigen::Vector3d(0, 0, 0), to_string(-1), 1, Eigen::Vector4d(0, 0, 0, 0), "text", i,
                                 10);
      }
      last_text_num = pts.size();
    }

    // Grid Tour
    auto grid_tour = expl_manager_->ed_->grid_tour_;
    visualization_->drawLines(
        grid_tour, 0.05,
        PlanningVisualization::getColor((expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        "grid_tour", 0, 10);
  }
}

bool FastExplorationFSM::checkExploreReplan(const shared_ptr<ExplData> &ed_ptr, const DivisionParam::Ptr &params,
                                            bool is_drone) {

  // Drone
  if (is_drone) {
    if (params->complete_expl_) { // Complete Explore
      bool complete_ori = ed_ptr->begin_ ? true : ed_ptr->ori_div_->ftrs_.empty() && ed_ptr->ori_div_->had_ftrs_;
      bool complete_next = ed_ptr->next_div_->ftrs_.empty() && ed_ptr->next_div_->had_ftrs_;
      bool complete = complete_ori && complete_next;
      bool no_ftr = ed_ptr->chosen_ftrs_.empty() && DivisionUtils::checkPointInside(fd_->odom_pos_, ed_ptr->next_div_);
      return complete || no_ftr;
    }

    else { // Partial Explore
      bool part = ed_ptr->next_div_->coverage_rate_ > params->coverage_rate_thr_;
      bool complete_next = ed_ptr->next_div_->ftrs_.empty() && ed_ptr->next_div_->had_ftrs_;
      bool no_ftr = ed_ptr->chosen_ftrs_.empty() && DivisionUtils::checkPointInside(fd_->odom_pos_, ed_ptr->next_div_);
      return part || complete_next || no_ftr;
    }
  }

  // Ground
  else {
    bool complete_ori = ed_ptr->ori_div_->ftrs_.empty() && ed_ptr->ori_div_->had_ftrs_;
    bool complete_next = ed_ptr->next_div_->ftrs_.empty() && ed_ptr->next_div_->had_ftrs_;
    bool complete = complete_ori && complete_next;

    return complete;
  }
}

bool FastExplorationFSM::checkGroundExplore() {

  // Wait for Drone Odom and IDs of Next Corner Divs
  if (!fd_->have_drone_odom_ || !fd_->have_next_cor_)
    return false;

  // Delay after get the IDs of Next Corner Divs
  if ((ros::Time::now() - fd_->get_next_cor_time_).toSec() < 5.0)
    return false;

  return true;
}

//
// RACER
//

void FastExplorationFSM::droneStateTimerCallback(const ros::TimerEvent &e) {

  // Timer 0.04s + Pub "/swarm_expl/drone_state"

  // Broadcast drone own state periodically

  exploration_manager::DroneState msg; // Pre-Defined Ros Message (For Pub and Sub)
  msg.drone_id = getId();
  auto &state = expl_manager_->ed_->swarm_state_[msg.drone_id - 1];

  // Get current state
  if (fd_->static_state_) {
    state.pos_ = fd_->odom_pos_;
    state.vel_ = fd_->odom_vel_;
    state.yaw_ = fd_->odom_yaw_;
  }

  else {
    LocalTrajData *info = &planner_manager_->local_traj_;
    double t_r = (ros::Time::now() - info->start_time_).toSec();
    state.pos_ = info->position_traj_.evaluateDeBoorT(t_r);
    state.vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
    state.yaw_ = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
  }

  // Broadcast message
  state.stamp_ = ros::Time::now().toSec();
  msg.pos = {float(state.pos_[0]), float(state.pos_[1]), float(state.pos_[2])};
  msg.vel = {float(state.vel_[0]), float(state.vel_[1]), float(state.vel_[2])};
  msg.yaw = state.yaw_;
  for (auto id : state.grid_ids_)
    msg.grid_ids.push_back(id);
  msg.recent_attempt_time = state.recent_attempt_time_;
  msg.stamp = state.stamp_;

  drone_state_pub_.publish(msg);
}

void FastExplorationFSM::droneStateMsgCallback(const exploration_manager::DroneStateConstPtr &msg) {

  // Sub "/swarm_expl/drone_state"

  // Update other drones' states from msg

  if (msg->drone_id == getId())
    return;

  // Simulate swarm communication loss
  // Eigen::Vector3d msg_pos(msg->pos[0], msg->pos[1], msg->pos[2]);
  // if ((msg_pos - fd_->odom_pos_).norm() > 6.0) return;

  // Update the swarm states
  auto &drone_state = expl_manager_->ed_->swarm_state_[msg->drone_id - 1];
  if (drone_state.stamp_ + 1e-4 >= msg->stamp)
    return; // Avoid unordered msg

  drone_state.pos_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  drone_state.vel_ = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
  drone_state.yaw_ = msg->yaw;

  drone_state.grid_ids_.clear();
  for (auto id : msg->grid_ids)
    drone_state.grid_ids_.push_back(id);

  drone_state.stamp_ = msg->stamp;
  drone_state.recent_attempt_time_ = msg->recent_attempt_time;

  // std::cout << "Drone " << getId() << " get drone " << int(msg->drone_id) << "'s state" <<
  // std::endl; std::cout << drone_state.pos_.transpose() << std::endl;
}

void FastExplorationFSM::optTimerCallback(const ros::TimerEvent &e) {

  // Timer 0.05s + Pub "/swarm_expl/pair_opt"

  if (state_ == INIT)
    return;

  // Select nearby drone not interacting with recently
  auto &states = expl_manager_->ed_->swarm_state_;
  auto &state1 = states[getId() - 1];
  // bool urgent = (state1.grid_ids_.size() <= 1 /* && !state1.grid_ids_.empty() */);
  bool urgent = state1.grid_ids_.empty();
  auto tn = ros::Time::now().toSec();

  // Avoid frequent attempt
  if (tn - state1.recent_attempt_time_ < fp_->attempt_interval_)
    return;

  int select_id = -1;
  double max_interval = -1.0;
  for (int i = 0; i < states.size(); ++i) {
    if (i + 1 <= getId())
      continue;
    // Check if have communication recently
    // or the drone just experience another opt
    // or the drone is interacted with recently /* !urgent &&  */
    // or the candidate drone dominates enough grids
    if (tn - states[i].stamp_ > 0.2)
      continue;
    if (tn - states[i].recent_attempt_time_ < fp_->attempt_interval_)
      continue;
    if (tn - states[i].recent_interact_time_ < fp_->pair_opt_interval_)
      continue;
    if (states[i].grid_ids_.size() + state1.grid_ids_.size() == 0)
      continue;

    double interval = tn - states[i].recent_interact_time_;
    if (interval <= max_interval)
      continue;
    select_id = i + 1;
    max_interval = interval;
  }
  if (select_id == -1)
    return;

  std::cout << "\nSelect: " << select_id << std::endl;
  ROS_WARN("Pair opt %d & %d", getId(), select_id);

  // Do pairwise optimization with selected drone, allocate the union of their domiance grids
  unordered_map<int, char> opt_ids_map;
  auto &state2 = states[select_id - 1];
  for (auto id : state1.grid_ids_)
    opt_ids_map[id] = 1;
  for (auto id : state2.grid_ids_)
    opt_ids_map[id] = 1;
  vector<int> opt_ids;
  for (auto pair : opt_ids_map)
    opt_ids.push_back(pair.first);

  std::cout << "Pair Opt id: ";
  for (auto id : opt_ids)
    std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Find missed grids to reallocated them
  vector<int> actives, missed;
  expl_manager_->hgrid_->getActiveGrids(actives);
  findUnallocated(actives, missed);
  std::cout << "Missed: ";
  for (auto id : missed)
    std::cout << id << ", ";
  std::cout << "" << std::endl;
  opt_ids.insert(opt_ids.end(), missed.begin(), missed.end());

  // Do partition of the grid
  vector<Eigen::Vector3d> positions = {state1.pos_, state2.pos_};
  vector<Eigen::Vector3d> velocities = {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)};
  vector<int> first_ids1, second_ids1, first_ids2, second_ids2;
  if (state_ != WAIT_TRIGGER) {
    expl_manager_->hgrid_->getConsistentGrid(state1.grid_ids_, state1.grid_ids_, first_ids1, second_ids1);
    expl_manager_->hgrid_->getConsistentGrid(state2.grid_ids_, state2.grid_ids_, first_ids2, second_ids2);
  }

  auto t1 = ros::Time::now();

  vector<int> ego_ids, other_ids;
  expl_manager_->allocateGrids(positions, velocities, {first_ids1, first_ids2}, {second_ids1, second_ids2}, opt_ids,
                               ego_ids, other_ids);

  double alloc_time = (ros::Time::now() - t1).toSec();

  std::cout << "Ego1  : ";
  for (auto id : state1.grid_ids_)
    std::cout << id << ", ";
  std::cout << "\nOther1: ";
  for (auto id : state2.grid_ids_)
    std::cout << id << ", ";
  std::cout << "\nEgo2  : ";
  for (auto id : ego_ids)
    std::cout << id << ", ";
  std::cout << "\nOther2: ";
  for (auto id : other_ids)
    std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Check results
  double prev_app1 = expl_manager_->computeGridPathCost(state1.pos_, state1.grid_ids_, first_ids1,
                                                        {first_ids1, first_ids2}, {second_ids1, second_ids2}, true);
  double prev_app2 = expl_manager_->computeGridPathCost(state2.pos_, state2.grid_ids_, first_ids2,
                                                        {first_ids1, first_ids2}, {second_ids1, second_ids2}, true);
  std::cout << "prev cost: " << prev_app1 << ", " << prev_app2 << ", " << prev_app1 + prev_app2 << std::endl;
  double cur_app1 = expl_manager_->computeGridPathCost(state1.pos_, ego_ids, first_ids1, {first_ids1, first_ids2},
                                                       {second_ids1, second_ids2}, true);
  double cur_app2 = expl_manager_->computeGridPathCost(state2.pos_, other_ids, first_ids2, {first_ids1, first_ids2},
                                                       {second_ids1, second_ids2}, true);
  std::cout << "cur cost : " << cur_app1 << ", " << cur_app2 << ", " << cur_app1 + cur_app2 << std::endl;
  if (cur_app1 + cur_app2 > prev_app1 + prev_app2 + 0.1) {
    ROS_ERROR("Larger cost after reallocation");
    if (state_ != WAIT_TRIGGER) {
      return;
    }
  }

  if (!state1.grid_ids_.empty() && !ego_ids.empty() &&
      !expl_manager_->hgrid_->isConsistent(state1.grid_ids_[0], ego_ids[0])) {
    ROS_ERROR("Path 1 inconsistent");
  }
  if (!state2.grid_ids_.empty() && !other_ids.empty() &&
      !expl_manager_->hgrid_->isConsistent(state2.grid_ids_[0], other_ids[0])) {
    ROS_ERROR("Path 2 inconsistent");
  }

  // Update ego and other dominace grids
  auto last_ids2 = state2.grid_ids_;

  // Send the result to selected drone and wait for confirmation
  exploration_manager::PairOpt opt;
  opt.from_drone_id = getId();
  opt.to_drone_id = select_id;
  // opt.msg_type = 1;
  opt.stamp = tn;
  for (auto id : ego_ids)
    opt.ego_ids.push_back(id);
  for (auto id : other_ids)
    opt.other_ids.push_back(id);

  for (int i = 0; i < fp_->repeat_send_num_; ++i)
    opt_pub_.publish(opt);

  ROS_WARN("Drone %d send opt request to %d, pair opt t: %lf, allocate t: %lf", getId(), select_id,
           ros::Time::now().toSec() - tn, alloc_time);

  // Reserve the result and wait...
  auto ed = expl_manager_->ed_;
  ed->ego_ids_ = ego_ids;
  ed->other_ids_ = other_ids;
  ed->pair_opt_stamp_ = opt.stamp;
  ed->wait_response_ = true;
  state1.recent_attempt_time_ = tn;
}

void FastExplorationFSM::optMsgCallback(const exploration_manager::PairOptConstPtr &msg) {

  // Sub "/swarm_expl/pair_opt" + Pub "/swarm_expl/pair_opt_res"

  // Receive msg from other drones to ego drone
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId())
    return;

  // Check stamp to avoid unordered/repeated msg
  if (msg->stamp <= expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] + 1e-4)
    return;
  expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] = msg->stamp;

  //
  auto &state1 = expl_manager_->ed_->swarm_state_[msg->from_drone_id - 1];
  auto &state2 = expl_manager_->ed_->swarm_state_[getId() - 1];

  exploration_manager::PairOptResponse response;
  response.from_drone_id = msg->to_drone_id;
  response.to_drone_id = msg->from_drone_id;
  response.stamp = msg->stamp; // reply with the same stamp for verificaiton

  // Check the request frequency
  if (msg->stamp - state2.recent_attempt_time_ < fp_->attempt_interval_) {
    // Reject this attempt to avoid frequent changes
    ROS_WARN("Reject frequent attempt");
    response.status = 2;
  }

  else {
    // No opt attempt recently, and the grid info between drones are consistent, the pair opt
    // request can be accepted
    response.status = 1;

    // Change grid ids from the msg
    state1.grid_ids_.clear();
    state2.grid_ids_.clear();
    for (auto id : msg->ego_ids)
      state1.grid_ids_.push_back(id);
    for (auto id : msg->other_ids)
      state2.grid_ids_.push_back(id);

    state1.recent_interact_time_ = msg->stamp;
    state2.recent_attempt_time_ = ros::Time::now().toSec();
    expl_manager_->ed_->reallocated_ = true;

    if (!state2.grid_ids_.empty()) {
      transiteState(PLAN_TRAJ, "optMsgCallback");
      ROS_WARN("Restart after opt!");
    }

    // if (!check_consistency(tmp1, tmp2)) {
    //   response.status = 2;
    //   ROS_WARN("Inconsistent grid info, reject pair opt");
    // } else {
    // }
  }

  // Repeat pub the response
  for (int i = 0; i < fp_->repeat_send_num_; ++i)
    opt_res_pub_.publish(response);
}

void FastExplorationFSM::optResMsgCallback(const exploration_manager::PairOptResponseConstPtr &msg) {

  // Sub "/swarm_expl/pair_opt_res"

  if (msg->from_drone_id == getId() || msg->to_drone_id != getId())
    return;

  // Check stamp to avoid unordered/repeated msg
  if (msg->stamp <= expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] + 1e-4)
    return;
  expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] = msg->stamp;

  auto ed = expl_manager_->ed_;
  // Verify the consistency of pair opt via time stamp
  if (!ed->wait_response_ || fabs(ed->pair_opt_stamp_ - msg->stamp) > 1e-5)
    return;

  ed->wait_response_ = false;
  ROS_WARN("get response %d", int(msg->status));

  if (msg->status != 1)
    return; // Receive 1 for valid opt

  auto &state1 = ed->swarm_state_[getId() - 1];
  auto &state2 = ed->swarm_state_[msg->from_drone_id - 1];
  state1.grid_ids_ = ed->ego_ids_;
  state2.grid_ids_ = ed->other_ids_;
  state2.recent_interact_time_ = ros::Time::now().toSec();
  ed->reallocated_ = true;

  if (!state1.grid_ids_.empty()) {
    transiteState(PLAN_TRAJ, "optResMsgCallback");
    ROS_WARN("Restart after opt!");
  }
}

void FastExplorationFSM::swarmTrajTimerCallback(const ros::TimerEvent &e) {

  // Timer 0.1s + Pub "/planning/swarm_traj"

  // Broadcast newest traj of this drone to others

  if (state_ == EXEC_TRAJ) {
    swarm_traj_pub_.publish(fd_->newest_traj_);
  }

  else if (state_ == WAIT_TRIGGER) {
    // Publish a virtual traj at current pose, to avoid collision
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = ros::Time::now();
    bspline.traj_id = planner_manager_->local_traj_.traj_id_;

    Eigen::MatrixXd pos_pts(4, 3);
    for (int i = 0; i < 4; ++i)
      pos_pts.row(i) = fd_->odom_pos_.transpose();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    NonUniformBspline tmp(pos_pts, planner_manager_->pp_.bspline_degree_, 1.0);
    Eigen::VectorXd knots = tmp.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    bspline.drone_id = expl_manager_->ep_->drone_id_;
    swarm_traj_pub_.publish(bspline);
  }
}

void FastExplorationFSM::swarmTrajCallback(const bspline::BsplineConstPtr &msg) {

  // Sub "/planning/swarm_traj"

  // Get newest trajs from other drones, for inter-drone collision avoidance
  auto &sdat = planner_manager_->swarm_traj_data_;

  // Ignore self trajectory
  if (msg->drone_id == sdat.drone_id_)
    return;

  // Ignore outdated trajectory
  if (sdat.receive_flags_[msg->drone_id - 1] == true &&
      msg->start_time.toSec() <= sdat.swarm_trajs_[msg->drone_id - 1].start_time_ + 1e-3)
    return;

  // Convert the msg to B-spline
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  // // Transform of drone's basecoor, optional step (skip if use swarm_pilot)
  // Eigen::Vector4d tf;
  // planner_manager_->edt_environment_->sdf_map_->getBaseCoor(msg->drone_id, tf);
  // double yaw = tf[3];
  // Eigen::Matrix3d rot;
  // rot << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  // Eigen::Vector3d trans = tf.head<3>();
  // for (int i = 0; i < pos_pts.rows(); ++i) {
  //   Eigen::Vector3d tmp = pos_pts.row(i);
  //   tmp = rot * tmp + trans;
  //   pos_pts.row(i) = tmp;
  // }

  sdat.swarm_trajs_[msg->drone_id - 1].setUniformBspline(pos_pts, msg->order, 0.1);
  sdat.swarm_trajs_[msg->drone_id - 1].setKnot(knots);
  sdat.swarm_trajs_[msg->drone_id - 1].start_time_ = msg->start_time.toSec();
  sdat.receive_flags_[msg->drone_id - 1] = true;

  if (state_ == EXEC_TRAJ) {
    // Check collision with received trajectory
    if (!planner_manager_->checkSwarmCollision(msg->drone_id)) {
      ROS_ERROR("Drone %d collide with drone %d.", sdat.drone_id_, msg->drone_id);
      fd_->avoid_collision_ = true;
      transiteState(PLAN_TRAJ, "swarmTrajCallback");
    }
  }
}

int FastExplorationFSM::getId() { return expl_manager_->ep_->drone_id_; }

void FastExplorationFSM::findUnallocated(const vector<int> &actives, vector<int> &missed) {

  // Create map of all active
  unordered_map<int, char> active_map;
  for (auto ativ : actives) {
    active_map[ativ] = 1;
  }

  // Remove allocated ones
  for (auto state : expl_manager_->ed_->swarm_state_) {
    for (auto id : state.grid_ids_) {
      if (active_map.find(id) != active_map.end()) {
        active_map.erase(id);
      } else {
        // ROS_ERROR("Inactive grid %d is allocated.", id);
      }
    }
  }

  missed.clear();
  for (auto p : active_map) {
    missed.push_back(p.first);
  }
}

} // namespace fast_planner
