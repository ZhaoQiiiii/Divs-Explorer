#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/DroneState.h>
#include <exploration_manager/PairOpt.h>
#include <exploration_manager/PairOptResponse.h>
#include <exploration_manager/NextCor.h>
#include <bspline/Bspline.h>


#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {

struct FSMData; 
struct FSMParam; 
class FastPlannerManager; 
class FastExplorationManager; 
class PlanningVisualization;

enum EXPL_STATE {INIT, WAIT_TRIGGER, WAIT_DRONE, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, IDLE, FINISH};


class FastExplorationFSM 
{
public:
  FastExplorationFSM() {}
  ~FastExplorationFSM() {}
  void initialize(ros::NodeHandle& nh);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


private:
  typedef exploration_manager::NextCor NextCor;
  typedef exploration_manager::NextCorConstPtr NextCorPtr;

  EXPL_STATE state_;
  shared_ptr<FSMParam> fp_; 
  shared_ptr<FSMData> fd_; 

  shared_ptr<FastExplorationManager> expl_manager_; 
  shared_ptr<FastPlannerManager> planner_manager_; 
  shared_ptr<PlanningVisualization> visualization_;

  ros::NodeHandle nh_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_, wait_drone_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, drone_odom_sub_, next_cor_sub_, expl_replan_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, expl_replan_pub_;

  ros::Publisher drone_state_pub_, opt_pub_, opt_res_pub_, swarm_traj_pub_, grid_tour_pub_, hgrid_pub_;
  ros::Subscriber drone_state_sub_, opt_sub_, opt_res_sub_, swarm_traj_sub_;
  ros::Timer drone_state_timer_, opt_timer_, swarm_traj_timer_;

  // Main API
  int callExplorationPlanner(); 

  // Helper
  void transiteState(EXPL_STATE new_state, string pos_call);
  void visualize();  
  bool checkExploreReplan(const shared_ptr<ExplData>& ed_ptr, const DivisionParam::Ptr &params, bool is_drone);
  bool checkGroundExplore();
  int getId();
  void findUnallocated(const vector<int>& actives, vector<int>& missed);

  // Callbacks
  void FSMCallback(const ros::TimerEvent& e); 
  void safetyCallback(const ros::TimerEvent& e);
  void terminalCallback(const ros::TimerEvent& e); 
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg); 
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void waitDroneCallback(const ros::TimerEvent& e);
  void droneOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void nextCorCallback(const NextCorPtr& msg);
  void explReplanCallback(const std_msgs::Empty& msg);

  // Swarm Callbacks 
  void droneStateTimerCallback(const ros::TimerEvent& e);
  void droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg);
  void optTimerCallback(const ros::TimerEvent& e);
  void optMsgCallback(const exploration_manager::PairOptConstPtr& msg);
  void optResMsgCallback(const exploration_manager::PairOptResponseConstPtr& msg);
  void swarmTrajCallback(const bspline::BsplineConstPtr& msg);
  void swarmTrajTimerCallback(const ros::TimerEvent& e);
};

}  // namespace fast_planner

#endif