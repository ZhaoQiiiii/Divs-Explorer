#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <vector>
#include <bspline/Bspline.h>
#include <active_perception/division.h>

using Eigen::Vector3d;
using std::vector;

namespace fast_planner {

struct FSMData {
  vector<string> state_str_;
  ros::Time fsm_init_time_, trigger_time_;

  bool have_odom_, static_state_, avoid_collision_;
  
  // Motion
  bspline::Bspline newest_traj_; 
  Eigen::Vector3d start_pos_, start_vel_, start_acc_, start_yaw_;

  // Odom
  Eigen::Vector3d odom_pos_, odom_vel_;  
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;

  // For Ground 
  ros::Time get_next_cor_time_;
  bool have_next_cor_, have_drone_odom_; 
  Eigen::Vector3d drone_pos_;
  Eigen::Quaterniond drone_orient_;
  double drone_yaw_;
};

struct FSMParam {
  int expl_method_;           // 0-Proposed, 1-Classic, 2-FUEL, 3-RACER
  bool show_ground_;          // Show the ground voxels of all Divs
  bool show_shape_;           // Show the 3D shape of all Divs (True - 3D, False - 2D)
  bool is_drone_; 

  double replan_thresh1_;     // time_to_end < replan_thresh1_
  double replan_thresh2_;     // t_cur_relative > replan_thresh2_ && isFrontierCovered() 
  double replan_thresh3_;     // t_cur_relative > replan_thresh3_ && !classic
  double replan_time_;        // Forward time to chose the start pos (Current Traj) for Next Traj 

  // Swarm
  double attempt_interval_;   // Min interval of opt attempt
  double pair_opt_interval_;  // Min interval of successful pair opt
  int repeat_send_num_;

  // Color
  Eigen::Vector4d color_traj_, color_cover_, color_expl_;
};

struct DroneState {
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  double yaw_;
  
  double stamp_;                 // Time Stamp of pos, vel, yaw
  double recent_attempt_time_;   // Time Stamp of latest opt attempt with any drone
  double recent_interact_time_;  // Time Stamp of latest opt with this drone

  vector<int> grid_ids_;       
};

struct ExplParam {
  bool is_drone_;
  int expl_task_, expl_method_, drone_id_, drone_num_; 
  ros::Time trigger_time_; 
  string result_dir_;   

  // Map
  int voxel_number;
  double box_min_x_, box_min_y_, box_min_z_;
  double box_max_x_, box_max_y_, box_max_z_; 
  Vector3d box_min_, box_max_;

  // FIS
  double max_decay_;  
  int top_view_num_;   

  // Planning
  int init_motion_plan_num_;  // Motion planning init number
  double relax_time_;         // Yaw Planning

  // Refine
  bool refine_local_;         // Do viewpoints refine
  int refined_num_;           // Num of clusteres need to refine 
  double refined_radius_;     // Radius of refine space 

  // Dir 
  string process_dir_;
  string compute_dir_;
  string tsp_dir_, mtsp_dir_;  
};

struct ExplData {

  // 
  // General
  //
  
  bool triggered_;        // Trigger the exploration (For record exploration process)
  bool begin_;            // First time to do exploration planning
  bool greedy_;           // Force to execute the greedy explore
  bool need_expl_replan_; // Allow to Explore Replan
  bool to_viewpoint_;     // Next pos is viewpoint
  bool loop_;             // Next Corner Div is loop
  bool follow_;           // Follow the Drone
  bool reset_;            // Reset Next Viewpoint if not find path

  int unknown_number_;    // Unknown voxels number  
  double coverage_;       // Coverage Rate

  int motion_plan_num_;   // Motion planning number (For init)
  int plan_num_;          // Expl planning number

  // Frontier
  vector<Vector3d> averages_;                 // Average Positon of cells of Frontiers
  vector<vector<Vector3d>> frontiers_;        // Position of each cells of Frontiers  
  vector<vector<Vector3d>> dead_frontiers_;    

  vector<pair<Vector3d, Vector3d>> frontier_boxes_;           // Bounding Box of each Frontiers

  Vector3d updated_box_min_, updated_box_max_;                // Updated Box
  pair<vector<Vector3d>, vector<Vector3d>> updated_box_vis_;  // Fov Vis 

  // Viewpoint
  vector<Vector3d> top_points_;  // Pos of Top Viewpoints
  vector<double> top_yaws_;      // Yaw of Top Viewpoints
  vector<Vector3d> views_;       // View Line of Top Viewpoints (For Vis) 
  vector<Frontier> ftrs_;        // All Frontiers (vector)

  vector<Vector3d> frontier_tour_;
  vector<vector<Vector3d>> other_tours_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;

  vector<Vector3d> path_next_goal_;
  vector<Vector3d> kino_path_;

  Vector3d next_goal_;  
  Vector3d next_pos_;   
  double next_yaw_;    

  // Refine
  vector<int> refined_ids_;             // Index of cluster whose top vp need to refine (Discard)
  vector<vector<Vector3d>> n_points_;   // Positon of viewpoints need to refine (Discard)
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_;
  vector<Vector3d> refined_views1_;
  vector<Vector3d> refined_views2_;
  vector<Vector3d> refined_tour_;

  // Division
  unordered_map<int, shared_ptr<Division>> all_divs_;

  shared_ptr<Division> cur_div_;      // Current Div of robot
  shared_ptr<Division> ori_div_;      // Start Div of Coverage Path
  shared_ptr<Division> next_div_;     // Last Div of Coverage Path
  
  
  //
  // Drone
  //

  bool finish_partial_; 
  bool find_coverage_;                  // Find the coverage path (For Explore Replanning)

  vector<Frontier> chosen_ftrs_;        // Chosen Frontiers for Exploration Planning

  vector<Vector3d> init_path_;          // A* Path for motion planning
  vector<Vector3d> coverage_path_;      // Coverage Path
  vector<Vector3d> exploration_path_;   // Exploration Path

  vector<Vector3d> grid_tour_;          
  vector<Vector3d> grid_tour2_;   
 
  // Swarm
  vector<DroneState> swarm_state_; 
  vector<int> last_grid_ids_;
  vector<double> pair_opt_stamps_, pair_opt_res_stamps_;
  vector<int> ego_ids_, other_ids_;
  double pair_opt_stamp_;
  bool reallocated_, wait_response_;

  // Division
  unordered_map<int, shared_ptr<Division>> activated_divs_;   // Divs which will join the Coverage Path Planning
  unordered_map<int, shared_ptr<Division>> basic_divs_;       // Basic Divs (Ori Div + Next Div) (For vis)

  shared_ptr<Division> next_corner_div_; // Corner Div corresponding to Next Div


  //
  // Ground
  //
 
  Vector3d drone_pos_;        // Drone Pos
  double drone_yaw_;          // Drone Yaw
  
  int next_cor_id_;           // ID of current Corner Div
  vector<int> next_cor_ids_;  // IDs of Next Corner Divs 
  
  shared_ptr<Division> next_corner_div_ground_;
  unordered_map<int, shared_ptr<Division>> forget_divs_;  // Explored Divs but remain ftrs
};


} // namespace fast_planner

#endif