#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <traj_utils/planning_visualization.h>
#include <active_perception/division.h>
#include <exploration_manager/NextCor.h>


using Eigen::Vector3d;
using Eigen::MatrixXd;

using std::vector;
using std::pair;
using std::tuple;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_set;
using std::unordered_map;

namespace fast_planner {

class EDTEnvironment;
class SDFMap;
class FastPlannerManager;
// class UniformGrid;
class HGrid;
class FrontierFinder;
class MultiDivManager;

struct ExplParam;
struct ExplData;
struct DivisionParam;

enum EXPL_RESULT { SUCCEED, FAIL, NO_DIVISION, NO_FRONTIER, NO_GRID};


class FastExplorationManager {
public:
  FastExplorationManager();
  ~FastExplorationManager();
  void initialize(ros::NodeHandle& nh);

  // Modules
  int updateFrontierStruct(const Vector3d& pos);
  int generateExploreTraj(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw);

  // Expl Planner API
  int planExploreDrone(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw); // Proposed
  int planExploreGround(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw, const Vector3d& drone_pos, const double& drone_yaw); 
  
  int racerExplore(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw); // RACER
  int fuelExpore(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw); // FUEL
  int classicExpore(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw); // Classic

  // RACER
  void allocateGrids(const vector<Vector3d>& positions,const vector<Vector3d>& velocities, const vector<vector<int>>& first_ids, const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<int>& ego_ids, vector<int>& other_ids);
  bool findGlobalTourOfGrid(const vector<Vector3d>& positions, const vector<Vector3d>& velocities, vector<int>& ids, vector<vector<int>>& others, bool init = false);
  double computeGridPathCost(const Vector3d& pos, const vector<int>& grid_ids, const vector<int>& first, const vector<vector<int>>& firsts, const vector<vector<int>>& seconds, const double& w_f);

  shared_ptr<ExplParam> ep_;
  shared_ptr<ExplData> ed_;
  shared_ptr<DivisionParam> dp_;
  shared_ptr<MultiDivManager> md_;

  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<EDTEnvironment> edt_environment_;
  shared_ptr<SDFMap> sdf_map_;
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<PlanningVisualization> vis_;

  shared_ptr<HGrid> hgrid_;
  // shared_ptr<UniformGrid> uniform_grid_;

  
private:
typedef exploration_manager::NextCor NextCor;
typedef exploration_manager::NextCorConstPtr NextCorPtr;

  ros::ServiceClient tsp_client_, acvrp_client_;
  ros::Publisher next_cor_pub_; // For Ground trigger

  // Coverage Path
  void getCandidateDivs(const Vector3d& cur_pos);
  void findCoveragePath(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, 
    unordered_map<int, shared_ptr<Division>> divs, shared_ptr<Division>& next_div, vector<Vector3d>& tour);
  void computeCoverageCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, 
    unordered_map<int, shared_ptr<Division>> divs, vector<int>& div_ids, MatrixXd& cost_mat);
  void getCoveragePathVis(unordered_map<int, shared_ptr<Division>> divs, vector<int> new_div_ids, vector<Vector3d>& tour);
  void getNextCornerDiv(shared_ptr<Division>& next_corner_div);

  // Exploration Path
  void getChosenFrontiers(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<Frontier>& vps_sets);
  int findExplorationPath(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, 
    const vector<Frontier>& vps_sets, Vector3d &next_pos, double &next_yaw, vector<Vector3d>& tour);
  void computeExplCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, 
    const vector<Frontier>& vps_sets, MatrixXd& cost_mat);                       
  void getExplorationPathVis(const Vector3d& cur_pos, const vector<int>& indices, 
    const vector<Frontier>& vps_sets, vector<Vector3d>& tour);

  // Greedy
  int greedyExplore(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, 
    const vector<Frontier>& ftrs, Vector3d& next_pos, double& next_yaw);
  int findBestViewpoint(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, 
    const Frontier& ftr, Vector3d& next_pos, double& next_yaw);

  // Helper
  void updateExplProcess();
  void updateDivsCost(const shared_ptr<Division>& divi, const shared_ptr<Division>& divj);
  void updateVpsCost(const vector<Viewpoint>::iterator& vpi, const vector<Viewpoint>::iterator& vpj);
  bool solveTSP(const Eigen::MatrixXd& cost_mat, vector<int>& indices, int problem_id = 1);
  void shortenPath(vector<Vector3d>& path);

  // FUEL
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw, vector<int>& indices);
  void getGlobalTourVis(const Vector3d& cur_pos, const vector<int>& indices, vector<Vector3d>& tour);

  // RACER
  void findGridAndFrontierPath(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids, vector<int>& ftr_ids); 
  void findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos, vector<int>& ids);
  void refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw, const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts, vector<double>& refined_yaws);

public:
  typedef shared_ptr<FastExplorationManager> Ptr;
};


}  // namespace fast_planner

#endif