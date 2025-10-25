#ifndef _UNIFORM_GRID_H_
#define _UNIFORM_GRID_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <utility>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

class RayCaster;

namespace fast_planner {

class EDTEnvironment;
class Astar;
class HGrid;

class GridInfo {
public:
  GridInfo() {}
  ~GridInfo() {}

  int unknown_num_;         // Number of unknown voxel
  Eigen::Vector3d center_;  // Center of unknown voxel

  int frontier_num_;
  unordered_map<int, int> frontier_cell_nums_;
  unordered_map<int, int> contained_frontier_ids_; // IDs of Frontiers which Average inside Grid

  bool is_updated_;   // Grid if has been updated (Avoid repeated computation)

  bool active_;       // Grid if need to Vis 
  bool need_divide_;  // Grid if need to Divide  

  bool is_cur_relevant_;    // Current Grid if need to be planned
  bool is_prev_relevant_;   // Grid if planned last time

  Eigen::Vector3d vmin_, vmax_;       // Boundary Box of uniform grid (Just use the xy plane)
  vector<Eigen::Vector3d> vertices_;  // Four Vertices of uniform grid
  vector<Eigen::Vector3d> normals_;   // Dir Vector of four Separating Lines (Vertices connection Lines in xy plane)
};


// Uniform Grid (Grid Lv1 or Grid Lv2)
class UniformGrid {
public:
  UniformGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh, const int& level);
  ~UniformGrid();

  void initGridData();

  // Main API
  void inputFrontiers(const vector<Eigen::Vector3d>& avgs);
  void updateGridsData(const int& drone_id, vector<int>& grid_ids, 
                       vector<int>& parti_ids, vector<int>& parti_ids_all);
  void updateGridData(const Eigen::Vector3i& id);

  // Helper
  void updateBaseCoor();
  void activateGrids(const vector<int>& ids);

  void getCostMatrix(const vector<Eigen::Vector3d>& positions,
                     const vector<Eigen::Vector3d>& velocities, const vector<int>& prev_first_grid,
                     const vector<int>& grid_ids, Eigen::MatrixXd& mat);
  void getGridTour(const vector<int>& ids, vector<Eigen::Vector3d>& tour);
  void getFrontiersInGrid(const int& grid_id, vector<int>& ftr_ids);
  void getGridMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2);

private:
  shared_ptr<EDTEnvironment> edt_;
  unique_ptr<Astar> path_finder_;

  // Vector of Grid Data
  vector<GridInfo> grid_data_;

  // Helper
  bool insideGrid(const Eigen::Vector3i& id);
  bool isRelevant(const GridInfo& grid);

  int toAddress(const Eigen::Vector3i& id);
  void adrToIndex(const int& adr, Eigen::Vector3i& idx);
  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, const double& inc, Eigen::Vector3d& pos);

  vector<int> relevant_id_;
  unordered_map<int, int> relevant_map_;
  bool initialized_;
  vector<int> extra_ids_;

  Eigen::Vector3d resolution_;  // Size of one grid
  Eigen::Vector3d min_, max_;   // Box of one grid
  Eigen::Vector3i grid_num_;    // Number of Grid
  int level_;

  int min_unknown_, min_frontier_, min_free_;
  double consistent_cost_, inside_ratio_;
  double w_unknown_;

  // Swarm tf
  Eigen::Matrix3d rot_sw_;    // Rotation (Drone Frame -> World Frame)
  Eigen::Vector3d trans_sw_;  // Translation (Drone Frame -> World Frame)
  bool use_swarm_tf_;

  friend HGrid;
};

}  // namespace fast_planner
#endif