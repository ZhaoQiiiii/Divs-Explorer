#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;


class RayCaster;


namespace fast_planner {

class EDTEnvironment;
class PerceptionUtils;
struct DivisionParam;


struct Viewpoint {
  Vector3d pos_;          // Position sampled at the average position of Frontier
  double yaw_;            // Optimal yaw
  int visib_num_;         // Number of visible voxels (Count by raycaster)
  list<double> costs_;    // Cost to other vp (Temp, Just used in the Exploration Path Planning)

  // double fraction_;    // Fraction of the cluster that can be covered
};


struct Frontier {
  int id_;  

  Vector3d average_;                // Average position of frontiers         
  Vector3d box_min_, box_max_;      // Bounding box of cluster, center & 1/2 side length
  
  vector<Vector3d> cells_;          // Voxels belonging to the cluster
  vector<Vector3d> filtered_cells_; // Down-sampled Voxels filtered by voxel grid filter

  vector<Viewpoint> vps_;           // Sampled Viewpoints 

  list<vector<Vector3d>> paths_;    // Paths to other clusers
  list<double> costs_;              // Time costs to other clusers
};


class FrontierFinder {
public:
  FrontierFinder(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  ~FrontierFinder();

  // Frontiers Container
  list<Frontier> frontiers_;
  list<Frontier> tmp_frontiers_;       // Temporarily contain the Frontiers, and insert it to old Frontiers
  list<Frontier> dormant_frontiers_;   // The Frontiers without finding viewpoints

  // Main API (FIS)
  void searchFrontiers();
  void computeFrontiersToVisit();
  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);
  void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Vector3d>& points, vector<double>& yaws, vector<Vector3d>& averages);
  void getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids, const int& view_num, const double& max_decay, vector<vector<Vector3d>>& points, vector<vector<double>>& yaws);
  void updateFrontierCostMatrix();
  void getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw, Eigen::MatrixXd& mat);

  // Coverage Path & Exploration Path
  void getSwarmCostMatrix(const vector<Vector3d>& positions, const vector<Vector3d>& velocities, const vector<double> yaws, Eigen::MatrixXd& mat);
  void getSwarmCostMatrix(const vector<Vector3d>& positions, const vector<Vector3d>& velocities, const vector<double>& yaws, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos, Eigen::MatrixXd& mat);
  void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path);

  // Frontier Finder Utils
  bool isFrontierCovered();
  void wrapYaw(double& yaw);
  int computeGainOfView(const Eigen::Vector3d& pos, const double& yaw);
  int deleteFrontiers(const vector<uint16_t>& ids);
  int addFrontiers(const vector<pair<Eigen::Vector3d, double>>& views);

  shared_ptr<PerceptionUtils> percep_utils_;

private:

  shared_ptr<EDTEnvironment> edt_env_;
  shared_ptr<DivisionParam> dp_;
  unique_ptr<RayCaster> raycaster_;

  // Data
  vector<char> frontier_flag_;
  vector<int> removed_ids_;
  list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_, min_candidate_clearance_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;

  // Helper
  void splitLargeFrontiers(list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, list<Frontier>& splits);
  void mergeFrontiers(Frontier& ftr1, const Frontier& ftr2);
  bool isFrontierChanged(const Frontier& ft);
  bool haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2);
  bool haveAnyOverlap(const Vector3d& min1, const Vector3d& max1, const vector<Vector3d>& mins,const vector<Vector3d>& maxs);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  void sampleViewpoints(Frontier& frontier);
  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);
  int countVisibleCells(const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster);

  bool isNearUnknown(const Vector3d& pos);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  int toadr(const Eigen::Vector3i& idx);
  bool knownfree(const Eigen::Vector3i& idx);
  bool inmap(const Eigen::Vector3i& idx);

  // Deprecated
  Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i& pt);
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx);
  bool canBeMerged(const Frontier& ftr1, const Frontier& ftr2);
  void findViewpoints(const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps);
};

}  // namespace fast_planner

#endif