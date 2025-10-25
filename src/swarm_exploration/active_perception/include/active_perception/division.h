#ifndef _DIVISION_H_
#define _DIVISION_H_

#include <Eigen/Eigen>
#include <list>
#include <memory>
#include <ros/ros.h>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <active_perception/frontier_finder.h>
#include <active_perception/graph_node.h>
#include <active_perception/kd_tree.h>
#include <active_perception/multi_div_manager.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector3d;
using Eigen::Vector3i;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unordered_map;
using std::unordered_set;
using std::vector;

namespace fast_planner {

class EDTEnvironment;
class SDFMap;
class FrontierFinder;
class MultiDivManager;

enum DIVSION_STATUS { SPARE, ACTIVATED, EXPLORED };

struct DivisionParam {
  bool is_drone_;            // Drone or Ground
  bool complete_expl_;       // Complete or Partial Explore
  double coverage_rate_thr_; // Coverage rate threshold (For Partial Expl)

  Vector3d box_min_; // Min boundary of Exploration Space
  Vector3d box_max_; // Max boundary of Exploration Space

  Vector3d div_origin_; // Origin of Divs

  double div_inflation_; // Inflation factor of Div (set t_lb when move to center)
  double div_size_;      // Divide Length for Divs
  Vector3d div_res_;     // Real Resolution of Divs (x_res, y_res, z_res)
  double map_res_;       // Resolution of Grid Map

  int div_total_number_; // Total Number of Div
  Vector3i div_number_;  // Number Vector of Div (x_num, y_num, z_num)

  int div_voxel_number_;       // Voxel Number in one Div
  int div_voxel_total_number_; // Voxel Number in all Divs

  unordered_set<int> ids_corner_; // IDs of four Corner Divs
  vector<int> ids_corner_cw_;     // IDs of four Corner Divs with clockwise

  typedef shared_ptr<DivisionParam> Ptr;
};

class Division {

public:
  Division();
  Division(const Vector3d &start, const Vector3d &end, const int &index, const shared_ptr<DivisionParam> &params,
           const shared_ptr<EDTEnvironment> &edt);
  ~Division();

  // Static Data
  int id_;                            // Own id
  int id_corner_;                     // Corner id (corresponding)
  unordered_set<int> ids_nbrs_four_;  // Neighbor ids (4 Expand)
  unordered_set<int> ids_nbrs_eight_; // Neighbor ids (8 Expand)

  Vector3d size_;    // Size (x_size, y_size, z_size = box_z) (Quasi Square)
  Vector3d center_;  // Geometric Center
  Vector3d box_min_; // Box Min Boundary
  Vector3d box_max_; // Box Max Boundary

  double cost_boundary;              // Cost of Corner Dist and Box Dist
  vector<Eigen::Vector3d> vertices_; // Four Vertices of Box(xy plane)
  vector<Eigen::Vector3d> normals_;  // Dir Vector of four Separating Lines (Vertices connection Lines in xy plane)

  pair<vector<Vector3d>, vector<Vector3d>> draw_; // Drawing Info of Box

  // Dynamic Data
  int status_;            // Exploration Status (Spare / Activated / Explored)
  list<double> costs_;    // Time cost to other Divs (Temp, Just used in the Coverage Path Planning)
  vector<Frontier> ftrs_; // Frontiers (average pos inside Div) (No need Sync because of Multi Map Manager)
  bool had_ftrs_;         // Explore Replan Flag (Div had frotiners before)

  // Dynamic Data (need Sync)
  int unknown_number_, free_number_;
  Vector3d unknown_center_, free_center_;
  vector<Vector3d> unknown_, free_, ground_;
  double coverage_rate_; // Coverage Rate = (Free + Occupied) / (Free + Unknown + Occupied)

  shared_ptr<EDTEnvironment> edt_;

private:
  // Division Constructor
  void findDivisionNeighbors(const Vector3i &number);
  void reformDivison(Vector3d &box_min, Vector3d &box_max);
  void getVerticesAndNormals(vector<Vector3d> &vertices, vector<Eigen::Vector3d> &normals);
};

// Division Utils
namespace DivisionUtils {
void divideExplorationSpace(const Vector3d &box_min, const Vector3d &box_max, const shared_ptr<EDTEnvironment> &edt,
                            shared_ptr<DivisionParam> &params, unordered_map<int, shared_ptr<Division>> &divs);
void updateDivData(const shared_ptr<FrontierFinder> &ftr_finder, const shared_ptr<DivisionParam> &params,
                   const unordered_map<int, shared_ptr<Division>> &divs, const int &ori_id);

bool countDivNumber(const unordered_map<int, shared_ptr<Division>> &divs);
bool isInDivsMap(const shared_ptr<DivisionParam> &params, const Vector3d &point, bool conserve = false);
bool checkPointInside(const Vector3d &point, const shared_ptr<Division> &div);
int findDivisionID(const shared_ptr<DivisionParam> &params, const Vector3d &point);

bool getValidCenter(const shared_ptr<Division> &div, Vector3d &center);
bool getValidUnknownCenter(const shared_ptr<Division> &div, Vector3d &center);
bool getValidFreeCenter(const shared_ptr<Division> &div, Vector3d &center);
bool findNearestFreePoint(const shared_ptr<Division> &div, const bool &unknown, Vector3d &point);

void getBoxDrawing(const Vector3d &box_min, const Vector3d &box_max, pair<vector<Vector3d>, vector<Vector3d>> &drawing,
                   double offset = 0.0);
unordered_set<int> getIntersection(unordered_set<int> set1, unordered_set<int> set2);

void getOverlapDivs(const Vector3d &updated_min, const Vector3d &updated_max,
                    const unordered_map<int, shared_ptr<Division>> &divs,
                    unordered_map<int, shared_ptr<Division>> &overlap_divs, unordered_set<int> &overlap_ids);
bool checkOverlap(const Vector3d &updated_min, const Vector3d &updated_max, const shared_ptr<Division> &div);

void computeBoundaryCost(const shared_ptr<Division> &div, const shared_ptr<DivisionParam> &params);
int getStepBetweenDivs(const shared_ptr<Division> &div1, const shared_ptr<Division> &div2, double size);

bool getRotationCornerDiv(const shared_ptr<Division> &next_div, const shared_ptr<Division> &next_corner_div,
                          const shared_ptr<DivisionParam> &params);
int getNextCornerDivID(int last_corner_id, vector<int> ids_corner, bool rotation);
} // namespace DivisionUtils

} // namespace fast_planner

#endif