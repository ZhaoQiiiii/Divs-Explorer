#ifndef _KD_TREE_H_
#define _KD_TREE_H_

#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <memory>
#include <utility>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using std::vector;
using std::priority_queue;
using std::pair;
using std::shared_ptr;
using Eigen::Vector3d;

namespace fast_planner 
{

struct KDCompare {
  bool operator()(const pair<double, Vector3d>& lhs, const pair<double, Vector3d>& rhs) {
    return lhs.first > rhs.first; 
  }
};


struct KDNode 
{
  Vector3d point;  
  KDNode* left;    
  KDNode* right;  

  KDNode(const Vector3d& p) : point(p), left(nullptr), right(nullptr) {}
};


class KDTree 
{
public:
  KDTree() : root(nullptr) {}

  void build(const vector<Vector3d>& points);
  vector<Vector3d> nearestK(const Vector3d& target, const shared_ptr<EDTEnvironment>& edt, 
                            int k, double clearance);

private:
  KDNode* root;
  KDNode* buildRecursive(const vector<Vector3d>& points, int depth);
  void nearestKRecursive(KDNode* node, const Vector3d& target, const int& depth, const shared_ptr<EDTEnvironment>& edt,
                         priority_queue<pair<double, Vector3d>, vector<pair<double, Vector3d>>, KDCompare>& pq, 
                         int k, double clearance);
};


}

#endif