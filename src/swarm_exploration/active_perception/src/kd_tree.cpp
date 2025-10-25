#include <active_perception/kd_tree.h>

namespace fast_planner 
{

// Build the KD Tree
void KDTree::build(const vector<Vector3d>& points) {
  root = buildRecursive(points, 0);
}

KDNode* KDTree::buildRecursive(const vector<Vector3d>& points, int depth) {
  if (points.empty()) return nullptr;

  // 选择轴：0=x, 1=y, 2=z
  int axis = depth % 3;
  auto sorted_points = points;

  // 按当前轴排序
  std::sort(sorted_points.begin(), sorted_points.end(), 
           [axis](const Vector3d& a, const Vector3d& b) { return a(axis) < b(axis); });

  // 找到中位数
  size_t median_idx = sorted_points.size() / 2;
  KDNode* node = new KDNode(sorted_points[median_idx]);

  // 递归构建子树
  vector<Vector3d> left_points(sorted_points.begin(), sorted_points.begin() + median_idx);
  vector<Vector3d> right_points(sorted_points.begin() + median_idx + 1, sorted_points.end());
  node->left = buildRecursive(left_points, depth + 1);
  node->right = buildRecursive(right_points, depth + 1);

  return node;
}


// Return K Nearest Neighbor Points with clearance
vector<Vector3d> KDTree::nearestK(const Vector3d& target, const shared_ptr<EDTEnvironment>& edt, 
                                  int k, double clearance) 
{
  priority_queue<pair<double, Vector3d>, vector<pair<double, Vector3d>>, KDCompare> pq; // 最小堆
  nearestKRecursive(root, target, 0, edt, pq, k, clearance);

  vector<Vector3d> result;
  while (!pq.empty()) {
    result.push_back(pq.top().second);
    pq.pop();
  }
  return result; 
}

void KDTree::nearestKRecursive(KDNode* node, const Vector3d& target, const int& depth, const shared_ptr<EDTEnvironment>& edt,
                               priority_queue<pair<double, Vector3d>, vector<pair<double, Vector3d>>, KDCompare>& pq, 
                               int k, double clearance) 
{
  if (!node) return;
  double dist = (node->point - target).squaredNorm();

  // 只考虑距离大于 clearance 的点
  if (dist > clearance * clearance && !edt->sdf_map_->getInflateOccupancy(node->point)) {
    if (pq.size() < k) {
      pq.push({dist, node->point});
    } 
    else if (dist < pq.top().first) {
      pq.pop();
      pq.push({dist, node->point});
    }
  }

  // 选择下一个轴
  int axis = depth % 3;
  KDNode* near_node = (target(axis) < node->point(axis)) ? node->left : node->right;
  KDNode* far_node = (target(axis) < node->point(axis)) ? node->right : node->left;

  // 递归进入近邻
  nearestKRecursive(near_node, target, depth + 1, edt, pq, k, clearance);

  // 检查是否需要搜索远邻
  if (pq.size() < k || fabs(target(axis) - node->point(axis)) < sqrt(pq.top().first)) {
    nearestKRecursive(far_node, target, depth + 1, edt, pq, k, clearance);
  }
}

}