#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>


namespace fast_planner {

// Static data
double ViewNode::vm_;
double ViewNode::am_;
double ViewNode::yd_;
double ViewNode::ydd_;
double ViewNode::w_dir_;
shared_ptr<Astar> ViewNode::astar_;
shared_ptr<RayCaster> ViewNode::caster_;
shared_ptr<SDFMap> ViewNode::map_;

// Graph node for viewpoints planning
ViewNode::ViewNode(const Vector3d& p, const double& y) {
  pos_ = p;
  yaw_ = y;
  parent_ = nullptr;
  vel_.setZero();  // vel is zero by default, should be set explicitly
}

double ViewNode::costTo(const ViewNode::Ptr& node) {
  vector<Vector3d> path;
  double c = ViewNode::computeCostLB(pos_, node->pos_, yaw_, node->yaw_, vel_, yaw_dot_, path);
  // std::cout << "cost from " << id_ << " to " << node->id_ << " is: " << c << std::endl;
  return c;
}


//
// Main API
//

// A* Search
double ViewNode::searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path, bool optimistic) 
{
  //
  // 1、Try to connect two points with straight line
  //

  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    if (map_->getInflateOccupancy(idx) == 1 || !map_->isInBox(idx)) {
      // map_->getOccupancy(idx) == SDFMap::UNKNOWN
      safe = false;
      break;
    }
  }
  if (safe) {
    path = { p1, p2 };
    return (p1 - p2).norm();
  }
  

  //
  // 2、Try to search a path using decreasing resolution with Astar
  //

  vector<double> res = { 0.4 };
  for (int k = 0; k < res.size(); ++k) {
    astar_->reset();
    astar_->setResolution(res[k]);
    if (astar_->search(p1, p2, optimistic) == Astar::REACH_END) {
      path = astar_->getPath();
      return astar_->pathLength(path);
    }
  }

  // Use Astar early termination cost as an estimate
  path = { p1, p2 };
  return 1000;
}

// Cost of Time Lower Bound
double ViewNode::computeCostLB(const Vector3d& p1, const Vector3d& p2, const double& y1, const double& y2, 
                               const Vector3d& v1, const double& yd1, vector<Vector3d>& path, bool optimistic) {

  // Cost of position change
  double pos_cost = ViewNode::searchPath(p1, p2, path, optimistic) / vm_;

  if (v1.norm() > 1e-3) { // Consider velocity change
    Vector3d dir = (p2 - p1).normalized();
    Vector3d vdir = v1.normalized();
    double diff = acos(vdir.dot(dir));
    pos_cost += w_dir_ * diff;
    // double vc = v1.dot(dir);
    // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    // if (vc < 0)
    //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
  }

  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = diff / yd_;
  
  return max(pos_cost, yaw_cost);

  // Consider yaw rate change
  // if (fabs(yd1) > 1e-3) {
  //   double diff1 = y2 - y1;
  //   while (diff1 < -M_PI)
  //     diff1 += 2 * M_PI;
  //   while (diff1 > M_PI)
  //     diff1 -= 2 * M_PI;
  //   double diff2 = diff1 > 0 ? diff1 - 2 * M_PI : 2 * M_PI + diff1;
  // }
}

// Cost of Vel Consistency 
double ViewNode::computeCostCon(const double& y1, const double& y2, const Vector3d& v1) {

  Vector3d dir_1;
  if (v1.norm() < 1e-3) dir_1 = { cos(y1), sin(y1), 0 }; 
  else dir_1 = v1.normalized();

  Vector3d toward = { cos(y2), sin(y2), 0 }; 
  Vector3d dir_2 = toward.normalized();
  
  return acos(dir_1.dot(dir_2)); // [0, PI]
} 

// Cost of Toward
double ViewNode::computeCostToward(const Vector3d& p1, const Vector3d& p2, const double& y1) {

  Vector3d dir_1 = (p2 - p1).normalized();

  Vector3d toward = { cos(y1), sin(y1), 0 }; 
  Vector3d dir_2 = toward.normalized(); 

  return acos(dir_1.dot(dir_2)); // [0, PI]
}

} // fast_planner