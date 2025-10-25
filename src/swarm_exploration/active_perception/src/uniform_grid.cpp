#include <active_perception/uniform_grid.h>
#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_env/multi_map_manager.h>

namespace fast_planner {

UniformGrid::UniformGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh, const int& level) 
{
  // Params
  this->edt_ = edt;
  nh.param("sdf_map/box_min_x", min_[0], 0.0);
  nh.param("sdf_map/box_min_y", min_[1], 0.0);
  nh.param("sdf_map/box_min_z", min_[2], 0.0);
  nh.param("sdf_map/box_max_x", max_[0], 0.0);
  nh.param("sdf_map/box_max_y", max_[1], 0.0);
  nh.param("sdf_map/box_max_z", max_[2], 0.0);
  nh.param("partitioning/min_unknown", min_unknown_, 10000);
  nh.param("partitioning/min_frontier", min_frontier_, 100);
  nh.param("partitioning/min_free", min_free_, 3000);
  nh.param("partitioning/consistent_cost", consistent_cost_, 3.5);
  nh.param("partitioning/w_unknown", w_unknown_, 3.5);

  double grid_size;
  nh.param("partitioning/grid_size", grid_size, 5.0);

  // resolution_ = size / 3;
  auto size = max_ - min_;
  for (int i = 0; i < 2; ++i) {
    int num = ceil(size[i] / grid_size);
    resolution_[i] = size[i] / double(num);
    for (int j = 1; j < level; ++j) {
      resolution_[i] *= 0.5;
    }
  }

  resolution_[2] = size[2];
  initialized_ = false;
  level_ = level;

  // path_finder_.reset(new Astar);
  // path_finder_->init(nh, edt);
}

UniformGrid::~UniformGrid() {
}

void UniformGrid::initGridData() 
{
  Eigen::Vector3d size = max_ - min_;
  for (int i = 0; i < 3; ++i) { grid_num_(i) = ceil(size(i) / resolution_[i]); }
  grid_data_.resize(grid_num_[0] * grid_num_[1] * grid_num_[2]);

  // std::cout << "data size: " << grid_data_.size() << std::endl;
  // std::cout << "grid num: " << grid_num_.transpose() << std::endl;
  // std::cout << "resolution: " << resolution_.transpose() << std::endl;

  for (int x = 0; x < grid_num_[0]; ++x) {
    for (int y = 0; y < grid_num_[1]; ++y) {
      for (int z = 0; z < grid_num_[2]; ++z) 
      {
        Eigen::Vector3i id(x, y, z);
        auto& grid = grid_data_[toAddress(id)];

        Eigen::Vector3d pos;
        indexToPos(id, 0.5, pos);
        if (use_swarm_tf_) pos = rot_sw_ * pos + trans_sw_;
        
        grid.center_ = pos;
        grid.unknown_num_ = resolution_[0] * resolution_[1] * resolution_[2] /
                            pow(edt_->sdf_map_->getResolution(), 3);

        grid.is_prev_relevant_ = true;
        grid.is_cur_relevant_ = true;
        grid.need_divide_ = false;
        if (level_ == 1) grid.active_ = true;
        else grid.active_ = false;
      }
    }
  }
}


//
// Main API
//

// Input Frontiers which Average inside into Grid
void UniformGrid::inputFrontiers(const vector<Eigen::Vector3d>& avgs) {
  for (auto& grid : grid_data_) grid.contained_frontier_ids_.clear();

  Eigen::Matrix3d Rt = rot_sw_.transpose();
  Eigen::Vector3d t_inv = -Rt * trans_sw_;

  for (int i = 0; i < avgs.size(); ++i) {
    Eigen::Vector3d pos = avgs[i];
    if (use_swarm_tf_) pos = Rt * pos + t_inv; // Average (World -> Grid)

    Eigen::Vector3i id; // ID of one grid in the Grids which Average inside
    posToIndex(pos, id);
    if (!insideGrid(id)) continue;

    auto& grid = grid_data_[toAddress(id)];
    grid.contained_frontier_ids_[i] = 1; 
  }
}

// Update the Grid Info + Allocate Grid IDs for single drone + Find grids which need to Fine
void UniformGrid::updateGridsData(const int& drone_id, vector<int>& grid_ids, 
                                  vector<int>& parti_ids, vector<int>& parti_ids_all) 
{
  for (auto& grid : grid_data_) grid.is_updated_ = false; 
  parti_ids.clear();

  //
  // 1、Get the Updated Boxes for Updating 
  //

  bool reset = (level_ == 2);

  // Get the Updated Box (ego-drone box)
  Vector3d update_min, update_max;
  edt_->sdf_map_->getUpdatedBox(update_min, update_max, reset);

  // Get the Chunk Updated Boxes (other-drones boxes)  
  vector<Eigen::Vector3d> update_mins, update_maxs;
  edt_->sdf_map_->mm_->getChunkBoxes(update_mins, update_maxs, reset);

  // Define the Lamda Function to check overlap (xy plane)
  auto have_overlap = [](const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2) 
  {
    for (int m = 0; m < 2; ++m) 
    {
      double bmin = max(min1[m], min2[m]);
      double bmax = min(max1[m], max2[m]);
      if (bmin > bmax + 1e-3) return false; 
    }
    return true;
  };


  //
  // 2、Check each uniform grids if overlap with any Updated Boxes and Update if necessary
  //

  vector<int> rediscovered_ids;
  for (int i = 0; i < grid_data_.size(); ++i) 
  {
    auto& grid = grid_data_[i];
    if (!grid.active_) continue;

    // Check if overlap with Chunk Updated Boxes
    bool overlap = false;
    for (int j = 0; j < update_mins.size(); ++j) {
      if (have_overlap(grid.vmin_, grid.vmax_, update_mins[j], update_maxs[j])) {
        overlap = true;
        break;
      }
    }

    // Check if overlap with Updated Box 
    bool overlap_with_fov = have_overlap(grid.vmin_, grid.vmax_, update_min, update_max);
    if (!overlap && !overlap_with_fov) continue; 


    // Update the grid if overlap with any Boxes
    Eigen::Vector3i idx; // ID of grid
    adrToIndex(i, idx);
    updateGridData(idx); // Free + Unknown + Relevant + Divide


    // Save the grid which need to divide into parti_ids_all
    if (grid.need_divide_) {
      parti_ids_all.push_back(i);
      grid.active_ = false;
    }

    // Re-discover the relevant grid for ego-drone 
    if (!overlap_with_fov) continue;
    if (!grid.is_prev_relevant_ && grid.is_cur_relevant_ && level_ > 1) {
      rediscovered_ids.push_back(i);
      // ROS_WARN("Grid %d is re-discovered", i);
    }
  }

  // Patch Code (To avoid incomplete update of Grid Lv2)
  for (auto id : extra_ids_) {
    if (grid_data_[id].is_updated_) continue; 

    Eigen::Vector3i idx;
    adrToIndex(id, idx);
    updateGridData(idx);
  }
  extra_ids_.clear();


  //
  // 3、Update the Allocated Grids for ego-drone
  //

  // Get the relevant grid
  relevant_id_.clear();
  relevant_map_.clear();
  for (int i = 0; i < grid_data_.size(); ++i) {
    if (isRelevant(grid_data_[i])) {
      relevant_id_.push_back(i);
      relevant_map_[i] = 1;
    }
  }

  // Update the Allocated Grids 
  if (!initialized_) 
  {
    if (drone_id == 1 && level_ == 1) { 
      grid_ids = relevant_id_; 
    }
    // else
    //   grid_ids = {};

    ROS_WARN("Init Grid Allocation.");
    initialized_ = true;
  } 

  else 
  {
    for (auto it = grid_ids.begin(); it != grid_ids.end();) 
    {
      // Remove irrelevant grids
      if (relevant_map_.find(*it) == relevant_map_.end()) {
        // std::cout << "Remove irrelevant: " << *it << std::endl;
        it = grid_ids.erase(it);
      } 

      // Partition and Remove coarse grids
      else if (grid_data_[*it].need_divide_) {
        // std::cout << "Remove divided: " << *it << std::endl;
        parti_ids.push_back(*it);
        it = grid_ids.erase(it);
        // grid_data_[*it].active_ = false;
      } 

      else ++it;
    }

    // Add re-discovered grids
    grid_ids.insert(grid_ids.end(), rediscovered_ids.begin(), rediscovered_ids.end());

    // sort(grid_ids.begin(), grid_ids.end());
  }
}

// Update the Info of one uniform grid (Free + Unknown + Relevant + Divide)
void UniformGrid::updateGridData(const Eigen::Vector3i& id) 
{
  // This grid overlaps with one certain Updated Box

  //
  // 1、Get the designated uniform grid with 3D index
  //

  int adr = toAddress(id); 
  auto& grid = grid_data_[adr]; 
  if (grid.is_updated_) { // Avoid repeated updating
    return;
  }
  grid.is_updated_ = true;
  grid.is_prev_relevant_ = grid.is_cur_relevant_;


  //
  // 2、Check the status of voxel and Count the unknown number for uniform grid
  //

  // Compute the Box Boundary of uniform grid 
  Eigen::Vector3d gmin, gmax;
  // indexToPos(id, 0.0, gmin);
  indexToPos(id, 1.0, gmax);

  // Define Lamda Function to Check one voxel if inside the rotated box
  auto inside_box = [](const Eigen::Vector3d& voxel, const GridInfo& grid) {
    for (int m = 0; m < 4; ++m) {
      if ((voxel - grid.vertices_[m]).dot(grid.normals_[m]) <= 0.0) return false;
    }
    return true;
  };

  // Count the number of Free & Unknown and Compute the Center of Unknown 
  const double res = edt_->sdf_map_->getResolution();
  grid.center_.setZero();
  grid.unknown_num_ = 0;
  int free = 0;
  for (double x = grid.vmin_[0]; x <= grid.vmax_[0]; x += res) {
    for (double y = grid.vmin_[1]; y <= grid.vmax_[1]; y += res) {
      for (double z = grid.vmin_[2]; z <= gmax[2]; z += res) {

        Eigen::Vector3d pos(x, y, z);
        if (!inside_box(pos, grid)) continue;

        int state = edt_->sdf_map_->getOccupancy(pos);
        if (state == SDFMap::FREE) {
          free += 1;
        } else if (state == SDFMap::UNKNOWN) {
          grid.center_ = (grid.center_ * grid.unknown_num_ + pos) / (grid.unknown_num_ + 1);
          grid.unknown_num_ += 1;
        }
      }
    }
  }


  //
  // 3、Check the uniform grid if need to plan and divide
  //

  grid.is_cur_relevant_ = isRelevant(grid);
  if (level_ == 1 && grid.active_ && free > min_free_) {
    grid.need_divide_ = true;
  }
}



//
// Helper
//

// Update the Base Coordinate and Basic Info for Grid
void UniformGrid::updateBaseCoor() 
{
  for (int i = 0; i < grid_data_.size(); ++i) 
  {
    auto& grid = grid_data_[i];
    // if (!grid.active_) continue;

    Eigen::Vector3i id;
    adrToIndex(i, id);

    // Compute the four Vertices of uniform grid (Drone Frame -> World Frame)
    Eigen::Vector3d left_bottom, right_top, left_top, right_bottom;
    indexToPos(id, 0.0, left_bottom);
    indexToPos(id, 1.0, right_top);
    left_top[0] = left_bottom[0];
    left_top[1] = right_top[1];
    left_top[2] = left_bottom[2];
    right_bottom[0] = right_top[0];
    right_bottom[1] = left_bottom[1];
    right_bottom[2] = left_bottom[2];
    right_top[2] = left_bottom[2];

    vector<Eigen::Vector3d> vertices = { left_bottom, right_bottom, right_top, left_top };
    if (use_swarm_tf_) { 
      for (auto& vert : vertices) vert = rot_sw_ * vert + trans_sw_; 
    }
    grid.vertices_ = vertices;


    // Compute the Boundary Box of uniform grid
    Eigen::Vector3d vmin, vmax;
    vmin = vmax = vertices[0];
    for (int j = 1; j < vertices.size(); ++j) {
      for (int k = 0; k < 2; ++k) {
        vmin[k] = min(vmin[k], vertices[j][k]);
        vmax[k] = max(vmax[k], vertices[j][k]);
      }
    }
    grid.vmin_ = vmin;
    grid.vmax_ = vmax;


    // Compute the Normals of four Separating Lines
    grid.normals_.clear();
    for (int j = 0; j < 4; ++j) {
      Eigen::Vector3d dir = (vertices[(j + 1) % 4] - vertices[j]).normalized();
      grid.normals_.push_back(dir);
    }


    // std::cout << "Vertices of grid " << toAddress(id) << std::endl;
    // for (auto v : grid.vertices_)
    //   std::cout << v.transpose() << "; ";
    // std::cout << "\nNormals: " << std::endl;
    // for (auto n : grid.normals_)
    //   std::cout << n.transpose() << "; ";
    // std::cout << "\nbox: " << grid.vmin_.transpose() << ", " << grid.vmax_.transpose()
    //           << std::endl;
  }
}

// Check if inside Grid with ID of Point
bool UniformGrid::insideGrid(const Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) {
    if (id[i] < 0 || id[i] >= grid_num_[i]) {
      return false;
    }
  }
  return true;
}

// Check the uniform grid if need to plan
bool UniformGrid::isRelevant(const GridInfo& grid) {
  // return grid.unknown_num_ >= min_unknown_ || grid.frontier_num_ >= min_frontier_;
  // return grid.unknown_num_ >= min_unknown_ || !grid.frontier_cell_nums_.empty();
  return grid.unknown_num_ >= min_unknown_ || !grid.contained_frontier_ids_.empty();
}

// Activate grids
void UniformGrid::activateGrids(const vector<int>& ids) {
  for (auto id : ids) {
    grid_data_[id].active_ = true;
  }
  extra_ids_ = ids;  // To avoid incomplete update
}


// 
void UniformGrid::getCostMatrix(const vector<Eigen::Vector3d>& positions, const vector<Eigen::Vector3d>& velocities, 
                                const vector<int>& prev_first_grid, const vector<int>& grid_ids, Eigen::MatrixXd& mat) {
}

void UniformGrid::getGridTour(const vector<int>& ids, vector<Eigen::Vector3d>& tour) {
  tour.clear();
  for (int i = 0; i < ids.size(); ++i) {
    tour.push_back(grid_data_[ids[i]].center_);
  }
}

void UniformGrid::getFrontiersInGrid(const int& grid_id, vector<int>& ftr_ids) {
  // Find frontier having more than 1/4 within the first grid
  auto& first_grid = grid_data_[grid_id];
  ftr_ids.clear();
  // for (auto pair : first_grid.frontier_cell_nums_) {
  //   ftr_ids.push_back(pair.first);
  // }
  for (auto pair : first_grid.contained_frontier_ids_) {
    ftr_ids.push_back(pair.first);
  }
}

void UniformGrid::getGridMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2) {

  Eigen::Vector3d p1 = min_;
  Eigen::Vector3d p2 = min_ + Eigen::Vector3d(max_[0] - min_[0], 0, 0);
  for (int i = 0; i <= grid_num_[1]; ++i) {
    Eigen::Vector3d pt1 = p1 + Eigen::Vector3d(0, resolution_[1] * i, 0);
    Eigen::Vector3d pt2 = p2 + Eigen::Vector3d(0, resolution_[1] * i, 0);
    pts1.push_back(pt1);
    pts2.push_back(pt2);
  }

  p1 = min_;
  p2 = min_ + Eigen::Vector3d(0, max_[1] - min_[1], 0);
  for (int i = 0; i <= grid_num_[0]; ++i) {
    Eigen::Vector3d pt1 = p1 + Eigen::Vector3d(resolution_[0] * i, 0, 0);
    Eigen::Vector3d pt2 = p2 + Eigen::Vector3d(resolution_[0] * i, 0, 0);
    pts1.push_back(pt1);
    pts2.push_back(pt2);
  }
  for (auto& p : pts1) p[2] = 0.5;
  for (auto& p : pts2) p[2] = 0.5;
}


// Convert 3D Index to Linear Address
int UniformGrid::toAddress(const Eigen::Vector3i& id) {
  return id[0] * grid_num_(1) * grid_num_(2) + id[1] * grid_num_(2) + id[2];
}

// Convert Linear Address to 3D Index
void UniformGrid::adrToIndex(const int& adr, Eigen::Vector3i& idx) 
{
  int tmp_adr = adr;
  const int a = grid_num_(1) * grid_num_(2);
  const int b = grid_num_(2);

  idx[0] = tmp_adr / a;
  tmp_adr = tmp_adr % a;
  idx[1] = tmp_adr / b;
  idx[2] = tmp_adr % b;
}

// Compute the 3D Index of uniform grid with one point
void UniformGrid::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) {
    id(i) = floor((pos(i) - min_(i)) / resolution_[i]);
  }
}

// Compute the Box Boundary of uniform grid with 3D Index and increment
void UniformGrid::indexToPos(const Eigen::Vector3i& id, const double& inc, Eigen::Vector3d& pos) {
  // inc: 0.0 for min, 1.0 for max, 0.5 for mid point
  for (int i = 0; i < 3; ++i) {
    pos(i) = (id(i) + inc) * resolution_[i] + min_(i);
  }
}

} // fast_planner
