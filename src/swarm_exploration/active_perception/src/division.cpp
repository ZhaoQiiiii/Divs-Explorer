#include <active_perception/division.h>

namespace fast_planner {

Division::Division() {
  id_ = -1;
  had_ftrs_ = false;
}

Division::Division(const Vector3d &start, const Vector3d &end, const int &index,
                   const shared_ptr<DivisionParam> &params, const shared_ptr<EDTEnvironment> &edt) {

  id_ = index;
  findDivisionNeighbors(params->div_number_);

  box_min_ = start;
  box_max_ = end;
  reformDivison(box_min_, box_max_);
  size_ = box_max_ - box_min_;
  center_ = box_min_ + size_ / 2.0;

  getVerticesAndNormals(vertices_, normals_);

  DivisionUtils::getBoxDrawing(box_min_, box_max_, draw_);
  edt_ = edt;

  status_ = SPARE;
  had_ftrs_ = false;
};

Division::~Division() {}

//
// Division Constructor
//

// Find the IDs of Div's neighbors
void Division::findDivisionNeighbors(const Vector3i &number) {

  // 
  int x_num = number[0];
  int y_num = number[1];
  int z_num = number[2];

  int row = id_ / x_num;
  int col = id_ % x_num;

  ids_nbrs_four_.clear();
  ids_nbrs_eight_.clear();

  // 4 Expand
  vector<pair<int, int>> offsets_1 = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

  for (const auto &offset : offsets_1) {
    int new_row = row + offset.first;
    int new_col = col + offset.second;

    if (new_row >= 0 && new_row < y_num && new_col >= 0 && new_col < x_num) {
      int neighbor_id = (new_row * x_num) + new_col;
      ids_nbrs_four_.insert(neighbor_id);
    }
  }

  // 8 Expand
  vector<pair<int, int>> offsets_2 = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

  for (const auto &offset : offsets_2) {
    int new_row = row + offset.first;
    int new_col = col + offset.second;

    if (new_row >= 0 && new_row < y_num && new_col >= 0 && new_col < x_num) {
      int neighbor_id = (new_row * x_num) + new_col;
      ids_nbrs_eight_.insert(neighbor_id);
    }
  }
}

// Corret the Boundary of Box of Div
void Division::reformDivison(Vector3d &box_min, Vector3d &box_max) {

  Vector3d reformed_box_min = box_min;
  Vector3d reformed_box_max = box_max;

  if (reformed_box_min[0] > reformed_box_max[0]) {
    std::swap(reformed_box_min[0], reformed_box_max[0]);
  }
  if (reformed_box_min[1] > reformed_box_max[1]) {
    std::swap(reformed_box_min[1], reformed_box_max[1]);
  }
  if (reformed_box_min[2] > reformed_box_max[2]) {
    std::swap(reformed_box_min[2], reformed_box_max[2]);
  }

  box_min = reformed_box_min;
  box_max = reformed_box_max;
}

// Get the Vertices and Normals of Div
void Division::getVerticesAndNormals(vector<Vector3d> &vertices, vector<Eigen::Vector3d> &normals) {

  // Four Vertices of Division (xy plane, z = box_min[2])
  Eigen::Vector3d left_bottom, right_top, left_top, right_bottom;

  left_bottom = box_min_;

  right_top = box_max_;
  right_top[2] = left_bottom[2];

  left_top[0] = left_bottom[0];
  left_top[1] = right_top[1];
  left_top[2] = left_bottom[2];

  right_bottom[0] = right_top[0];
  right_bottom[1] = left_bottom[1];
  right_bottom[2] = left_bottom[2];

  vertices = {left_bottom, right_bottom, right_top, left_top};

  // Normals of four separating lines of Four Vertices
  normals.clear();
  for (int j = 0; j < 4; ++j) {
    Eigen::Vector3d dir = (vertices[(j + 1) % 4] - vertices[j]).normalized();
    normals.push_back(dir);
  }
}

//
// Division Utils
//

// Divide the whole Exlporation Space
void DivisionUtils::divideExplorationSpace(const Vector3d &box_min, const Vector3d &box_max,
                                           const shared_ptr<EDTEnvironment> &edt, shared_ptr<DivisionParam> &params,
                                           unordered_map<int, shared_ptr<Division>> &divs) {

  //
  // 1、Set the Division Params
  //

  // Box & Origin
  const Vector3d box = box_max - box_min;
  params->box_min_ = box_min;
  params->box_max_ = box_max;
  params->div_origin_ = {box_min[0], box_min[1], box_min[2]};

  // Size & Number
  double size = params->div_size_; // Divide Length
  int x_num = ceil((box[0]) / size);
  int y_num = ceil((box[1]) / size);
  int z_num = 1; // Just one Lawyer for Exploration Space
  params->div_number_ = {x_num, y_num, z_num};
  params->div_total_number_ = x_num * y_num * z_num;

  // Real Division Resolution (Size -> Number -> Resolution)
  Vector3d resolution;
  resolution[0] = box[0] / double(x_num);
  resolution[1] = box[1] / double(y_num);
  resolution[2] = box[2] / double(z_num);
  params->div_res_ = resolution;

  // Map Resolution
  params->map_res_ = edt->sdf_map_->getResolution();

  // Voxel Number (Just in the Divs)
  double voxel_total_number = 0;
  const double res = edt->sdf_map_->getResolution();
  for (double x = box_min[0]; x <= box_max[0]; x += res) {
    for (double y = box_min[1]; y <= box_max[1]; y += res) {
      for (double z = box_min[2]; z <= box_max[2]; z += res) {
        voxel_total_number++;
      }
    }
  }
  params->div_voxel_total_number_ = voxel_total_number;
  params->div_voxel_number_ = voxel_total_number / (x_num * y_num * z_num);

  //
  // 2、Create the Divs with Division Params
  //

  divs.clear();
  int index = 0;
  for (int z = 0; z < z_num; z++) {
    for (int y = 0; y < y_num; y++) {
      for (int x = 0; x < x_num; x++) {
        Vector3d start = {box_min[0] + resolution[0] * x, box_min[1] + resolution[1] * y, box_min[2]};

        Vector3d end = {box_min[0] + resolution[0] * x + resolution[0], box_min[1] + resolution[1] * y + resolution[1],
                        box_max[2]};

        auto div = make_shared<Division>(start, end, index, params, edt);
        divs.insert({index, div});
        index++;
      }
    }
  }

  //
  // 3、Initial the Center and Coverage Rate of all Divs
  //

  for (const auto &[id, div] : divs) {
    Vector3d box_min = div->box_min_;
    Vector3d box_max = div->box_max_;

    // Free & Unknown & Ground
    Vector3d &unknown_center = div->unknown_center_;
    int &unknown_number = div->unknown_number_;
    vector<Vector3d> &unknown_ = div->unknown_;
    unknown_center.setZero();
    unknown_number = 0;
    unknown_.clear();

    Vector3d &free_center = div->free_center_;
    int &free_number = div->free_number_;
    vector<Vector3d> &free_ = div->free_;
    free_center.setZero();
    free_number = 0;
    free_.clear();

    vector<Vector3d> &ground_ = div->ground_;
    ground_.clear();

    for (double x = box_min[0]; x <= box_max[0]; x += res) {
      for (double y = box_min[1]; y <= box_max[1]; y += res) {
        for (double z = box_min[2]; z <= box_max[2]; z += res) {

          Eigen::Vector3d pos(x, y, z);
          if (!DivisionUtils::checkPointInside(pos, div))
            continue;

          int state = div->edt_->sdf_map_->getOccupancy(pos);
          if (state == SDFMap::FREE) {
            free_center = (free_center * free_number + pos) / double(free_number + 1);
            free_number += 1;
            free_.push_back(pos);
            if (pos(2) == 0)
              ground_.push_back(pos);
          }

          else if (state == SDFMap::UNKNOWN) {
            unknown_center = (unknown_center * unknown_number + pos) / double(unknown_number + 1);
            unknown_number += 1;
            unknown_.push_back(pos);
          }

          else if (state == SDFMap::OCCUPIED) {
            if (pos(2) == 0)
              ground_.push_back(pos);
          }
        }
      }
    }

    // Coverage Rate
    div->coverage_rate_ = double(params->div_voxel_number_ - unknown_number) / double(params->div_voxel_number_);
  }

  //
  // 4、Get the IDs of Corner Divs
  //

  // Clockwise
  Vector3d center_corner_1 = {box_min[0] + 0.01, box_min[1] + 0.01, box_min[2]};
  Vector3d center_corner_2 = {box_min[0] + 0.01, box_max[1] - 0.01, box_min[2]};
  Vector3d center_corner_3 = {box_max[0] - 0.01, box_max[1] - 0.01, box_min[2]};
  Vector3d center_corner_4 = {box_max[0] - 0.01, box_min[1] + 0.01, box_min[2]};
  vector<Vector3d> centers_corner = {center_corner_1, center_corner_2, center_corner_3, center_corner_4};

  int id_corner_1 = DivisionUtils::findDivisionID(params, center_corner_1);
  int id_corner_2 = DivisionUtils::findDivisionID(params, center_corner_2);
  int id_corner_3 = DivisionUtils::findDivisionID(params, center_corner_3);
  int id_corner_4 = DivisionUtils::findDivisionID(params, center_corner_4);
  vector<int> ids_corner = {id_corner_1, id_corner_2, id_corner_3, id_corner_4};

  // Get the IDs of four Corner Divs (for Division params)
  params->ids_corner_cw_ = ids_corner;
  for (int id : ids_corner)
    params->ids_corner_.insert(id);

  // Get the corresponding Corner ID (for each Div)
  for (auto [id, div] : divs) {
    int index;
    double dist_min = std::numeric_limits<double>::max();
    for (int i = 0; i < centers_corner.size(); i++) {
      double dist = (div->center_ - centers_corner[i]).norm();
      if (dist < dist_min) {
        index = i;
        dist_min = dist;
      }
    }
    div->id_corner_ = ids_corner[index];
  }

  //
  // 5、Compute the Boundary Cost
  //

  for (auto [id, div] : divs)
    DivisionUtils::computeBoundaryCost(div, params);
  ROS_WARN("The Exploration Space has been Divided.");
}

// Update the Divs Data
void DivisionUtils::updateDivData(const shared_ptr<FrontierFinder> &ftr_finder, const shared_ptr<DivisionParam> &params,
                                  const unordered_map<int, shared_ptr<Division>> &divs, const int &ori_id) {

  double res = params->map_res_;

  for (const auto &[id, div] : divs) {

    // Frontier
    div->ftrs_.clear();
    for (const auto &ftr : ftr_finder->frontiers_) {
      if (DivisionUtils::checkPointInside(ftr.average_, div))
        div->ftrs_.push_back(ftr);
    }

    // Free & Unknown & Ground
    Vector3d &unknown_center = div->unknown_center_;
    int &unknown_number = div->unknown_number_;
    vector<Vector3d> &unknown_ = div->unknown_;
    unknown_center.setZero();
    unknown_number = 0;
    unknown_.clear();

    Vector3d &free_center = div->free_center_;
    int &free_number = div->free_number_;
    vector<Vector3d> &free_ = div->free_;
    free_center.setZero();
    free_number = 0;
    free_.clear();

    vector<Vector3d> &ground_ = div->ground_;
    ground_.clear();

    Vector3d box_min = div->box_min_;
    Vector3d box_max = div->box_max_;

    for (double x = box_min[0]; x <= box_max[0]; x += res) {
      for (double y = box_min[1]; y <= box_max[1]; y += res) {
        for (double z = box_min[2]; z <= box_max[2]; z += res) {

          Eigen::Vector3d pos(x, y, z);
          if (!DivisionUtils::checkPointInside(pos, div))
            continue;

          int state = div->edt_->sdf_map_->getOccupancy(pos);
          if (state == SDFMap::FREE) {
            free_center = (free_center * free_number + pos) / double(free_number + 1);
            free_number += 1;
            free_.push_back(pos);
            if (pos(2) == 0)
              ground_.push_back(pos);
          }

          else if (state == SDFMap::UNKNOWN) {
            unknown_center = (unknown_center * unknown_number + pos) / double(unknown_number + 1);
            unknown_number += 1;
            unknown_.push_back(pos);
          }

          else if (state == SDFMap::OCCUPIED) {
            if (pos(2) == 0)
              ground_.push_back(pos);
          }
        }
      }
    }

    // Coverage Rate
    div->coverage_rate_ = double(params->div_voxel_number_ - unknown_number) / double(params->div_voxel_number_);

    // Status
    if (params->complete_expl_ && params->is_drone_) { // Complete Explore
      for (auto [id, div] : divs) {
        if (id == ori_id)
          continue;
        if (div->status_ == EXPLORED && !div->ftrs_.empty())
          div->status_ = ACTIVATED;
        else if (div->status_ != EXPLORED && div->ftrs_.empty() && div->had_ftrs_)
          div->status_ = EXPLORED;
      }
    }

    else { // Partial Explore || Ground
      for (auto [id, div] : divs) {
        if (id == ori_id)
          continue;
        if (div->status_ == EXPLORED && div->coverage_rate_ <= params->coverage_rate_thr_)
          div->status_ = ACTIVATED;
        else if (div->status_ != EXPLORED && div->coverage_rate_ > params->coverage_rate_thr_)
          div->status_ = EXPLORED;
      }
    }

    // Explore Replan Flag
    if (!div->had_ftrs_ && !div->ftrs_.empty())
      div->had_ftrs_ = true;
  }
}

// Count number of status
bool DivisionUtils::countDivNumber(const unordered_map<int, shared_ptr<Division>> &divs) {
  int spare = 0, activated = 0, explored = 0;
  for (auto [id, div] : divs) {
    if (div->status_ == SPARE)
      spare++;
    else if (div->status_ == ACTIVATED)
      activated++;
    else if (div->status_ == EXPLORED)
      explored++;
  }
  return activated == 0;
}

// Check one point if inside one Div
bool DivisionUtils::checkPointInside(const Vector3d &point, const shared_ptr<Division> &div) {

  // Check the dir dot with normals
  for (int m = 0; m < 4; ++m) {
    if ((point - div->vertices_[m]).dot(div->normals_[m]) <= 0.0)
      return false;
  }
  return true;
}

// Check one point if inside Divs Map
bool DivisionUtils::isInDivsMap(const shared_ptr<DivisionParam> &params, const Vector3d &point, bool conserve) {

  if (!conserve) {
    Vector3d box_min = params->box_min_;
    Vector3d box_max = params->box_max_;

    if (box_min(0) <= point(0) && point(0) <= box_max(0) && box_min(1) <= point(1) && point(1) <= box_max(1) &&
        box_min(2) <= point(2) && point(2) <= box_max(2))
      return true;
  }

  else {
    double reduce = 0.5;
    Vector3d box_min = {params->box_min_(0) + reduce, params->box_min_(1) + reduce, params->box_min_(2)};
    Vector3d box_max = {params->box_max_(0) - reduce, params->box_max_(1) - reduce, params->box_max_(2)};

    if (box_min(0) <= point(0) && point(0) <= box_max(0) && box_min(1) <= point(1) && point(1) <= box_max(1) &&
        box_min(2) <= point(2) && point(2) <= box_max(2))
      return true;
  }

  return false;
}

// Find id of division to which one point belongs
int DivisionUtils::findDivisionID(const shared_ptr<DivisionParam> &params, const Vector3d &point) {

  // Point not inside Div Map
  if (!DivisionUtils::isInDivsMap(params, point))
    return -1;

  // Get the 3D ID
  Vector3i id;
  for (int i = 0; i < 2; ++i) {
    id(i) = floor((point[i] - params->div_origin_[i]) / params->div_res_[i]);
  }
  id[2] = 0;

  // Get the Linear ID
  int id_address = id[1] * params->div_number_[0] + id[0];
  return id_address;
}

// Get the Valid Unknown or Free Center of Div
bool DivisionUtils::getValidCenter(const shared_ptr<Division> &div, Vector3d &center) {

  bool unknown = div->unknown_number_ >= div->free_number_;
  if (unknown)
    return getValidUnknownCenter(div, center);
  else
    return getValidFreeCenter(div, center);
}

bool DivisionUtils::getValidUnknownCenter(const shared_ptr<Division> &div, Vector3d &center) {

  // Unknown Center
  center = div->unknown_center_;
  if (div->edt_->sdf_map_->getInflateOccupancy(center) == 1) {
    if (!DivisionUtils::findNearestFreePoint(div, true, center))
      return false;
  }
  return true;
}

bool DivisionUtils::getValidFreeCenter(const shared_ptr<Division> &div, Vector3d &center) {

  // Free Center
  center = div->free_center_;
  if (div->edt_->sdf_map_->getInflateOccupancy(center) == 1) {
    if (!DivisionUtils::findNearestFreePoint(div, false, center))
      return false;
  }
  return true;
}

// Find the nearest free point of Div with KD tree
bool DivisionUtils::findNearestFreePoint(const shared_ptr<Division> &div, const bool &unknown, Vector3d &center) {

  // Choose Free or Unknown Voxels Set
  vector<Vector3d> points;
  if (unknown)
    points = div->unknown_;
  else
    points = div->free_;

  // Regenerate New Center with KD Tree
  KDTree kdtree;
  kdtree.build(points);

  int k = 20;
  double safety_clerance = 1.5;
  vector<Vector3d> nearest_points = kdtree.nearestK(center, div->edt_, k, safety_clerance);

  for (const Vector3d &pt : nearest_points) {
    if (!div->edt_->sdf_map_->getInflateOccupancy(pt)) {
      center = pt;
      return true;
    }
  }
  return false;
}

// Get the Drawing Info of Division Box
void DivisionUtils::getBoxDrawing(const Vector3d &box_min, const Vector3d &box_max,
                                  pair<vector<Vector3d>, vector<Vector3d>> &drawing, double offset) {

  bool draw_3d = false;

  vector<Vector3d> list1;
  vector<Vector3d> list2;

  double x_min = box_min[0];
  double x_max = box_max[0];

  double y_min = box_min[1];
  double y_max = box_max[1];

  double z_min = box_min[2] + offset;
  double z_max = box_max[2] + offset;

  Vector3d bottom_upper_left(x_min, y_max, z_min);
  Vector3d bottom_upper_right(x_max, y_max, z_min);
  Vector3d bottom_lower_left(x_min, y_min, z_min);
  Vector3d bottom_lower_right(x_max, y_min, z_min);

  Vector3d top_upper_left(x_min, y_max, z_max);
  Vector3d top_upper_right(x_max, y_max, z_max);
  Vector3d top_lower_left(x_min, y_min, z_max);
  Vector3d top_lower_right(x_max, y_min, z_max);

  if (draw_3d) {
    // Top rectangle
    list1.push_back(top_upper_left);
    list2.push_back(top_upper_right);

    list1.push_back(top_upper_right);
    list2.push_back(top_lower_right);

    list1.push_back(top_lower_right);
    list2.push_back(top_lower_left);

    list1.push_back(top_lower_left);
    list2.push_back(top_upper_left);

    // Bottom rectangle
    list1.push_back(bottom_upper_left);
    list2.push_back(bottom_upper_right);

    list1.push_back(bottom_upper_right);
    list2.push_back(bottom_lower_right);

    list1.push_back(bottom_lower_right);
    list2.push_back(bottom_lower_left);

    list1.push_back(bottom_lower_left);
    list2.push_back(bottom_upper_left);

    // Side
    list1.push_back(top_upper_left);
    list2.push_back(bottom_upper_left);

    list1.push_back(top_upper_right);
    list2.push_back(bottom_upper_right);

    list1.push_back(top_lower_left);
    list2.push_back(bottom_lower_left);

    list1.push_back(top_lower_right);
    list2.push_back(bottom_lower_right);
  }

  else {
    list1.push_back(bottom_upper_left);
    list2.push_back(bottom_upper_right);

    list1.push_back(bottom_upper_right);
    list2.push_back(bottom_lower_right);

    list1.push_back(bottom_lower_right);
    list2.push_back(bottom_lower_left);

    list1.push_back(bottom_lower_left);
    list2.push_back(bottom_upper_left);
  }

  drawing = make_pair(list1, list2);
}

// Get the intersection of two unordered sets
unordered_set<int> DivisionUtils::getIntersection(unordered_set<int> set1, unordered_set<int> set2) {

  unordered_set<int> intersection;
  for (const auto &element : set1) {
    if (set2.find(element) != set2.end())
      intersection.insert(element);
  }
  return intersection;
}

// Get the Divs which overlap with Updated Box
void DivisionUtils::getOverlapDivs(const Vector3d &updated_min, const Vector3d &updated_max,
                                   const unordered_map<int, shared_ptr<Division>> &divs,
                                   unordered_map<int, shared_ptr<Division>> &overlap_divs,
                                   unordered_set<int> &overlap_ids) {

  // Define the Lamda Function to check overlap (xy plane)
  auto have_overlap = [](const Vector3d &min1, const Vector3d &max1, const Vector3d &min2, const Vector3d &max2) {
    for (int m = 0; m < 2; ++m) {
      double bmin = max(min1[m], min2[m]);
      double bmax = min(max1[m], max2[m]);
      if (bmin > bmax + 1e-3)
        return false;
    }
    return true;
  };

  // Get the Overlap Divs
  overlap_divs.clear();
  overlap_ids.clear();
  for (const auto &[id, div] : divs) {
    if (have_overlap(div->box_min_, div->box_max_, updated_min, updated_max)) {
      overlap_divs.insert({id, div});
      overlap_ids.insert(id);
    }
  }
}

// Check one Div if overlap with Updated Box
bool DivisionUtils::checkOverlap(const Vector3d &updated_min, const Vector3d &updated_max,
                                 const shared_ptr<Division> &div) {

  // Define the Lamda Function to check overlap (xy plane)
  auto have_overlap = [](const Vector3d &min1, const Vector3d &max1, const Vector3d &min2, const Vector3d &max2) {
    for (int m = 0; m < 2; ++m) {
      double bmin = max(min1[m], min2[m]);
      double bmax = min(max1[m], max2[m]);
      if (bmin > bmax + 1e-3)
        return false;
    }
    return true;
  };

  return have_overlap(div->box_min_, div->box_max_, updated_min, updated_max);
}

// Compute the Boundary Cost of Div
void DivisionUtils::computeBoundaryCost(const shared_ptr<Division> &div, const shared_ptr<DivisionParam> &params) {

  double size = params->div_size_;
  Vector3d box_min = params->box_min_;
  Vector3d box_max = params->box_max_;

  // double x_dist = min(abs(div->center_(0) - box_min(0)), abs(div->center_(0) - box_max(0))) - size / 2.0;
  // double y_dist = min(abs(div->center_(1) - box_min(1)), abs(div->center_(1) - box_max(1))) - size / 2.0;

  double x_dist = min(abs(div->center_(0) - box_min(0)), abs(div->center_(0) - box_max(0)));
  double y_dist = min(abs(div->center_(1) - box_min(1)), abs(div->center_(1) - box_max(1)));

  int step_x = static_cast<int>(round(x_dist / size));
  int step_y = static_cast<int>(round(y_dist / size));

  div->cost_boundary = size * (min(step_x, step_y) + 1);
  if (params->ids_corner_.find(div->id_) != params->ids_corner_.end())
    div->cost_boundary -= size;
}

// Get the Step between two Divs
int DivisionUtils::getStepBetweenDivs(const shared_ptr<Division> &div1, const shared_ptr<Division> &div2, double size) {

  double dx = abs(div2->center_(0) - div1->center_(0));
  double dy = abs(div2->center_(1) - div1->center_(1));
  double dz = abs(div2->center_(2) - div1->center_(2));

  int step_x = static_cast<int>(round(dx / size));
  int step_y = static_cast<int>(round(dy / size));
  int step_z = static_cast<int>(round(dz / size));

  return step_x + step_y;
  // return step_x + step_y + step_z;
}

// Get the Rotation of Corner Div
bool DivisionUtils::getRotationCornerDiv(const shared_ptr<Division> &next_div,
                                         const shared_ptr<Division> &next_corner_div,
                                         const shared_ptr<DivisionParam> &params) {

  // True - Counter Clockwise
  // False - Clockwise

  Vector3d box_min = params->box_min_;
  Vector3d box_max = params->box_max_;
  Vector3d origin = (box_max + box_min) / 2.0;
  origin(2) = next_div->center_(2);

  Vector3d next_corner_center = next_corner_div->center_;
  Vector3d dir1 = (origin - next_corner_center).normalized();
  Vector3d dir2 = (next_div->center_ - next_corner_center).normalized();

  // dir2 in the left side of dir1 (cross > 0) -> Counter Clockwise
  // dir2 in the right side of dir1 (cross < 0) -> Clockwise
  double cross = dir1.x() * dir2.y() - dir1.y() * dir2.x();
  return cross >= 0;
}

// Get the ID of Next Corner Div
int DivisionUtils::getNextCornerDivID(int last_corner_id, vector<int> ids_corner, bool rotation) {

  // Office1
  // 0 -> 20 -> 23 -> 3 -> 0 (Counter Clockwise) (rotation = True)
  // 0 -> 3 -> 23 -> 20 -> 0 (Clockwise) (rotation = False)

  // Check valid of last id
  auto it = std::find(ids_corner.begin(), ids_corner.end(), last_corner_id);
  if (it == ids_corner.end())
    return -1;

  // Find new id
  int new_index;
  int cur_index = std::distance(ids_corner.begin(), it);
  if (rotation)
    new_index = (cur_index == 0) ? ids_corner.size() - 1 : cur_index - 1; // Counter Clockwise
  else
    new_index = (cur_index + 1) % ids_corner.size(); // Clockwise
  return ids_corner[new_index];
}

} // namespace fast_planner