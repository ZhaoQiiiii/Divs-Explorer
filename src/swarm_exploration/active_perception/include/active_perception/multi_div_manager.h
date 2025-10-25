#ifndef _MULTI_DIV_MANAGER_H
#define _MULTI_DIV_MANAGER_H

#include <ros/ros.h>
#include <memory>
#include <random>
#include <vector>
#include <unordered_map>

#include <active_perception/division.h>
#include <active_perception/DivisionPkg.h>
#include <active_perception/DivisionData.h>


using std::shared_ptr;
using std::vector;
using std::unordered_map;

namespace fast_planner {

class FastExplorationManager;
class Division;

class MultiDivManager {
public:
  MultiDivManager();
  ~MultiDivManager();
  void init(const shared_ptr<DivisionParam>& params, const unordered_map<int, shared_ptr<Division>>& divs);

  // Main API
  void syncDivData(const unordered_map<int, shared_ptr<Division>>& divs);
  typedef shared_ptr<MultiDivManager> Ptr;

private:
  // Data Type
  typedef active_perception::DivisionPkg DivisionPkg;
  typedef active_perception::DivisionPkgConstPtr DivisionPkgPtr;
  typedef active_perception::DivisionData DivisionData;

  ros::NodeHandle nh_;
  ros::Publisher data_pub_;
  ros::Subscriber data_sub_;

  // Callbacks
  void recvDataCallback(const DivisionPkgPtr& msg);

  // Helper
  void convertDivToData(const shared_ptr<Division>& div, DivisionData& data);
  void convertDataToDiv(const DivisionData& data, shared_ptr<Division>& div);

  // Params
  bool is_drone_;
  int drone_id_, drone_num_, div_num_;
  shared_ptr<DivisionParam> dp_;

  // Data
  unordered_map<int, shared_ptr<Division>> all_divs_;   // All Divisions in this drone

  double last_recv_msg_time_;               // Record last receive message time in this drone
  vector<double> last_update_divs_times_;   // Record last update Divs time in this drone

  friend FastExplorationManager;
};

} // namespace fast_planner

#endif