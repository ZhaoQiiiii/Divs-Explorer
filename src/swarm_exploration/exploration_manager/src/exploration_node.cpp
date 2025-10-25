#include <exploration_manager/fast_exploration_fsm.h>
#include <plan_manage/backward.hpp>
#include <ros/ros.h>

namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char **argv) {
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh("~");

  FastExplorationFSM expl_fsm;
  expl_fsm.initialize(nh);
  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
