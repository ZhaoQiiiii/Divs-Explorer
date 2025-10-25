#include <string.h>

#include <Eigen/Eigen>
#include <iostream>

#include "armadillo"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "pose_utils.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"

using namespace arma;
using namespace std;

bool is_drone_;
int drone_id_, drone_num_;
static string mesh_resource;

static double cov_scale;
bool cross_config = false;
bool tf45 = false;
bool cov_pos = false;
bool cov_vel = false;
bool cov_color = false;
bool origin = false;
bool isOriginSet = false;
colvec poseOrigin(6);

ros::Publisher meshPub;
ros::Publisher posePub;
ros::Publisher pathPub;
ros::Publisher velPub;
ros::Publisher covPub;
ros::Publisher covVelPub;
ros::Publisher trajPub;
ros::Publisher sensorPub;
ros::Publisher heightPub;

tf::TransformBroadcaster* broadcaster;
geometry_msgs::PoseStamped poseROS;
nav_msgs::Path pathROS;
visualization_msgs::Marker velROS;
visualization_msgs::Marker covROS;
visualization_msgs::Marker covVelROS;
visualization_msgs::Marker trajROS;
visualization_msgs::Marker sensorROS;
sensor_msgs::Range heightROS;
string frame_id_;

Eigen::Vector4d getColor(const double& h, double alpha) {
  double h1 = h;
  if (h1 < 0.0 || h1 > 1.0) {
    std::cout << "h out of range" << std::endl;
    h1 = 0.0;
  }

  double lambda;
  Eigen::Vector4d color1, color2;
  if (h1 >= -1e-4 && h1 < 1.0 / 6) {
    lambda = (h1 - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);
  } else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) {
    lambda = (h1 - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);
  } else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) {
    lambda = (h1 - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);
  } else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) {
    lambda = (h1 - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);
  } else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) {
    lambda = (h1 - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) {
    lambda = (h1 - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}

// Sub "/state_ukf/odom"
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  if (msg->header.frame_id == string("null")) return;

  // 
  // Pub Mesh model
  //

  // Basic
  visualization_msgs::Marker meshROS;
  meshROS.header.frame_id = frame_id_;
  meshROS.header.stamp = msg->header.stamp;
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;

  // Position
  meshROS.pose.position.x = msg->pose.pose.position.x;
  meshROS.pose.position.y = msg->pose.pose.position.y;
  meshROS.pose.position.z = msg->pose.pose.position.z;

  // Orientation 
  colvec q(4);
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;

  if (cross_config) {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }
  
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);

  // Color & Scale
  if (is_drone_) {
    auto color = getColor((drone_id_ - 1) / double(drone_num_ - 1), 1);
    meshROS.color.r = color[0];
    meshROS.color.g = color[1];
    meshROS.color.b = color[2];
    meshROS.color.a = color[3];
    meshROS.scale.x = 1.0;
    meshROS.scale.y = 1.0;
    meshROS.scale.z = 1.0;
  }

  else {
    meshROS.color.r = 0.0;
    meshROS.color.g = 0.0;
    meshROS.color.b = 0.0;
    meshROS.color.a = 1.0;
    meshROS.scale.x = 1.5;
    meshROS.scale.y = 1.5;
    meshROS.scale.z = 1.5;
  }

  meshROS.mesh_resource = mesh_resource;
  meshPub.publish(meshROS);
}

// Sub "cmd"
void cmd_callback(const quadrotor_msgs::PositionCommand cmd) {
  if (cmd.header.frame_id == string("null")) return;

  colvec pose(6);
  pose(0) = cmd.position.x;
  pose(1) = cmd.position.y;
  pose(2) = cmd.position.z;
  colvec q(4);
  q(0) = 1.0;
  q(1) = 0.0;
  q(2) = 0.0;
  q(3) = 0.0;
  pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));

  // Mesh model
  visualization_msgs::Marker meshROS;
  meshROS.header.frame_id = frame_id_;
  meshROS.header.stamp = cmd.header.stamp;
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = cmd.position.x;
  meshROS.pose.position.y = cmd.position.y;
  meshROS.pose.position.z = cmd.position.z;

  if (cross_config) {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }
  meshROS.pose.orientation.w = q(0);
  meshROS.pose.orientation.x = q(1);
  meshROS.pose.orientation.y = q(2);
  meshROS.pose.orientation.z = q(3);
  meshROS.scale.x = 2.0;
  meshROS.scale.y = 2.0;
  meshROS.scale.z = 2.0;
  meshROS.color.a = 1.0;
  meshROS.color.r = 0.0;
  meshROS.color.g = 0.0;
  meshROS.color.b = 0.0;
  meshROS.mesh_resource = mesh_resource;
  meshPub.publish(meshROS);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_visualization");
  ros::NodeHandle nh("~");

  // Params
  nh.param("is_drone", is_drone_, true);
  nh.param("drone_id", drone_id_, 1);
  nh.param("drone_num", drone_num_, 1);
  nh.param("frame_id", frame_id_, string("world"));
  // nh.param("frame_id", frame_id_, string("/world"));
  nh.param("mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/hummingbird.mesh"));
  nh.param("tf45", tf45, false);
  nh.param("origin", origin, false);
  nh.param("cross_config", cross_config, false);
  nh.param("covariance_scale", cov_scale, 100.0);
  nh.param("covariance_position", cov_pos, false);
  nh.param("covariance_velocity", cov_vel, false);
  nh.param("covariance_color", cov_color, false);

  // Ros Utils
  ros::Subscriber sub_odom = nh.subscribe("odom", 100, odom_callback);
  ros::Subscriber sub_cmd = nh.subscribe("cmd", 100, cmd_callback);
  meshPub = nh.advertise<visualization_msgs::Marker>("robot", 100, true);
  
  posePub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100, true);
  pathPub = nh.advertise<nav_msgs::Path>("path", 100, true);
  velPub = nh.advertise<visualization_msgs::Marker>("velocity", 100, true);
  covPub = nh.advertise<visualization_msgs::Marker>("covariance", 100, true);
  covVelPub = nh.advertise<visualization_msgs::Marker>("covariance_velocity", 100, true);
  trajPub = nh.advertise<visualization_msgs::Marker>("trajectory", 100, true);
  sensorPub = nh.advertise<visualization_msgs::Marker>("sensor", 100, true);
  heightPub = nh.advertise<sensor_msgs::Range>("height", 100, true);

  ros::spin();
  return 0;
}
