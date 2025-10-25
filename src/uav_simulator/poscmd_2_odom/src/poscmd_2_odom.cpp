#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PositionCommand.h"

ros::Subscriber cmd_sub_;
ros::Publisher odom_pub_;

quadrotor_msgs::PositionCommand cmd_;
bool rcv_cmd_ = false;

double init_x_, init_y_, init_z_;
int drone_id_;

// Sub "/planning/pos_cmd"
void rcvPosCmdCallBack(const quadrotor_msgs::PositionCommand cmd) {	
	rcv_cmd_ = true;
	cmd_ = cmd;
}

// Pub "/state_ukf/odom"
void pubOdom() {	
  // 
  // Simulate Kinematics Model to Pub Odom
  //

	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "world";
	odom.child_frame_id = std::to_string(drone_id_);


	if(rcv_cmd_) { 

    // Position                                                    
    odom.pose.pose.position.x = cmd_.position.x;
    odom.pose.pose.position.y = cmd_.position.y;
    odom.pose.pose.position.z = cmd_.position.z;

    // Orientation
    Eigen::Matrix3d R;
    Eigen::Vector3d alpha = Eigen::Vector3d(cmd_.acceleration.x, cmd_.acceleration.y, cmd_.acceleration.z) 
                            + 9.8 * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d xC = {cos(cmd_.yaw), sin(cmd_.yaw), 0};
    Eigen::Vector3d yC = {-sin(cmd_.yaw), cos(cmd_.yaw), 0};
    Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
    Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
    Eigen::Vector3d zB = xB.cross(yB);
    R.col(0) = xB;
    R.col(1) = yB;
    R.col(2) = zB;

    Eigen::Quaterniond q(R);
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    // Linear Vel
    odom.twist.twist.linear.x = cmd_.velocity.x;
    odom.twist.twist.linear.y = cmd_.velocity.y;
    odom.twist.twist.linear.z = cmd_.velocity.z;

    // Angular Vel (Simplify)
    odom.twist.twist.angular.x = cmd_.acceleration.x;
    odom.twist.twist.angular.y = cmd_.acceleration.y;
    odom.twist.twist.angular.z = cmd_.acceleration.z;
                  

    // else { // Ground 

    //   // Position
    //   odom.pose.pose.position.x = cmd_.position.x;
    //   odom.pose.pose.position.y = cmd_.position.y;
    //   odom.pose.pose.position.z = center_height_; // cmd_.position.z 

    //   // Orientation
    //   double yaw = std::atan2(cmd_.velocity.y, cmd_.velocity.x);
    //   Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    //   odom.pose.pose.orientation.w = q.w();
    //   odom.pose.pose.orientation.x = q.x();
    //   odom.pose.pose.orientation.y = q.y();
    //   odom.pose.pose.orientation.z = q.z();

    //   // Linear Vel
    //   odom.twist.twist.linear.x = cmd_.velocity.x;
    //   odom.twist.twist.linear.y = cmd_.velocity.y;
    //   odom.twist.twist.linear.z = 0.0; // cmd_.velocity.z

    //   // Angular Vel
    //   odom.twist.twist.angular.x = 0.0; 
    //   odom.twist.twist.angular.y = 0.0; 
    //   odom.twist.twist.angular.z = std::abs(cmd_.velocity.x) > 1e-6 ? 2.0 * cmd_.velocity.y / cmd_.velocity.x : 0.0; 
    // }
	}

	else {
		odom.pose.pose.position.x = init_x_;
    odom.pose.pose.position.y = init_y_;
    odom.pose.pose.position.z = init_z_;

    odom.pose.pose.orientation.w = 1;
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
	}

  odom_pub_.publish(odom); // "/state_ukf/odom"
}


int main (int argc, char** argv) {    

  ros::init(argc, argv, "odom_generator");
  ros::NodeHandle nh("~");

  nh.param("init_x", init_x_, 0.0);
  nh.param("init_y", init_y_, 0.0);
  nh.param("init_z", init_z_, 0.0);
  nh.param("drone_id", drone_id_, 1);

  cmd_sub_ = nh.subscribe("command", 1, rcvPosCmdCallBack); 
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odometry", 1);                      

  ros::Rate rate(100);
  while(ros::ok()) {
    pubOdom();                   
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}