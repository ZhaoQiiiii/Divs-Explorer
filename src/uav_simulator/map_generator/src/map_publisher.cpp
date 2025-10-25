#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "map_pub");
  ros::NodeHandle nh("~");

  string map_path;
  double box_min_x, box_min_y, box_max_x, box_max_y;
  nh.param("map_path", map_path, string("null"));
  nh.param("box_min_x", box_min_x, -7.0);
  nh.param("box_min_y", box_min_y, -15.0);
  nh.param("box_max_x", box_max_x, 7.0);
  nh.param("box_max_y", box_max_y, 15.0);

  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 10, true);
  ros::Duration(1.0).sleep();

  // Load Point Cloud from .pcd 
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, cloud) == -1) { 
    ROS_ERROR("[Map Pub] Can't Read Map File."); 
    return -1;
  }

  // Add Ground Point Cloud
  for (double x = box_min_x; x <= box_max_x; x += 0.1) {
    for (double y = box_min_y; y <= box_max_y; y += 0.1) {
      cloud.push_back(pcl::PointXYZ(x, y, 0));
    }
  }
  
  // Pub the Point Cloud
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "world";
  while (ros::ok()) {
    ros::Duration(0.2).sleep();
    cloud_pub.publish(msg);
  }

  return 0;
}