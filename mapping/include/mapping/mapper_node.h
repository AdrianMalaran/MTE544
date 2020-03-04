#ifndef MAPPER_NODE_H
#define MAPPER_NODE_H

#include <ros/console.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include "ros/ros.h"

using namespace std;

class Mapping {
private:
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber real_pose_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher map_pub_;

  double occupancy_threshold_ = 0.5;
  vector<vector<double>> map_;

public:
  Mapping();
  void CreateMap();
  void PoseCallback(const gazebo_msgs::ModelStates& msg);
  void PrintMap(vector<vector<double>> map);
  void ScanCallback(const sensor_msgs::LaserScan& msg);
};

#endif
