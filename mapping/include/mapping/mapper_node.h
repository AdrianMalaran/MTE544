#ifndef MAPPER_NODE_H
#define MAPPER_NODE_H

#include <ros/console.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include "ros/ros.h"

using namespace std;

enum State {
  UNKNOWN=0,
  OCCUPIED=1,
  FREE=2
};

struct Cell {
  double log_odds = 0.5;
  State state = UNKNOWN;
};

class Mapping {
private:
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber real_pose_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber position_sub_;
  ros::Publisher map_pub_;

  bool position_set_ = false;
  geometry_msgs::Pose pose_;
  tf::Transform world_to_robot_;

  double occupancy_threshold_ = 0.5;
  int width_;
  int height_;
  vector<vector<Cell>> map_;

public:
  Mapping();
  void CreateMap();
  void PoseCallback(const gazebo_msgs::ModelStates& msg);
  void PrintMap(vector<vector<Cell>> map);
  void ScanCallback(const sensor_msgs::LaserScan& msg);
  void IndoorPositionCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
};

#endif
