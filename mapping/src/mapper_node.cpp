// Mapping Node Implementation
#include <mapping/mapper_node.h>


Mapping::Mapping() {
  // map_ = vector< >
  int length = 10;
  int width = 10;
  vector<vector<double>> temp_map(length, vector<double>(width, -1.0));

  pose_sub_ = nh_.subscribe("/gazebo/model_states", 1, &Mapping::PoseCallback, this);
  //TODO: Switch this to the position from the bag
  real_pose_sub_ = nh_.subscribe("/gazebo/model_states", 1, &Mapping::PoseCallback, this);
  scan_sub_ = nh_.subscribe("/scan", 1, &Mapping::ScanCallback, this);

  // nav_msgs::OccupancyGrid& msg;
  // TODO: Need a publisher for occupancy grid
  // ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, false);
  PrintMap(temp_map);
}

void Mapping::PrintMap(vector<vector<double>> map) {
  string map_debug_string = "\n";
  for (int row = 0; row < map.size(); row ++) {
    for (int col = 0; col < map[row].size(); col++) {
      map_debug_string += to_string(map[row][col]) + ", ";
    }
    map_debug_string += "\n";
  }

  ROS_INFO("Map: %s", map_debug_string.c_str());
}

void Mapping::PoseCallback(const gazebo_msgs::ModelStates& msg) {
  ROS_INFO("Got Model State (%f)", 1.0);
}


void Mapping::ScanCallback(const sensor_msgs::LaserScan& msg) {
  ROS_INFO("Got Scan");

  //TODO: Voodoo Magic to populate map_;
  /*
  Algorithm:
  - Get Position
  - Do math to calculate the probability that a cell is occupied or not
  */

}
