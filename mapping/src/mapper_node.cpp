// Mapping Node Implementation
#include <mapping/mapper_node.h>


Mapping::Mapping() {
  // map_ = vector< >
  int height_ = 1000;
  int width_ = 1000;
  Cell default_cell;
  vector<vector<Cell>> temp_map(width_, vector<Cell>(height_, default_cell));

  pose_sub_ = nh_.subscribe("/gazebo/model_states", 1, &Mapping::PoseCallback, this);
  //TODO: Switch this to the position from the bag
  real_pose_sub_ = nh_.subscribe("/gazebo/model_states", 1, &Mapping::PoseCallback, this);
  scan_sub_ = nh_.subscribe("/scan", 1, &Mapping::ScanCallback, this);
  position_sub_ = nh_.subscribe("/indoor_pos", 1, &Mapping::IndoorPositionCallback, this);

  // nav_msgs::OccupancyGrid& msg;
  // TODO: Need a publisher for occupancy grid
  // ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, false);
  PrintMap(temp_map);
}

void Mapping::PrintMap(vector<vector<Cell>> map) {
  string map_debug_string = "\n";
  for (int row = 0; row < map.size(); row ++) {
    for (int col = 0; col < map[row].size(); col++) {
      map_debug_string += to_string(map[row][col].log_odds) + ", ";
    }
    map_debug_string += "\n";
  }

  ROS_INFO("Map: %s", map_debug_string.c_str());
}

void Mapping::PoseCallback(const gazebo_msgs::ModelStates& msg) {
  ROS_INFO("Got Model State (%f)", 1.0);
}


void Mapping::ScanCallback(const sensor_msgs::LaserScan& msg) {
  if (!position_set_) {
    ROS_WARN("Position not yet set, not using this scan data until position i set");
    return;
  }


  /*
  Algorithm:
  [x] Get Position
  - Do math to calculate the probability that a cell is occupied or not
  */

  double increment = msg.angle_increment;

  ROS_INFO("Num Ranges (%lu)", msg.ranges.size());

  for (int i = 0; i < msg.ranges.size(); i++) {
    double angle = msg.angle_min + msg.angle_increment * i;
    double range = msg.ranges[i];
    double x = range * sin(angle);
    double y = range * cos(angle);

    // TESTING
    // angle = -0.463;
    // range = 2.236;
    // x = range * sin(angle);
    // y = range * cos(angle);
    // tf::Transform test_to_test;
    // geometry_msgs::Pose test_pose;
    // test_pose.position.x = 2;
    // test_pose.position.y = 2;
    // // test_pose.orientation.w = 1.0;
    // tf::poseMsgToTF(test_pose ,world_to_robot_);

    // ROS_INFO("World_to_robot %f, %f", world_to_robot_.getOrigin().getX(), world_to_robot_.getOrigin().getY());
    if (!isnan(range)) {
      ROS_INFO("Angle (%f) Pos(%f, %f)", angle, x, y);
      ROS_INFO("Angle (%f) Pos(%f, %f)", angle, x, y);

      geometry_msgs::Pose scan_pose;
      scan_pose.position.x = x;
      scan_pose.position.y = y;
      tf::Transform robot_to_scan;
      tf::poseMsgToTF(scan_pose, robot_to_scan);

      ROS_INFO("robot_to_scan %f, %f", robot_to_scan.getOrigin().getX(), robot_to_scan.getOrigin().getY());

      tf::Transform world_to_scan = world_to_robot_ * robot_to_scan;
      ROS_INFO("  World to Scan Pose (%f, %f)", world_to_scan.getOrigin().getX(), world_to_scan.getOrigin().getY());
      geometry_msgs::Pose world_to_scan_pose;
      tf::poseTFToMsg(world_to_scan, world_to_scan_pose);

      int16_t occupancy_grid_x = world_to_scan_pose.position.x + width_/2;
      int16_t occupancy_grid_y = world_to_scan_pose.position.y + height_/2;
      ROS_INFO("    Getting Probability for Map(%f, %f)", world_to_scan_pose.position.x, world_to_scan_pose.position.y);
      // Laser Scan hits a target
      // double probability = 1 - 1.0/(1.0 + exp());
    }
  }
  // for (float )

}

void Mapping::IndoorPositionCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  position_set_ = true;
  pose_ = msg.pose.pose;
  pose_.position.z = 0;
  ROS_INFO("Received Position: (%f, %f, %f)", pose_.position.x, pose_.position.y, pose_.position.z);

  tf::poseMsgToTF(pose_, world_to_robot_);


}
