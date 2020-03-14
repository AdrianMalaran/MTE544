// Mapping Node Implementation
#include <mapping/mapper_node.h>


Mapping::Mapping() {
  Cell default_cell;

  // width_ = 233;
  // height_ = 195;
  width_ = 3000;
  height_ = 3000;
  x_offset_ = width_ / 2;
  y_offset_ = height_ / 2;

  vector<vector<Cell>> temp_map(width_, vector<Cell>(height_, default_cell));

  pose_sub_ = nh_.subscribe("/gazebo/model_states", 1, &Mapping::PoseCallback, this);
  //TODO: Switch this to the position from the bag
  real_pose_sub_ = nh_.subscribe("/gazebo/model_states", 1, &Mapping::PoseCallback, this);
  scan_sub_ = nh_.subscribe("/scan", 1, &Mapping::ScanCallback, this);
  position_sub_ = nh_.subscribe("/indoor_pos", 1, &Mapping::IndoorPositionCallback, this);

  // odom_pose_sub_ = nh_.subscribe("/odom", 1, &Mapping::OdomPositionCallback, this);

  // nav_msgs::OccupancyGrid& msg;
  // TODO: Need a publisher for occupancy grid
  // ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, false);

  for (int row = 0; row < temp_map.size(); row ++) {
    for (int col = 0; col < temp_map[row].size(); col++) {
      temp_map[row][col].log_odds = log_odds(unknown_constant_);
    }
  }
  map_ = temp_map;
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

double Mapping::log_odds(double prob) {
  return log(prob / (1 - prob));
}

void Mapping::convertToOccupancyFrame(double world_x, double world_y, int& occ_x, int& occ_y) {
  occ_x = (world_x * 100 + x_offset_);
  occ_y = (world_y * 100 + y_offset_);
}


void Mapping::ScanCallback(const sensor_msgs::LaserScan& msg) {
  if (!position_set_) {
    ROS_WARN("Position not yet set, not using this scan data until position i set");
    return;
  }

  iteration_ ++;
  if (iteration_ == 3) {
    iteration_ = 0;
    return;
  }
  double increment = msg.angle_increment;

  for (int i = 0; i < msg.ranges.size(); i++) {

    if (i % 2 != 0) {
      continue;
    } else {

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
      // test_pose.orientation.w = 1.0;
      // tf::poseMsgToTF(test_pose ,world_to_robot_);

      // ROS_INFO("World_to_robot %f, %f", world_to_robot_.getOrigin().getX(), world_to_robot_.getOrigin().getY());
      if (!isnan(range)) {
        // ROS_INFO("Angle (%f) Pos(%f, %f)", angle, x, y);
        // ROS_INFO("Angle (%f) Pos(%f, %f)", angle, x, y);

        geometry_msgs::Pose scan_pose;

        // TODO: Experimenting delete later
        // scan_pose.position.x = 4;
        // scan_pose.position.y = 0;

        scan_pose.position.x = x;
        scan_pose.position.y = y;

        tf::Transform robot_to_scan;
        tf::poseMsgToTF(scan_pose, robot_to_scan);

        // ROS_INFO("robot_to_scan %f, %f", robot_to_scan.getOrigin().getX(), robot_to_scan.getOrigin().getY());

        tf::Transform world_to_scan = world_to_robot_ * robot_to_scan;
        // ROS_INFO("  World to Scan Pose (%f, %f)", world_to_scan.getOrigin().getX(), world_to_scan.getOrigin().getY());
        geometry_msgs::Pose world_to_scan_pose;
        tf::poseTFToMsg(world_to_scan, world_to_scan_pose);

        // max_x = !isnan(world_to_scan_pose.position.x) ? max(max_x, world_to_scan_pose.position.x) : max_x;
        // min_x = !isnan(world_to_scan_pose.position.x) ? min(min_x, world_to_scan_pose.position.x) : min_x;
        //
        // max_y = !isnan(world_to_scan_pose.position.y) ? max(max_y, world_to_scan_pose.position.y) : max_y;
        // min_y = !isnan(world_to_scan_pose.position.y) ? min(min_y, world_to_scan_pose.position.y) : min_y;
        //
        // ROS_INFO("X max(%f) min(%f) | Y max(%f) min(%f)", max_x, min_x, max_y, min_y);

        // TODO: See if rounding to the nearest number makes it better
        // int64_t occupancy_to_scan_x = static_cast<int>(world_to_scan_pose.position.x) + width_/2;
        // int64_t occupancy_to_scan_y = static_cast<int>(world_to_scan_pose.position.y) + height_/2;

        int occupancy_to_scan_x;
        int occupancy_to_scan_y;
        convertToOccupancyFrame(world_to_scan_pose.position.x, world_to_scan_pose.position.y, occupancy_to_scan_x, occupancy_to_scan_y);
        // ROS_INFO("    World->Scan(%f, %f) Mapped Occupancy(%f, %f)",
        //   world_to_scan_pose.position.x, world_to_scan_pose.position.y,
        //   occupancy_to_scan_x, occupancy_to_scan_y);

        int occupancy_to_robot_x;
        int occupancy_to_robot_y;
        convertToOccupancyFrame(world_to_robot_.getOrigin().getX(), world_to_robot_.getOrigin().getY(), occupancy_to_robot_x, occupancy_to_robot_y);
        // ROS_INFO("      World->Robot(%f, %f) Mapped Occupancy(%f, %f)",
        //   world_to_robot_.getOrigin().getX(), world_to_robot_.getOrigin().getY(),
        //   occupancy_to_robot_x, occupancy_to_robot_y);


        std::vector<Coordinate> coordinates_to_clear = findAllPointsAlongLine(occupancy_to_robot_x, occupancy_to_robot_y, occupancy_to_scan_x, occupancy_to_scan_y);
        // std::vector<Coordinate> coordinates_to_clear = findAllPointsAlongLine(world_to_robot_.getOrigin().getX(), world_to_robot_.getOrigin().getY(), scan_pose.position.x, scan_pose.position.y);


        // ROS_INFO("        Coords to Fill:");
        for (int j = 0; j < coordinates_to_clear.size(); j++) {
          if (coordinates_to_clear[j].x < -1 || coordinates_to_clear[j].y < -1 ) {
              // ROS_ERROR("Coordinate less than -1");
          } else if (coordinates_to_clear[j].x < 0 || coordinates_to_clear[j].y < 0) {
              // ROS_WARN("Dropping -1 Coord");
              continue;
          }
          // ROS_INFO("          Coord(%i, %i)", coordinates_to_clear[j].x, coordinates_to_clear[j].y);
          // Probability that we add is one of either
          // - P_occupied = 0.7
          // - P_free = 0.3
          // - P_unknown = 0.5
          double probability = free_constant_;

          if (j == coordinates_to_clear.size() -1) {
            probability = occupied_constant_;
          } else {
            probability = free_constant_;
          }

          Cell& cell = map_[coordinates_to_clear[j].x][coordinates_to_clear[j].y];
          double old_cell_log_odds = cell.log_odds;
          cell.log_odds = cell.log_odds + log_odds(probability) - log_odds(unknown_constant_);
          // ROS_INFO("             Prev. Cell.log_odds(%f) New Cell.log_odds(%f)", old_cell_log_odds, cell.log_odds);
        }
      }
    }
  }
}

void Mapping::IndoorPositionCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  position_set_ = true;
  pose_ = msg.pose.pose;
  // pose_.position.x = 0;
  // pose_.position.y = 0;
  pose_.position.z = 0;
  // pose_.orientation.x = 0;
  // pose_.orientation.y = 0;
  // pose_.orientation.z = 0;
  // pose_.orientation.w = 1.0;
  // ROS_INFO("Received Position: (%f, %f, %f)", pose_.position.x, pose_.position.y, pose_.position.z);

  tf::poseMsgToTF(pose_, world_to_robot_);
  PublishMap();
}

void Mapping::OdomPositionCallback(const nav_msgs::Odometry& msg) {
  position_set_ = true;
  pose_ = msg.pose.pose;
  pose_.position.z = 0;
  tf::poseMsgToTF(pose_, world_to_robot_);
  PublishMap();
}




std::vector<Coordinate> Mapping::findAllPointsAlongLine(int x1, int y1, int x2, int y2) {
    std::vector<Coordinate> ptsAlongLine;

    int slope_x1 = 0;
    int slope_y1 = 0;
    int slope_x2 = 0;
    int slope_y2 = 0;

    int width = x2 - x1;
    int height = y2 - y1;
    int xTracker = x1;
    int yTracker = y1;

    if (width < 0) {
        slope_x1 = -1;
        slope_x2 = -1;
    } else if (width > 0) {
        slope_x1 = 1;
        slope_x2 = 1;
    }

    if (height < 0) {
        slope_y1 = -1;
    } else if (height > 0) {
        slope_y1 = 1;
    }

    int longest;
    int shortest;

    if (!(abs(width) > abs(height))) {
        longest = abs(height);
        shortest = abs(width);
        if (height < 0) {
            slope_y2 = -1;
        } else if (height > 0) {
            slope_y2 = 1;
        }
        slope_x2 = 0;
    } else {
        longest = abs(width);
        shortest = abs(height);
    }

    int numerator = longest >> 1;

    for (int i=0; i<=longest; i++) {
        Coordinate c;
        c.x = xTracker;
        c.y = yTracker;
        ptsAlongLine.push_back(c);

        numerator += shortest;

        if (!(numerator < longest)) {
            numerator -= longest;
            xTracker += slope_x1;
            yTracker += slope_y1;
        } else {
            xTracker += slope_x2;
            yTracker += slope_y2;
        }
    }

    return ptsAlongLine;
}

void Mapping::PublishMap() {
  nav_msgs::OccupancyGrid occ_grid;
  occ_grid.info.resolution = 1.0;
  occ_grid.info.width = width_;
  occ_grid.info.height = height_;

  // int data[4526775];
  vector<signed char> data_map;
  // for (int i = 0; i < map_.size(); i ++) {
  //   for (int j = 0; j < map_[i].size(); j++) {
  //     Cell& cell = map_[i][j];
  //     if (i < map_.size() /2 ) {
  //       cell.log_odds = -10.0;
  //     } else {
  //       cell.log_odds = 10.0;
  //     }
  //   }
  // }

  for (int i = 0; i < map_.size(); i ++) {
    for (int j = 0; j < map_[i].size(); j++) {
        Cell cell = map_[i][j];
        double probability = (1 - 1/(1 + exp(cell.log_odds))) * 100;
        data_map.push_back(static_cast<int>(probability)); // TODO: Change index back
        // Convert to Probability and multiply to 100
    }
  }
  occ_grid.data = data_map;

  ROS_WARN("Publishing Occ Grid");
  map_pub_.publish(occ_grid);
}
