#include <ros/ros.h>
#include <ros/console.h>
#include "mapping/mapper_node.h"

int main(int argc, char **argv)
{
  ROS_INFO("Initalizing Mapper Node!");
  ros::init(argc, argv, "mapper");

  Mapping mapping_node;

  ros::spin();
  return 0;
}

// How to Run:
//
// Terminal 1:
// roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
//
// Terminal 2:
// roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
//
// Terminal 3:
// roscore
//
// Terminal 4:
// rosrun mapping mapping
//
// Terminal 5:
// rosbag play run2.bag
