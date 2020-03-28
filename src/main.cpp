#include "ros/ros.h"
#include "movingtracker.h"
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "movingtracker_node");

  //Create an object of class datmo
  MovingTracker  movingtracker_object;

  ros::spin();

  return 0;
}
