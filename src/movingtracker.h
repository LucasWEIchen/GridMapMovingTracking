#include <ros/ros.h>
#include <math.h>       /* atan */
#include <omp.h>      //Multi-threading
#include <vector>
#include <random>
#include <algorithm> // for sort(), min()

#include <chrono>
#include <iostream>
#include <fstream>

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <movingtracker/TrackArray.h>
#include <movingtracker/Track.h>

typedef std::pair<double, double> Point;
typedef std::pair<int, int> Cell;
typedef std::vector<Cell> CellList;
typedef std::vector<double> l_shape;
typedef std::vector<l_shape> l_shapes;
typedef std::vector<Point> pointList;

#include "cluster.h"

using namespace std;
// This node segments the point cloud based on the break-point detector algorithm.
// This algorithm is based on "L-Shape Model Switching-Based Precise Motion Tracking
// of Moving Vehicles Using Laser Scanners.
class MovingTracker
{
public:
  MovingTracker();
  // ~movingtracker();

  void callback(const nav_msgs::OccupancyGrid::ConstPtr &);
  void addtoqueue(deque<Cell>&, deque<Cell>&, Cell);
  void BFS(deque<Cell>&, Cell&);
  void publishmap();
  void visualiseGroupedPoints(const deque<deque<Cell>> &);
  // void transformPointList(const pointList& , pointList& );

  tf::TransformListener tf_listener;
private:

  ros::Publisher pub_marker_array;
  ros::Publisher pub_tracks_mean;
  ros::Publisher pub_tracks_mean_kf;
  ros::Publisher pub_tracks_box;
  ros::Publisher pub_marked_map;

  ros::Subscriber sub_map;
  nav_msgs::OccupancyGrid map;
  vector<Cluster> clusters;


  //Tuning Parameteres
  double dt;
  ros::Time time;

  unsigned long int cg       = 1;//group counter to be used as id of the clusters
  //initialised as one, because 0 index take the msgs that fail to be initialized
  unsigned long int cclusters= 0;//counter for the cluster objects to be used as id for the markers

  //Parameters
  double dth;
  double euclidean_distance;
  bool p_marker_pub;
  string lidar_frame;
  string world_frame;

};
