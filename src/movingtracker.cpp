#include "movingtracker.h"

MovingTracker::MovingTracker(){
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  ROS_INFO("Starting Detection And Tracking of Moving Objects");

  n_private.param("lidar_frame", lidar_frame, string(""));
  n_private.param("world_frame", world_frame, string("map"));
  n_private.param("threshold_distance", dth, 0.2);
  n_private.param("euclidean_distance", euclidean_distance, 0.25);
  n_private.param("pub_markers", p_marker_pub, false);



  pub_tracks_mean    = n.advertise<movingtracker::TrackArray>("tracks/mean", 10);
  pub_tracks_mean_kf = n.advertise<movingtracker::TrackArray>("tracks/mean_kf", 10);
  pub_tracks_box     = n.advertise<movingtracker::TrackArray>("tracks/box", 10);
  pub_marker_array   = n.advertise<visualization_msgs::MarkerArray>("marker_array", 10);
  pub_marked_map     = n.advertise<nav_msgs::OccupancyGrid>("pmap", 10);

  sub_map = n.subscribe("/map", 1, &MovingTracker::callback, this);

}


void MovingTracker::callback(const nav_msgs::OccupancyGrid::ConstPtr& map_in){
    // delete all Markers
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markera;
    marker.action =3;
    markera.markers.push_back(marker);
    pub_marker_array.publish(markera);

    map = *map_in;

    // int occucount = 0;
    deque<deque<Cell>> cellLists;
    // ROS_INFO("Map received");
    //Find the number of non inf laser scan values and save them in c_points
    for (unsigned int i = 0; i < map.info.width; ++i){
      for (unsigned int j = 0; j < map.info.height; ++j){
        if(map.data[i+j*map.info.width] == 100){
          deque<Cell> cellList;
          Cell cell;
          cell.first = i;
          cell.second = j;
          // ROS_INFO("Visiting Cell %i, %i", cell.first,cell.second);
          MovingTracker::BFS(cellList, cell);
          cellLists.push_back(cellList);
          // occucount++;
        }
      }
    }
    // ROS_INFO("Number of Clasters %i", cellLists[0].size());
    // MovingTracker::publishmap();
    MovingTracker::visualiseGroupedPoints(cellLists);
}

void MovingTracker::BFS(deque<Cell>& cellList, Cell& cell){
  // list<Cell> cellList;
  cellList.push_back(cell);
  // ROS_INFO("first island");
  map.data[cell.first+cell.second*map.info.width] = 50;
  deque<Cell> cellList_temp;
  cellList_temp.push_back(cell);

  while (!cellList.empty()){
    cell = cellList.front();
    cellList.pop_front();
    MovingTracker::addtoqueue(cellList, cellList_temp, Cell(cell.first+1, cell.second));
    MovingTracker::addtoqueue(cellList, cellList_temp, Cell(cell.first+1, cell.second+1));
    MovingTracker::addtoqueue(cellList, cellList_temp, Cell(cell.first+1, cell.second-1));
    MovingTracker::addtoqueue(cellList, cellList_temp, Cell(cell.first, cell.second+1));
    MovingTracker::addtoqueue(cellList, cellList_temp, Cell(cell.first, cell.second-1));
    MovingTracker::addtoqueue(cellList, cellList_temp, Cell(cell.first-1, cell.second));
    MovingTracker::addtoqueue(cellList, cellList_temp, Cell(cell.first-1, cell.second+1));
    MovingTracker::addtoqueue(cellList, cellList_temp, Cell(cell.first-1, cell.second-1));
  }
  cellList = cellList_temp;
  // ROS_INFO("Number In the Claster %i", cellList.size());
}

void MovingTracker::addtoqueue(deque<Cell>& cellList, deque<Cell>& cellList_temp, Cell cell){
  if (cell.first < map.info.width){
    if (cell.second < map.info.height ){
      if (map.data[cell.first+cell.second*map.info.width] == 100){
        map.data[cell.first+cell.second*map.info.width] = 50;
        cellList.push_back(cell);
        cellList_temp.push_back(cell);
        // ROS_INFO("Push Cell %i, %i", cell.first,cell.second);
      }
    }
  }
}
void MovingTracker::publishmap(){
  nav_msgs::OccupancyGrid newmap = map;
  newmap.header.frame_id = world_frame;
  newmap.header.stamp = ros::Time(0);
  pub_marked_map.publish(newmap);
}

void MovingTracker::visualiseGroupedPoints(const deque<deque<Cell>>& cellLists){
  //Publishing the clusters with different colors
  visualization_msgs::MarkerArray marker_array;
  //Populate grouped points message
  visualization_msgs::Marker gpoints;
  gpoints.header.frame_id = world_frame;
  gpoints.header.stamp = ros::Time(0);
  gpoints.ns = "clustered_points";
  gpoints.action = visualization_msgs::Marker::ADD;
  gpoints.pose.orientation.w = 1.0;
  gpoints.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  gpoints.scale.x = 0.13;
  gpoints.scale.y = 0.13;
  for(unsigned int i=0; i<cellLists.size(); ++i){

    gpoints.id = cg;
    cg++;

    float randomg = rand() / double(RAND_MAX);
    float randomb = rand() / double(RAND_MAX);
    float randomr = rand() / double(RAND_MAX);
    gpoints.color.g = randomg;
    gpoints.color.b = randomb;
    gpoints.color.r = randomr;
    gpoints.color.a = 1.0;
    gpoints.lifetime = ros::Duration(0.1);
    for(unsigned int j=0; j<cellLists[i].size(); ++j){
      geometry_msgs::Point p;
      p.x = (cellLists[i][j].first * map.info.resolution + map.info.origin.position.x);
      p.y = (cellLists[i][j].second * map.info.resolution + map.info.origin.position.y);
      p.z = 0;
      gpoints.points.push_back(p);
      // ROS_INFO("Adding Marker, %i ,%i",p.x, p.y);
    }
    marker_array.markers.push_back(gpoints);
    gpoints.points.clear();
  }
  pub_marker_array.publish(marker_array);
}
