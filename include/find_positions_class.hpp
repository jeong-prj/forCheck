#ifndef INCLUDE_FIND_POSITIONS_CLASS_HPP_
#define INCLUDE_FIND_POSITIONS_CLASS_HPP_


#include <ros/ros.h>
#include <ros/console.h>

#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/MapMetaData.h"
#include <visualization_msgs/Marker.h>

//#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <queue>
#include <array>

//#include <boost/bind.hpp>

#include "tsp.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <omp.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_fn/potarr_point.h>
#include "global_planning_handler.hpp"

extern "C" {
#include <concorde.h>
}
#define DIST_HIGH  (1.0e10)

namespace find_positions
{

using namespace std;
  
class FindPositions{
public:
  FindPositions();
  virtual ~FindPositions();
  
  array<double, 2> gridmapToworld(int x, int y);
  array<int, 2> getPoseTo2D(int pose);
  int get2DToPose(int x, int y);
  
  //void setMarker(visualization_msgs::Marker& m_points, visualization_msgs::Marker& line_strip){
  void setMarker(){
    m_points.header.frame_id = line_strip.header.frame_id= "map";
    m_points.header.stamp = line_strip.header.stamp = ros::Time(0);
    m_points.ns = line_strip.ns = "way_points";
    m_points.id=1000;
    m_points.type=visualization_msgs::Marker::POINTS;
    line_strip.type=visualization_msgs::Marker::LINE_STRIP;
    
    m_points.action = line_strip.action =visualization_msgs::Marker::ADD;
    m_points.pose.orientation.w = line_strip.pose.orientation.w =1.0;
    
    m_points.scale.x = 0.1;
    m_points.scale.y = 0.1;
    
    line_strip.scale.x = 0.03;

    m_points.color.b = 1.0f;
    m_points.color.a = 1.0;
    
    line_strip.color.b=1.0f;
    line_strip.color.a=1.0;
    
    m_points.lifetime = ros::Duration();
    
    ///////print path
    m_path_points.header.frame_id = path_line.header.frame_id= "map";
    m_path_points.header.stamp = path_line.header.stamp = ros::Time(0);
    m_path_points.ns = path_line.ns = "way_apath";
    m_path_points.id=2000;
    m_path_points.type=visualization_msgs::Marker::POINTS;
    path_line.type=visualization_msgs::Marker::LINE_STRIP;
    
    m_path_points.action = path_line.action =visualization_msgs::Marker::ADD;
    m_path_points.pose.orientation.w = path_line.pose.orientation.w =1.0;
    
    m_path_points.scale.x = 0.05;
    m_path_points.scale.y = 0.05;
    
    path_line.scale.x = 0.02;

    m_path_points.color.g = 1.0f;
    m_path_points.color.a = 1.0;
    
    path_line.color.g=1.0f;
    path_line.color.a=1.0;
    
    m_path_points.lifetime = ros::Duration(); 
  }
  
  geometry_msgs::PoseStamped StampedPosefromSE2( const float& x, const float& y, const float& yaw_radian ){
    geometry_msgs::PoseStamped outPose ;
    outPose.pose.position.x = x ;
    outPose.pose.position.y = y ;

    float c[3] = {0,};
    float s[3] = {0,};
    c[0] = cos(yaw_radian/2) ;
    c[1] = cos(0) ;
    c[2] = cos(0) ;
    s[0] = sin(yaw_radian/2) ;
    s[1] = sin(0) ;
    s[2] = sin(0) ;

    float qout[4] = {0,};
    qout[0] = c[0]*c[1]*c[2] + s[0]*s[1]*s[2];
    qout[1] = c[0]*c[1]*s[2] - s[0]*s[1]*c[2];
    qout[2] = c[0]*s[1]*c[2] + s[0]*c[1]*s[2];
    qout[3] = s[0]*c[1]*c[2] - c[0]*s[1]*s[2];

    outPose.pose.orientation.w = qout[0] ;
    outPose.pose.orientation.x = qout[1] ;
    outPose.pose.orientation.y = qout[2] ;
    outPose.pose.orientation.z = qout[3] ;

    outPose.header.frame_id = "map" ;
    outPose.header.stamp = ros::Time::now() ;

    return outPose;
  }
  
  void writeDataFile (vector<array<int,2>> positions);
  ////void solving_tsp_concorde(vector<int> * tour, int flag);
  void solving_tsp_concorde(int flag);
  void padding(int pose, int siz);
  ////void findWaypointsDistance(vector<int> *tour);
  void findWaypointsDistance(int *tour);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  
  //int eraseInvalidByDist();
  //int bfs(int node_size, vector<int>* graph, vector<int>& sequence);
  void makeMapForPath();
  double calculatePath(double ax, double ay, double bx, double by);
  void checkRealPath();
  int m_mapAvailable = 0;
  
  vector<array<double,2>> result_waypoints;
  
protected:
  //                     up ri  do  le
  array<int,4> x_move = { 0, 1, 0, -1};
  array<int,4> y_move = {-1, 0, 1,  0};
  
  ros::NodeHandle m_nh;
  
  ros::Subscriber m_gridmapsub;
  ros::Publisher m_wayPointsPub;
  ros::Publisher m_pathPub;
  
  TSP::TSPSolver m_TSPSolver;
  int map_width;
  int map_height;
  double map_resolution;
  array<double, 2> gridmap_origin_pose;
  vector<signed char> origin_map;
  vector<signed char> map_flags;
  vector<array<double,2>> waypointsWorld;
  //ej_marker
  visualization_msgs::Marker m_points, line_strip;
  visualization_msgs::Marker m_path_points, path_line;
  vector<array<int,2>> middles;
  vector<array<int,2>> waypoints;
  
  costmap_2d::Costmap2D* mpo_costmap;
};
  
}


#endif
