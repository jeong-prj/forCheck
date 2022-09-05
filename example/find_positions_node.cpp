#include "find_positions_class.hpp"

using namespace find_positions;

int main(int argc, char **argv){
  ros::init(argc, argv, "find_positions");
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  FindPositions m_fp;
  system("adb connect 10.77.80.203");
  
  while(!m_fp.m_mapAvailable || !m_fp.m_sposeAvailable){
    ros::spinOnce();
  }
  system("adb connect 10.77.80.203");
  
  m_fp.totalStartTime = ros::WallTime::now();
  
  
  m_fp.setMarker();
  
  m_fp.tsp1StartTime = ros::WallTime::now();
  m_fp.solving_tsp_concorde(0);//15sec
  m_fp.tsp1EndTime = ros::WallTime::now();
  
  m_fp.tsp1Time = m_fp.takeATime(m_fp.tsp1StartTime, m_fp.tsp1EndTime);
  
  ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n First tsp time: %lf (ms)\n ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n", m_fp.tsp1Time);
  
  m_fp.tsp2StartTime = ros::WallTime::now();
  m_fp.solving_tsp_concorde(1);//30sec
  m_fp.tsp2EndTime = ros::WallTime::now();
  
  m_fp.tsp2Time = m_fp.takeATime(m_fp.tsp2StartTime, m_fp.tsp2EndTime);
  
  ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n Second tsp time: %lf (ms)\n ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n", m_fp.tsp2Time);

  //clock_t start_0=clock();
  //while((clock()-start_0)/CLOCKS_PER_SEC <= 10) ;//10sec
  
  m_fp.scanningStartTime = ros::WallTime::now();
  for(int i = 0; i < m_fp.result_waypoints.size(); i++){
    ROS_INFO("Send goal %d", i+1);
    //cout<<"node_waypoints: "<<m_fp.result_waypoints[i][0]<<", " <<m_fp.result_waypoints[i][1]<<endl;
    double x=m_fp.result_waypoints[i][0], y=m_fp.result_waypoints[i][1];
    
    move_base_msgs::MoveBaseGoal waypoint_goal;
    waypoint_goal.target_pose.header.frame_id = "map";
    waypoint_goal.target_pose.header.stamp = ros::Time::now();
    
    waypoint_goal.target_pose.pose.position.x = x;
    waypoint_goal.target_pose.pose.position.y = y;
    waypoint_goal.target_pose.pose.orientation.w = 1.0;
    
    ac.sendGoal(waypoint_goal);
    
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("The robot has arrived at the goal location");
      clock_t start_1=clock();
      while((clock()-start_1)/CLOCKS_PER_SEC <= 2) ;
      
      ROS_INFO("start scanning,,,");
      system("adb shell input tap 800 2300");
      
      //32 is the best time. I think,,
      //ros::Duration(32).sleep();
      clock_t start_2=clock();
      while((clock()-start_2)/CLOCKS_PER_SEC <= 35) ;
    }
    else{
      ROS_INFO("The robot failed to reach the goal location for some reason");
    }
  }
  
  m_fp.scanningEndTime = ros::WallTime::now();
  
  m_fp.scanningTime = m_fp.takeATime(m_fp.scanningStartTime, m_fp.scanningEndTime);
  
  ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n Scanning time: %lf (ms)\n ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n", m_fp.scanningTime);
  
  m_fp.totalEndTime = ros::WallTime::now();
  m_fp.totalTime = m_fp.takeATime(m_fp.totalStartTime, m_fp.totalEndTime);
  
  //Print the waypoints on the map
  ROS_INFO("done find_positions_node");
  ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ \n total time: %lf ms \n tsp1 time: %lf ms\n tsp2 time: %lf ms\n scanning time: %lf ms\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++", m_fp.totalTime, m_fp.tsp1Time, m_fp.tsp2Time, m_fp.scanningTime);
  
  return 0;
}
