#include "find_positions_class.hpp"

using namespace find_positions;

int main(int argc, char **argv){
  ros::init(argc, argv, "find_positions");
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  FindPositions m_fp;
  
  while(!m_fp.m_mapAvailable){
    ros::spinOnce();
  }
  m_fp.setMarker();
  
  m_fp.solving_tsp_concorde(0);
  
  m_fp.solving_tsp_concorde(1);
  
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
      ROS_INFO("10 sec 1_s");
      clock_t start_1=clock();
      while((clock()-start_1)/CLOCKS_PER_SEC <= 10) ;
      //system("adb shell input tap 800 2300");
      ROS_INFO("10 sec 1_e");
      //32 is the best time. I think,,
      clock_t start_2=clock();
      while((clock()-start_2)/CLOCKS_PER_SEC <= 10) ;
      ROS_INFO("10 sec 2_e");
      //ros::Duration(5).sleep();
    }
    else{
      ROS_INFO("The robot failed to reach the goal location for some reason");
    }
  }
  
  //Print the waypoints on the map
  ROS_INFO("done find_positions_node");
  
  return 0;
}
