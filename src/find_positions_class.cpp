#include "find_positions_class.hpp"
#include <fstream>
namespace find_positions
{

FindPositions::FindPositions(){
  m_wayPointsPub = m_nh.advertise<visualization_msgs::Marker>("way_point_marker", 10000);
  m_tspPointsPub = m_nh.advertise<visualization_msgs::Marker>("tsp_marker", 10000);
  m_addPointsPub = m_nh.advertise<visualization_msgs::Marker>("added_marker", 10000);
  m_pathPub = m_nh.advertise<visualization_msgs::Marker>("path_marker", 10000);
  m_gridmapsub = m_nh.subscribe<nav_msgs::OccupancyGrid>("m_gridmap_fin", 1000, &FindPositions::mapCallback, this); 
  m_globalCostmapSub = m_nh.subscribe("move_base_node/global_costmap/costmap", 100, &FindPositions::globalCostmapCallBack, this );
  
  if (mp_cost_translation_table == NULL){
    mp_cost_translation_table = new uint8_t[101];

    // special values:
    mp_cost_translation_table[0] = 0;  // NO obstacle
    mp_cost_translation_table[99] = 253;  // INSCRIBED obstacle
    mp_cost_translation_table[100] = 254;  // LETHAL obstacle
//   mp_cost_translation_table[-1] = 255;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 99; i++){
      mp_cost_translation_table[ i ] = uint8_t( ((i-1)*251 -1 )/97+1 );
    }
  }
}
FindPositions::~FindPositions(){
  delete [] mp_cost_translation_table;
}

array<double, 2> FindPositions::gridmapToworld(int x, int y){
  //0.05 is resolution of the map
  double wx = (x * map_resolution) + gridmap_origin_pose[0];
  double wy = (y * map_resolution) + gridmap_origin_pose[1];
  
  return {wx, wy};
}

array<int, 2> FindPositions::getPoseTo2D(int pose){
  int x = pose % map_width;
  int y = pose / map_width;
  
  return {x, y};  
}

int FindPositions::get2DToPose(int x, int y){
  return (y*map_width) + x;
}

double distance(double x, double y, double xx, double yy){
  return sqrt(((x-xx)*(x-xx))+((y-yy)*(y-yy)));
}

void FindPositions::writeDataFile (vector<array<int,2>> positions) {
  ROS_INFO("function write data file");
  FILE *out = fopen (m_TSPSolver.name, "w");
  m_TSPSolver.ncount = positions.size();
  
  fprintf (out, "NAME: concorde%d\n",m_TSPSolver.ncount);
  fprintf (out, "TYPE: TSP\n");
  fprintf (out, "DIMENSION: %d\n", m_TSPSolver.ncount);
  fprintf (out, "EDGE_WEIGHT_TYPE: EUC_2D\n");
  fprintf (out, "NODE_COORD_SECTION\n");
  
  for (int i = 0; i < m_TSPSolver.ncount; i++) {
    fprintf(out, "%d %d %d\n", i, positions[i][0], positions[i][1]);
  }

  fprintf(out, "EOF\n");
  fclose(out);
  
  ROS_INFO("done the write task");
}

//Receive a symmetric 2D distance matrix (dist) and create a TSP optimal tour (tour)
////void FindPositions::solving_tsp_concorde(vector<int> * tour, int flag){
void FindPositions::solving_tsp_concorde(int flag){
  int rval = 0; //Concorde functions return 1 if something fails
  double szeit; //Measure cpu time
  
  string str = "map.tsp";
  m_TSPSolver.name = new char[str.size() + 1];
  copy(str.begin(), str.end(), m_TSPSolver.name);
  m_TSPSolver.name[str.size()]='\0';
  
  if(flag ==0)
    writeDataFile(middles);
  else
    writeDataFile(waypoints);
  
  CCrandstate rstate;
  int seed = rand();
  CCutil_sprand(seed, &rstate); //Initialize the portable random number generator
  m_TSPSolver.out_tour = CC_SAFE_MALLOC (m_TSPSolver.ncount, int);
  
  CCdatagroup dat;
  cout<<"=================initialize================="<<endl;
  //Initialize a CCdatagroup
  CCutil_init_datagroup (&dat);
  CCutil_gettsplib (m_TSPSolver.name, &m_TSPSolver.ncount, &dat);
  
  cout<<"=================solve data================="<<endl;
  //Solves the TSP over the graph specified in the datagroup
  rval = CCtsp_solve_dat (m_TSPSolver.ncount, &dat, m_TSPSolver.in_tour, m_TSPSolver.out_tour,
                              m_TSPSolver.in_val, &m_TSPSolver.optval, &m_TSPSolver.success,
                              &m_TSPSolver.foundtour, m_TSPSolver.name, m_TSPSolver.timebound,
                              &m_TSPSolver.hit_timebound, m_TSPSolver.silent, &rstate);
  //
  
  cout <<m_TSPSolver.ncount<<endl;
  
  if(flag ==1){
    for (int i = 0; i < m_TSPSolver.ncount; i++) {
      int turn= m_TSPSolver.out_tour[i];
      int x=waypoints[turn][0], y=waypoints[turn][1];
      array<double, 2> world_position = gridmapToworld(x,y);
      waypointsWorld.push_back({world_position[0], world_position[1]});
      geometry_msgs::Point p;
      p.x=world_position[0]; p.y=world_position[1]; p.z=0.0;
        
      tsp_points.points.push_back(p);
      //line_strip.points.push_back(p);
    }
    
    m_tspPointsPub.publish(tsp_points);
    //m_wayPointsPub.publish(line_strip);
    ////eraseInvalidByDist();
    //eraseInvalidByWall();
    checkRealPath();
    
  }
  else{
    findWaypointsDistance(m_TSPSolver.out_tour); 
  }
  
  cout<<endl<<"done: "<<flag<<endl;
  
  CC_IFFREE (m_TSPSolver.out_tour, int);
}

void FindPositions::padding(int pose, int siz, int ch){
  array<int, 2> grid = getPoseTo2D(pose);
  for(int i=grid[0]-siz;i<=grid[0]+siz;i++){
    for(int j=grid[1]-siz;j<=grid[1]+siz;j++){
      if(i<0||i>=map_width||j<0||j>=map_height){
        continue;
      }
      int flag_pose=get2DToPose(i, j);
      if(origin_map[flag_pose]==0){
        if(ch==1){
          map_flags[flag_pose]=2;
        }
        else{
          map_for_path[flag_pose]=100;
        }
        
      }
    }
  }
}

void FindPositions::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  nav_msgs::OccupancyGrid m_gridmap = *msg;
  ROS_INFO("map callback");
  
  this->map_flags = m_gridmap.data;
  this->origin_map = m_gridmap.data;
  this->map_for_path = m_gridmap.data;
  this->map_resolution = m_gridmap.info.resolution;
  this->map_width = m_gridmap.info.width;
  this->map_height = m_gridmap.info.height;
  this->gridmap_origin_pose = {m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y};
  m_mapAvailable = 1;
    
  ROS_INFO("%lf Got map %d, %d, size: %lu", map_resolution, map_width, map_height, origin_map.size());
  
  for(int i=0; i<origin_map.size();i++){
    if(origin_map[i]!=0){
      padding(i, 9, 1);
      padding(i, 5, 2);
    }
  }
  
  int square_size = 3;
  for(int x = square_size/2; x<map_width; x+=square_size){
    for(int y = square_size/2; y<map_height; y+=square_size){
      int check_pose = get2DToPose(x, y);
      
      if(map_flags[check_pose] == 0){
        middles.push_back({x, y});
      }
    }
  }
}

void FindPositions::globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	//const std::unique_lock<mutex> lock(mutex_costmap);
//ROS_INFO("cm callback is called \n");
	m_globalcostmap = *msg ;
	m_globalcostmap_rows = m_globalcostmap.info.height ;
	m_globalcostmap_cols = m_globalcostmap.info.width ;
}


////void FindPositions::findWaypointsDistance(vector<int> *tour){
void FindPositions::findWaypointsDistance(int* tour){
  for(int i=0; i<m_TSPSolver.ncount; i++){
    int turn = tour[i];

    array<int, 2> cur_2D = middles[turn];
    int x=cur_2D[0], y=cur_2D[1];
    int pose=get2DToPose(x, y);
    if(map_flags[pose]==0){
      waypoints.push_back({x, y});
      
      padding(pose, 32, 1);
    }
  }
}

//ej_invalid_by distance
//waypointsWorld = the result of second TSP with waypoints..
//need to fix it !!!!!!!!!!!

//calculate length of path
//return length

void FindPositions::makeMapForPath(){
  ROS_INFO("make map for path");
  std::vector<signed char> cmdata;
  cmdata  =m_globalcostmap.data;
  
  mpo_costmap = new costmap_2d::Costmap2D();
  
  this->mpo_costmap->resizeMap( this->map_width, this->map_height, this->map_resolution, this->gridmap_origin_pose[0], this->gridmap_origin_pose[1] );
  //ROS_INFO("mpo_costmap has been reset \n");
  unsigned char* pmap = this->mpo_costmap->getCharMap() ;
  //ROS_INFO("w h datlen : %d %d %d \n", map_width, map_height, cmdata.size() );

  for(uint32_t ridx = 0; ridx < this->map_height; ridx++){
    for(uint32_t cidx=0; cidx < this->map_width; cidx++){
      uint32_t idx = ridx * this->map_width + cidx ;
      signed char val = map_for_path[idx];
      pmap[idx] = val < 0 ? 255 : mp_cost_translation_table[val];
    }
  }
  
  ROS_INFO("done map path");
}

double FindPositions::calculatePath(double ax, double ay, double bx, double by,int mode){
 // ROS_INFO("calculate len..");
  GlobalPlanningHandler o_gph( *mpo_costmap );
  o_gph.reinitialization();
  
  std::vector<geometry_msgs::PoseStamped> plan;
  
  geometry_msgs::PoseStamped start = StampedPosefromSE2( ax, ay, 0.f );
  start.header.frame_id = "map";
  
  geometry_msgs::PoseStamped goal = StampedPosefromSE2( bx, by, 0.f );
  goal.header.frame_id = "map" ;

  bool bplansuccess = o_gph.makePlan(start, goal, plan);
      
  //caclulate path length..//
  if(!bplansuccess){
    ROS_INFO("couldnt find a path..");
    return 0.0;
  }
  
  double path_len=0;
  int add_node=0;
  geometry_msgs::Point p;
  if(mode==1){
    p.x=plan[0].pose.position.x; p.y=plan[0].pose.position.y; p.z=0.0;
    m_path_points.points.push_back(p);
  }
  for(int k=0;k<plan.size()-1;k++){
    double pose_ax=plan[k].pose.position.x;
    double pose_ay=plan[k].pose.position.y;
    double pose_bx=plan[k+1].pose.position.x;
    double pose_by=plan[k+1].pose.position.y;
        
    path_len += distance(pose_ax, pose_ay, pose_bx, pose_by);
    
    if(mode==1){
      if(path_len>=2.5){
        //result_waypoints.push_back({pose_bx, pose_by});
        add_node++;
        path_len=0;
      }
      p.x=pose_bx; p.y=pose_by; p.z=0.0;
      m_path_points.points.push_back(p);
    }
  }
  
  if(mode==1){
    int equi_dist=plan.size()/(add_node+1);
    for(int i=1;i<=add_node;i++){
      double add_x=plan[equi_dist*i].pose.position.x;
      double add_y=plan[equi_dist*i].pose.position.y;
      
      //have visited node under 1.4m than not add node
      int not_add=0;
      for(int ch_node=0;ch_node<result_waypoints.size();ch_node++){
        double ch_x = result_waypoints[ch_node][0];
        double ch_y = result_waypoints[ch_node][1];
        if(distance(add_x, add_y, ch_x, ch_y)<1.4){
          double ch_path = calculatePath(add_x, add_y, ch_x, ch_y, 0);
          if(ch_path<2.0){
            not_add=1;
            break;
          }
        }
      }
      if(not_add!=1){
        result_waypoints.push_back({add_x, add_y});
        p.x=add_x; p.y=add_y; p.z=0.0;
        add_points.points.push_back(p);
      }
    }
  }
  return path_len+(add_node*2.5);
}

void FindPositions::checkRealPath(){
  ROS_INFO("Function: check real path %lu", waypointsWorld.size());
  
  makeMapForPath();
  //cut the invalid sequence.
  //change the sequence 
  //i i+1 to i-ch i
  int ch=1;
  for(int i=0;i<waypointsWorld.size(); i++){
    int pre_i= (i-ch)==-1? waypointsWorld.size()-1:i-ch;
    double path_len = calculatePath(waypointsWorld[pre_i][0], waypointsWorld[pre_i][1]
                                    , waypointsWorld[i][0], waypointsWorld[i][1], 1);
    if(path_len==0.0){
      ROS_INFO("path is not found..");
      //do it something.
      ch++;
    }
    else{
      ch=1;
      int not_add=0;
      for(int ch_node=0;ch_node<result_waypoints.size();ch_node++){
        double ch_x = result_waypoints[ch_node][0];
        double ch_y = result_waypoints[ch_node][1];
        if(distance(waypointsWorld[i][0], waypointsWorld[i][1], ch_x, ch_y)<1.4){
          double ch_path = calculatePath(waypointsWorld[i][0], waypointsWorld[i][1], ch_x, ch_y, 0);
          if(ch_path<2.0){
            not_add=1;
            break;
          }
        }
      }
      if(not_add!=1){
        result_waypoints.push_back(waypointsWorld[i]);
      }
      //result_waypoints.push_back(waypointsWorld[i]);
    }
  }
  
  
  
  /*
  vector<vector<array<double,2>>> v;
  
  makeMapForPath();
  //cut the invalid sequence.
  int flag=0;
  v.push_back({waypointsWorld[0]});
  for(int i=0;i<waypointsWorld.size(); i++){
    int next_i= i==waypointsWorld.size()-1? 0:i+1;
    double path_len = calculatePath(waypointsWorld[i][0], waypointsWorld[i][1], waypointsWorld[next_i][0], waypointsWorld[next_i][1]);
    //check the length is under 3.0 m (first just ???)
    if(path_len==0.0){
      ROS_INFO("path is not found..");
      //do it something.
    }
    else if(path_len>=3.0){
      flag++;
      v.push_back({waypointsWorld[next_i]});
    }
    else{
      v[flag].push_back(waypointsWorld[next_i]);
    }
  }
  /*
  ROS_INFO("Graph size: %lu", v.size());
  for(int i=0;i<v.size(); i++){
    ROS_INFO("Graph node: %d", i);
    for(int j=0;j<v[i].size(); j++){
      ROS_INFO("x: %lf, y: %lf", v[i][j][0], v[i][j][1]);
    }
    cout<<endl<<endl;
  }
  */
  /*
  //make loop 
  if(v.size()>1){
    if(v[0][0][0]==v.back().back()[0]
      &&v[0][0][1]==v.back().back()[1]){
      v.back().pop_back();
      v[0].insert(v[0].begin(), v.back().begin(), v.back().end());
      v.pop_back();
    }
  }
  
  //print the graph size and each waypoints 
  ROS_INFO("Graph size: %lu", v.size());
  for(int i=0;i<v.size(); i++){
    ROS_INFO("Graph node: %d", i);
    for(int j=0;j<v[i].size(); j++){
      ROS_INFO("x: %lf, y: %lf", v[i][j][0], v[i][j][1]);
    }
  }
  
  ROS_INFO("find near node");
  for(int i=0; i<v[0].size();i++){
    result_waypoints.push_back(v[0][i]);
  }
  if(v.size()>1){
    int node_idx=0;
    
    //vector<vector<array<double,2>>> v_check;
    //v_check.assign(v.begin(), v.end());
    
    /////ej_find_connected_graph
    while(!v.empty()){
      ROS_INFO("Node index is %d", node_idx);
      for(int i=0;i<v[node_idx].size(); i++){
        //node_idx for target
        int near=0;
        int near_node=-1;
        int near_wp=-1;
        double near_len=DBL_MAX;
        for(int n=0;n<v.size(); n++){
          //check the n'th index node in v_check
          if(near==1||node_idx==n) break;
          for(int m=0;m<v[n].size(); m++){
            //m: waypoint's index
            double path_len_ch = calculatePath(v[node_idx][i][0], v[node_idx][i][1],
                                            v[n][m][0], v[n][m][1]);
            if(path_len_ch<3.0){
              for(int ii=m; ii<v[n].size();ii++){
                result_waypoints.push_back(v[n][ii]);
              }
              for(int ii=m-1; ii>=0;ii--){
                result_waypoints.push_back(v[n][ii]);
              }
              vector<vector<array<double,2>>>::iterator nth_1 = v.begin() + node_idx;
              v.erase(nth_1);
              delete *nth_1;
              node_idx = n;
              near=1;
              break;
            }
            if(path_len_ch<near_len){
              near_len=path_len_ch;
              near_node=n;
              near_wp=m;
            }
          }
        }
        if(near!=1){
          for(int ii=near_wp; ii<v[0].size();ii++){
            result_waypoints.push_back(v[near_node][ii]);
          }
          for(int ii=near_wp-1; ii>=0;ii--){
            result_waypoints.push_back(v[near_node][ii]);
          }
          vector<vector<array<double,2>>>::iterator nth_2 = v.begin() + node_idx;
          v.erase(nth_2);
          delete *nth_2;
          node_idx = near_node;
          near=1;
        }
      }
    }
  }
  */
  
  ////ej_visual 
  /*
  for(int i=0;i<v.size(); i++){
    //ROS_INFO("Graph node: %d", i);
    for(int j=0;j<v[i].size(); j++){
      //ROS_INFO("x: %lf, y: %lf", v[i][j][0], v[i][j][1]);
      ////ej_visual
      geometry_msgs::Point p;
      std_msgs::ColorRGBA p_c;
      
      p.x=v[i][j][0]; p.y=v[i][j][1]; p.z=0.0;
      p_c.a = (j==0||j==v[i].size()-1) ? 0.0 : 1.0;
      p_c.b = (j==0||j==v[i].size()-1) ? 0.0 : 1.0;
      m_points.points.push_back(p);
      line_strip.colors.push_back(p_c);
      line_strip.points.push_back(p);
      ////
    }
  }
  */
  ROS_INFO("result_waypoints: size:: %lu", result_waypoints.size());
  for(int i=0;i<result_waypoints.size(); i++){
    geometry_msgs::Point p;
      
    p.x=result_waypoints[i][0]; p.y=result_waypoints[i][1]; p.z=0.0;
    m_points.points.push_back(p);
    line_strip.points.push_back(p);
  }
 
  ////ej_visual
  m_addPointsPub.publish(add_points);
  m_pathPub.publish(m_path_points);
  m_wayPointsPub.publish(m_points);
  m_wayPointsPub.publish(line_strip);
  
    ///////make map test.txt to isualize map////////////
  /*
  ofstream ofile;
  ofile.open("/home/ej/test.txt");
  
  ROS_INFO("file open?: %d", ofile.is_open());
  
  unsigned char* testmap = this->mpo_costmap->getCharMap() ;
  for(uint32_t ridx = 0; ridx < this->map_height; ridx++){
    for(uint32_t cidx=0; cidx < this->map_width; cidx++){
      uint32_t idx = ridx * this->map_width + cidx ;
      if (ofile.is_open()) {
        ofile <<static_cast<unsigned>(testmap[idx]) << ", ";
        cout <<static_cast<unsigned>(testmap[idx])<< ", ";
      }
    }
  }
  
  ofile.close();
  *////////////////////////////////////////////////////
  ////
  //*/
  //ROS_INFO("Sequence: %d, %d\n", i, next_idx);
}


}
