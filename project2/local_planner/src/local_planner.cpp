#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <global_planner/GlobalPath.h>
#include "../../../include/util.h"
#include "../../../include/search.h"
#include <math.h>
#include <iostream>

#define grid_cells_per_m 10
#define ROBOT_RADIUS 0.157

using namespace std;

struct coordinate {
  double x;
  double y;
  double theta;
};

struct trajectory {
  float w;
  float v;
  vector<coordinate> path;
};

// Robot and goal poses
list<coordinate> global_path;
coordinate currPose;
ros::Publisher local_obstacles_pub;
ros::Publisher local_grid_pub;
ros::Publisher traj_pub;
Matrix<float> local_grid;
Matrix<float> I;
static bool waiting_for_global_path = true;
int updateNum = 0;


// Debug functions
void PrintMatrix(Matrix<float> &grid) {
  cout << "local obstacle wavefronted grid:" << endl;
  for (int i = 0; i < grid.maxY(); i++) {
    for (int j = 0; j < grid.maxX(); j++) {
      cout << setw(3) << (int)grid(i,j) << " ";
    }
    cout << endl;
  }
  cout << endl;
}

list<State>& get4Successors(State state)
{
  State successor;
  list<State>* successorStates = new list<State>();
  float cellWidth = 1;
  //cout << "inside get successors" << endl;
  
    for(int i = -1; i <= 1; i++)
    {
      for(int j = -1; j <= 1; j++)
      {
        if(abs(i) != abs(j))
        {  
          successor = state;
          successor.x += i*cellWidth;
          successor.y += j*cellWidth;
          
	  //cout << "eval successor: {" << successor.x << "," << successor.y << "}" << endl;

	  //if (abs(successor.x - 27) < 0.1)
	  //cout << "got (27,y)" << endl;

          // Check to make sure the successor isn't off the grid and isn't
          //     another obstacle
          if( (successor.x >= 0 && successor.x <= local_grid.maxX() - 1) &&
              (successor.y >= 0 && successor.y <= local_grid.maxY() - 1) &&
	      !local_grid.equals(successor.x,successor.y,0))
          	successorStates->push_back(successor);
        }
      }
    }
    return *successorStates;
}

bool farEnoughAwayFromStartState(Node currNode)
{
  //cout << "inside far enough away" << endl;

  Node* tempNode = NULL;
  Node* nextNode = &currNode;
  do
    {
      tempNode = nextNode;
      nextNode = nextNode->parent;
    }while(nextNode);
  
  float distance = Utility::euclideanDistance(tempNode->state,currNode.state);
  bool result = distance > 5.0*ROBOT_RADIUS * grid_cells_per_m;
  return result;
}

// Get current robot position (x,y,theta)
void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped path) {
  
  cout << "Got amcl pose" << endl;
  cout << "currPose={" << path.pose.pose.position.x << "," << path.pose.pose.position.y << endl;
}

// Given a distance away from object, r, this function returns the intrinsic
//     object cost at this location
float objectCostCalculatorFn(float r)
{
    
  float epsilon_distance = 5.0*ROBOT_RADIUS*grid_cells_per_m;
  float epsilon = 1;  // Value we should be at or below at epsilon_distance
  float sigma = log(4/epsilon)/(epsilon_distance);
  float f = 0;
  if(r <= ROBOT_RADIUS * grid_cells_per_m)
    f = 100;
  else if(r >= epsilon_distance)
    f = 0;
  else
    f = 100*exp(-sigma*(r-ROBOT_RADIUS*grid_cells_per_m));
    
  return f;
}

// Get a laser scan update and create a new model of local obstacles
void laserScanCallback(const sensor_msgs::LaserScan scan) {
  
  updateNum++;
  if (waiting_for_global_path || updateNum%30 != 0)
    return;
  cout << "updateNum=" << updateNum << endl;
  
  tf::TransformListener listener;

  // Filter the laser scan data
  list<geometry_msgs::Point> local_obstacles;
  geometry_msgs::Point curr;

  nav_msgs::GridCells obstacles;
  obstacles.cell_width = 0.1;
  obstacles.cell_height = 0.1;

  double min_y = 1000;
  double max_y = -1000;
  double max_x = -1000;;

  list<State> objectStates;
  int index = 0;
  for (float angle = scan.angle_min; angle <= scan.angle_max; angle += scan.angle_increment) {     
    float ptDistance = scan.ranges[index];
    if (ptDistance > scan.range_min && ptDistance < scan.range_max) {
      curr.x = ptDistance*cos(angle);
      curr.y = ptDistance*sin(angle);
      curr.z = 0;
      if (curr.y < min_y) {min_y = curr.y;}
      if (curr.y > max_y) {max_y = curr.y;}
      if (curr.x > max_x) {max_x = curr.x;}
      //obstacles.cells.push_back(curr);
      objectStates.push_back(State(floor(curr.x * grid_cells_per_m),
				   curr.y * grid_cells_per_m));
    }
    index++;
    //cout << "angle=" << angle*180/PI << endl;
  }
  
  // Move the grid over so we don't have negative values for y
  for (list<State>::iterator iter = objectStates.begin(); iter != objectStates.end(); iter++) {
    iter->y = floor(iter->y - (min_y * grid_cells_per_m));
  }

  //cout << "max_y=" << max_y << ", min_y=" << min_y << ", max_x=" << max_x << endl;
  //cout << "num_obstacles=" << objectStates.size() << endl;

  int grid_x = round(max_x * grid_cells_per_m);
  int grid_y = round((max_y - min_y) * grid_cells_per_m);
  //cout << "grid_x=" << grid_x << " grid_y=" << grid_y << endl;
  /*
  for (int i = 0; i < (int)local_obstacles.size(); i++) {
    //cout << "loop iter=" << i << endl;
    obstacles.cells.push_back(local_obstacles.front());
    local_obstacles.pop_front();
 
  obstacles.header.stamp = ros::Time::now();
  obstacles.header.frame_id = "laser";

  // Publish the current obstacle grid cells
  local_obstacles_pub.publish(obstacles);
  */

  // Create a grid so that the workspace is split into 10cm x 10cm cells, 
  // with each cell initialized to 100
  local_grid.resize(grid_x,grid_y,100);
  for (list<State>::iterator iter = objectStates.begin(); iter != objectStates.end(); iter++)
  {
    //cout << "cell_x=" << obstacles.cells[i].x << ", cell_y=" << obstacles.cells[i].x << endl;
    local_grid.assign(iter->x,
		      iter->y,
		      0);
  }
  // Update the workspace grid by wavefront propagating the obstacle costs

  // Run a wavefront from each object to the surrounding cells.  "grid_local" will be
  //     filled out with the distances to the nearest object from each cell
  Search search;
  search.bfs(objectStates,&get4Successors,&farEnoughAwayFromStartState,local_grid);

  //PrintMatrix(local_grid);

  for(int x = 0; x < local_grid.maxX(); x++)
    {
      for(int y = 0; y < local_grid.maxY(); y++)
	{
	  if(local_grid(x,y) > 100)
	    local_grid.assign(x,y,0);
	  else
	    local_grid.assign(x,y,objectCostCalculatorFn(local_grid(x,y)));
	}
    }

  //PrintMatrix(local_grid);

  // Create a list of possible trajectories
  vector<trajectory> possible_traj;
  const int NUM_SAMPLES = 4;
  const float min_w = -0.4, max_w = 0.4, min_v = 0.4, max_v = 1.0;
  for (float w = min_w; w < max_w; w += (max_w-min_w)/NUM_SAMPLES) {
    for (float v = min_v; v < max_v; v += (max_v - min_v)/NUM_SAMPLES) {
      vector<coordinate> emptyPath;
      trajectory curr = {w, v, emptyPath};
      possible_traj.push_back(curr);
    }
  }
  cout << "got trajectories" << endl;
  
  visualization_msgs::Marker points;
  points.header.frame_id = "laser";
  points.header.stamp = ros::Time::now();
  points.ns = "trajectories";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.05;
  points.scale.y = 0.05;
  points.color.r = 1.0;
  points.color.a = 1.0;

  // Create a set of set of pts for publishing & vector of possible trajectories
  for (int i = 0; i < (int)possible_traj.size(); i++) {
     for (float t = 0.0; t < 1.6; t += 0.2) {
      geometry_msgs::Point p;
      float w = possible_traj[i].w;
      float v = possible_traj[i].v;
      p.x = (v*t)*cos(w*t);
      p.y = (v*t)*sin(w*t);
      p.z = 0;
      coordinate curr = {p.x, p.y, 0};
      possible_traj[i].path.push_back(curr);
      points.points.push_back(p);
    }
  }
  traj_pub.publish(points);
  
  // For each trajectory, find maximum obstacle cost cell it passes through
  vector<float> traj_obstacle_costs(possible_traj.size());
  for (int i = 0; i < (int)possible_traj.size(); i++) {
    float max_cost = 0;
    for (int j = 0; j < (int)possible_traj[i].path.size(); j++) {
      int x = round((possible_traj[i].path[j].x)*grid_cells_per_m);
      int y = round(((possible_traj[i].path[j].y - min_y)*grid_cells_per_m));
      //cout << "evaluating traj pt " << x << "," << y << " with cost " << local_grid(x,y) << endl;
      if (local_grid(x,y) > max_cost) {max_cost = local_grid(x,y);}
    }
    traj_obstacle_costs[i] = max_cost;
    //cout << "trajectory " << i << " has cost " << traj_obstacle_costs[i] << endl << endl;
  }

  // For each trajectory, find the distance from the endpoint to the global path
  
  vector<float> traj_path_proximity_costs(possible_traj.size());
  for (int i = 0; i < (int)possible_traj.size(); i++) {
    float min_distance = 1000;
    for (list<coordinate>::iterator iter = global_path.begin(); iter != global_path.end(); iter++) {
      coordinate currGlobalPt = global_path.back();
      currGlobalPt.x = iter->x;
      currGlobalPt.y = iter->y;
      // Find Euclidean distance 
     }
  }

  // For each trajectory, find the distance from the endpoint to the goal
  vector<float> traj_goal_distance_costs(possible_traj.size());
  
  // Path cost function: 
  // obstacle cost (highest cost cell through which trajectory passes)
  // + distance to global path (min distance from trajectory endpoint to global path)
  // + distance to goal (distance from trajectory endpoint to goal)
  // Display the local paths and the chosen path
}

// Get a copy of the global path
void globalPlanCallback(const global_planner::GlobalPath path)
{
  cout << "inside global path callback" << endl;
  
  // Store the global path (should be in the map frame)
  for (int i = 0; i < (int)path.points.size(); i++) {
    coordinate newPoint;
    newPoint.x = path.points[i].point.x;
    newPoint.y = path.points[i].point.y; 
    newPoint.theta = 0;
    global_path.push_back(newPoint);
  }
  cout << "got global path of size " << global_path.size() << endl;
  waiting_for_global_path = false;
}


#define BUFFER_SIZE 1
int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_planner");
  ros::NodeHandle n;

  cout << "initialized local planner" << endl;

  ros::Subscriber sub = n.subscribe("/global_path", BUFFER_SIZE, globalPlanCallback);
  ros::Subscriber sub1 = n.subscribe("/amcl_pose", BUFFER_SIZE, amclPoseCallback);
  ros::Subscriber sub3 = n.subscribe("/scan", BUFFER_SIZE, laserScanCallback);
  local_obstacles_pub = n.advertise<nav_msgs::GridCells>("local_obstacles_pub", 10);
  traj_pub = n.advertise<visualization_msgs::Marker>("trajectories", 10);

  cout << "finished subscriptions" << endl;

  ros::spin();
  return(0);
}
