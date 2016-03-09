#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include "../../../include/search.h"
#include "global_planner/GlobalPath.h"
#include "global_planner/NavFunction.h"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>

using namespace std;

#define ROBOT_RADIUS 0.157 // Robot's radius in meters
#define WALL_VALUE 100
#define INITIALIZER_LARGE 100000
#define INITIALIZER_SMALL -INITIALIZER_LARGE

float METERS_PER_CELL = 0; 
Matrix<int> occupancyGrid; // Stores walls (WALL_VALUE) and empty (0) cells
Matrix<float> distancesToObstacles; // Stores distances from each cell to nearest obstacle
Matrix<float> I;  // Stores intrinsic cost of each cell due to nearest obstacle
list<State> objectStates; // (x,y) locations where objects are present
Matrix<float> h; // Heuristic distances
//
ros::Publisher pubGlobalPath;
ros::Publisher pubNavFn;
//ros::Publisher pubObstacleLocs;
//ros::Publisher pubIdisp;
//ros::Publisher pubhdisp;

// Robot and goal poses
State robotState = State(0,0);
State goalState = State(0,0);

// Given a distance away from object, r, this function returns the intrinsic
//     object cost at this location
float objectCostCalculatorFn(float r)
{
  if(METERS_PER_CELL <= 0)
    throw("global_planner.cpp::objectCostCalculatorFn(): Invalid METERS_PER_CELL");
    
  float epsilon_distance = 2*ROBOT_RADIUS/METERS_PER_CELL;
  float epsilon = 1;  // Value we should be at or below at epsilon_distance
  float sigma = log(4/epsilon)/(epsilon_distance);
  float f = 0;
  if(r <= ROBOT_RADIUS/METERS_PER_CELL)
    f = WALL_VALUE;
  else if(r >= epsilon_distance)
    f = 0;
  else
    f = WALL_VALUE*exp(-sigma*(r-ROBOT_RADIUS/METERS_PER_CELL));
    
  return f;
}

// Successor retrievers
list<State>& get4Successors(State state)
{
  State successor;
  list<State>* successorStates = new list<State>();
  float cellWidth = 1;

    for(int i = -1; i <= 1; i++)
    {
      for(int j = -1; j <= 1; j++)
      {
        if(abs(i) != abs(j))
        {  
          successor = state;
          successor.x += i*cellWidth;
          successor.y += j*cellWidth;
          
          // Check to make sure the successor isn't off the grid and isn't
          //     another obstacle
          if( (successor.x >= 0 && successor.x < occupancyGrid.maxX()) &&
              (successor.y >= 0 && successor.y < occupancyGrid.maxY()) &&
              !occupancyGrid.equals(successor.x,successor.y,WALL_VALUE) )
              {
          	successorStates->push_back(successor);
              }
        }
      }
    }
    return *successorStates;
}

list<State>& get8Successors(State state)
{
  State successor;
  list<State>* successorStates = new list<State>();
  float cellWidth = 1;
  
    for(int i = -1; i <= 1; i++)
    {
      for(int j = -1; j <= 1; j++)
      {
        if(i!=0 || j!=0)
        {  
          successor = state;
          successor.x += i*cellWidth;
          successor.y += j*cellWidth;
          
          // Check to make sure the successor isn't off the grid and isn't
          //     another obstacle
          if( (successor.x >= 0 && successor.x < occupancyGrid.maxX()) &&
              (successor.y >= 0 && successor.y < occupancyGrid.maxY()) &&
              !occupancyGrid.equals(successor.x,successor.y,WALL_VALUE) )
          {
          	successorStates->push_back(successor);
          }
        }
      }
    }
    return *successorStates;
}

// Goal conditions
bool farEnoughAwayFromStartState(Node currNode)
{
  if(METERS_PER_CELL <= 0)
    throw("global_planner.cpp::farEnoughAwayFromStartState(): Invalid METERS_PER_CELL");

  Node* tempNode = NULL;
  Node* nextNode = &currNode;
  do
  {
    tempNode = nextNode;
    nextNode = nextNode->parent;
  }while(nextNode);
  
  float distance = Utility::euclideanDistance(tempNode->state,currNode.state);
  bool result = distance > 2.0*ROBOT_RADIUS/METERS_PER_CELL;
  return result;
}

bool isRobotState(Node currNode)
{    
  return (currNode.state == robotState);
}

bool isGoalState(Node currNode)
{
  return (currNode.state == goalState);
}

bool nullGoal(Node currNode)
{
  return false;
}

void updateH()
{
  Search search;
  
  // With I, compute potential at each cell by starting at the goal
  h.resize(I.maxX(),I.maxY(),INITIALIZER_LARGE);
  search.wavefrontPotential(goalState,&get4Successors,&isRobotState,I,h);
  
  /*ValueIndexPair<float> vip = h.max(INITIALIZER_LARGE);
  float hmax = vip.value;
  visualization_msgs::Marker hdisp;
  hdisp.header.frame_id = "/map";
  hdisp.header.stamp = ros::Time::now();
  hdisp.ns = "Idisp";
  hdisp.action = visualization_msgs::Marker::ADD;
  hdisp.pose.orientation.w = 1.0;
  hdisp.scale.x = 0.05;
  hdisp.scale.y = 0.05;
  hdisp.id = 0;
  hdisp.type = visualization_msgs::Marker::POINTS;
  hdisp.color.a = 1;
  hdisp.color.r = 1;
  for(int x = 0; x < h.maxX(); x++)
  {
    for(int y = 0; y < h.maxY(); y++)
    {
      if(h(x,y) < INITIALIZER_LARGE)
      {
      geometry_msgs::Point pt;
      pt.x = x*METERS_PER_CELL;
      pt.y = y*METERS_PER_CELL;
      
      std_msgs::ColorRGBA color;
      color.a = 1;
      float color_set = h(x,y);
      int max_color = 40;
      color_set = (color_set > max_color)?0:color_set/max_color;
      color.r = color_set;
      color.g = color_set;
      color.b = color_set;
      //color.b = h(x,y)/hmax;
      hdisp.colors.push_back(color);
      hdisp.points.push_back(pt);
      }
     }
   }
   pubhdisp.publish(hdisp);
   cout << "Published hdisp with " << hdisp.points.size() << " points with hmax=" << hmax << endl;*/
}

// Responds to the send_global_path request
void sendGlobalPath()
{  
  // Case there is no goalState and/or robotState
  if(robotState.isNull() || goalState.isNull() || h.isEmpty())
    return;
    
  geometry_msgs::PointStamped nextPoint;
  global_planner::GlobalPath msg;
  msg.resolution = METERS_PER_CELL;
  
  // Start at robotState, move a short distance in direction of gradient 
  //     (interpolate at this location), then stop and repeat process
  float h_orientation;
  float step_size = 0.5;
  float distance_from_goal = 1.0;
  
  State state = robotState;
  // Keep doing this movement until we're within distance_from_goal
  //     of the goalState
  int i = 0;
  while(Utility::euclideanDistance(state,goalState) > distance_from_goal && i < 5000)
  {   
    // Compute the gradient of the potential function at every point
    h_orientation = h.gradient_orientation(state);
    
    state.x -= step_size*cos(h_orientation);
    Utility::bound(state.x,0,h.maxX()-1);

    state.y -= step_size*sin(h_orientation);
    Utility::bound(state.y,0,h.maxY()-1);
  
    nextPoint.header.stamp = ros::Time::now();
    nextPoint.header.frame_id = "map";
    nextPoint.point.x = state.x*METERS_PER_CELL;
    nextPoint.point.y = state.y*METERS_PER_CELL;
    try
      {
	msg.points.push_back(nextPoint);
      }
    catch (const bad_alloc &)
      {
	cout << "got bad alloc" << endl;
	//pubGlobalPath.publish(msg);
	return;
      }   
  }
  pubGlobalPath.publish(msg);
  cout << "Published GlobalPath with numSteps=" << msg.points.size() << endl;
}
  
// Responds to the send_nav_fn request
void sendNavFn()
{        
  if(robotState.isNull() || goalState.isNull() || h.isEmpty())
    return;
    
  //cout << "In sendNavFnCallback()" << endl;
  global_planner::NavFunction nav_msg;
  nav_msg.header.stamp = ros::Time::now();
  nav_msg.header.frame_id = "map";
  nav_msg.resolution = METERS_PER_CELL;
  nav_msg.height = h.maxX();
  nav_msg.width = h.maxY();
  for(int x = 0; x < h.maxX(); x++)
  {
    for(int y = 0; y < h.maxY(); y++)
    {
      float value = h(x,y);
      if(value < INITIALIZER_LARGE)
      {
        // Only send values that are less than the initialized INITIALIZER_LARGE value
        nav_msg.h_val.push_back(value);
        nav_msg.h_x.push_back(x);
        nav_msg.h_y.push_back(y);
      }
    }
  }
  
  // Also send the max value in the potential field for display purposes
  ValueIndexPair<float> valueIndexPair = h.max(INITIALIZER_LARGE);
  nav_msg.maxValue  = valueIndexPair.value;
        
  pubNavFn.publish(nav_msg);
  cout << "Published NavFunction" << endl;
  
}

// Robot position (x,y,theta)
void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped amcl_pose)
{
  robotState.x = round(float(amcl_pose.pose.pose.position.x)/METERS_PER_CELL);
  robotState.y = round(float(amcl_pose.pose.pose.position.y)/METERS_PER_CELL);
  robotState.theta = tf::getYaw(amcl_pose.pose.pose.orientation);  
  
  cout << "robotState=(" << robotState.x << "," << robotState.y << ")" << endl;
}

// Obstacle locations (x,y)
void goalPoseCallback(const geometry_msgs::PoseStamped goalPose)
{
  goalState.x = round(float(goalPose.pose.position.x)/METERS_PER_CELL);
  goalState.y = round(float(goalPose.pose.position.y)/METERS_PER_CELL);
  goalState.theta = tf::getYaw(goalPose.pose.orientation);
  
  cout << "goalState=(" << goalState.x << "," << goalState.y << ")" << endl;
  updateH();
  sendNavFn();
  sendGlobalPath();
}

// Wall locations (x,y) - will only get called once on startup of this .exe or on startup of map_server
void occupancyGridCallback(const nav_msgs::OccupancyGrid occupancy_grid)
{  
  // For the object wavefront calculation
  occupancyGrid = Matrix<int>::initMatrixWithValue(occupancy_grid.info.width,occupancy_grid.info.height,0);
  distancesToObstacles = Matrix<float>::initMatrixWithValue(occupancyGrid.maxX(),occupancyGrid.maxY(),INITIALIZER_LARGE);
  METERS_PER_CELL = occupancy_grid.info.resolution;
  objectStates.clear();
  
  /*nav_msgs::GridCells obstacles;
  obstacles.header.frame_id = "/map";
  obstacles.header.stamp = ros::Time::now();
  obstacles.cell_width = METERS_PER_CELL;
  obstacles.cell_height = METERS_PER_CELL;*/
  
  // Copy object information from occupancy_grid.data to occupancyGrid
  for(int x = 0; x < occupancyGrid.maxX(); x++)
  {
    for(int y = 0; y < occupancyGrid.maxY(); y++)
    {
      int data = (int)occupancy_grid.data[y*occupancyGrid.maxX()+x];
      if(data != 0)
      {
        // Set the object wavefront at this location to 0 (bc it is an object)
        occupancyGrid.assign(x,y,WALL_VALUE);
        distancesToObstacles.assign(x,y,0);
        objectStates.push_back(State(x,y));
        
        /*geometry_msgs::Point pt;
        pt.x = x*METERS_PER_CELL;
        pt.y = y*METERS_PER_CELL;
        pt.z = 0;
        obstacles.cells.push_back(pt);*/
      }
    }
  }
  
  //pubObstacleLocs.publish(obstacles);
  //cout << "Published obstacleLocs with " << obstacles.cells.size() << " points" << endl;
  
  Search search;
  // Run a wavefront from each object to the surrounding cells.  distancesToObstacles will be
  //     filled out with the distances to the nearest object from each cell
  search.ucs(objectStates,&get4Successors,&farEnoughAwayFromStartState,distancesToObstacles);
  
  // For all the values in the occupancyGridInvertedCopy that are still very large (>WALL_VALUE),
  //     reset them to 0, signifying that they aren't very close to any objects.  Also, 
  //     pass the distances from objects to the cost calculator function.
  I.resize(distancesToObstacles.maxX(),distancesToObstacles.maxY(),INITIALIZER_LARGE);
  
  for(int x = 0; x < distancesToObstacles.maxX(); x++)
  {
    for(int y = 0; y < distancesToObstacles.maxY(); y++)
    {
      if(distancesToObstacles(x,y) > WALL_VALUE)
        I.assign(x,y,0);
      else
        I.assign(x,y,objectCostCalculatorFn(distancesToObstacles(x,y)));
    }
  }
  
  /*ValueIndexPair<float> vip = I.max(INITIALIZER_LARGE);
  float Imax = vip.value;
  visualization_msgs::Marker Idisp;
  Idisp.header.frame_id = "/map";
  Idisp.header.stamp = ros::Time::now();
  Idisp.ns = "Idisp";
  Idisp.action = visualization_msgs::Marker::ADD;
  Idisp.pose.orientation.w = 1.0;
  Idisp.scale.x = 0.05;
  Idisp.scale.y = 0.05;
  Idisp.id = 0;
  Idisp.type = visualization_msgs::Marker::POINTS;
  Idisp.color.r = 1;
  Idisp.color.a = 1;
  for(int x = 0; x < I.maxX(); x++)
  {
    for(int y = 0; y < I.maxY(); y++)
    {
      if(I(x,y) < INITIALIZER_LARGE)
      {
      geometry_msgs::Point pt;
      pt.x = x*METERS_PER_CELL;
      pt.y = y*METERS_PER_CELL;
      
      std_msgs::ColorRGBA color;
      color.r = I(x,y)/Imax;
      color.a = 1;
      Idisp.colors.push_back(color);
      Idisp.points.push_back(pt);
      }
     }
   }
   pubIdisp.publish(Idisp);
   cout << "Published Idisp with " << Idisp.points.size() << " points with Imax=" << Imax << endl;*/
}

#define BUFFER_SIZE 1
int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle nh;
  //
  pubGlobalPath = nh.advertise<global_planner::GlobalPath>("/global_path",BUFFER_SIZE,true);
  pubNavFn = nh.advertise<global_planner::NavFunction>("/nav_fn",BUFFER_SIZE,true);
  //pubObstacleLocs = nh.advertise<nav_msgs::GridCells>("/obstacle_locs",BUFFER_SIZE,true);
  //pubIdisp = nh.advertise<visualization_msgs::Marker>("/Idisp",BUFFER_SIZE,true);
  //pubhdisp = nh.advertise<visualization_msgs::Marker>("/hdisp",BUFFER_SIZE,true);
  //
  ros::Subscriber sub = nh.subscribe("/amcl_pose", BUFFER_SIZE, robotPoseCallback);
  ros::Subscriber sub1 = nh.subscribe("/map", BUFFER_SIZE, occupancyGridCallback);
  ros::Subscriber sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", BUFFER_SIZE, goalPoseCallback);
  
  ros::spin();
  
  return(0);
}






