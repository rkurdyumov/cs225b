#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include "my_amcl.h"
using namespace std;

my_amcl amcl(20000);  // Access point into all amcl functionalities

// Receives user's initial pose estimate of robot
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& initialpose_msg)
{
  amcl.initialPoseCallback(initialpose_msg);
}

// Receives laser scan data
void laserScanCallback(const sensor_msgs::LaserScan& scan)
{
  amcl.laserScanCallback(scan);
}

// Receives odometry data of robot position
void movementCallback(const nav_msgs::Odometry& pos)
{
  amcl.movementCallback(pos);
}

// Receives the occupancy_grid map (objects and free spaces) from the map_server
void occupancyGridCallback(const nav_msgs::OccupancyGrid& occupancy_grid)
{
  amcl.occupancyGridCallback(occupancy_grid);
}

#define BUFFER_SIZE 1
#define LATCHED_TOPIC true
int main(int argc, char** argv)
{
  ros::init(argc,argv,"my_amcl");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber sub_laser = nh.subscribe("/scan", BUFFER_SIZE, laserScanCallback);
  ros::Subscriber sub_move = nh.subscribe("/odom", BUFFER_SIZE, movementCallback);
  ros::Subscriber sub_grid = nh.subscribe("/map", BUFFER_SIZE, occupancyGridCallback);
  ros::Subscriber sub_pose = nh.subscribe("initialpose", BUFFER_SIZE, initialPoseCallback);
  //
  ros::Publisher pubPoses = nh.advertise<geometry_msgs::PoseArray>("/my_amcl_poses",BUFFER_SIZE,LATCHED_TOPIC);  
  ros::Publisher pubLdisp = nh.advertise<visualization_msgs::Marker>("/my_amcl_Ldisp",BUFFER_SIZE,LATCHED_TOPIC);
  //
  amcl.setPosePublisher(pubPoses);
  amcl.setLikelihoodPublisher(pubLdisp);

  while(ros::ok())
  {
    ros::spinOnce();
    amcl.drawPoses();
    loop_rate.sleep();
  }
  
  return 0;
}
