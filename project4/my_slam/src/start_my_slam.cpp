#include <ros/ros.h>
#include "my_slam.h"
using namespace std;

my_slam* slam;  // Access point into all slam functionalities

// Receives laser scan data
void laserScanCallback(const sensor_msgs::LaserScan& scan)
{
	slam->laserScanCallback(scan);
}

// Receives odometry data of robot position
void movementCallback(const nav_msgs::Odometry& pos)
{
	slam->movementCallback(pos);
}

#define BUFFER_SIZE 1
#define LATCHED_TOPIC true
int main(int argc, char** argv)
{
	ros::init(argc,argv,"my_slam");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);
	//
	// Start up my_slam
	slam = new my_slam();
	//
	ros::Subscriber sub_laser = nh.subscribe("/scan", BUFFER_SIZE, laserScanCallback);
	ros::Subscriber sub_move = nh.subscribe("/odom", BUFFER_SIZE, movementCallback);
	//
	ros::Publisher pub_occupancy_grid = nh.advertise<nav_msgs::GridCells>("/my_occupancy_grid",BUFFER_SIZE,LATCHED_TOPIC);
	slam->setOccupancyGridPublisher(pub_occupancy_grid);
	ros::Publisher pub_graph_nodes = nh.advertise<geometry_msgs::PoseArray>("/my_graph_nodes",BUFFER_SIZE,LATCHED_TOPIC);
	ros::Publisher pub_graph_odoms = nh.advertise<geometry_msgs::PoseArray>("/my_graph_odoms",BUFFER_SIZE,LATCHED_TOPIC);
	ros::Publisher pub_graph_scans = nh.advertise<visualization_msgs::Marker>("/my_graph_scans",BUFFER_SIZE,LATCHED_TOPIC);
	ros::Publisher pub_graph_barycenters = nh.advertise<visualization_msgs::Marker>("/my_graph_barycenters",BUFFER_SIZE,LATCHED_TOPIC);
	ros::Publisher pub_graph_edges = nh.advertise<visualization_msgs::Marker>("/my_graph_edges",BUFFER_SIZE,LATCHED_TOPIC);
	slam->setGraphPublishers(pub_graph_nodes,pub_graph_scans,pub_graph_barycenters,pub_graph_odoms,pub_graph_edges);
	//
	while(ros::ok())
	{
		ros::spinOnce();
		// If we haven't moved in awhile, consider building the Occupancy Grid
		if(slam->shouldBuildOccupancyGrid())
			slam->buildOccupancyGrid();
		loop_rate.sleep();
	}
	delete slam;
	return 0;
}
