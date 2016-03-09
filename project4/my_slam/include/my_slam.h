#pragma once
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "karto_scan_matcher.h"
#include "GraphManager.h"
#include <math.h>

struct PoseMeanAndCovariance
{
	geometry_msgs::Pose2D mean;
	Eigen::Matrix3f covariance;
};

#define DEFAULT_BASE_TO_LASER_X 0.0//0.12
#define DEFAULT_BASE_TO_LASER_Y 0.0
#define eps 0.001

class my_slam
{
public:
	my_slam();
	void processNewScan(const sensor_msgs::LaserScan& scan);
	void laserScanCallback(const sensor_msgs::LaserScan& scan);
	void movementCallback(const nav_msgs::Odometry& pos);

	void resetOdomInitialAndDeltaPose();
	void setOccupancyGridPublisher(ros::Publisher& pub_occupancy_grid);
	void setGraphPublishers(ros::Publisher& pub_graph_nodes,ros::Publisher& pub_graph_scans,
			                ros::Publisher& pub_graph_barycenters,ros::Publisher& pub_graph_odoms,
			                ros::Publisher& pub_graph_edges);

	void buildOccupancyGrid();
	bool shouldBuildOccupancyGrid();

private:
	bool odomFirstTime;
	bool sensorFirstTime;
	karto_scan_matcher::KartoScanMatcher* scanMatcher;
	vector<karto_scan_matcher::ScanWithPose> reference_scans;
	geometry_msgs::Pose2D odomInitialPose;
	geometry_msgs::Pose2D odomDeltaPose;
	nav_msgs::Odometry odomCurrent;
	float z_max;

	GraphManager graphManager;
	bool hasBuiltOccupancyGrid;
	util::Matrix<float> occupancyGrid;
	int occGridMaxX, occGridMinX, occGridMaxY, occGridMinY;
	float sigma;
	ros::Publisher occupancy_grid_pub;
	ros::Time lastOdomUpdateTime;

	void addLocalRegistrationConstraints(const sensor_msgs::LaserScan& scan);
	Eigen::Matrix3d getOdomCovarianceMatrix(geometry_msgs::Pose2D& parent_pose, geometry_msgs::Pose2D& deltaPose);
	util::Pair<int>& getIndicesForGlobalCoordinate(util::Pair<float>& coord);
	util::Pair<float>& getGlobalCoordinateForIndices(util::Pair<int>& indices);
	vector<util::Pair<int> >& rayTrace(util::Pair<int>& startIndices, util::Pair<int>& endIndices);
	float drawProbObst(float m, float z);
	float drawProbNoObst(float z);

	float BASE_TO_LASER_X;
	float BASE_TO_LASER_Y;
};
