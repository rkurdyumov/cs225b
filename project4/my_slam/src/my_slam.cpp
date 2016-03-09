#include "my_slam.h"

using namespace std;

my_slam::my_slam()
{
	this->sensorFirstTime = true;
	this->odomFirstTime = true;
	this->z_max = 0;
	this->sigma = 0;
	this->lastOdomUpdateTime = ros::Time::now();
}

void my_slam::setOccupancyGridPublisher(ros::Publisher& pub_occupancy_grid)
{
	this->occupancy_grid_pub = pub_occupancy_grid;
}

void my_slam::setGraphPublishers(ros::Publisher& pub_graph_nodes,ros::Publisher& pub_graph_scans,
		ros::Publisher& pub_graph_barycenters,ros::Publisher& pub_graph_odoms,
		ros::Publisher& pub_graph_edges)
{
	this->graphManager.graph_nodes_pub = pub_graph_nodes;
	this->graphManager.graph_scans_pub = pub_graph_scans;
	this->graphManager.graph_barycenters_pub = pub_graph_barycenters;
	this->graphManager.graph_odoms_pub = pub_graph_odoms;
	this->graphManager.graph_edges_pub = pub_graph_edges;
}

void my_slam::processNewScan(const sensor_msgs::LaserScan& scan)
{
	// For plotting the pure odom pose
	geometry_msgs::Pose2D currOdomPose;
	currOdomPose.x = this->odomCurrent.pose.pose.position.x;
	currOdomPose.y = this->odomCurrent.pose.pose.position.y;
	currOdomPose.theta = tf::getYaw(this->odomCurrent.pose.pose.orientation);
	this->graphManager.odoms.push_back(currOdomPose);
	if(this->sensorFirstTime)
	{
		geometry_msgs::Pose2D laser_offset_pose;
		try
		{
			// Get the laser's pose, relative to base.
			tf::TransformListener tf;
			tf::Stamped<tf::Pose> ident;
			tf::Stamped<btTransform> laser_to_base;
			ident.setIdentity();
			ident.frame_id_ = scan.header.frame_id;
			ident.stamp_ = ros::Time();
			//
			if (!tf.waitForTransform("base_link", ident.frame_id_, ros::Time(0), ros::Duration(10.0))) {
				throw "";
			}
			tf.transformPose("base_link", ident, laser_to_base);
			//
			geometry_msgs::Pose2D laser_offset_pose;
			laser_offset_pose.x = laser_to_base.getOrigin().x();
			laser_offset_pose.y = laser_to_base.getOrigin().y();
			laser_offset_pose.theta = tf::getYaw(laser_to_base.getRotation());

			this->BASE_TO_LASER_X = laser_to_base.getOrigin().x();
			this->BASE_TO_LASER_Y = laser_to_base.getOrigin().y();
		}
		catch(...)
		{
			laser_offset_pose.x = DEFAULT_BASE_TO_LASER_X;
			laser_offset_pose.y = DEFAULT_BASE_TO_LASER_Y;
			laser_offset_pose.theta = 0;

			this->BASE_TO_LASER_X = DEFAULT_BASE_TO_LASER_X;
			this->BASE_TO_LASER_Y = DEFAULT_BASE_TO_LASER_Y;
		}
		//
		// Initialize the scan matcher and add new scan to graph
		DoubleVector search_space_sizes;
		search_space_sizes.push_back(0.6);
		search_space_sizes.push_back(0.2);
		//
		DoubleVector search_grid_resolutions;
		search_grid_resolutions.push_back(0.05);
		search_grid_resolutions.push_back(0.01);
		//
		this->scanMatcher = new karto_scan_matcher::KartoScanMatcher(scan, laser_offset_pose, search_space_sizes, search_grid_resolutions);
		//
		geometry_msgs::Pose2D initial_pose = this->odomInitialPose;
		initial_pose.x += this->odomDeltaPose.x; initial_pose.y += this->odomDeltaPose.y; initial_pose.theta += this->odomDeltaPose.theta;
		karto_scan_matcher::ScanWithPose scanWithPose(scan,initial_pose);
		this->z_max = scan.range_max;
		//
		// Add the scan with pose into the graph manager
		this->graphManager.addNode(scanWithPose);

		// Reset the odomInitialPose=(odomInitialPose+odomDeltaPose) and odomDeltaPose=0
		this->resetOdomInitialAndDeltaPose();

		this->sensorFirstTime = false;
		return;
	}
	//
	// Retrieve the corrected parent's pose in the world frame and estimate the child's pose based on odometry
	geometry_msgs::Pose2D parent_pose = this->graphManager.getCurrentScanWithPose().pose;
	geometry_msgs::Pose2D child_pose_estimate = parent_pose;

	util::Random random;
	child_pose_estimate.x += this->odomDeltaPose.x;// + random.gaussian(0.0,0.01);
	child_pose_estimate.y += this->odomDeltaPose.y;// + random.gaussian(0.0,0.01);
	child_pose_estimate.theta += this->odomDeltaPose.theta;// + random.gaussian(0.0,0.001);

	// Match the current scan with the last K scans
	vector<ScanWithPose> referenceScansWithPoses = this->graphManager.getScansWithPosesForIndices(this->graphManager.getLastKNodes());
	karto_scan_matcher::ScanMatchResult res;
	try
	{
		res = this->scanMatcher->scanMatch(scan, child_pose_estimate, referenceScansWithPoses);
	}
	catch(...)
	{
		cout << "Scan match error with parent_pose=" << parent_pose.x << "," << parent_pose.y << "," << parent_pose.theta << ")" << endl;
	}
	// Correct the child's pose estimate in the world frame
	karto_scan_matcher::ScanWithPose scanWithPose(scan,res.pose);
	this->graphManager.addNode(scanWithPose);
	int childIndex = this->graphManager.getLatestScanWithPoseIndex();
	int parentIndex = childIndex-1;

	// Add an odometry constraint in the global frame, given the indices, the child's mean and covariance both in the global frame
	//cout << "Odometry:" << endl << "-----" << endl;
	this->graphManager.addConstraint(parentIndex,childIndex,child_pose_estimate,this->getOdomCovarianceMatrix(parent_pose,this->odomDeltaPose));

	// Add a scan match constraint in the global frame, given the indices, the child's mean and covariance both in the global frame
	//cout << "Scan Match:" << endl << "-----" << endl;
	this->graphManager.addConstraint(parentIndex,childIndex,res.pose,res.cov.cast<double>());

	// Add local registration constraints
	this->addLocalRegistrationConstraints(scan);

	// Reorder the poses so as to minimize the error specified by the constraints
	this->graphManager.optimizeGraph();

	// Visualize the newest scan (in red) and the previous K scans in other different colors
	this->graphManager.visualizeGraph();

	// Reset the odomInitialPose=(odomInitialPose+odomDeltaPose) and odomDeltaPose=0
	this->resetOdomInitialAndDeltaPose();
}

#define MIN_LOCAL_REGISTRATION_RESPONSE 0.3
void my_slam::addLocalRegistrationConstraints(const sensor_msgs::LaserScan& scan)
{
	int childIndex = this->graphManager.getLatestScanWithPoseIndex();
	geometry_msgs::Pose2D childPose = this->graphManager.getCurrentScanWithPose().pose;
	int closestIndex = 0;

	// Find map containing keys with pose indices and values with their corresponding barycenter
	//     distances to childIndex's barycenter
	map<int,float> neighborIndicesAndBarycenterDists = this->graphManager.getNeighborIndicesAndBarycenterDistances(childIndex);
	while(!neighborIndicesAndBarycenterDists.empty())
	{
		// Get the index corresponding to closest barycenter in map
		closestIndex = this->graphManager.findKeyCorrespondingToMaxValueInMap(neighborIndicesAndBarycenterDists);
		vector<int> closestIndexNeighbors = this->graphManager.findClosestNeighborsAroundIndex(closestIndex);

		// Perform scan matching
		vector<ScanWithPose> referenceScansWithPoses = this->graphManager.getScansWithPosesForIndices(closestIndexNeighbors);
		karto_scan_matcher::ScanMatchResult res;
		try
		{
			res = this->scanMatcher->scanMatch(scan, childPose, referenceScansWithPoses);
		}
		catch(...)
		{
			cout << "Scan match error in local registration" << endl;
		}

		// Add a constraint between this neighbor and the child only if the res.response value is large enough
		if(res.response > MIN_LOCAL_REGISTRATION_RESPONSE)
		{
			this->graphManager.addConstraint(closestIndex,childIndex,res.pose,res.cov.cast<double>());
			ParentChildPair parentChildPair;
			parentChildPair.parent = closestIndex;
			parentChildPair.child = childIndex;
			this->graphManager.edges.push_back(parentChildPair);
		}

		// Remove the entry of closestIndex and its nearest N neighbors on either side from the map (if they exist)
		for(int i = 0; i < (int)closestIndexNeighbors.size(); i++)
		{
			int index = closestIndexNeighbors[i];
			if(neighborIndicesAndBarycenterDists.count(index)>0)
				neighborIndicesAndBarycenterDists.erase(index);
		}
	}
}

#define MIN_TRANSLATION_THRESH 0.3         // in meters
#define MIN_ROTATION_THRESH 10.0*PI/180.0  // in radians
void my_slam::laserScanCallback(const sensor_msgs::LaserScan& scan)
{
	// Case we've moved enough to consider using this laser scan update
	if(sqrt(pow(this->odomDeltaPose.x,(float)2.0)+pow(this->odomDeltaPose.y,(float)2.0))>MIN_TRANSLATION_THRESH ||
			abs(this->odomDeltaPose.theta) > MIN_ROTATION_THRESH)
		this->processNewScan(scan);
}

void my_slam::movementCallback(const nav_msgs::Odometry& pos)
{
	if(this->odomFirstTime)
	{
		this->odomInitialPose.x = pos.pose.pose.position.x;
		this->odomInitialPose.y = pos.pose.pose.position.y;
		this->odomInitialPose.theta = tf::getYaw(pos.pose.pose.orientation);
		this->odomFirstTime = false;
	}
	this->odomDeltaPose.x = pos.pose.pose.position.x-this->odomInitialPose.x;
	this->odomDeltaPose.y = pos.pose.pose.position.y-this->odomInitialPose.y;
	this->odomDeltaPose.theta = tf::getYaw(pos.pose.pose.orientation)-this->odomInitialPose.theta;

	// Update most current odom (x,y,theta) and (dx/dt,dy/dt,dtheta/dt) info
	this->odomCurrent = pos;
	this->lastOdomUpdateTime = ros::Time::now();
}

// Returns the inverse a matrix found using odometry between parent and child in odom frame
Eigen::Matrix3d my_slam::getOdomCovarianceMatrix(geometry_msgs::Pose2D& parent_pose, geometry_msgs::Pose2D& deltaPose)
{
	float alpha1 = 0.01; 	     // (error in overshoot)/(total trans), m/m
	float alpha2 = 0.03; 	     // (error in drift)/(total trans), m/m
	float alpha3 = 0.02;         // (error in drift)/(total angle), m/rad
	float alpha4 = 36.0/360.0;   // (error in angle)/(total angle), rad/rad
	float alpha5 = 1.0*PI/180.0; // (error in angle)/(total trans), rad/m
	float delta_trans = sqrt(pow(deltaPose.x,2.0)+pow(deltaPose.y,2.0));

	// Compute angle between direction of travel axis and odom axis
	float gamma = atan2(deltaPose.y,deltaPose.x);

	// Create rotation matrix between direction of travel frame and parent frame
	Eigen::Matrix2d R_t_to_p;
	float c = cos(gamma-parent_pose.theta);
	float s = sin(gamma-parent_pose.theta);
	R_t_to_p << c,-s, s,c;

	// Create odometry covariance matrix in travel frame
	Eigen::Matrix2d sigma_xy_t;
	sigma_xy_t << alpha1*delta_trans+alpha3*abs(parent_pose.theta-gamma),0, 0,alpha2*delta_trans+alpha3*abs(parent_pose.theta-gamma);

	// Convert odometry covariance matrix to parent frame
	Eigen::Matrix2d sigma_xy_p = R_t_to_p*sigma_xy_t*R_t_to_p.inverse();

	// Create full matrix (including theta term) in parent frame
	Eigen::Matrix3d sigma_p;
	float child_pose_theta = parent_pose.theta+deltaPose.theta;
	float theta_theta = alpha4*abs(parent_pose.theta-gamma)+alpha5*delta_trans+alpha4*abs(child_pose_theta-gamma);
	sigma_p << sigma_xy_p(0,0),sigma_xy_p(0,1),0, sigma_xy_p(1,0),sigma_xy_p(1,1),0, 0,0,theta_theta;

	// Create rotation matrix between parent frame and odom frame
	Eigen::Matrix3d R_p_to_o;
	c = cos(parent_pose.theta);
	s = sin(parent_pose.theta);
	R_p_to_o << c,-s,0, s,c,0, 0,0,1;

	// Create odometry covariance matrix in odom frame
	Eigen::Matrix3d sigma_o = R_p_to_o*sigma_p*R_p_to_o.inverse();
	return sigma_o;
}

void my_slam::resetOdomInitialAndDeltaPose()
{
	this->odomInitialPose.x += this->odomDeltaPose.x;
	this->odomInitialPose.y += this->odomDeltaPose.y;
	this->odomInitialPose.theta += this->odomDeltaPose.theta;
	this->odomDeltaPose.x = this->odomDeltaPose.y = this->odomDeltaPose.theta = 0;
}

#define MAX_ODOM_WAIT_TIME 10 // in seconds
// Return true if (and we haven't built the occupancy grid previously)
bool my_slam::shouldBuildOccupancyGrid()
{
	static ros::Time lastNotMovingMuchTime = ros::Time::now();
	static bool hasBuiltBefore = false;

	if(hasBuiltBefore)
	{
		return false;
	}

	// Case the timer has expired
	if((ros::Time::now()-lastNotMovingMuchTime > ros::Duration(MAX_ODOM_WAIT_TIME) && lastNotMovingMuchTime > ros::Time(0)) ||
	   (ros::Time::now()-this->lastOdomUpdateTime > ros::Duration(MAX_ODOM_WAIT_TIME) && this->lastOdomUpdateTime  > ros::Time(0)))
	{
		hasBuiltBefore = true;
		return true;
	}

	// Case the timer has not yet expired
	bool notMovingMuch = abs(this->odomCurrent.twist.twist.linear.x)+abs(this->odomCurrent.twist.twist.angular.z) < eps;

	// Case the most recent estimate is that the robot is moving, so keep refreshing the timer
	if(!notMovingMuch)
		lastNotMovingMuchTime = ros::Time::now();

	return false;
}

#define OCC_CELL_SPACING 0.10 // spacing between grid cells, in meters
#define D 10 // Padding around each edge of occupancy grid.  To allow for laser scans to stay on the grid when
             //     the robot is near the edges.
// Uses odds likelihood to probabilistically construct the occupancy grid of the map
void my_slam::buildOccupancyGrid()
{
	const int SKIP_AMOUNT = 2;
	geometry_msgs::Point point;
	vector<float> scanX_inGlobal, scanY_inGlobal;

	cout << "Building occupancy grid" << endl;

	// Loop through each pose and place (x,y) coordinate of each laser scan endpoint in global frame
	for(int scanIndex = 0; scanIndex < (int)this->graphManager.numPoses(); scanIndex++)
	{
		ScanWithPose sWP = this->graphManager.getScanWithPoseForIndex(scanIndex);
		geometry_msgs::Pose2D pose = sWP.pose;
		sensor_msgs::LaserScan scan = sWP.scan;
		// Loop through each laser scan in this pose
		int angleIndex = 0;
		for(double theta = scan.angle_min; theta <= scan.angle_max; theta += scan.angle_increment*SKIP_AMOUNT)
		{
			// The measured laser reading for this pose at this scan angle
			float distance = scan.ranges[angleIndex];

			// Point in laser frame
			btVector3 point_in_laser(distance*cos(theta), distance*sin(theta), 0);
			// Convert from the laser frame to the base frame
			btVector3 point_in_base = btTransform(tf::createQuaternionFromYaw(0), btVector3(-this->BASE_TO_LASER_X, -this->BASE_TO_LASER_Y, 0))*point_in_laser;

			// Transform to the odom frame
			btVector3 point_in_odom = btTransform(tf::createQuaternionFromYaw(pose.theta),
					btVector3(pose.x, pose.y, 0))*point_in_base;
			scanX_inGlobal.push_back(point_in_odom.x());
			scanY_inGlobal.push_back(point_in_odom.y());

			angleIndex += SKIP_AMOUNT;
		}
	}

	// Find max and min of scanX_inGlobal and scanY_inGlobal
	vector<float>::iterator iterXmax = max_element(scanX_inGlobal.begin(),scanX_inGlobal.end()); this->occGridMaxX = *iterXmax;
	vector<float>::iterator iterXmin = min_element(scanX_inGlobal.begin(),scanX_inGlobal.end()); this->occGridMinX = *iterXmin;

	vector<float>::iterator iterYmax = max_element(scanY_inGlobal.begin(),scanY_inGlobal.end()); this->occGridMaxY = *iterYmax;
	vector<float>::iterator iterYmin = min_element(scanY_inGlobal.begin(),scanY_inGlobal.end()); this->occGridMinY = *iterYmin;

	// Create grid where x=[0:OCC_CELL_SPACING:maxX+2*D] and y=[0:OCC_CELL_SPACING:maxY+2*D]
	this->occupancyGrid.resize(int((this->occGridMaxX-this->occGridMinX+2*D)/OCC_CELL_SPACING),
			int((this->occGridMaxY-this->occGridMinY+2*D)/OCC_CELL_SPACING),float(0.0));

	// Loop back through each pose to do ray tracing
	for(int scanIndex = 0; scanIndex < (int)this->graphManager.numPoses(); scanIndex++)
	{
		ScanWithPose sWP = this->graphManager.getScanWithPoseForIndex(scanIndex);
		geometry_msgs::Pose2D pose = sWP.pose;

		// Convert displacement to laser from base_link in odom frame
		btVector3 displacement_to_laser_in_base(this->BASE_TO_LASER_X, this->BASE_TO_LASER_Y, 0);
		btVector3 displacement_to_laser_in_odom = btTransform(tf::createQuaternionFromYaw(pose.theta),
				btVector3(pose.x, pose.y, 0))*displacement_to_laser_in_base;
		util::Pair<float> laserStartCoord(displacement_to_laser_in_odom[0],displacement_to_laser_in_odom[1]);

		sensor_msgs::LaserScan scan = sWP.scan;
		// Loop through each laser scan in this pose
		int angleIndex = 0;
		for(double theta = scan.angle_min; theta <= scan.angle_max; theta += scan.angle_increment*SKIP_AMOUNT)
		{
			// The measured laser reading for this pose at this scan angle
			float z = scan.ranges[angleIndex];

			// Point in laser frame
			btVector3 point_in_laser(z*cos(theta), z*sin(theta), 0);
			// Convert from the laser frame to the base frame
			btVector3 point_in_base = btTransform(tf::createQuaternionFromYaw(0), btVector3(-this->BASE_TO_LASER_X, -this->BASE_TO_LASER_Y, 0))*point_in_laser;

			// Transform to the odom frame
			btVector3 point_in_odom = btTransform(tf::createQuaternionFromYaw(pose.theta),
					btVector3(pose.x, pose.y, 0))*point_in_base;
			util::Pair<float> laserEndCoord((float)point_in_odom.x(),(float)point_in_odom.y());

			// Perform ray tracing to find the indices this scan passes through between poseCoord and scanCoord
			util::Pair<int> laserStartIndices = this->getIndicesForGlobalCoordinate(laserStartCoord);
			util::Pair<int> laserEndIndices = this->getIndicesForGlobalCoordinate(laserEndCoord);
			vector<util::Pair<int> > indicesToVisit = this->rayTrace(laserStartIndices, laserEndIndices);

			util::Pair<int> indices;
			util::Pair<float> globalCoordinate;
			// Loop through every grid cell passed through via ray tracing
			for(int i = 0; i < (int)indicesToVisit.size(); i++)
			{
				indices = indicesToVisit[i];

				// Convert each index into its global coordinate, and compute distance from pose
				globalCoordinate = this->getGlobalCoordinateForIndices(indices);
				float m = util::Utility::euclideanDistance(pose.x,pose.y,globalCoordinate.x,globalCoordinate.y);

				// Create probability distribution for occupied and unoccupied given m
				float P_obst = this->drawProbObst(m,z);
				float P_no_obst = this->drawProbNoObst(z);

				// Add to the log of the odds at this index in this->occupancyGrid
				float currVal = this->occupancyGrid(indices.x,indices.y);
				this->occupancyGrid.assign(indices.x,indices.y,currVal+log(P_obst/P_no_obst));
			}

			angleIndex += SKIP_AMOUNT;
		}
	}

	const float NUMBER_WALL_HITS = 0.10;
	const float oddsOfExactHit = 1.0/(sqrt(2.0*PI)*this->sigma);
	float wallCutoff = NUMBER_WALL_HITS*log(oddsOfExactHit);

	// this->occupancyGrid is now filled in with the cumulative log likelihoods of being occupied
	nav_msgs::GridCells gridCells;
	gridCells.header.frame_id = "odom";
	gridCells.header.stamp = ros::Time::now();
	gridCells.cell_width = OCC_CELL_SPACING;
	gridCells.cell_height = OCC_CELL_SPACING;

	// Loop back over every cell in the grid
	float minValue = 10000;
	float maxValue = -10000;
	float sumValues = 0;
	for(int x = 0; x < this->occupancyGrid.maxX(); x++)
	{
		for(int y = 0; y < this->occupancyGrid.maxY(); y++)
		{
			float value = this->occupancyGrid(x,y);
			if(value > wallCutoff)
			{
				util::Pair<int> indices(x,y);
				util::Pair<float> globalGridCenter = this->getGlobalCoordinateForIndices(indices);
				point.x = globalGridCenter.x;
				point.y = globalGridCenter.y;
				gridCells.cells.push_back(point);
			}

			if(value > maxValue)
				maxValue = value;
			else if(value < minValue)
				minValue = value;
			sumValues += value;
		}
	}
	/*cout << "maxValue=" << maxValue << " minValue=" << minValue << " meanValues=" <<
			sumValues/(this->occupancyGrid.maxX()*this->occupancyGrid.maxY()) <<
			" maxX=" << this->occupancyGrid.maxX() << " maxY=" << this->occupancyGrid.maxY() << endl;*/
	cout << "Built occupancy grid!" << endl;

	this->occupancy_grid_pub.publish(gridCells);
}

// Converts from global coordinates to indices in the matrix which is padded by D on each side
//     and has a global step size of OCC_CELL_SPACING
util::Pair<int>& my_slam::getIndicesForGlobalCoordinate(util::Pair<float>& coord)
{
	util::Pair<int>* indices = new util::Pair<int>();

	if(coord.x < (this->occGridMinX-D) || coord.x > (this->occGridMaxX+D) ||
			coord.y < (this->occGridMinY-D) || coord.y > (this->occGridMaxY+D))
	{
		cout << "getIndicesForGlobalCoordinate()::coord=(" << coord.x << "," << coord.y << ") out of range!" << endl;
		return *indices;
	}

	indices->x = (coord.x-this->occGridMinX+D)/OCC_CELL_SPACING;
	indices->y = (coord.y-this->occGridMinY+D)/OCC_CELL_SPACING;
	return *indices;
}

// Converts from indices to global coordinates in the matrix which is padded by D on each side
//     and has a global step size of OCC_CELL_SPACING
util::Pair<float>& my_slam::getGlobalCoordinateForIndices(util::Pair<int>& indices)
{
	util::Pair<float>* coord = new util::Pair<float>();

	if(indices.x < 0 || indices.x > this->occupancyGrid.maxX() ||
			indices.y < 0 || indices.y > this->occupancyGrid.maxY())
	{
		cout << "getGlobalCoordinateForIndices()::indices=(" << indices.x << "," << indices.y << ") out of range!" << endl;
		return *coord;
	}

	coord->x = (indices.x+0.5)*OCC_CELL_SPACING+this->occGridMinX-D;
	coord->y = (indices.y+0.5)*OCC_CELL_SPACING+this->occGridMinY-D;
	return *coord;
}

// Traverse from indices0 to indices1 using ray tracing to determine all intermediate grid cells in between.
//     Cut ray tracing off after min(indices1+3*sigma, z_max).  Let startIndices=poseIndices and endIndices=endOfLaserIndices
vector<util::Pair<int> >& my_slam::rayTrace(util::Pair<int>& startIndices, util::Pair<int>& endIndices)
{
	const float RAY_TRACE_PAST_ENDPOINT_AMOUNT = 1.0/OCC_CELL_SPACING;
	vector<util::Pair<int> >* indicesToVisit = new vector<util::Pair<int> >();
	util::Pair<float> startGlobalCoords = this->getGlobalCoordinateForIndices(startIndices);
	util::Pair<int> currIndices = startIndices;

	// Instead of stopping at the desired endIndices, traverse 3*sigma past endIndices
	float angle = util::Pair<int>::angle(startIndices,endIndices);
	endIndices.x += int(RAY_TRACE_PAST_ENDPOINT_AMOUNT*cos(angle)); endIndices.y += int(RAY_TRACE_PAST_ENDPOINT_AMOUNT*sin(angle));

	while(true)
	{
		// End when we've traveled further than z_max from startIndices (in meters) or we've traveled to (endIndices+3*sigma)
		util::Pair<float> currGlobalCoords = this->getGlobalCoordinateForIndices(currIndices);
		float distFromStart = util::Utility::euclideanDistance(startGlobalCoords.x,startGlobalCoords.y,currGlobalCoords.x,currGlobalCoords.y);
		if((currIndices.x==endIndices.x && currIndices.y==endIndices.y) || (distFromStart>=this->z_max))
			break;

		// Angle between currIndices and endIndices.  angle=0 means endIndices is directly to the right of currIndices.
		//     angle=pi/2 means endIndices is directly above currIndices.
		angle = util::Pair<int>::angle(currIndices,endIndices);

		// Case we need to move right
		if(angle >= -PI/4 && angle < PI/4)
		{
			currIndices.x = currIndices.x+1;
		}
		// Case we need to move upwards
		else if(angle >= PI/4 && angle < 3*PI/4)
		{
			currIndices.y = currIndices.y+1;
		}
		// Case we need to move downwards
		else if(angle >= -3*PI/4 && angle < -PI/4)
		{
			currIndices.y = currIndices.y-1;
		}
		// Case we need to move left
		else
		{
			currIndices.x = currIndices.x-1;
		}
		if(currIndices.x >= 0 && currIndices.x < this->occupancyGrid.maxX() &&
				currIndices.y >= 0 && currIndices.y < this->occupancyGrid.maxY())
			indicesToVisit->push_back(currIndices);
		else
		{
			cout << "rayTrace()::currIndices=(" << currIndices.x << "," << currIndices.y << ") out of range!" << endl;
			break;
		}
	}
	return *indicesToVisit;
}

// Compute the probability of z given an m and that an obstacle is present
float my_slam::drawProbObst(float m, float z)
{
	this->sigma = this->z_max/300.0;       // std of Gaussian
	float a = (m-this->sigma);             // where uniform cuts off and Gaussian begins

	// Case in the uniform region, m>z and we are in a space closer than the laser distance reading
	if(z < a)
	{
		return 1.0/this->z_max;
	}
	// Case in the Gaussian region, m~z so we are nearby the laser distance reading (or at least less than
	//     the range_max reading
	else if(z >= a && z < this->z_max-eps)
	{
		return 1.0/(sqrt(2.0*PI)*this->sigma)*exp(-pow((z-m)/this->sigma,2.0));
	}
	// Case in the z_max region, this reading could possibly be a range_max
	else
	{
		return 0.1;
	}
}

// Compute the probability of z given that an obstacle is not present
float my_slam::drawProbNoObst(float z)
{
	// Case in the uniform region, m<z and we are in a space closer than range_max reading
	if(z < this->z_max-eps)
	{
		return 1.0/this->z_max;
	}
	// Case in the z_max region, this reading could possibly be a range_max
	else
	{
		return 0.3;
	}
}

