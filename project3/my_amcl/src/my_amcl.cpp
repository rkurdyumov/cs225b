#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>
#include "../../../include/search.h"
#include "PointCloud.h"
#include "my_amcl.h"
using namespace std;

my_amcl::my_amcl(int numPoses/* = DEFAULT_NUM_POSES*/)
{
      this->alpha1 = 1/360; // deg/deg
      this->alpha2 = 1*PI/180; // rads/m
      this->alpha3 = 0.06; // m/m
      this->lastState = State(0,0);
      this->posesChangedSinceLastDraw = false;
      this->pc.setNumPoses(numPoses);
}

void my_amcl::setPosePublisher(ros::Publisher pub)
{
  this->pc.setPublisher(pub);
}

void my_amcl::setLikelihoodPublisher(ros::Publisher pub)
{
  this->likelihoodPublisher = pub;
}

void my_amcl::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& initialpose_msg)
{ 
  bool runLocalizedGaussian = false;  // Allows user to select type of initialization and gets rid of compiler warnings derived from commenting out instead
  // Case the poses should be distributed locally in a Gaussian shape around the robot's initial pose
  if(runLocalizedGaussian)
  {
    float meanX = initialpose_msg.pose.pose.position.x;
    float varX = 0.5;
    float meanY = initialpose_msg.pose.pose.position.y;
    float varY = 0.5;
    float meanTheta = tf::getYaw(initialpose_msg.pose.pose.orientation);
    float varTheta = 0.3;
    
    boost::function<bool (State)> isFreeState = boost::bind(&my_amcl::isFreeState,this,_1);
    this->pc.gaussianDistribution(meanX,varX,meanY,varY,meanTheta,varTheta,isFreeState);
    this->updatePoseMotionFirstTime = true;
    this->updateBeliefsFromSensorReadingsFirstTime = true;
    this->posesChangedSinceLastDraw = false;
  }
  // Case we want to distribute the poses' (x,y,theta) randomly and uniformly
  else
  {
    float startX = 0; float startY = 0;
    float lengthX = (this->occupancyGrid.maxX()- 2*D)*this->METERS_PER_CELL;
    float lengthY = (this->occupancyGrid.maxY()- 2*D)*this->METERS_PER_CELL;
    float thetaMin = 0; float thetaMax = 2*PI;
    
    boost::function<bool (State)> isFreeState = boost::bind(&my_amcl::isFreeState,this,_1);
    this->pc.uniformDistribution(startX,startY,lengthX,lengthY,thetaMin,thetaMax,isFreeState);
    this->updatePoseMotionFirstTime = true;
    this->updateBeliefsFromSensorReadingsFirstTime = true;
    this->posesChangedSinceLastDraw = false;
  }
}

void my_amcl::laserScanCallback(const sensor_msgs::LaserScan& scan)
{
    if(this->pc.numPoses() <= 0)
      return;
      
    // Update self.beliefs and resample using scanStates info
    // Transform the scanStates to each poses frame and then evaluate
    // Consider only doing this after moving a significant amount
    if(this->updateBeliefsFromSensorReadingsFirstTime)
      this->updateBeliefsFromSensorReadingsFirstTime = false;
    else
    {
      if(!this->shouldUpdateFromSensors)
        return;
    }

    static int numUpdatesSinceReseed = 0;
    float alpha_max = 0.1, alpha_rand = 0.8;
    float alpha_hit = 1 - alpha_max - alpha_rand;
    const int SKIP_AMOUNT = 10;
    const float SCAN_MAX_TOLERANCE = 0.1;
    const float OUTSIDE_MAP_PROBABILITY = 1e-4;
    float P_rand = 1 / (scan.range_max - scan.range_min);
    float P_max = 0; 
    vector<float> log_P_z_x((int)this->pc.numPoses(),0.0);
    float min_sum_log = 0, max_sum_log = -1e7;
    
    // Cycle through each pose
    for (int i = 0; i < (int)this->pc.numPoses(); i++) 
      {
	float sum_log_P_z_x = 0;
	int index = 0;
	// Cycle through every SKIP_AMOUNT laser scan reading
	for (float angle = scan.angle_min; angle <= scan.angle_max; 
	     angle += SKIP_AMOUNT * scan.angle_increment)
	  {
	    // Compute the (x,y,theta) of this laser scan reading in laser frame
	    float ptDistance = scan.ranges[index];
	    float P_hit;
	    float new_direction = Utility::wrap(angle + this->pc.poses[i].theta);
	    float scan_x = ptDistance*cos(new_direction);
	    float scan_y = ptDistance*sin(new_direction);
	    
	    // Transform laser scan reading to likelihood cell frame
	    float x_cells = (this->pc.poses[i].x + scan_x + ROBOT_RADIUS/2.0 + D*this->METERS_PER_CELL) / this->METERS_PER_CELL;
	    float y_cells = (this->pc.poses[i].y + scan_y + ROBOT_RADIUS/2.0 + D*this->METERS_PER_CELL) / this->METERS_PER_CELL;
	    // Case this reading is a "max" reading
	    if (scan.range_max - ptDistance < SCAN_MAX_TOLERANCE)
	      {
	        // Since this is a max reading, ignore the chance of it hitting an object by setting P_hit=0
		P_max = 1;
		P_hit = 0;
	     }
	    else 
	      {
		P_max = 0;
		// Case this laser scan reading lands outside the map, so discount its probability
		if (x_cells < 0 || x_cells >= this->L.maxX() || y_cells < 0 || y_cells >= this->L.maxY())    
		  {
		    P_hit = OUTSIDE_MAP_PROBABILITY;
		  }
		  // Otherwise, this is a valid sensor reading so look up its value in the likelihood grid
		else
		  {
		    P_hit = this->L(x_cells, y_cells);
		  }
	      }
	      // Log linear combination of P_hit, P_max, and P_rand for this laser scan reading
	    float P_tot = log(alpha_hit*P_hit + alpha_max*P_max + alpha_rand*P_rand);
	    // Keep a running tally of P_tot for all scan readings for this pose
	    sum_log_P_z_x += P_tot;
	    
	    // Move to next laser scan reading (SKIP_AMOUNT away)
	    index += SKIP_AMOUNT;
	  }
	
	// Track the smallest and largest laser scan log probability for this pose
	if (min_sum_log > sum_log_P_z_x)
	  min_sum_log = sum_log_P_z_x;
	if (max_sum_log < sum_log_P_z_x)
	  max_sum_log = sum_log_P_z_x;
	
	// Store the log sum likelihood for this pose
	log_P_z_x[i] = sum_log_P_z_x;
      }
      
      // Shift the log likelihood values for all poses up by a constant amount to
      //     avoid underflow/overflow issues
    float C = 0;
    if(max_sum_log-min_sum_log > 50)
      C = max_sum_log + 30;
    else
      C = (max_sum_log + min_sum_log) / 2.0;

    // Convert out of log domain
    vector<float> P_z_x((int)this->pc.numPoses(),0.0);
    float sum_P_z_x = 0;
    for (int i = 0; i < (int)this->pc.numPoses(); i++) 
    {
      P_z_x[i] = exp(log_P_z_x[i] - C);
      
      // Keep a running sum of all pose likelihoods to prepare to divide by
      //     total probability
      sum_P_z_x += P_z_x[i];
    }

    // Create a ring of weights, which will be used for resampling
    vector<float> ring((int)this->pc.numPoses(),0.0);
    // The zeroth entry of the ring begins at zero rads and has width equal to
    //     the zeroth pose probability (normalized by the total probability)
    ring[0] = 0 + P_z_x[0] / sum_P_z_x * 2 * PI; 
    
    // Fill in the rest of the slices of the ring   
    for (int i = 0; i < (int)P_z_x.size(); i++)
    {
	ring[i] = ring[i-1] + P_z_x[i] / sum_P_z_x * 2 * PI;
    }
    // To make sure the ring always has an upper bound of 2*PI
    ring[(int)P_z_x.size() - 1] = 2*PI;
    
    // Decide to increase or decrease size of pose set based on variance of poses
    State varState = this->pc.variance();
    
    // Compute the new hypothesized size based on the variance in x and y
    const float MIN_NUM_POSES = 200;  // Absolute minimum number of poses to ever use
    const float MAX_VAR_PER_NUM_POSES_X = 2.4e6/20e3; // Experimentally-obtained maximum variance in x direction
    const float MAX_VAR_PER_NUM_POSES_Y = 5.0e6/20e3; // Experimentally-obtained maximum variance in y direction
    // There is a linear relationship between needed pose size and variance of poses
    float pose_sizeX = MIN_NUM_POSES+this->pc.numStartPoses*varState.x/(this->pc.numPoses()*MAX_VAR_PER_NUM_POSES_X);
    float pose_sizeY = MIN_NUM_POSES+this->pc.numStartPoses*varState.y/(this->pc.numPoses()*MAX_VAR_PER_NUM_POSES_Y);
    
    // Make the official new size of poses the mean of the two estimates computed above
    int pose_size = round(Utility::geometricMean(pose_sizeX,pose_sizeY));
    
    // Make a copy of the poses
    vector<State> poses_copy = this->pc.poses;
    this->pc.poses.clear();
    
    // For rescattering poses when the robot is lost 
    float percent_poses_to_scatter = 0;
    const float MIN_LOCALIZED_LOG_PROB = -84; // Experimentally-determined value that indicates we are lost
                                              //     if the log probabilty for the best pose is below this value
    const float RATIO_POSES_TO_SCATTER = 0.9; // Choose to scatter this ratio of the poses around the map when lost
    const float MIN_NUM_STEPS_BETWEEN_RESCATTERS = 15; // Prevents re-scattering poses too often 
    // Case we have a high probability of being lost
    if(max_sum_log < MIN_LOCALIZED_LOG_PROB && numUpdatesSinceReseed > MIN_NUM_STEPS_BETWEEN_RESCATTERS)
    {
      pose_size = this->pc.numStartPoses;
      percent_poses_to_scatter = RATIO_POSES_TO_SCATTER;
      numUpdatesSinceReseed = 0;
    }
    // Compute the number of poses to scatter based on the ratio found above (could be zero if we're not lost)
    int num_poses_to_scatter = round(pose_size*percent_poses_to_scatter);

    // Amount of randomness to add onto resampled poses in (x,y,theta) dimensions
    const float XY_SPREAD_VAR = 0.01;
    const float THETA_SPREAD_VAR = 0.001;
    // Uniformly sample ring to get a new set of poses.  If we want to rescatter some poses, then only
    //     resample from the ring (pose_size-num_poses_to_scatter) poses
    for (int i = 0; i < pose_size-num_poses_to_scatter; i++)
    {
        // Randomly choose an angle between 0 and 2*PI to sample from the ring for this pose
	float ring_angle = this->random.uniform(0,2*PI);
	
	// Binary search function (ring, ring_angle), returns index of the upper bound.
	//     If we search for pi/2 from the ring (0, pi/4, pi, 2*pi), we return the index of pi
	int k = Utility::ringBinarySearch(ring, ring_angle);

	// Add some random error in point placement to avoid point multiplicity
	State resampledPoint;
	resampledPoint.x = poses_copy[k].x + this->random.gaussian(0, XY_SPREAD_VAR); 
	resampledPoint.y = poses_copy[k].y + this->random.gaussian(0, XY_SPREAD_VAR);
	resampledPoint.theta = poses_copy[k].theta + this->random.gaussian(0, THETA_SPREAD_VAR);
	this->pc.poses.push_back(resampledPoint);
    }
    
    // Randomly scatter points around the map if we are not well-localized
    for(int i = 0; i < num_poses_to_scatter; i++)
    {
      float startX = 0; float startY = 0;
      float lengthX = this->occupancyGrid.maxX(); float lengthY = this->occupancyGrid.maxY();
      float thetaMin = 0; float thetaMax = 2*PI;
      boost::function<bool (State)> isFreeState = boost::bind(&my_amcl::isFreeState,this,_1);
      State scatteredPoint = this->pc.uniformlyDistributePose(startX,startY,lengthX,lengthY,thetaMin,thetaMax,isFreeState);
      this->pc.poses.push_back(scatteredPoint);
    }

    this->shouldUpdateFromSensors = false;
    this->posesChangedSinceLastDraw = true;
    numUpdatesSinceReseed++;
  }

void my_amcl::movementCallback(const nav_msgs::Odometry& pos)
{
  State currState;
  currState.x = pos.pose.pose.position.x;
  currState.y = pos.pose.pose.position.y;
  currState.theta = tf::getYaw(pos.pose.pose.orientation);
  
    if(this->pc.numPoses() <= 0)
      return;
      
    if(this->updatePoseMotionFirstTime)
    {
      this->lastState = currState;
      this->updatePoseMotionFirstTime = false;
      return;
    }
  
    const float MIN_DISTANCE = 0.3; // Minimum distance to have move between recalls of this function (in meters)
    const float MIN_ANGLE = 10.0;   // Minimum angle to have turned between recalls of this function (in degrees)
  
    // Case we haven't moved "far" enough
    if(Utility::euclideanDistance(currState,lastState)<MIN_DISTANCE && abs(currState.theta-lastState.theta)<MIN_ANGLE*PI/180.0)
      return;
  
    // Compute measured movement between last and current state
    float d_trans = sqrt(pow(currState.x-lastState.x,2.0)+pow(currState.y-lastState.y,2.0));
    float d_rot1 = atan2(currState.y-lastState.y,currState.x-lastState.x) - lastState.theta;
    float d_rot2 = currState.theta - lastState.theta - d_rot1;
  
    // Create three gaussians with mean=0 and variance specified by alpha params
    //     and d_trans, d_rot1, d_rot2
    float var_rot1 = this->alpha1*abs(d_rot1) + this->alpha2*abs(d_trans);
    float var_rot2 = this->alpha1*abs(d_rot2) + this->alpha2*abs(d_trans);
    float var_trans = this->alpha3*abs(d_trans);
  
    // For every point in cloud, randomly sample from 3 gaussians with means of
    //     (d_trans,d_rot1,d_rot2) and variances of (var_rot1,var_rot2,var_trans)
    for(int i = 0; i < (int)this->pc.numPoses(); i++)
      {
	float d_trans_hat = this->random.gaussian(d_trans,var_trans);
	float d_rot1_hat = this->random.gaussian(d_rot1,var_rot1);
	float d_rot2_hat = this->random.gaussian(d_rot2,var_rot2);
    
	// Update the (x,y,theta) using (d_trans_hat,d_rot1_hat,d_rot2_hat)
	float newX = this->pc.poses[i].x + d_trans_hat*cos(this->pc.poses[i].theta+d_rot1_hat);
	float newY = this->pc.poses[i].y + d_trans_hat*sin(this->pc.poses[i].theta+d_rot1_hat);
	
	// Case the (newX,newY) keep us in bounds and are free space on the map
	if(newX >= 0 && newX < this->occupancyGrid.maxX() && newY >= 0 && newY < this->occupancyGrid.maxY() &&
	   this->occupancyGrid(floor(newX/this->METERS_PER_CELL),floor(newY/this->METERS_PER_CELL))==IS_FREE_SPACE)
	{
	  this->pc.poses[i].x = newX;
	  this->pc.poses[i].y = newY;
	  this->pc.poses[i].theta += Utility::wrap(d_rot1_hat+d_rot2_hat, 0, 2*PI);
	}
      }
    
    this->lastState = currState;
    this->shouldUpdateFromSensors = true;
    this->posesChangedSinceLastDraw = true;
  }

// Wall locations (x,y) - will only get called once on startup of this .exe or on startup of map_server
void my_amcl::occupancyGridCallback(const nav_msgs::OccupancyGrid& occupancy_grid)
{  
  // For the object wavefront calculation
  this->occupancyGrid = Matrix<int>::initMatrixWithValue(occupancy_grid.info.width + 2*this->D,
						   occupancy_grid.info.height + 2*this->D,0);
  // Holds the raw values from occupancy_grid in its original dimensions						   
  Matrix<bool> occupancyGrid_originalSize = Matrix<bool>::initMatrixWithValue(occupancy_grid.info.width, occupancy_grid.info.height,false);	
  // Stores distances from each cell to nearest obstacle				   
  Matrix<float> distancesToObstacles = Matrix<float>::initMatrixWithValue(occupancyGrid.maxX(),
							    occupancyGrid.maxY(),INITIALIZER_LARGE);
  this->METERS_PER_CELL = occupancy_grid.info.resolution;
  list<State> objectStates; // (x,y) locations where objects are present
  
  // Copy object information from occupancy_grid.data to occupancyGrid
  for(int x = 0; x < (int)occupancy_grid.info.width; x++)
  {
    for(int y = 0; y < (int)occupancy_grid.info.height; y++)
    {
      int data = (int)occupancy_grid.data[y*occupancy_grid.info.width+x];
      
      // Setting occupancyGrid_originalSize to true means it's free space
      if(data==MAP_FREE_SPACE)
        occupancyGrid_originalSize.assign(x,y,IS_FREE_SPACE);
 
      // Case this cell is holding an object
      if(data > MAP_FREE_SPACE)
      {
        // Set the object wavefront at this location to 0
        this->occupancyGrid.assign(x+this->D,y+this->D,WALL_VALUE);
        distancesToObstacles.assign(x+this->D,y+this->D,0);
        objectStates.push_back(State(x+this->D,y+this->D));
      }
    }
  }
  // Run a wavefront from each object to the surrounding cells.  distancesToObstacles will be
  //     filled out with the distances to the nearest object from each cell
  Search search;
  boost::function<list<State>& (State&)> get4Successors = boost::bind(&my_amcl::get4Successors,this,_1);
  boost::function<bool(Node&)> farEnoughAwayFromStartState = boost::bind(&my_amcl::farEnoughAwayFromStartState,this,_1);
  search.ucs(objectStates,get4Successors,farEnoughAwayFromStartState,distancesToObstacles);
  
  // Stores probability of laser ray hitting obstacle in current cell
  this->L = Matrix<float>::initMatrixWithValue(distancesToObstacles.maxX(),distancesToObstacles.maxY(),INITIALIZER_LARGE);
  
  for(int x = 0; x < distancesToObstacles.maxX(); x++)
  {
    for(int y = 0; y < distancesToObstacles.maxY(); y++)
    {
      // Case there is an object at this location
      if(distancesToObstacles(x,y) > WALL_VALUE)
        this->L.assign(x,y,0);
      else
        this->L.assign(x,y,this->objectCostCalculatorFn(distancesToObstacles(x,y)));
    }
  }

  // Publish the likelihood field of the map
  ValueIndexPair<float> vip = this->L.max(INITIALIZER_LARGE);
  float Lmax = vip.value;
  visualization_msgs::Marker Ldisp;
  Ldisp.header.frame_id = "/map";
  Ldisp.header.stamp = ros::Time::now();
  Ldisp.ns = "Ldisp";
  Ldisp.action = visualization_msgs::Marker::ADD;
  Ldisp.pose.orientation.w = 1.0;
  Ldisp.scale.x = 0.05;
  Ldisp.scale.y = 0.05;
  Ldisp.id = 0;
  Ldisp.type = visualization_msgs::Marker::POINTS;
  Ldisp.color.r = 1;
  Ldisp.color.a = 1;
  for(int x = 0; x < distancesToObstacles.maxX(); x++)
    {
      for(int y = 0; y < distancesToObstacles.maxY(); y++)
	{
	  if(this->L(x,y) < INITIALIZER_LARGE)
	    {
	      geometry_msgs::Point pt;
	      pt.x = (x-D)*METERS_PER_CELL;
	      pt.y = (y-D)*METERS_PER_CELL;
      
	      std_msgs::ColorRGBA color;
	      color.r = this->L(x,y)/Lmax;
	      color.a = 1;
	      Ldisp.colors.push_back(color);
	      Ldisp.points.push_back(pt);
	    }
	}
    }
  this->likelihoodPublisher.publish(Ldisp);
}

// Private member variables

void my_amcl::broadcastMapToOdomTransform()
{
  State meanState = this->pc.mean();
  // base_link to map transform
  tf::Transform T_bl_m = tf::Transform(tf::createQuaternionFromYaw(meanState.theta), tf::Vector3(meanState.x, meanState.y, 0.0));
  
  // Odom to base_link transform
  tf::StampedTransform T_o_bl; 
  // Listen for the odom to base_link transform
    tf::TransformListener listener;
    try{
      listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform("odom", "base_link",  
                               ros::Time(0),T_o_bl);
    }
    catch (tf::TransformException ex){
      return;
    } 
  
    // Calculate the odom to map transform
    tf::StampedTransform T_o_m;
    T_o_m.mult(T_bl_m, T_o_bl);

    // Broadcast the odom to map transform
    tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(T_o_m, ros::Time::now(), "map", "odom"));
}

void my_amcl::drawPoses()
{
  if(this->posesChangedSinceLastDraw)
    this->pc.drawPoses();
  
  this->posesChangedSinceLastDraw = false;
  cout << "Published pose array of size " << (int)this->pc.numPoses() << endl;
  
  // Broadcast the map to odom transform
  this->broadcastMapToOdomTransform();
  cout << "Published map to odom tranform" << endl;
}

// Given a distance away from object, r, this function returns the intrinsic
//     object cost at this location
float my_amcl::objectCostCalculatorFn(float r)
{    
  int num_sigma = 4;  // Number of sigmas until the function is assumed to decay to zero
  float var = pow(D,2.0) / pow(num_sigma,2.0); // Variance that satisfies this rate of decay
  float f = 0;
  
  // Case the distance away from center is less than the max value to consider
  if(r < D)
    f = exp(-pow(r,2.0) / (2.0 * var));
  else
    f = 0;

  return f;
}

bool my_amcl::farEnoughAwayFromStartState(Node currNode)
{
  Node* tempNode = NULL;
  Node* nextNode = &currNode;
  do
  {
    tempNode = nextNode;
    nextNode = nextNode->parent;
  }while(nextNode);
  
  float distance = Utility::euclideanDistance(tempNode->state,currNode.state);
  bool result = distance > D;
  return result;
}

list<State>& my_amcl::get4Successors(State state)
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
          if( (successor.x >= 0 && successor.x < this->occupancyGrid.maxX()) &&
              (successor.y >= 0 && successor.y < this->occupancyGrid.maxY()) &&
              !this->occupancyGrid.equals(successor.x,successor.y,WALL_VALUE) )
              {
          	successorStates->push_back(successor);
              }
        }
      }
    }
    return *successorStates;
}

bool my_amcl::isFreeState(State state)
{
  float x = state.x; float y = state.y;
  bool result = this->occupancyGrid(floor(x/this->METERS_PER_CELL),floor(y/this->METERS_PER_CELL))==IS_FREE_SPACE;
  return result;
}
