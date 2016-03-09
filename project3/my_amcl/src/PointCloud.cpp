#include "PointCloud.h"

PointCloud::PointCloud(int newNumStartPoses/* = DEFAULT_NUM_POSES*/)
    {
      this->numStartPoses = newNumStartPoses;
    }
    
    /*void PointCloud::setGridAndCheckValueAndMetersPerCell(Matrix<int>newGrid, int newValidStateValue, float newMetersPerCell)
    {
      this->grid = newGrid;
      this->validStateValue = newValidStateValue;
      this->metersPerCell = newMetersPerCell;
    }*/
    
    State& PointCloud::uniformlyDistributePose(float x0, float y0, float L, float W,float theta_min,float theta_max,boost::function<bool (State)>& isValidState)
    {
        float x,y;
	// Make sure the (x,y) isn't in an unauthorized coordinate (such as a wall)
	do
	{
	  x = this->random.uniform(x0,x0+L);
	  y = this->random.uniform(y0,y0+W);
	}while(!isValidState(State(x,y)));
	float theta = this->random.uniform(theta_min,theta_max);
	
	State* pose = new State(x,y,theta);
	return *pose;
    }
  void PointCloud::uniformDistribution(float x0, float y0, float L, float W,float theta_min,float theta_max,boost::function<bool (State)>& isValidState)
  {
    this->poses.clear();
    this->poses.resize(this->numStartPoses);
    for(int i = 0; i < (int)this->poses.size(); i++)
    {
      this->poses[i] = this->uniformlyDistributePose(x0,y0,L,W,theta_min,theta_max,isValidState);
    }
  }
  
  State& PointCloud::gaussianDistributePose(float mu_x, float sigma2_x,float mu_y, float sigma2_y,
			    float mu_theta,float sigma2_theta,boost::function<bool (State)>& isValidState)
  {
    float x,y;
	// Make sure the (x,y) isn't outside the bounds of the valid area
	do
	{
	  x = this->random.gaussian(mu_x,sigma2_x);
	  y = this->random.gaussian(mu_y,sigma2_y);
	}while(!isValidState(State(x,y)));
	  
	float theta = this->random.gaussian(mu_theta,sigma2_theta);
	State* pose = new State(x,y,theta);
	return *pose;
  }
  void PointCloud::gaussianDistribution(float mu_x, float sigma2_x,float mu_y, float sigma2_y,
			    float mu_theta,float sigma2_theta,boost::function<bool (State)>& isValidState)
  {
    this->poses.clear();
    this->poses.resize(this->numStartPoses);
    for(int i = 0; i < (int)this->poses.size(); i++)
      {
	this->poses[i] = this->gaussianDistributePose(mu_x,sigma2_x,mu_y,sigma2_y,mu_theta,sigma2_theta,isValidState);
      }
  }
  
  State& PointCloud::mean()
  {
    State sumState;
    int numPoses = (int)this->poses.size();
    for(int i = 0; i < numPoses; i++)
    {
      sumState.x += this->poses[i].x;
      sumState.y += this->poses[i].y;
      sumState.theta += this->poses[i].theta;
    }
    State* meanState = new State(sumState.x/numPoses,sumState.y/numPoses,sumState.theta/numPoses);
    return *meanState;
  }
  
  State& PointCloud::variance()
  {
    State& meanState = this->mean();
    State* varianceState = new State();
    for(int i = 0; i < (int)this->poses.size(); i++)
    {
      varianceState->x += pow(this->poses[i].x-meanState.x,2.0);
      varianceState->y += pow(this->poses[i].y-meanState.y,2.0);
      varianceState->theta += pow(this->poses[i].theta-meanState.theta,2.0);
    }
    return *varianceState;
  }
  
  void PointCloud::setNumPoses(int newSize)
  {
    if(newSize != this->numPoses())
      this->poses.resize(newSize);
  }
  
  int PointCloud::numPoses()
  {
    return (int)this->poses.size();
  }
  
  void PointCloud::setPublisher(ros::Publisher& newPublisher)
  {
    this->posePublisher = newPublisher;
  }
  
  void PointCloud::drawPoses()
{

  if(!this->posePublisher || this->numPoses() <= 0)
    return;
    
  geometry_msgs::PoseArray amcl_poses;
  amcl_poses.header.frame_id = "/map";
  amcl_poses.header.stamp = ros::Time::now();
      
  for(int i = 0; i < (int)this->poses.size(); i++)
  {
      geometry_msgs::Point p1;
      geometry_msgs::Quaternion q1;
      p1.x = this->poses[i].x;
      p1.y = this->poses[i].y;
      q1 = tf::createQuaternionMsgFromYaw(this->poses[i].theta);
      geometry_msgs::Pose pose;
      pose.position = p1;
      pose.orientation = q1;
      amcl_poses.poses.push_back(pose);
  }
  this->posePublisher.publish(amcl_poses);
}
