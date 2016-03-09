#pragma once
#include "../../../include/util.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <iostream>

#define DEFAULT_NUM_POSES 500
class PointCloud
{
 public:
  PointCloud(int newNumStartPoses = DEFAULT_NUM_POSES);

    // Distribute poses according to uniform distribution across a rectangular space given by corners:
    //     {(x0,y0),(x0,y0+W),(x0+L,y0),(x0+L,y0+W)} and with theta values between theta_min and theta_max
    State& uniformlyDistributePose(float x0, float y0, float L, float W,float theta_min,float theta_max,boost::function<bool (State)>& isValidState);
    void uniformDistribution(float x0, float y0, float L, float W,float theta_min,float theta_max,boost::function<bool (State)>& isValidState);
  
    // Distribute poses according to a normal distribution with means: (mu_x,mu_y,mu_theta) and variances:
    //     (sigma2_x,sigma2_y,sigma2_theta)
    State& gaussianDistributePose(float mu_x, float sigma2_x,float mu_y, float sigma2_y,
			    float mu_theta,float sigma2_theta,boost::function<bool (State)>& isValidState);		    
    void gaussianDistribution(float mu_x, float sigma2_x,float mu_y, float sigma2_y,
			    float mu_theta,float sigma2_theta,boost::function<bool (State)>& isValidState);
  
  // Compute the mean and variance of this->poses in the (x,y,theta) dimensions
  State& mean();
  State& variance();
  
  // Setter and getter for the number of poses in this->poses
  void setNumPoses(int newSize);
  int numPoses();
  
  // For visualization in rviz
  void setPublisher(ros::Publisher& newPublisher);
  void drawPoses();
 
 int numStartPoses;
 vector<State> poses;
  
 private:
  Random random;
  
  // Visualization
  ros::Publisher posePublisher;
};







