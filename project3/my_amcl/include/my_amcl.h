#pragma once
#include "PointCloud.h"

#define ROBOT_RADIUS 0.157 // in meters
#define WALL_VALUE 100
#define MAP_FREE_SPACE 0
#define IS_FREE_SPACE true

class my_amcl
{
  public:
    my_amcl(int numPoses = DEFAULT_NUM_POSES);
    // Receives user's initial pose estimate of robot
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& initialpose_msg);

    // Receives laser scan data
void laserScanCallback(const sensor_msgs::LaserScan& scan);

    // Receives odometry data of robot position
void movementCallback(const nav_msgs::Odometry& pos);

// Receives the occupancy_grid map (objects and free spaces) from the map_server
void occupancyGridCallback(const nav_msgs::OccupancyGrid& occupancy_grid);

// Draws the poses in rviz
void drawPoses();

// Set publishers
void setPosePublisher(ros::Publisher pub);
void setLikelihoodPublisher(ros::Publisher pub);

  private:
    // Publish a map to odom transform for rviz to use
    void broadcastMapToOdomTransform();
    
    // This current node has reached a sufficient Euclidean distance from the start node,
    //     which we find by backtracing through the parents
    bool farEnoughAwayFromStartState(Node currNode);
    
    // Find the 4 nearest neighbors to a state
    list<State>& get4Successors(State state);
    
    // Given a distance away from object, r, this function returns the intrinsic
    //     object cost at this location
    float objectCostCalculatorFn(float r);
    
    // Returns true if, at the given state, there is free space in occupancyGrid
    bool isFreeState(State state);
    
    // Used for resampling points in a PointCloud
    static int ringBinarySearch(vector<float>& ring, float ring_angle);
    
    Matrix<int> occupancyGrid; // Occupancy grid of wall and free spaces
    float METERS_PER_CELL;
    int D; // Border size for likelihood around original grid size
    Matrix<float> L; // Likelihood grid of size occupancyGrid.size()+2D
    
    // Access into the PointCloud class
    PointCloud pc;
    
    // Access to random functionalities
    Random random;
    
    // Publishers
    ros::Publisher pubLdisp;
    
    // Flag variables
    bool updatePoseMotionFirstTime;
  bool updateBeliefsFromSensorReadingsFirstTime;
  
  // Movement variables
  State lastState;
  float alpha1, alpha2, alpha3;
  bool firstTime;
    
  // Sensor variables
  bool shouldUpdateFromSensors;
  
  // Visualization
  ros::Publisher likelihoodPublisher;
  bool posesChangedSinceLastDraw;
};
