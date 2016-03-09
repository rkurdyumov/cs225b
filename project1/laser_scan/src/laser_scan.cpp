#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <laser_geometry/laser_geometry.h>
#include <cmath>

#define PI 3.14

ros::Publisher closest_laser_scan_pt_pub;
visualization_msgs::Marker line_strip;

void laserScanCallback(const sensor_msgs::LaserScan laserScan)
{
  tf::TransformListener listener(ros::Duration(10));

  // Find the closest laser scan point
  float ptDistance = 0;
  float minDistance = 1000000;
  float closestAngle = 0;
  int index = 0, closestIndex = 0;
  //std::cout << "min angle = " << laserScan.angle_min << std::endl;
  //std::cout << "max angle = " << laserScan.angle_max << std::endl;
  for(float myAngle = laserScan.angle_min; myAngle <= laserScan.angle_max; myAngle+=laserScan.angle_increment)
  {
    ptDistance = laserScan.ranges[index];
    //std::cout << "ptDistance = " << ptDistance << std::endl;
    if(ptDistance < minDistance && ptDistance > 0.1)
    {
      minDistance = ptDistance;
      closestAngle = myAngle;
      closestIndex = index;
      std::cout << "ptDistance = " << ptDistance << std::endl;
    }
    index++;
  }
  
  

  // Convert the closest point coordinates (r,theta)->(x,y,z)
  geometry_msgs::PointStamped laser_point, odom_point;
  laser_point.point.x = laserScan.ranges[closestIndex]*cos(closestAngle);
  std::cout << "r = " <<  laserScan.ranges[closestIndex] << std::endl;
  std::cout << "theta = " << closestAngle << " cos(theta): " << cos(closestAngle) << std::endl;
  std::cout << "sin(theta): " << sin(closestAngle) << std::endl;
  laser_point.point.y = laserScan.ranges[closestIndex]*sin(closestAngle);
  laser_point.point.z = 0.0;
  laser_point.header.stamp = ros::Time::now();
  laser_point.header.frame_id = "laser";
  std::cout << "point: " << laser_point.point.x << "," << laser_point.point.y << std::endl;

  // Map from the laser frame to the odom frame
  try
  {
    listener.transformPoint("odom", laser_point, odom_point);
  }
  catch(tf::TransformException& ex)
  {
    odom_point = laser_point;
  }

  // Find circle points surrounding the closest point (in the odom frame)
  line_strip.points.clear();
  geometry_msgs::Point circle_point;
  int num_circle_points = 10;
  float radius = 0.05;

  for(float myAngle = 0.0; myAngle <= 360.0; myAngle += (360.0-0.0)/num_circle_points)
  {
    circle_point.x = odom_point.point.x + radius*cos(myAngle*PI/180.0);
    circle_point.y = odom_point.point.y + radius*sin(myAngle*PI/180.0);
    circle_point.z = odom_point.point.z;
    line_strip.points.push_back(circle_point);
  }

  // Publish the points that are in a circle around the closest point
  line_strip.header.stamp = ros::Time::now();
  closest_laser_scan_pt_pub.publish(line_strip);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_scan_listener");
  sensor_msgs::LaserScan scan;
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1000, laserScanCallback);
  closest_laser_scan_pt_pub = n.advertise<visualization_msgs::Marker>("closest_laser_scan_pt_pub", 10);

  ros::Rate rate(10.0);
  line_strip.header.frame_id = "/odom";
  line_strip.ns = "laser_scan";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.02;
  line_strip.color.a = 1.0;
  line_strip.color.r = 1.0;
  ros::spin();

  while(ros::ok()) {
    rate.sleep();
  }
}
