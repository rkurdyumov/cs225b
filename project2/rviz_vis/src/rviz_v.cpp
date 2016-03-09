#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <global_planner/GlobalPath.h>
#include <global_planner/NavFunction.h>
#include "../../../include/util.h"
#include <iostream>
#include <vector>

using namespace std;

ros::Publisher global_path_pub;
ros::Publisher nav_fn_pub;

void navFnCallback(const global_planner::NavFunction nav)
{
  cout << "got nav function" << endl;
		
  visualization_msgs::Marker nav_fn;
  nav_fn.header.frame_id = "/map";
  nav_fn.header.stamp = ros::Time::now();
  nav_fn.ns = "nav_traj";
  nav_fn.action = visualization_msgs::Marker::ADD;
  nav_fn.id = 0;
  nav_fn.type = visualization_msgs::Marker::POINTS;
  nav_fn.scale.x = 0.05;
  nav_fn.scale.y = 0.05;
  nav_fn.pose.orientation.w = 1.0;
  nav_fn.color.r = 1.0;
  nav_fn.color.a = 1.0;
      
  cout << "nav function has size " << nav.h_val.size() << endl;
  for(int i = 0; i < (int)nav.h_val.size(); i++){

      geometry_msgs::Point  p1;
      std_msgs::ColorRGBA color;
      p1.x = nav.resolution*nav.h_x[i];
      p1.y = nav.resolution*nav.h_y[i];
      float max_color = 20;
      color.r = color.b = color.g = (nav.h_val[i] > max_color) ? 0 : nav.h_val[i]/max_color;
      color.a = 1;
      nav_fn.points.push_back(p1);
      nav_fn.colors.push_back(color);
  }
  
  cout << "max nav function value = " << nav.maxValue << endl;
  nav_fn_pub.publish(nav_fn);
  cout << "sent nav fn visual" << endl;
};

void pathCallback(const global_planner::GlobalPath path ){
 
  cout << "got global path" << endl;

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "/map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "global_path";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.05;
  line_strip.color.b = 1.;		//blue lines
  line_strip.color.a = 1.;
  for(int i = 0; i < (int)path.points.size(); i++){
    geometry_msgs::Point p;
    p.x =  path.points[i].point.x;
    p.y =  path.points[i].point.y;
    p.z =  path.points[i].point.z;
    line_strip.points.push_back(p);
  }
  global_path_pub.publish(line_strip);
  cout << "sent path visual" << endl;
};

int main (int argc, char** argv) {

  ros::init(argc, argv, "rviz_vis");
	
  ros::NodeHandle n;
  cout << "running visualization" << endl;
  	
  ros::Subscriber sub = n.subscribe<global_planner::NavFunction>("/nav_fn",1,navFnCallback);
  ros::Subscriber sub1 = n.subscribe<global_planner::GlobalPath>("/global_path",1,pathCallback);
  
  // Create latched topics so we can publish the global path and nav fn once
  global_path_pub = n.advertise<visualization_msgs::Marker>("global_path_vis", 1, true);
  nav_fn_pub = n.advertise<visualization_msgs::Marker>("nav_fn_vis", 1, true);

  ros::spin();
};
