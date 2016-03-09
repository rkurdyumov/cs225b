#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;



class Map_Odom {

public:
  Map_Odom();
  void broadcasterLoop();

private:
  //tf::StampedTransform T_ip_m;
  tf::TransformListener listener;
};

tf::Transform T_ip_m;

Map_Odom::Map_Odom() {

}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped initialpose_msg)
{

  tf::Quaternion orientation = tf::Quaternion(initialpose_msg.pose.pose.orientation.x, 
					      initialpose_msg.pose.pose.orientation.y,
					      initialpose_msg.pose.pose.orientation.y,
					      initialpose_msg.pose.pose.orientation.w);

  tf::Vector3 position = tf::Vector3(initialpose_msg.pose.pose.position.x,
				     initialpose_msg.pose.pose.position.y,
				     initialpose_msg.pose.pose.position.z);


  //Transform from initialpose to map
  T_ip_m = tf::Transform(orientation, position);
}


void Map_Odom::broadcasterLoop() {

  tf::TransformBroadcaster br;

  T_ip_m = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-10, -15, 0.0));

  tf::StampedTransform T_o_bl;

  ros::Rate rate(20.0);
  while (ros::ok()){
    
    ros::spinOnce();  

    tf::TransformListener listener;
    try{
      listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform("odom", "base_link",  
                               ros::Time(0),T_o_bl);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    } 
  
    tf::StampedTransform T_o_m;
    T_o_m.mult(T_ip_m, T_o_bl);

    br.sendTransform(tf::StampedTransform(T_o_m, ros::Time::now(), "map", "odom"));

    cout << "transform sent" << endl;    

    rate.sleep();
  }

  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_odom");
  
  Map_Odom map_odom;

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000, initialPoseCallback);

  map_odom.broadcasterLoop();

 
  return(0);
}
