#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

using namespace std;

int kfd = 0;
struct termios cooked, raw;
#define KEYCODE_LETTER_T 0x74

class FrameBroadcaster
{
public:
  FrameBroadcaster();
  void keyLoop();
  void watchdog();
  ros::Publisher last_pose_marker_pub;
  visualization_msgs::Marker line_strip;

private:
  tf::StampedTransform lastTransform, updatedTransform;
  ros::Time last_publish_, first_publish_;
  boost::mutex publish_mutex_;
  tf::TransformBroadcaster br;
  
};

FrameBroadcaster::FrameBroadcaster()
{
}

void FrameBroadcaster::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  tf::TransformListener listener;
  static bool first_time = true;
  try{
      listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("odom", "base_link",  
                               ros::Time(0), updatedTransform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    } 
    cout << "updatedTransform: {" << updatedTransform.getOrigin().x() << "," <<  updatedTransform.getOrigin().y() << "," <<  updatedTransform.getOrigin().z() << "}" << endl;

    if (first_time) {
	lastTransform = updatedTransform;
	first_time = false;
    }

  cout << "last_pose Transform: {" << lastTransform.getRotation().x() << "," <<  lastTransform.getRotation().y() << "," <<  lastTransform.getRotation().z() << "," << lastTransform.getRotation().w() << "}" << endl;
      br.sendTransform(tf::StampedTransform(lastTransform, ros::Time::now(), "odom", "last_pose"));
      line_strip.points.clear();

      // Get two points to draw between
      geometry_msgs::Point p1,p2;
      tf::StampedTransform mapToBaseLinkTransform = updatedTransform;
      tf::StampedTransform mapToLastPoseTransform = lastTransform;

      p1.x = mapToBaseLinkTransform.getOrigin().x();
      p2.x = mapToLastPoseTransform.getOrigin().x();
      p1.y = mapToBaseLinkTransform.getOrigin().y();
      p2.y = mapToLastPoseTransform.getOrigin().y();
      p1.z = mapToBaseLinkTransform.getOrigin().z();
      p2.z = mapToLastPoseTransform.getOrigin().z();

      cout << "p1: {" << p1.x << "," << p1.y << "," << p1.z <<  "}" << endl;
      cout << "p2: {" << p2.x << "," << p2.y << "," << p2.z <<  "}" << endl;
      //p1.x = p2.x = p1.y = p2.y = p1.z = 1;
      //p2.z = 0;


      // Publish the line_strip message for rviz
      line_strip.points.push_back(p1); line_strip.points.push_back(p2);
      line_strip.header.stamp = ros::Time::now();
      last_pose_marker_pub.publish(line_strip);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_broadcaster");
  FrameBroadcaster frameBroadcaster;
  ros::NodeHandle n;
  frameBroadcaster.last_pose_marker_pub = n.advertise<visualization_msgs::Marker>("last_pose_marker_pub", 10);
  frameBroadcaster.line_strip.header.frame_id = "/odom";
  frameBroadcaster.line_strip.ns = "points_and_lines";
  frameBroadcaster.line_strip.action = visualization_msgs::Marker::ADD;
  frameBroadcaster.line_strip.pose.orientation.w = 1.0;
  frameBroadcaster.line_strip.id = 0;
  frameBroadcaster.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  frameBroadcaster.line_strip.scale.x = 0.01;
  frameBroadcaster.line_strip.color.a = 1.0;
  frameBroadcaster.line_strip.color.r = 1.0;
  frameBroadcaster.line_strip.color.b = 1.0;

  boost::thread my_thread(boost::bind(&FrameBroadcaster::keyLoop, &frameBroadcaster));
  
  
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&FrameBroadcaster::watchdog, &frameBroadcaster));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;
      
  return(0);
}

void FrameBroadcaster::keyLoop()
{

  ros::Rate rate(10.0);

  // Key reading stuff                                                            
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  char c;

  while (ros::ok()){
    
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
	perror("read():");
	exit(-1);
    }

    boost::mutex::scoped_lock lock(publish_mutex_);
    if(c == KEYCODE_LETTER_T)
      {
	lastTransform = updatedTransform;
      }  

    rate.sleep();
  }
  return;
};
