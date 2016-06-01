#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <unistd.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

// This class implements the DNS algorithm
#include "Controller.h"

using namespace std;

// I know global variables are dangerous but how else is this to be done?
Controller * aController;

//===========================================================================
// Topic subscriber callbacks:
//===========================================================================

// The bearings contained in the msg should be measured with respect to the front 
// of the robot (ie. in the local from of the robot). Bearings CCW should from 
// [0, pi], and bearings CW should be from[0, -pi]
void NeighbourBearingsCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg){
  // Pass the vector of neighbour bearings to the controller
  aController->UpdateNeighbourBearings(msg->data);  
}

// The compass heading contained in msg should follow the same convention as 
// the bearings to neighbours ie. [pi,-pi]
void CompassCallBack(const std_msgs::Float64::ConstPtr& msg){
  // Pass the headint to the controller object
  aController->UpdateHeading(msg->data);
}

//===========================================================================
// Main Function
//===========================================================================
int main(int argc,char** argv){  
  
  // Create a ROS node
  ros::init(argc,argv, "DNS_Node");
  ros::NodeHandle nodeHandle;
  
  // Make sure that rosmaster is indeed running.
  if(!ros::master::check()){
    printf("ROS check failure...exiting\n");
    printf("Make sure to run roscore before rosrun\n");
    return 0;
  } 
  
  // Subscribe to topics /neighbourBearings and /compass 
  ros::Subscriber neighbourBearingSubscriber = nodeHandle.subscribe("/neighbourBearings",1, NeighbourBearingsCallBack);
  ros::Subscriber compassSubscriber = nodeHandle.subscribe("/compass",1, CompassCallBack);
  
  // Advertize topic /cmd_vel which publishes a twist msg
  ros::Publisher velocityCommandPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  
  // Set the update frequency (10hz)
  ros::Rate loop_rate(10);
  
  // Create an instance of a Controller object
  aController = new Controller();
  
  // The start of the control loop
  printf("DNS node started...\n");
  
  while (ros::ok()){
    // Update the controllers response (velocity vector)
    aController->UpdateResponse();

    // Now publish the controllers response to the /cmd_vel topic
    velocityCommandPublisher.publish(aController->GetTwist());    
    
    // handle ROS messages:
    ros::spinOnce();
  }
  
  // Close down the node.
  delete aController;
  
  ros::shutdown();
  printf("...DNS node stopped!\n");
  return 0;
}

