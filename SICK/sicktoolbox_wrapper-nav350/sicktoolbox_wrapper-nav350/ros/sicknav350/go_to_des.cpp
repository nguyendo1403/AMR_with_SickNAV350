/// @file go_to_goal_x_y.cpp
/// @author Addison Sears-Collins
/// @date May 7, 2021
/// 
/// This program takes the coordinates of the turtlesim robot and a goal 
/// waypoint (published by waypoint_publisher.cpp) as input and outputs 
/// a velocity command as a Twist message. The velocity command
/// will be calculated so as to get the turtlesim robot to move to the user's
/// desired goal waypoint (x, y coordinate). The x and y values can be any 
/// number from 0 to 11.
 
#include <cstdlib> // Use for the absolute value method abs()
#include <iostream> // Enables command line input and output
 
#include "ros/ros.h" // Necessary header files for ROS
#include "geometry_msgs/Twist.h" // Twist messages (linear & angular velocity)
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
#include "turtlesim/Pose.h" // x, y, theta, linear & angular velocity
 
// Remove the need to use std:: prefix
using namespace std;
 
// Key variable declarations 
geometry_msgs::Twist velCommand; // Linear and angular velocity in m/s 
geometry_msgs::Pose2D current; // Current x, y, and theta 
geometry_msgs::Pose2D waypointGoal; // Waypoint x, y, and theta (the waypoint)
ros::Publisher velocityPub; // Object used for publishing velocity command
 
const double PI = 3.141592654;
const double K_l = 0.5;
const double K_a = 0.5;
const double distanceTolerance = 0.1;
const double angleTolerance = 0.1;
bool goToWaypoint = false;
 
// Initialized variables and take care of other setup tasks
void setup() {
 
  // We initialize with the default starting coordinate 
  // for the turtlesim simulator
  waypointGoal.x = 5.544445;
  waypointGoal.y = 5.544445;
   

  velCommand.linear.x = 0.0;
  velCommand.linear.y = 0.0;
  velCommand.linear.z = 0.0;
  velCommand.angular.x = 0.0;
  velCommand.angular.y = 0.0;
  velCommand.angular.z = 0.0;
}
 
// Get the distance between the current x,y coordinate and 
// the desired waypoint x,y coordinate.
double getDistanceToWaypoint() {
  return sqrt(pow(waypointGoal.x - current.x, 2) + pow(
    waypointGoal.y - current.y, 2));
}
 
// Get the heading error
// i.e. how many radians does the robot need 
// to turn to head towards the waypoint  
double getHeadingError() {
 
  double deltaX = waypointGoal.x - current.x;
  double deltaY = waypointGoal.y - current.y;
  double waypointHeading = atan2(deltaY, deltaX);
  double headingError = waypointHeading - current.theta;   
   
  // Make sure heading error falls within -PI to PI range
  if (headingError > PI) {
    headingError = headingError - (2 * PI);
  } 
  if (headingError < -PI) {
    headingError = headingError + (2 * PI);
  } 
   
  return headingError;
}
 
// If we haven't yet reached the goal, set the velocity value.
// Otherwise, stop the robot.
void setVelocity() {
 
  double distanceToWaypoint = getDistanceToWaypoint();
  double headingError = getHeadingError();
 
  // If we are not yet at the waypoint
  if (goToWaypoint == true && (abs(distanceToWaypoint) > distanceTolerance)) {
    
    // If the robot's heading is off, fix it.
    if (abs(headingError) > angleTolerance) {
      velCommand.linear.x = 0.0;
      velCommand.angular.z = K_a  * headingError;
    }
    // Just fix the distance gap between current pose and waypoint.
    // The magnitude of the robot's velocity is directly
    // proportional to the distance the robot is from the 
    // goal.
    else {
      velCommand.linear.x = K_l * distanceToWaypoint;
      velCommand.angular.z = 0.0;    
    }
  }
  else {
    cout << "Goal has been reached!" << endl << endl;
    velCommand.linear.x = 0.0;
    velCommand.angular.z = 0.0; 
    goToWaypoint == false;
  }
}
 
// This callback function updates the current position and 
// orientation of the robot. 
void updatePose(const turtlesim::PoseConstPtr &currentPose) {
  current.x = currentPose->x;
  current.y = currentPose->y;
  current.theta = currentPose->theta;
}
 
// This callback function updates the desired waypoint when a waypoint
// message is published to the /waypoint topic
void updateWaypoint(const geometry_msgs::Pose2D &waypointPose) {
  waypointGoal.x = waypointPose.x;
  waypointGoal.y = waypointPose.y;
  goToWaypoint = true;  
}
 
int main(int argc, char **argv) {
 
  setup();  
 
  // Initiate ROS
  ros::init(argc, argv, "go_to_goal_x_y");
     
  // Create the main access point to communicate with ROS
  ros::NodeHandle node;
 
  // Subscribe to the robot's pose
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  // Every time a new pose is received, update the robot's pose.
  ros::Subscriber currentPoseSub = node.subscribe("turtle1/pose", 0, updatePose);
     
  // Subscribe to the user's desired waypoint
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  // Every time a new waypoint is received, update the robot's 
  // desired waypoint.
  // The tcpNoDelay is to reduce latency between nodes and to make sure we are
  // not missing any critical waypoint messages.
  ros::Subscriber waypointPoseSub = node.subscribe("waypoint", 0, updateWaypoint, 
    ros::TransportHints().tcpNoDelay());
 
  // Publish velocity commands to a topic.
  // Hold no messages in the queue. Automatically throw away 
  // any messages that are received that are not able to be
  // processed quickly enough.
  velocityPub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);
 
  
  ros::Rate loop_rate(10); 
 
  while (ros::ok()) {
 
    ros::spinOnce();
 
    setVelocity();
    velocityPub.publish(velCommand);
 
    cout << "Current (x,y) = " << "(" << current.x << "," << current.y << ")"
         << endl
         << "Waypoint (x,y) = " << "(" << waypointGoal.x << ","
         << waypointGoal.y << ")"
         << endl
         << "Distance to Waypoint = " << getDistanceToWaypoint() << " m"
         << endl
         << "Linear Velocity (x) = " << velCommand.linear.x << " m/s"
         << endl << endl;
 

    loop_rate.sleep();
  }
 
  return 0;
}