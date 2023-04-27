
#include "std_msgs/String.h"
 
#include <cstdlib> // Use for the absolute value method abs()
#include <iostream> // Enables command line input and output
 
#include "ros/ros.h" // Necessary header files for ROS
#include "geometry_msgs/Twist.h" // Twist messages (linear & angular velocity)
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation

using namespace std;
 
//khoi tao cac bien 
geometry_msgs::Twist velCommand; // Linear and angular velocity in m/s 
geometry_msgs::Pose2D current; // Current x, y, and theta 
geometry_msgs::Pose2D waypointGoal; // Waypoint x, y, and theta (the waypoint)
 
const double PI = 3.141592654;
const double K_l = 0.5;
const double K_a = 0.5;
const double distanceTolerance = 0.1;
const double angleTolerance = 0.1;
bool goToWaypoint = false;
////////////////

void setup() {
 

  waypointGoal.x = 0.0;
  waypointGoal.y = 0.0;
  velCommand.linear.x = 0.0;
  velCommand.linear.y = 0.0;
  velCommand.linear.z = 0.0;
  velCommand.angular.x = 0.0;
  velCommand.angular.y = 0.0;
  velCommand.angular.z = 0.0;
}
///ham tinh khoang cach toi dich////
double getDistanceToWaypoint() {
  return sqrt(pow(waypointGoal.x - current.x, 2) + pow(
    waypointGoal.y - current.y, 2));
}
/////////////////////////////////
//ham tinh goc lech huong///////
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
 //////////////////////////////////////
/////tinh van toc////
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
void Get_Odometry(const nav_msgs::Odometry::ConstPtr& Odometry_data)
{
  double current = odom_msg.geometry_msgs.Pose2D;
  current.x =  Odometry_data.pose.pose.position.y;
  current.y =  Odometry_data.pose.pose.position.y;
  current.theta =  Odometry_data.pose.pose.orientation.z;
}

 
void updateWaypoint(const geometry_msgs::Pose2D &waypointPose) {
  waypointGoal.x = waypointPose.x;
  waypointGoal.y = waypointPose.y;
  goToWaypoint = true;  
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "caculate_data");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("odom", 1000, Get_Odometry);
  ros::Subscriber waypointPoseSub = n.subscribe("goal_2d", 0, updateWaypoint, 
    ros::TransportHints().tcpNoDelay());
  velocityPub = node.advertise<geometry_msgs::Twist>("vel_amr", 0);
  ros::spin();
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
  return 0;
}