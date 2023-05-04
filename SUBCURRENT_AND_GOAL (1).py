#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage

from geometry_msgs.msg import PoseStamped

from pymodbus.client.sync import ModbusSerialClient
from geometry_msgs.msg import Twist

import math

PI = 3.141592654
K_l = 0.5
K_a = 0.5
distanceTolerance = 0.1
angleTolerance = 0.1
goToWaypoint = False
currentx = 0.0
currenty = 0.0
waypointGoaly = 0.0
waypointGoalx = 0.0
currenttheta = 0.0

velCommand=Twist()

def getDistanceToWaypoint():
    global currentx, currenty,waypointGoalx, waypointGoaly, currenttheta
    return math.sqrt(math.pow(waypointGoalx - currentx, 2) + math.pow(
    waypointGoaly - currenty, 2))

def getHeadingError():
    global currentx, currenty,waypointGoalx, waypointGoaly, currenttheta
    deltaX = waypointGoalx - currentx
    deltaY = waypointGoaly - currenty
    waypointHeading = math.atan2(deltaY, deltaX) 
    headingError = waypointHeading - currenttheta
   
  
    if (headingError > PI) :
        headingError = headingError - (2 * PI)
  
    elif (headingError < -PI) :
        headingError = headingError + (2 * PI)
           
   
    return headingError


def call_back_current(data):  
    global currentx, currenty , currenttheta
    for transform in data.transforms:
        if transform.header.frame_id == 'map' and transform.child_frame_id == 'nav350':
            currentx = transform.transform.translation.x
            currenty = transform.transform.translation.y 
            currenttheta =   transform.transform.rotation.z
        

               
def call_back_goal(data):
   global waypointGoalx, waypointGoaly, goToWaypoint
   waypointGoalx = data.pose.position.x
   waypointGoaly = data.pose.position.y
   goToWaypoint = True


def setVelocity():
    global distanceTolerance , angleTolerance
    global velCommand, goToWaypoint
    distanceToWaypoint = getDistanceToWaypoint()
    headingError = getHeadingError()
    pub =rospy.Publisher("cmd_vel",Twist,queue_size=10)

    if (goToWaypoint == True and (abs(distanceToWaypoint) > distanceTolerance)):
        if (abs(headingError) > angleTolerance):
            velCommand.linear.x = 0.0
            velCommand.angular.z = K_a  * headingError
            pub.publish(velCommand)
        else:
            velCommand.linear.x = K_l * distanceToWaypoint
            velCommand.angular.z = 0.0;    
            pub.publish(velCommand)

    else:
        velCommand.linear.x = 0.0
        velCommand.angular.z = 0.0
        goToWaypoint == False
        pub.publish(velCommand)
    



    

while True:
     rospy.init_node("feedbackposego", anonymous= True)     
     rospy.Subscriber("/move_base_simple/goal",PoseStamped,call_back_goal)      
     rospy.Subscriber("/tf",TFMessage,call_back_current)
     setVelocity()
     rate = rospy.Rate(10)
     rate.sleep()

        
