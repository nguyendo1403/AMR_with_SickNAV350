#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage

from geometry_msgs.msg import PoseStamped

from pymodbus.client.sync import ModbusSerialClient
import math

PI = 3.141592654
distanceTolerance = 0.1
angleTolerance = 0.5
goToWaypoint = False
currentx = 0.0
currenty = 0.0
waypointGoaly = 0.0
waypointGoalx = 0.0
currenttheta = 0.0
th_1 = False
th_2 = False
th_3 = False
th_4 = False
values_trai = 0
values_phai = 0
client = ModbusSerialClient(method="rtu", port = "/dev/ttyUSB0", baudrate = 115200,stopbits = 1, parity = "N",timeout = 1)
client.connect()
jog_accel1 = client.write_registers(address=0x2E,values=50,unit=0x01)
jog_decel1 = client.write_registers(address=0X2F,values=50,unit=0x01)
jog_velocity1 = client.write_registers(address=0x30,values=values_phai,unit=0x01)

jog_accel2 = client.write_registers(address=0x2E,values=50,unit=0x02)
jog_decel2 = client.write_registers(address=0X2F,values=50,unit=0x02)
jog_velocity2 = client.write_registers(address=0x30,values=values_trai,unit=0x02)
start_jog2 = client.write_registers(address=0X7C,values=0X96,unit=0x02)
start_jog1 = client.write_registers(address=0X7C,values=0X96,unit=0x01)


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
#     pub = rospy.Publisher("pose_current",Float32,queue_size=10)
     
     global currentx, currenty , currenttheta
    
   
     for transform in data.transforms:
          if transform.header.frame_id == 'map' and transform.child_frame_id == 'nav350':
        
            # print("Transform between", transform.header.frame_id, "and", transform.child_frame_id)
            currentx = transform.transform.translation.x
            currenty = transform.transform.translation.y 
            currenttheta =   transform.transform.rotation.z
          #   print ("Current (x,y) = ",currentx,currenty)
           
    


# def call_back_current():            # velCommand.angular.z = K_a  * headingError xu ly toc do quay

    
    
   
#     for transform in data.transforms:
#           if transform.header.frame_id == 'map' and transform.child_frame_id == 'nav350':
        
#             # print("Transform between", transform.header.frame_id, "and", transform.child_frame_id)
#             print("translation_x:", transform.transform.translation.x)
#             print("translation_y:",  transform.transform.translation.y )
#             print("rotation_z:",  transform.transform.rotation.z)
               
def call_back_goal(data):
   global waypointGoalx, waypointGoaly, goToWaypoint
    
   waypointGoalx = data.pose.position.x
   waypointGoaly = data.pose.position.y
   goToWaypoint = True


def setVelocity():
    global distanceTolerance , angleTolerance, th_1,th_2,th_3,th_4, values_phai, values_trai
    distanceToWaypoint = getDistanceToWaypoint()
    headingError = getHeadingError()
    if goToWaypoint == True and (abs(distanceToWaypoint) > distanceTolerance):
        if (abs(headingError) > angleTolerance) :
            #xe re phai
            if headingError > 0 and th_1 == False:
                    print ("xe re phai")
                    jog_velocity1 = client.write_registers(address=0x30,values=4800,unit=0x01)
                    jog_velocity2 = client.write_registers(address=0x30,values=0,unit=0x02) 
                    th_1 = True
                    th_2 = False
                    th_3 = False
                    th_4 = False


            # xe re trai    
            elif headingError < 0 and th_2 == False:
                print("xe re trai")
                jog_velocity1 = client.write_registers(address=0x30,values=0,unit=0x01)
                jog_velocity2 = client.write_registers(address=0x30,values=2**16 - abs(-4800),unit=0x02) 
                th_1 = False
                th_2 = True
                th_3 = False
                th_4 = False
                
            
            
            # velCommand.linear.x = 0.0
            # velCommand.angular.z = K_a  * headingError xu ly toc do quay
        elif (abs(headingError) <= angleTolerance) and th_3 == False :
            # velCommand.linear.x = K_l * distanceToWaypoint xu ly van toc quay
            # velCommand.angular.z = 0.0;   
            print("xe di thang")
            jog_velocity1 = client.write_registers(address=0x30,values=4800,unit=0x01)
            jog_velocity2 = client.write_registers(address=0x30,values=2**16 - abs(-4800),unit=0x02) 
            th_1 = False
            th_2 = False
            th_3 = True
            th_4 = False
 
    else :
        if th_4 == False:
          print("Goal has been reached!")
          jog_velocity1 = client.write_registers(address=0x30,values=0,unit=0x01)
          jog_velocity2 = client.write_registers(address=0x30,values=0,unit=0x02) 
          goToWaypoint == False
          th_1 = False
          th_2 = False
          th_3 = False
          th_4 = True
    
# def updateWaypoint(data1):
# #     global goToWaypoint
# #     for pose in data1.poses:

#         waypointGoal.x = data1.pose

     #    waypointGoal.y = datdef talk_pose_current():
#      rospy.init_node("feedbackposecurrent",anonymous= True)
     
#     #  rate = rospy.Rate(1)
#      rospy.Subscriber("/tf",TFMessage,call_back_current)
#      rospy.spin()a1.pose.position.y
               
               
    
# def subscriberr():
#     rospy.init_node("take_adata",anonymous= True)
#     rospy.Subscriber("/tf",TFMessage,callback)
#     rospy.spin()

# def talk_pose_current():
#      rospy.init_node("feedbackposecurrent",anonymous= True)
     
#     #  rate = rospy.Rate(1)
#      rospy.Subscriber("/tf",TFMessage,call_back_current)
#      rospy.spin()
     
#      # pub = rospy.Publisher("feedback_pose",st,queue_size=10)

# def talk_pose_goal():
#     rospy.init_node("feedbackposego", anonymous= True)
#     rospy.Subscriber("/move_base_simple/goal",PoseStamped,call_back_goal)
#     rospy.spin()



# if __name__ == '__main__':
#     try:
       
     #     talk_pose_goal()rospy.init_node("feedbackposego", anonymous= True)
     #     talk_pose_current()
     # rospy.init_node("feedbackposego", anonymous= True)     
     # rospy.Subscriber("/move_base_simple/goal",PoseStamped,call_back_goal)
          
     # rospy.Subscriber("/tf",TFMessage,call_back_current)
     # rospy.spin()

while True:
     rospy.init_node("feedbackposego", anonymous= True)     
     rospy.Subscriber("/move_base_simple/goal",PoseStamped,call_back_goal)
          
     rospy.Subscriber("/tf",TFMessage,call_back_current)
     # setVelocity()
     # print ("Current (x,y) = ",currentx,",",currenty)
     # print ("Waypoint (x,y) = ",waypointGoalx,",",waypointGoaly)
     # print ("Distance to Waypoint = ",getDistanceToWaypoint(),"m")
     setVelocity()
     # print(getHeadingError())
     
     
         
     
#     rospy.spin()

#     except rospy.ROSInterruptException:
#          pass
# rospy.init_node("feedbackposego", anonymous= True)

    
#     rospy.Subscriber("/move_base_simple/goal", PoseStamped,updateWaypoint)
#     rospy.spin()
     rate = rospy.Rate(10)
    
   
     rate.sleep()
         
        
     #    rate.sleep()
     #    print ("Current (x,y) = ",current.x,",",current.y)
     #    print ("Waypoint (x,y) = ",waypointGoal.x,waypointGoal.y)
      
        
