cd import rospy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D



pose=Pose2D()


def callback(data):
    global pose
    pub = rospy.Publisher("ODOM_TOPIC",Int32,queue_size=10)
    for transform in data.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'nav350':
               pose.x = transform.transform.translation.x
               pose.y=transform.transform.translation.y
               pose.theta= transform.transform.rotation.z
               print("translation_x:", pose.x)
               print("translation_y:",  pose.y )
               print("rotation_z:",  pose.theta)
               pub.publish(pose)

def talk():
     rospy.init_node("feedbackpose",anonymous= True)
     rate = rospy.Rate(10)
     rospy.Subscriber("/tf",TFMessage,callback)
     rospy.spin()
     pub = rospy.Publisher("feedback_pose",Int32,queue_size=10)


if __name__ == '__main__':
    try:
          talk()
    except rospy.ROSInterruptException:
         pass
