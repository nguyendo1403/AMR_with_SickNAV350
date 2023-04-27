#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

# Khai báo biến toàn cục
odom = Odometry()

def tf_callback(data):
    global odom
    for transform in data.transforms:
        if transform.header.frame_id == 'map' and transform.child_frame_id == 'nav350':
            odom.header = transform.child_frame_id
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = transform.transform.translation.x
            odom.pose.pose.position.y = transform.transform.translation.y
            odom.pose.pose.position.z = transform.transform.translation.z
            odom.pose.pose.orientation.x = transform.transform.rotation.x
            odom.pose.pose.orientation.y = transform.transform.rotation.y
            odom.pose.pose.orientation.z = transform.transform.rotation.z
            odom.pose.pose.orientation.w = transform.transform.rotation.w
     
            odom.twist.twist.angular.z = transform.transform.rotation.z


def main():
    # Khai báo node
    rospy.init_node('tf_to_odom_node')

    # Tạo subscriber nhận message từ chủ đề /tf
    rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, tf_callback)

    # Tạo publisher để gửi message với chủ đề /odom
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)

    # Khai báo tần số cập nhật
    rate = rospy.Rate(10) # 10 Hz

    # Vòng lặp chính để publish message
    while not rospy.is_shutdown():
        # Điền thông tin cho twist message
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0

     #  odom.twist.twist.angular.z = data.transform.rotation.z

        # Publish message
        odom_pub.publish(odom)

        # Delay để đảm bảo tần số publish
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass