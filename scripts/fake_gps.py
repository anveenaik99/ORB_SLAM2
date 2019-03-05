#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('/mavros/fake_gps/mocap/pose',PoseStamped, queue_size=10)
    rospy.init_node('gps', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "camera_pose"
        pose.pose.position.x = 500
        pose.pose.position.y = 5
        pose.pose.position.z = 1000

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
