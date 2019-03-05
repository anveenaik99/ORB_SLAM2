#!/usr/bin/env python
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError

# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False
bridge = CvBridge()

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub_left = rospy.Publisher("/camera/left/image_raw", Image, queue_size = 10)
        self.image_pub_right = rospy.Publisher("/camera/right/image_raw", Image, queue_size = 10)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber_left = rospy.Subscriber("/camera/left/compressed",
            CompressedImage, self.callback_left,  queue_size = 1)
        self.subscriber = rospy.Subscriber("/camera/right/compressed",
            CompressedImage, self.callback_right,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /camera/image/compressed"


    def callback_left(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        # #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        left = cv2.imdecode(np_arr, 0)
        #left = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        left_msg = bridge.cv2_to_imgmsg(left, "mono8")
        left_msg.header.stamp = rospy.Time.now()
        self.image_pub_left.publish(left_msg)
        
        #self.subscriber.unregister()
    def callback_right(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        # #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        right = cv2.imdecode(np_arr, 0)
        #right = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        right_msg = bridge.cv2_to_imgmsg(right, "mono8")
        right_msg.header.stamp = rospy.Time.now()
        self.image_pub_right.publish(right_msg)
        
        #self.subscriber.unregister()

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_feature', anonymous=True)
    ic = image_feature()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)