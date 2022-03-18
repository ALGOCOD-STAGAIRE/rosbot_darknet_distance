#!/usr/bin/env python

from operator import truediv
from re import X
from turtle import distance
import rospy
from math import pi , sqrt
from sensor_msgs.msg import Image , CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
import os,sys

"""
    This Script is to calculate the Real Distance from Object
    To the robot   
    
"""
class CalculatingDistance(object):
    """
    This is the constructor of the class FenceDetectorNode
    this is called ONCE in the beginning
    """
    def __init__(self):
        #rospy.init_node('FenceDetectorNode', anonymous=True)

        """
        Initialize local topics value with empty data valrosbotues = 0
        and store them
        """

        self.center_point       = Pose2D() 
        self.depth_image        = Image() 
        self.bridge             = CvBridge()


        """Publisher
        """
        self.pub_distances      = rospy.Publisher('rosbot_darknet_distance/distance', String    , queue_size=5)
    
        """Subscriber
        """
        self.sub_depth_image    = rospy.Subscriber('/camera/depth/image_rect_raw'   ,Image  , self.depth_image_cb)
        self.sub_center_point   = rospy.Subscriber('rosbot_darknet_distance/center' ,Pose2D , self.center_point_cb)

    def depth_image_cb(self,image_raw):
        # self.depth_image = image_raw
        try:
            cv_image       = self.bridge.imgmsg_to_cv2(image_raw, image_raw.encoding)
            # pix            = (image_raw.width/2, image_raw.height/2)
            pix            = (int(self.center_point.x), int(self.center_point.y))
            distance_in_cm = cv_image[pix[1], pix[0]]/10
            rospy.logwarn()
            # sys.stdout.write(' Depth at center(%d, %d): %f(cm)\r' % (pix[0], pix[1], cv_image[pix[1], pix[0]]/10))
            #sys.stdout.flush()
        except CvBridgeError as e:
            print(e)
            return

    def center_point_cb(self, pc):
        self.center_point = pc
        # rospy.logwarn("Center Point X {}".format(self.center_point.x))
        # rospy.logwarn("Center Point Y {}".format(self.center_point.y))

    def shutdown(self):
        rospy.loginfo("Stopping this topic...")
        #rospy.sleep(2)


if __name__ == '__main__':
    rospy.init_node('CalculatingDistance', anonymous=True)
    try:
        values_calcu = CalculatingDistance()
        rospy.on_shutdown(values_calcu.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("{0} node finished.".format(rospy.get_name()))
