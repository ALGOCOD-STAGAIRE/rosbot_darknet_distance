#!/usr/bin/env python
import rospy
from math import pi , sqrt
from sensor_msgs.msg import Image 
from std_msgs.msg import String
import message_filters
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge, CvBridgeError
import sys

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
        Initialize local topics value with empty data val rosbotues = 0
        and store them
        """
        # PARAMETERS
        self.distance_rgb   = String()
        self.distance_depth = String()
        self.angle_value    = String()
        self.center_point   = Pose2D()
        self.bridge         = CvBridge()
        self.depth_image    = Image() 


        """Params
        """ 
        self.hfov     = rospy.get_param('~hfov'     , 91.2  )
        self.vfov     = rospy.get_param('~vfov'     , 65.5  )
        self.center_w = rospy.get_param('~center_w' , 240.0 )
        self.center_h = rospy.get_param('~center_h' , 320.0 )

        """Publisher
        """
        self.pub_distances      = rospy.Publisher('rosbot/darknet/distance'         , String    , queue_size=5)
        self.pub_depth_distance = rospy.Publisher('rosbot/darknet/depth/distance'   , String    , queue_size=5)
        self.pub_angle          = rospy.Publisher('rosbot/darknet/angle'            , String    , queue_size=5)
        self.pub_center_point   = rospy.Publisher('rosbot/darknet/center'           , Pose2D    , queue_size=5)  

        """Subscriber
        """
        self.sub_darknet_bounding = rospy.Subscriber('/darknet_ros/bounding_boxes'  , BoundingBoxes , self.calculating_distance)
        self.sub_depth_image      = rospy.Subscriber('/camera/depth/image_rect_raw' , Image         , self.depth_image_cb)

    """ Main function """
    def calculating_distance(self,data):
        # determined by the first position in the shape tuple, in this case 1.
        for i in data.bounding_boxes:
            rospy.loginfo("---------- For this Frame ---------- ")
            rospy.loginfo(" I recieved {} with probability {} ".format(i.Class,i.probability))
            rospy.loginfo(" xmin= {}, xmax= {}, ymin= {}, ymax = {}".format(i.xmin,i.xmax,i.ymin,i.ymax))

            self.center_point.x, self.center_point.y = self.center(i.xmax,i.xmin,i.ymax,i.ymin)
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(self.depth_image, self.depth_image.encoding)
            except CvBridgeError as e:
                print(e)
                return
            self.pix_center     = (int(self.center_point.x), int(self.center_point.y))

            #Publish Depth Distance
            self.distance_depth.data    = str(self.cv_image[self.pix_center[1], self.pix_center[0]]/10)

            #Distance RGB
            self.distance_rgb.data       = str(self.distance(i.xmax,i.xmin,i.ymax,i.ymin))

            #Angle RGB
            self.angle_value.data       = str(self.angle(i.xmax,i.xmin,i.ymax,i.ymin))

            rospy.logwarn("Depth at center({}, {})= {}cm".format(self.pix_center[0], self.pix_center[1],self.distance_depth.data))
            rospy.loginfo(" Calculating Distance --{} in cm".format(self.distance_rgb.data))
            rospy.loginfo(" Calculating Angle -- {} in degres".format(self.angle_value.data))

            # rospy.logwarn(anglei)
            self.pub_depth_distance.publish(self.distance_depth)
            self.pub_center_point.publish(self.center_point)
            self.pub_distances.publish(self.distance_rgb.data)
            self.pub_angle.publish(self.angle_value.data)
        



    """ Calculating Distance based on optic Formula 
    REF : https://github.com/paul-pias/Object-Detection-and-Distance-Measurement#how-the-distance-measurement-works 
    """
    def distance(self,xmax,xmin,ymax,ymin):
        width     = float(xmax)-float(xmin)
        height    = float(ymax)-float(ymin)
        distancei = (2.0*pi* 180.0) / (float(width) + float(height)  * 360.0) * 1000.0 + 3.0
        distancei = round(distancei * 2.54)
        return distancei

    """ Calculating Angle - 
    Let's assume center of the image is the camera center then as the camera characteristics for resolution is prior known (D400 specs datasheet). 
    You can easily get the horizontal angle and vertical angle for any self.pix_centerel by dividing it the self.pix_centerel distance divided 
    by length or width of image multiplied by corresponding horizontal FOV or Vertical FOV.
    REF : https://github.com/IntelRealSense/librealsense/issues/5553#issuecomment-569441474 
    """
    def angle(self,xmax,xmin,ymax,ymin):
        XMAX    = float(xmax)
        XMIN    = float(xmin)
        YMAX    = float(ymax)
        YMIN    = float(ymin)
        XCENTER = (XMAX+XMIN)/2
        YCENTER = (YMAX+YMIN)/2
        H_Angle = ((XCENTER - self.center_w)/self.center_w)*(self.hfov/2)
        V_Angle = ((YCENTER - self.center_h)/self.center_h)*(self.vfov/2)
        rospy.logwarn(H_Angle)
        rospy.logwarn(V_Angle)
        angle_eu = sqrt(H_Angle**2 + V_Angle**2)
        return angle_eu
    
    def center(self,xmax,xmin,ymax,ymin):
        xcenter=(xmax+xmin)/2
        ycenter=(ymax+ymin)/2
        return xcenter,ycenter

    def depth_image_cb(self,image_raw):
        self.depth_image = image_raw


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
