#!/usr/bin/env python

from turtle import distance
import rospy
from math import pi , sqrt
from sensor_msgs.msg import Image as SensorImage
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes

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
        # PARAMETERS
        self.distance_value      = String()
        self.angle_value      = String() 
        self.center_point      = String() 
        """Params
        """ 
        """Publisher
        """
        self.pub_distances      = rospy.Publisher('rosbot_darknet_distance/distance', String, queue_size=5)
        self.pub_angle      = rospy.Publisher('rosbot_darknet_distance/angle', String, queue_size=5)
        self.pub_center_point      = rospy.Publisher('rosbot_darknet_distance/center', String, queue_size=5)        
        """Subscriber
        """
        self.sub_darknet_bounding = rospy.Subscriber('/darknet_ros/bounding_boxes' ,BoundingBoxes, self.calculating_distance)
        

    """ Main function """
    def calculating_distance(self,data):
        # determined by the first position in the shape tuple, in this case 1.
        for i in data.bounding_boxes:
            rospy.loginfo("---------- For this Frame ---------- ")
            rospy.loginfo(" I recieved %s with probability %s ",i.Class,i.probability)
            rospy.loginfo(" xmin= %s, xmax= %s, ymin= %s, ymax = %s",i.xmin,i.xmax,i.ymin,i.ymax)
            center = self.distance(i.xmax,i.xmin,i.ymax,i.ymin)
            distancei = self.distance(i.xmax,i.xmin,i.ymax,i.ymin)
            anglei = self.angle(i.xmax,i.xmin,i.ymax,i.ymin)
            rospy.logwarn(anglei)
            self.distance_value.data = str(distancei)
            self.angle_value.data = str(anglei)
            rospy.loginfo(" Calculating Distance -- %s in cm",self.distance_value.data)
            rospy.loginfo(" Calculating Angle -- %s in degres \n",self.angle_value.data)
            self.pub_distances.publish(self.distance_value.data)
            self.pub_angle.publish(self.angle_value.data)
            self.pub_angle.publish(self.center.data)

            
        
    """ Calculating Distance based on optic Formula 
    REF : https://github.com/paul-pias/Object-Detection-and-Distance-Measurement#how-the-distance-measurement-works 
    """
    def distance(self,xmax,xmin,ymax,ymin):
        width = xmax-xmin
        height= ymax-ymin
        distancei = (2*pi* 180) / (float(width) + float(height)  * 360) * 1000 + 3
        distancei = round(distancei * 2.54)
        return distancei

    """ Calculating Angle - 
    Let's assume center of the image is the camera center then as the camera characteristics for resolution is prior known (D400 specs datasheet). 
    You can easily get the horizontal angle and vertical angle for any pixel by dividing it the pixel distance divided 
    by length or width of image multiplied by corresponding horizontal FOV or Vertical FOV.
    REF : https://github.com/IntelRealSense/librealsense/issues/5553#issuecomment-569441474 
    """
    def angle(self,xmax,xmin,ymax,ymin):
        # CAMERA_FOCAL_LENGTH=1.93 #focal length in mm | Horizontal Field of View 91.2* | Vertical Field of View 65.5* | Diagonal Field of View 100.6*
        HFOV = 91.2
        VFOV = 65.5
        CENTER_W = 240.0
        CENTER_H =320.0
        XMAX=float(xmax)
        XMIN=float(xmin)
        YMAX=float(ymax)
        YMIN=float(ymin)
        XCENTER=(XMAX+XMIN)/2
        YCENTER=(YMAX+YMIN)/2
        H_Angle = ((XCENTER - CENTER_W)/CENTER_W)*(HFOV/2) 
        V_Angle = ((YCENTER - CENTER_H)/CENTER_H)*(VFOV/2) 
        rospy.logwarn(H_Angle)
        rospy.logwarn(V_Angle)
        angle_eu = sqrt(H_Angle**2 + V_Angle**2)
        return angle_eu
    
    def center(self,xmax,xmin,ymax,ymin):
        XMAX=float(xmax)
        XMIN=float(xmin)
        YMAX=float(ymax)
        YMIN=float(ymin)
        XCENTER=(XMAX+XMIN)/2
        YCENTER=(YMAX+YMIN)/2
        return XCENTER,YCENTER

    
    def unsubscribe(self):
        # use the saved subscriber object to unregister the subscriber
        pass

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
