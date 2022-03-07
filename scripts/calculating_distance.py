#!/usr/bin/env python

import rospy
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
        """Params
        """ 
        """Publisher
        """
        self.pub_distances      = rospy.Publisher('rosbot_darknet_distance/distance', String, queue_size=5)
        
        """Subscriber
        """
        self.sub_darknet_bounding = rospy.Subscriber('/darknet_ros/bounding_boxes' ,BoundingBoxes, self.calculating_distance)


    def calculating_distance(self,data):
        # determined by the first position in the shape tuple, in this case 1.
        for i in data.bounding_boxes:
            rospy.loginfo("---------- For this Frame ---------- ")
            rospy.loginfo(" I recieved %s with probability %s ",i.Class,i.probability)
            rospy.loginfo(" xmin= %s, xmax= %s, ymin= %s, ymax = %s",i.xmin,i.xmax,i.ymin,i.ymax)
            distancei = (2* 3.14* 180) / (i.xmax + i.ymax * 360) * 1000 + 3
            self.distance_value.data = str(round(distancei * 2.54))
            rospy.loginfo(" Calculating Distance -- %s in inches to %s cm \n",round(distancei),self.distance_value.data)
            self.pub_distances.publish(self.distance_value.data)

            
        

        

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
