#!/usr/bin/env python
from ast import Sub
import rospy, math
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from sara_msgs.msg import BoundingBoxes3D
from geometry_msgs.msg import PolygonStamped, Point32

"""
    This Script is to convert sara_msg to ObstacleArrayMsg
    
"""
class ConvertSaraObs(object):
    """
    This is the constructor of the class FenceDetectorNode
    this is called ONCE in the beginning
    """
    def __init__(self):

        # PARAMETERS
        self.obstacle_msg   = ObstacleArrayMsg()


        """Publisher
        """
        self.pub_obstacle = rospy.Publisher('/obstacles', ObstacleArrayMsg, queue_size=10)

        """Subscriber
        """
        
        self.sub_darknet_bounding = rospy.Subscriber('/frame_to_boxes/bounding_boxes', BoundingBoxes3D, self.publish_obstacle_msg)


    """ Main function """
    def publish_obstacle_msg(self,data):
        if data.header.stamp == 0.0:
            rospy.loginfo("NULL ")
        self.obstacle_msg.header.stamp = data.header.stamp
        self.obstacle_msg.header.frame_id = "obstacle" # CHANGE HERE: odom/map 
        Obstacleid=0
        for i in data.boundingBoxes:        
            rospy.loginfo("{} | probability {} ".format(i.Class,i.probability))
            rospy.loginfo(" Depth= {}, Width= {}, Height = {}".format(i.Depth,i.Width,i.Height))
            self.obstacle_msg.obstacles.append(ObstacleMsg())
            self.obstacle_msg.obstacles[0].id = Obstacleid
            self.obstacle_msg.obstacles[0].polygon.points = [Point32()]
            self.obstacle_msg.obstacles[0].polygon.points[0].x = i.Center.x
            self.obstacle_msg.obstacles[0].polygon.points[0].y = i.Center.y
            self.obstacle_msg.obstacles[0].polygon.points[0].z = i.Center.z
            self.obstacle_msg.obstacles[0].radius = 1.0

            Obstacleid=Obstacleid+1

            self.pub_obstacle.publish(self.obstacle_msg)        


    def shutdown(self):
        rospy.loginfo("Stopping this topic...")
        #rospy.sleep(2)


if __name__ == '__main__':
    rospy.init_node('CalculatingDistance', anonymous=True)
    try:
        convert_msg = ConvertSaraObs()
        rospy.on_shutdown(convert_msg.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("{0} node finished.".format(rospy.get_name()))
