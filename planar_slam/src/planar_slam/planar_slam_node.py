#!/usr/bin/env python

import rospy
import occamsam

class PlanarSLAMNode(object):


    def loop(self):
    
        rospy.init_node('planar_slam_node', anonymous=True)
    
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            rate.sleep()


if __name__ == '__main__':

    node = PlanarSLAMNode()
    node.loop()
