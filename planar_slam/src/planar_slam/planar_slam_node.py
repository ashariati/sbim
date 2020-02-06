#!/usr/bin/env python

import threading

import rospy
import message_filters
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, TransformStamped

import sbim_msgs.msg
from sbim_msgs.msg import PrincipalPlaneArray

import occamsam
from occamsam import variable, factor, factorgraph, optim


class PlanarSlamNode(object):

    def __init__(self):

        # parameters
        self.freq = rospy.get_param('frequency', 5)

        # subscribers
        pose_sub = message_filters.Subscriber('/scan_aggregator/keyframe', PoseStamped)
        compass_sub = message_filters.Subscriber('/point_cloud_compass_node/compass_transform', TransformStamped)
        plane_sub = message_filters.Subscriber('/plane_detector_node/principal_planes', PrincipalPlaneArray)
        sync = message_filters.ApproximateTimeSynchronizer(
                fs=[pose_sub, compass_sub, plane_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self.callback)

        # factorgraph
        self._fg = factorgraph.GaussianFactorGraph(free_point_window = 10*self.freq)
        self._lock = threading.Lock()

    def callback(self, pose, R, plane_array):
            print('bar')


    def loop(self):
    
        rospy.init_node('planar_slam_node', anonymous=True)
    
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            print('foo')
            rate.sleep()


if __name__ == '__main__':

    node = PlanarSlamNode()
    node.loop()
