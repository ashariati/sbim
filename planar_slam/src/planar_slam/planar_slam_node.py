#!/usr/bin/env python

import threading
import numpy as np

import rospy
import message_filters
import tf
from tf import transformations
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, TransformStamped

import sbim_msgs.msg
from sbim_msgs.msg import PrincipalPlaneArray

import occamsam
from occamsam import variable, factor, factorgraph, optim

def transformMsgToMatrix(msg):
    q = np.array([msg.rotation.x, 
        msg.rotation.y, 
        msg.rotation.z, 
        msg.rotation.w])
    t = np.array([msg.translation.x, msg.translation.y, msg.translation.z])

    T = transformations.translation_matrix(t)
    R = transformations.quaternion_matrix(q)
    G = np.dot(T,  R)

    return G

def poseMsgToMatrix(msg):
    q = np.array([msg.orientation.x, 
        msg.orientation.y, 
        msg.orientation.z, 
        msg.orientation.w])
    t = np.array([msg.position.x, msg.position.y, msg.position.z])

    T = transformations.translation_matrix(t)
    R = transformations.quaternion_matrix(q)
    G = np.dot(T,  R)

    return G

class PlanarSlamNode(object):

    def __init__(self):

        # parameters
        self.freq = rospy.get_param('frequency', 5)
        self.sigma_t = rospy.get_param('odometry_noise', 0.01)
        self.sigma_d = rospy.get_param('observation_noise', 0.03)

        # subscribers
        pose_sub = message_filters.Subscriber('/scan_aggregator/keyframe', PoseStamped)
        compass_sub = message_filters.Subscriber('/point_cloud_compass_node/compass_transform', TransformStamped)
        plane_sub = message_filters.Subscriber('/plane_detector_node/principal_planes', PrincipalPlaneArray)
        sync = message_filters.ApproximateTimeSynchronizer(
                fs=[pose_sub, compass_sub, plane_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self.callback)

        # lock
        self._lock = threading.Lock()

        # initialize members 
        self._G_ws = np.eye(4)
        self._G_cs = np.eye(4)
        self._point_var = variable.PointVariable(3)
        self._fg = factorgraph.GaussianFactorGraph(free_point_window=10*self.freq)
        self._optimizer = optim.Occam(self._fg)
        self._is_init = False


    def callback(self, pose_ws, transform_cs, c_plane_array):

        # convert messages to numpy
        G_ws = poseMsgToMatrix(pose_ws.pose)
        G_cs = transformMsgToMatrix(transform_cs.transform)

        # initialization
        if not self._is_init:

            # save for next iteration
            self._G_ws = G_ws
            self._G_cs = G_cs

            # anchor starting point
            t0 = transformations.translation_from_matrix(G_ws)
            prior_factor = factor.PriorFactor(self._point_var, np.eye(3), t0, 1e-6*np.ones(3))
            self._fg.add_factor(prior_factor)

            self._is_init = True
            return

        # frame-to-frame translation 
        G_ss = np.dot(np.linalg.inv(self._G_ws), G_ws)
        t_s = transformations.translation_from_matrix(G_ss)

        # generate odometry factor
        point_var = variable.PointVariable(3)
        odom_factor = factor.OdometryFactor(self._point_var, point_var, 
                self._G_cs[:3, :3], t_s, self.sigma_t*np.eye(3))

        # generate observation factors
        obsv_factors = []
        for plane in c_plane_array.planes:

            plane_var = variable.LandmarkVariable(1, plane.label.data, plane.intensity.data)
            v_c = np.array([[plane.plane.coef[0], plane.plane.coef[1], plane.plane.coef[2]]])
            d_c = plane.plane.coef[3]
            plane_factor = factor.ObservationFactor(point_var, plane_var, v_c, d_c, sigma=self.sigma_d)
            obsv_factors.append(plane_factor)

        # insert into factorgraph
        factor_list = [odom_factor] + obsv_factors
        self._lock.acquire(True)
        for f in factor_list:
            self._fg.add_factor(f)
        self._lock.release()

        # save for next iteration
        self._G_ws = G_ws
        self._G_cs = G_cs
        self._point_var = point_var


    def loop(self):
    
        rospy.init_node('planar_slam_node', anonymous=True)
    
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():

            rate.sleep()

            if not self._is_init:
                continue

            # run optimizaiton
            self._lock.acquire(True)
            self._optimizer.optimize()
            self._optimizer.update()
            print('Points = {:}, Landmarks = {:}'.format(len(self._fg.free_points), len(self._fg.landmarks)))
            self._lock.release()


if __name__ == '__main__':

    node = PlanarSlamNode()
    node.loop()


