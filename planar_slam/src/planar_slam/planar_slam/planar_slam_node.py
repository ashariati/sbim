#!/usr/bin/env python3

import threading
import numpy as np
import cvxpy as cp

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import euler_from_quaternion, quaternion_from_euler, translation_matrix, quaternion_matrix, translation_from_matrix

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from sbim_msgs.msg import PrincipalPlaneArray, PrincipalPlane, CorrespondenceMap, Trajectory

import occamsam
from occamsam import variable, factor, factorgraph, optim

def transformMsgToMatrix(msg):
    q = np.array([msg.rotation.x, 
        msg.rotation.y, 
        msg.rotation.z, 
        msg.rotation.w])
    t = np.array([msg.translation.x, msg.translation.y, msg.translation.z])

    T = translation_matrix(t)
    R = quaternion_matrix(q)
    G = np.dot(T,  R)

    return G

def poseMsgToMatrix(msg):
    q = np.array([msg.orientation.x, 
        msg.orientation.y, 
        msg.orientation.z, 
        msg.orientation.w])
    t = np.array([msg.position.x, msg.position.y, msg.position.z])

    T = translation_matrix(t)
    R = quaternion_matrix(q)
    G = np.dot(T,  R)

    return G

class PlanarSlamNode(Node):

    def __init__(self):
        super().__init__('planar_slam_node')

        # parameters
        self.freq = self.declare_parameter('frequency', 5).value
        self.pose_window = self.declare_parameter('pose_window', 10).value
        self.sigma_t = self.declare_parameter('odometry_noise', 0.01).value
        self.sigma_d = self.declare_parameter('observation_noise', 0.03).value
        assoc_range = self.declare_parameter('association_range', 0.5).value

        # subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/keyframe', self.pose_callback, qos_profile_sensor_data)
        self.compass_sub = self.create_subscription(
            TransformStamped, '/compass_transform', self.compass_callback, qos_profile_sensor_data)
        self.plane_sub = self.create_subscription(
            PrincipalPlaneArray, '/planes', self.plane_callback, qos_profile_sensor_data)

        # publishers
        self._traj_pub = self.create_publisher(Trajectory, '/planar_slam_node/trajectory', 10)
        self._layout_pub = self.create_publisher(PrincipalPlaneArray, '/planar_slam_node/layout_planes', 10)
        self._cmap_pub = self.create_publisher(CorrespondenceMap, '/planar_slam_node/correspondence_map', 10)

        # lock
        self._lock = threading.Lock()

        # initialize members for optimization
        self._G_ws = np.eye(4)
        self._G_cs = np.eye(4)
        self._point_var = variable.PointVariable(3)
        self._fg = factorgraph.GaussianFactorGraph(free_point_window=self.pose_window)
        self._optimizer = optim.Occam(self._fg, assoc_range=assoc_range)
        self._is_init = False

        # intialize bookkeepers
        self._label_orientation_map = {}
        self._var_id_map = {}
        self._point_stamp_map = {}

        # create timer for the loop
        self.timer = self.create_timer(1.0 / self.freq, self.loop)

        # message buffer
        self.msg_buffer = []

    def pose_callback(self, msg):
        self.msg_buffer.append(('pose', msg))

    def compass_callback(self, msg):
        self.msg_buffer.append(('compass', msg))

    def plane_callback(self, msg):
        self.msg_buffer.append(('plane', msg))

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
            t0 = translation_from_matrix(G_ws)
            prior_factor = factor.PriorFactor(self._point_var, np.eye(3), t0, 1e-6*np.ones(3))
            self._fg.add_factor(prior_factor)

            self._point_stamp_map[self._point_var] = pose_ws.header.stamp

            self._is_init = True
            return

        # frame-to-frame translation 
        G_ss = np.dot(np.linalg.inv(self._G_ws), G_ws)
        t_s = translation_from_matrix(G_ss)

        # generate odometry factor
        point_var = variable.PointVariable(3)
        odom_factor = factor.OdometryFactor(self._point_var, point_var, 
                self._G_cs[:3, :3], t_s, self.sigma_t*np.ones(3))

        # generate observation factors
        obsv_factors = []
        for plane in c_plane_array.planes:

            plane_var = variable.LandmarkVariable(1, plane.label.data, plane.intensity.data)
            v_c = np.array([[plane.plane.coef[0], plane.plane.coef[1], plane.plane.coef[2]]])
            d_c = -plane.plane.coef[3]
            plane_var.facing = -1 if d_c > 0 else 1
            plane_factor = factor.ObservationFactor(point_var, plane_var, v_c, d_c, np.array([self.sigma_d]))
            obsv_factors.append(plane_factor)

            # bookkeeping
            self._var_id_map[plane_var] = plane.id.data
            if plane.label.data not in self._label_orientation_map:
                self._label_orientation_map[plane.label.data] = v_c

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

        # save point timestamp
        self._point_stamp_map[point_var] = pose_ws.header.stamp

    def loop(self):
        if not self.msg_buffer:
            return

        pose_ws = None
        transform_cs = None
        c_plane_array = None

        while self.msg_buffer:
            msg_type, msg = self.msg_buffer.pop(0)
            if msg_type == 'pose':
                pose_ws = msg
            elif msg_type == 'compass':
                transform_cs = msg
            elif msg_type == 'plane':
                c_plane_array = msg

        if pose_ws and transform_cs and c_plane_array:
            self.callback(pose_ws, transform_cs, c_plane_array)
            self.publish_results()

    def publish_results(self):
        # run optimization
        self._lock.acquire(True)
        try:
            self._optimizer.optimize()
            self._optimizer.update()
        except cp.error.SolverError as e:
            self.get_logger().warn("Bad solve in occamsam : %s" % e.message)
        finally:
            free_points = self._fg.free_points
            layout_planes = self._fg.landmarks
            plane_parent_map = self._fg.correspondence_map.root_map()
            plane_group_map = self._fg.correspondence_map.set_map()
            self._lock.release()

        now = self.get_clock().now().to_msg()

        # publish pose array
        pose_array = Trajectory()
        pose_array.header.frame_id = 'building'
        pose_array.header.stamp = now
        for point in free_points:
            pose = PoseStamped()
            pose.header.frame_id = 'building'
            pose.header.stamp = self._point_stamp_map[point]
            pose.pose.position.x = point.position[0]
            pose.pose.position.y = point.position[1]
            pose.pose.position.z = point.position[2]
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            pose_array.poses.append(pose)
        self._traj_pub.publish(pose_array)

        # publish planes
        b_plane_array = PrincipalPlaneArray()
        b_plane_array.header.frame_id = 'building'
        b_plane_array.header.stamp = now
        for lp in layout_planes:
            plane = PrincipalPlane()
            plane.plane.coef[:3] = self._label_orientation_map[lp.class_label][0]
            plane.plane.coef[3] = -lp.position
            plane.facing.data = lp.facing
            plane.intensity.data = np.sum([k.mass for k in plane_group_map[lp]])
            plane.label.data = lp.class_label
            plane.id.data = self._var_id_map[plane_parent_map[lp]]
            b_plane_array.planes.append(plane)
        self._layout_pub.publish(b_plane_array)

        # publish correspondence map
        correspondence_map = CorrespondenceMap()
        correspondence_map.header.frame_id = 'building'
        correspondence_map.header.stamp = now
        for (k, v) in list(plane_parent_map.items()):
            k_str = String()
            v_str = String()
            k_str.data = self._var_id_map[k]
            v_str.data = self._var_id_map[v]
            correspondence_map.plane_ids.append(k_str)
            correspondence_map.parent_ids.append(v_str)
        self._cmap_pub.publish(correspondence_map)


def main(args=None):
    rclpy.init(args=args)
    node = PlanarSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
