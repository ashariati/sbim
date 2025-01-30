#!/usr/bin/env python3

import numpy as np
import threading
import copy
import collections
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from message_filters import ApproximateTimeSynchronizer, Subscriber

from std_msgs.msg import UInt32
from geometry_msgs.msg import Point
from sbim_msgs.msg import (Trajectory, PrincipalPlaneArray, CorrespondenceMap, LayoutSegmentArray,
                           FloorplanArray, Floorplan, SceneNode, SceneEdge, UInt32Pair)

import bimpy
from bimpy import models, estimators


class FloorplanEstimationNode(Node):

    def __init__(self):
        super().__init__('floorplan_estimation_node')

        # parameters
        self.freq = self.declare_parameter('frequency', 1).value
        self.speculation_horizon = self.declare_parameter('speculation_horizon', 0).value
        self.min_free_ratio = self.declare_parameter('min_free_ratio', 0.02).value
        self.boundary_coverage_threshold = self.declare_parameter('boundary_coverage_threshold', 0.5).value
        self.boundary_height_threshold = self.declare_parameter('boundary_height_threshold', 2.5).value
        self.width_max = self.declare_parameter('width_max', 300).value
        self.length_max = self.declare_parameter('length_max', 300).value

        # subscribers
        self.trajectory_sub = Subscriber(self, Trajectory, '/planar_slam_node/trajectory', qos_profile_sensor_data)
        self.layout_plane_sub = Subscriber(self, PrincipalPlaneArray, '/planar_slam_node/layout_planes', qos_profile_sensor_data)
        self.correspondence_sub = Subscriber(self, CorrespondenceMap, '/planar_slam_node/correspondence_map', qos_profile_sensor_data)
        
        self.slam_sync = ApproximateTimeSynchronizer(
            [self.trajectory_sub, self.layout_plane_sub, self.correspondence_sub], queue_size=100, slop=0.05)
        self.slam_sync.registerCallback(self.slam_callback)

        self.segment_sub = self.create_subscription(
            LayoutSegmentArray, '/layout_extractor_node/layout_segments', 
            self.scene_parsing_callback, qos_profile_sensor_data)
        
        # publishers
        self._floorplan_pub = self.create_publisher(FloorplanArray, '/floorplan_estimation_node/floorplan', 10)

        self._lock = threading.Lock()
        self._pose_at_time = {}
        self._plane_model = {}
        self._plane_label = {}
        self._upward_facing = []
        self._segment_list = []
        self._boundary_list = []
        self._plane_evidence = collections.defaultdict(list)

    def slam_callback(self, trajectory, layout_planes, correspondence_map):
        parent_plane = {plane_id.data: parent_id.data for plane_id, parent_id in
                        zip(correspondence_map.plane_ids, correspondence_map.parent_ids)}

        with self._lock:
            segment_list = copy.copy(self._segment_list)
            plane_model = copy.copy(self._plane_model)
            plane_label = copy.copy(self._plane_label)
            pose_at_time = copy.copy(self._pose_at_time)

        upward_facing = []
        for plane in layout_planes.planes:
            coef = np.array([plane.plane.coef[0], plane.plane.coef[1], plane.plane.coef[2], plane.plane.coef[3]])
            plane_model[plane.id.data] = models.Plane(coef)
            plane_label[plane.id.data] = plane.label.data
            if plane.label.data == "0" and plane.facing.data == 1:
                upward_facing.append(plane.id.data)

        for pose in trajectory.poses:
            time = (pose.header.stamp.sec, pose.header.stamp.nanosec)
            pose_at_time[time] = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

        # process segments
        boundary_list = []
        plane_evidence = collections.defaultdict(list)
        for segment in segment_list:
            if segment.plane_id.data not in parent_plane:
                continue
            plane = plane_model[parent_plane[segment.plane_id.data]]

            time = (segment.header.stamp.sec, segment.header.stamp.nanosec)
            pose = pose_at_time[time]

            # register to position in world
            vertices = []
            for u in segment.vertices:
                v = np.array([u.x, u.y, u.z])
                vertices.append(pose + v)

            # project to corrected plane position
            projected = []
            for v in vertices:
                projected.append(plane.project(v))

            edges = list(zip(range(len(projected)), range(1, len(projected))))
            edges.append((len(projected) - 1, 0))

            # add to evidence list if on z-plane or boundary list if otherwise
            if segment.label.data == "0":
                plane_evidence[parent_plane[segment.plane_id.data]].append(
                    models.ConvexPolygon2D(np.array(projected), set(edges)))
            else:
                boundary_list.append(models.Polygon3D(np.array(projected), set(edges), plane))

        with self._lock:
            self._plane_model = plane_model
            self._plane_label = plane_label
            self._upward_facing = upward_facing
            self._boundary_list = boundary_list
            self._plane_evidence = plane_evidence
            self._pose_at_time = pose_at_time

    def scene_parsing_callback(self, layout_segments):
        with self._lock:
            for segment in layout_segments.layout_segments:
                self._segment_list.append(segment)

    def loop(self):
        rate = self.create_rate(self.freq)
        while rclpy.ok():
            rate.sleep()

            with self._lock:
                plane_model = copy.copy(self._plane_model)
                plane_label = copy.copy(self._plane_label)
                upward_facing = copy.copy(self._upward_facing)
                boundary_list = copy.copy(self._boundary_list)
                plane_evidence = copy.copy(self._plane_evidence)
                pose_at_time = copy.copy(self._pose_at_time)

            evidence = []
            for plane_id in plane_evidence:
                evidence.extend(plane_evidence[plane_id])
            for position in pose_at_time.values():
                evidence.append(models.Point(position))

            floorplan_array_msg = FloorplanArray()
            floorplan_array_msg.header.frame_id = 'building'
            floorplan_array_msg.header.stamp = self.get_clock().now().to_msg()
            for z_id in upward_facing:
                # initialize cell complex at height
                z_ref = -plane_model[z_id].coefficients[3]
                cell_complex = models.CellComplex2D(z_ref, self.width_max, self.length_max, evidence=evidence)

                # insert partitions
                for plane_id in plane_model:
                    if plane_label[plane_id] == "0":
                        continue
                    cell_complex.insert_partition(plane_model[plane_id])

                # insert boundaries
                for boundary in boundary_list:
                    cell_complex.insert_boundary(boundary, height_threshold=(z_ref + self.boundary_height_threshold))

                # infer floorplan
                floorplan_speculator = estimators.FloorPlanSpeculator(cell_complex,
                                                                      horizon=self.speculation_horizon,
                                                                      min_ratio=self.min_free_ratio,
                                                                      coverage_threshold=self.boundary_coverage_threshold)
                floorplan = floorplan_speculator.floorplan()

                # convert to message and save to array
                floorplan_msg = self._floorplan_to_msg(floorplan)
                floorplan_array_msg.floorplans.append(floorplan_msg)

            self._floorplan_pub.publish(floorplan_array_msg)

    @staticmethod
    def _floorplan_to_msg(floorplan):
        floorplan_msg = Floorplan()

        node_id = {}
        for i, u in enumerate(floorplan.nodes):
            node_id[u] = i
            scene_node_msg = SceneNode()
            scene_node_msg.free_ratio.data = u.free_ratio
            scene_node_msg.index_map_keys = [UInt32(data=k) for k in u.vertex_index_map.keys()]
            scene_node_msg.index_map_values = [UInt32(data=v) for v in u.vertex_index_map.values()]
            for vertex in u.vertices:
                point_msg = Point()
                point_msg.x = vertex[0]
                point_msg.y = vertex[1]
                point_msg.z = vertex[2]
                scene_node_msg.vertices.append(point_msg)
            for edge in u.edges:
                pair_msg = UInt32Pair()
                pair_msg.u.data = edge[0]
                pair_msg.v.data = edge[1]
                scene_node_msg.edges.append(pair_msg)

            floorplan_msg.nodes.append(scene_node_msg)

        for u, v, data in floorplan.edges(data=True, keys=False):
            scene_edge_msg = SceneEdge()
            scene_edge_msg.u.data = node_id[u]
            scene_edge_msg.v.data = node_id[v]

            shared_edge = data['shared_edge']
            if shared_edge is not None:
                scene_edge_msg.shared_edge = [UInt32(data=shared_edge[0]), UInt32(data=shared_edge[1])]

            if data['boundary_interval'] is not None:
                interval = data['boundary_interval']
                start_point = Point()
                start_point.x = interval[0][0]
                start_point.y = interval[0][1]
                start_point.z = interval[0][2]
                scene_edge_msg.boundary.append(start_point)
                stop_point = Point()
                stop_point.x = interval[1][0]
                stop_point.y = interval[1][1]
                stop_point.z = interval[1][2]
                scene_edge_msg.boundary.append(stop_point)
            floorplan_msg.edges.append(scene_edge_msg)

        return floorplan_msg


def main(args=None):
    rclpy.init(args=args)
    node = FloorplanEstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
