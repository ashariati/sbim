#!/usr/bin/env python

import numpy as np
import threading
import copy
import collections
import time

import rospy
import message_filters

import sbim_msgs.msg
from sbim_msgs.msg import Trajectory, PrincipalPlaneArray, CorrespondenceMap, LayoutSegmentArray
from sbim_msgs.msg import FloorplanArray, Floorplan, SceneNode, SceneEdge

import geometry_msgs.msg
from geometry_msgs.msg import Point

import bimpy
from bimpy import models, estimators


class FloorplanEstimationNode(object):

    def __init__(self):

        # parameters
        self.freq = rospy.get_param('frequency', 1)
        self.speculation_horizon = rospy.get_param('speculation_horizon', 0)
        self.width_max = rospy.get_param('width_max', 300)
        self.length_max = rospy.get_param('length_max', 300)
        self.boundary_height_threshold = rospy.get_param('boundary_height_threshold', 2.5)

        # subscribers
        trajectory_sub = message_filters.Subscriber('/planar_slam_node/trajectory', Trajectory)
        layout_plane_sub = message_filters.Subscriber('/planar_slam_node/layout_planes', PrincipalPlaneArray)
        correspondence_sub = message_filters.Subscriber('/planar_slam_node/correspondence_map', CorrespondenceMap)
        slam_sync = message_filters.ApproximateTimeSynchronizer(
            fs=[trajectory_sub, layout_plane_sub, correspondence_sub], queue_size=100, slop=0.05)
        slam_sync.registerCallback(self.slam_callback)

        segment_sub = rospy.Subscriber('/layout_extractor_node/layout_segments', LayoutSegmentArray,
                                       self.scene_parsing_callback, queue_size=100)
        # segment_sub = message_filters.Subscriber('/layout_extractor_node/layout_segments', LayoutSegmentArray)
        # door_sub = message_filters.Subscriber('/door_detector_node/doors', LayoutSegmentArray)
        # parsing_sync = message_filters.ApproximateTimeSynchronizer(
        #     fs=[segment_sub, door_sub], queue_size=10, slop=0.05)
        # parsing_sync.registerCallback(self.scene_parsing_callback)

        # publishers
        self._floorplan_pub = rospy.Publisher('/floorplan_estimation_node/floorplan', FloorplanArray, queue_size=10)

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

        self._lock.acquire(True)
        segment_list = copy.copy(self._segment_list)
        plane_model = copy.copy(self._plane_model)
        plane_label = copy.copy(self._plane_label)
        self._lock.release()

        upward_facing = []
        for plane in layout_planes.planes:
            coef = np.array([plane.plane.coef[0], plane.plane.coef[1], plane.plane.coef[2], plane.plane.coef[3]])
            plane_model[plane.id.data] = models.Plane(coef)
            plane_label[plane.id.data] = plane.label.data
            if plane.label.data == "0" and plane.facing.data == 1:
                upward_facing.append(plane.id.data)

        for pose in trajectory.poses:
            time = (pose.header.stamp.secs, pose.header.stamp.nsecs)
            self._pose_at_time[time] = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

        # process segments
        boundary_list = []
        plane_evidence = collections.defaultdict(list)
        for segment in segment_list:

            if segment.plane_id.data not in parent_plane:
                continue
            plane = plane_model[parent_plane[segment.plane_id.data]]

            time = (segment.header.stamp.secs, segment.header.stamp.nsecs)
            pose = self._pose_at_time[time]

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

        self._lock.acquire(True)
        self._plane_model = plane_model
        self._plane_label = plane_label
        self._upward_facing = upward_facing
        self._boundary_list = boundary_list
        self._plane_evidence = plane_evidence
        self._lock.release()

    def scene_parsing_callback(self, layout_segments):

        self._lock.acquire(True)
        for segment in layout_segments.layout_segments:
            self._segment_list.append(segment)
        self._lock.release()

    def loop(self):

        rospy.init_node('floorplan_estimation_node', anonymous=True)

        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():

            rate.sleep()

            self._lock.acquire(True)
            plane_model = copy.copy(self._plane_model)
            plane_label = copy.copy(self._plane_label)
            upward_facing = copy.copy(self._upward_facing)
            boundary_list = copy.copy(self._boundary_list)
            plane_evidence = copy.copy(self._plane_evidence)
            self._lock.release()

            evidence = []
            for plane_id in plane_evidence:
                evidence.extend(plane_evidence[plane_id])

            floorplan_array_msg = FloorplanArray()
            floorplan_array_msg.header.frame_id = 'building'
            floorplan_array_msg.header.stamp = rospy.Time.now()
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
                floorplan_speculator = estimators.FloorPlanSpeculator(cell_complex, horizon=self.speculation_horizon)
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
            for vertex in u.vertices:
                point_msg = Point()
                point_msg.x = vertex[0]
                point_msg.y = vertex[1]
                point_msg.z = vertex[2]
                scene_node_msg.vertices.append(point_msg)
            floorplan_msg.nodes.append(scene_node_msg)

        for u, v, data in floorplan.edges(data=True):
            scene_edge_msg = SceneEdge()
            scene_edge_msg.u.data = node_id[u]
            scene_edge_msg.v.data = node_id[v]
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



if __name__ == '__main__':
    node = FloorplanEstimationNode()
    node.loop()
