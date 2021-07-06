#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
import quaternion as npqt

d2r = math.pi / 180

from visualization_msgs.msg import Marker as MsgMarker
from visualization_msgs.msg import MarkerArray as MsgMarkerArray
from geometry_msgs.msg import Point as MsgPoint
from std_msgs.msg import ColorRGBA as MsgColorRGBA

from ros_viz.rviz_mesh import RVizMesh
from ros_viz.rviz_wireframe import RVizWireFrame

def rotX(angle):
    cs,sn = math.cos(angle), math.sin(angle)
    return np.array([
        [1,0,0,],
        [0,cs,-sn],
        [0,sn,cs]
    ])

def rotY(angle):
    cs,sn = math.cos(angle), math.sin(angle)
    return np.array([
        [cs,0,sn],
        [0,1,0],
        [-sn,0,cs]
    ])

def rotZ(angle):
    cs,sn = math.cos(angle), math.sin(angle)
    return np.array([
        [cs,-sn,0,],
        [sn,cs,0],
        [0,0,1]
    ])

def roll_pitch_yaw(roll, pitch, yaw):
    return rotZ(yaw) @ rotY(pitch) @ rotX(roll)

def try_otherwise(function, default_value=None, *args , **kwargs):
    #return function(*args , **kwargs)
    try:
        return function(*args , **kwargs)
    #except e:
    except Exception as e: 
        print (e  , args)
        return default_value

class Node:
    def __init__(self):
        pass

    def run(self):
        rospy.init_node("node")
        self.frame_id =           try_otherwise(rospy.get_param, "frame", "~frame_id")
        self.rotation =          try_otherwise(rospy.get_param, "0.0 0.0 0.0", "~rotation")
        self.rotation = [float(degree)*d2r for degree in self.rotation.split(" ")[:3]]
        self.quaternion = npqt.from_rotation_matrix(roll_pitch_yaw(*self.rotation))
        self.quaternion = [self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w]
        # print(self.quaternion)
        self.frustum_type =       try_otherwise(rospy.get_param, "rectangular", "~frustum_type")
        self.frustum_mask =       try_otherwise(rospy.get_param, "", "~frustum_mask")
        self.hfov =               try_otherwise(rospy.get_param, 45.0, "~hfov")
        self.vfov =               try_otherwise(rospy.get_param, 40.0, "~vfov")
        self.hfov *= d2r
        self.vfov *= d2r
        self.near =               try_otherwise(rospy.get_param, 1.0, "~near")
        self.far =                try_otherwise(rospy.get_param, 10.0, "~far")
        self.face_color =         try_otherwise(rospy.get_param, "1.0 0.0 0.0", "~face_color")
        self.wire_color =         try_otherwise(rospy.get_param, "0.0 0.0 0.0", "~wire_color")
        self.face_color = list(map(float,self.face_color.split(" ")))[:3]
        self.wire_color = list(map(float,self.wire_color.split(" ")))[:3]
        self.face_alpha =         try_otherwise(rospy.get_param, 0.5, "~face_alpha")
        self.wire_alpha =         try_otherwise(rospy.get_param, 1.0, "~wire_alpha")
        self.wire_width =         try_otherwise(rospy.get_param, 0.01, "~wire_width")
        self.enable_central_ray = try_otherwise(rospy.get_param, True, "~enable_central_ray")
        self.frame_locked =       try_otherwise(rospy.get_param, True, "~frame_locked")
        self.namespace =          try_otherwise(rospy.get_param, "viz_view_frustum", "~namespace")
        self.lifetime =           try_otherwise(rospy.get_param, 0.0, "~lifetime")
        self.lifetime = rospy.Duration(self.lifetime)
        self.rate_param =         try_otherwise(rospy.get_param, 100, "~rate")
        self.circle_resolution =  try_otherwise(rospy.get_param, 16, "~circle_resolution")
        try:
            self.circle_resolution = int(self.circle_resolution)
        except:
            self.circle_resolution = 16

        valid_frustum_types = ["rectangular", "circular", "mask"]
        assert( self.frustum_type in valid_frustum_types )
        assert( self.circle_resolution >= 3)
        if self.frustum_type == "rectangular":
            self.mesh, self.wires = self.rectangular_view_frustum(
                self.hfov, self.vfov, self.near, self.far, 
                self.enable_central_ray,
                self.face_color, self.wire_color, self.face_alpha, self.wire_alpha)

        elif self.frustum_type == "circular":
            self.mesh, self.wires = self.circular_view_frustum(
                self.hfov, self.vfov, self.near, self.far, self.circle_resolution, 
                self.enable_central_ray,
                self.face_color, self.wire_color, self.face_alpha, self.wire_alpha)

        elif self.frustum_type == "mask":
            self.mesh, self.wires = self.mask_view_frustum(
                self.hfov, self.vfov, self.near, self.far, self.frustum_mask, 
                self.enable_central_ray,
                self.face_color, self.wire_color, self.face_alpha, self.wire_alpha)

        self.pub = rospy.Publisher("~out/viz", MsgMarkerArray, queue_size=1)
        # self.pub = rospy.Publisher("~out/viz", MsgMarker, queue_size=1)
        self.msg = self.marker_array_of([
            self.mesh.to_marker(
                frame_id = self.frame_id,
                frame_locked = self.frame_locked,
                lifetime = self.lifetime,
                ns = self.namespace,
                quaternion = self.quaternion,
            ),
            self.wires.to_marker(
                frame_id = self.frame_id,
                frame_locked = self.frame_locked,
                lifetime = self.lifetime,
                ns = self.namespace,
                width = self.wire_width,
                quaternion = self.quaternion,
            )],
            ns = self.namespace
        )
        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate_param),callback=self.on_timer)
        rospy.spin()
        # self.rate = rospy.Rate(self.rate_param)
        # while not rospy.is_shutdown():
        #     for k in range(len(self.msg.markers)):
        #         self.msg.markers[k].header.stamp = rospy.Time.now()
        #     # self.msg.header.stamp = rospy.Time.now()
        #     self.pub.publish(self.msg)
        #     try:
        #         self.rate.sleep()
        #     except rospy.exceptions.ROSTimeMovedBackwardsException:
        #         pass
    def on_timer(self, evt):
        for k in range(len(self.msg.markers)):
            self.msg.markers[k].header.stamp = rospy.Time.now()
        # self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)


    def marker_array_of(self, markers, ns="", clear=True):
        msg = MsgMarkerArray()
        if clear:
            msg.markers.append(self.clear_markers(ns))
        msg.markers += markers
        for k in range(len(msg.markers)):
            msg.markers[k].id = k
        return msg

    def clear_markers(self, ns=""):
        msg = MsgMarker()
        msg.action = msg.DELETEALL
        msg.ns = ns
        return msg

    def rectangular_view_frustum(self, 
            hfov, vfov, near, far, 
            enable_central_ray,
            face_color, wire_color, face_alpha, wire_alpha):
        htan = math.tan(hfov/2)
        vtan = math.tan(vfov/2)
        near_points = [
            (-htan*near, -vtan*near, near),
            (-htan*near, +vtan*near, near),
            (+htan*near, +vtan*near, near),
            (+htan*near, -vtan*near, near),
        ]
        far_points = [
            (-htan*far, -vtan*far, far),
            (-htan*far, +vtan*far, far),
            (+htan*far, +vtan*far, far),
            (+htan*far, -vtan*far, far),
        ]
        return self.connect_near_and_far(
            near_points, far_points, 
            enable_central_ray, 
            face_color, wire_color, face_alpha, wire_alpha)

    def circular_view_frustum(self, 
            hfov, vfov, near, far, circle_resolution, 
            enable_central_ray,
            face_color, wire_color, face_alpha, wire_alpha):
        angles = np.linspace(0,math.pi*2,circle_resolution+1)[:-1]
        cs, sn = np.cos(angles), np.sin(angles)
        htan = math.tan(hfov/2)
        vtan = math.tan(vfov/2)
        near_points = []
        far_points = []
        for cos,sin in zip(cs,sn):
            near_points.append(
                (htan * near * cos, vtan * near * sin, near)
            )
            far_points.append(
                (htan * far * cos, vtan * far * sin, far)
            )
        return self.connect_near_and_far(
            near_points, far_points, 
            enable_central_ray, 
            face_color, wire_color, face_alpha, wire_alpha)

    def mask_view_frustum(self,
            hfov, vfov, near, far, mask,
            enable_central_ray,
            face_color, wire_color, face_alpha, wire_alpha
            ):
        if type(mask) == str:
            mask = cv2.imread(mask, cv2.IMREAD_GRAYSCALE)
        assert(mask.dtype == np.uint8)
        assert(len(mask.shape) == 2)

        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours = [contour.reshape(-1,2) for contour in contours]
        assert(len(contours) > 0)
        if len(contours) > 1:
            contours = list(sorted(contours, key=lambda c: -len(c)))
        contour = contours[0]
        eps = 1e-3# + 1e-4)
        approx_contour = cv2.approxPolyDP(
            curve = contour,
            epsilon = eps*cv2.arcLength(contour,True),
            closed = True).reshape(-1,2)
        htan = math.tan(hfov/2)
        vtan = math.tan(vfov/2)
        h,w = mask.shape
        near_points = []
        far_points = []
        for u,v in approx_contour:
            # remap u from [0..w-1] to [-1..+1]
            # remap v from [0..h-1] to [-1..+1]
            x = -1. + 2. * u / (w-1.)
            y = -1. + 2. * v / (h-1.)
            near_points.append(
                (htan * near * x, vtan * near * y, near)
            )
            far_points.append(
                (htan * far * x, vtan * far * y, far)
            )

        return self.connect_near_and_far(
            near_points, far_points, 
            enable_central_ray, 
            face_color, wire_color, face_alpha, wire_alpha)



    def connect_near_and_far(self, 
            near_points, far_points, 
            enable_central_ray,
            face_color, wire_color, face_alpha, wire_alpha):
        assert(len(near_points) == len(far_points))
        num = len(near_points)
        mesh = RVizMesh()
        wires = RVizWireFrame()
        r,g,b = face_color
        face_color_a = [(r,g,b,face_alpha)]
        face_colors_4 = face_color_a * 4
        face_colors_3 = face_color_a * 3
        # face_colors_2 = face_color_a * 2
        r,g,b = wire_color
        wire_color_a = [(r,g,b,wire_alpha)]
        wire_colors_4 = wire_color_a * 4
        # wire_colors_3 = wire_color_a * 3
        wire_colors_2 = wire_color_a * 2
        for k in range(num):
            i = (k+1) % num
            points = [
                near_points[k],
                near_points[i],
                far_points[i],
                far_points[k]
            ]
            mesh.add_quad(points, face_colors_4)
            wires.add_quad(points, wire_colors_4)
        
        near_center = np.array(near_points).mean(axis=0)
        far_center = np.array(far_points).mean(axis=0)
        for k in range(num):
            i = (k+1) % num
            _near_points = [
                near_center,
                near_points[k],
                near_points[i],
            ]
            _far_points = [
                far_center,
                far_points[k],
                far_points[i],
            ]
            mesh.add_triangle(_near_points, face_colors_3)
            mesh.add_triangle(_far_points, face_colors_3)

        if enable_central_ray:
            wires.add_line([near_center, far_center], wire_colors_2)

        return mesh, wires

def main():
    node = Node()
    node.run()

main()
# try:
    # main()
# except rospy.ROSInterruptException:
    # pass

