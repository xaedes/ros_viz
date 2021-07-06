#!/usr/bin/env python

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped 
from geometry_msgs.msg import Transform 
from geometry_msgs.msg import Vector3 
from aruco_msgs.msg import Marker
from aruco_msgs.msg import MarkerArray

class Node:
    def __init__(self):
        pass

    def run(self):
        rospy.init_node("viz_markers")
        self.marker_tf_prefix = rospy.get_param("~tf_prefix", "marker_")
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()

        self.sub_markers = rospy.Subscriber("markers", MarkerArray, self.callback_markers)
        rospy.spin()

    def callback_markers(self, msg):
        for marker in msg.markers:
            child_frame_id = self.marker_tf_prefix + str(marker.id)
            msg_tf = TransformStamped(
                header = marker.header, 
                child_frame_id = child_frame_id, 
                transform = Transform(
                    translation = Vector3(marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z),
                    rotation = marker.pose.pose.orientation
                )
            )
            self.transform_broadcaster.sendTransform(msg_tf)

def main():
    node = Node()
    node.run()

try:
    main()
except rospy.ROSInterruptException:
    pass