
from visualization_msgs.msg import Marker as MsgMarker
from geometry_msgs.msg import Point as MsgPoint
from std_msgs.msg import ColorRGBA as MsgColorRGBA

class RVizWireFrame:
    def __init__(self):
        self.lines = []

    def add_line(self, points, colors):
        assert(len(points) == 2)
        assert(len(points) == len(colors))
        self.lines.append((points, colors))

    def add_polygon(self, points, colors):
        assert(len(points) == len(colors))
        num = len(points)
        for k in range(num):
            i = (k+1) % num
            self.add_line(
                [points[k], points[i]],
                [colors[k], colors[i]])

    def add_triangle(self, points, colors):
        self.add_polygon(points, colors)

    def add_quad(self, points, colors):
        self.add_polygon(points, colors)

    def to_marker(self, 
            header=None, stamp=None, frame_id=None, 
            ns="", id=0, lifetime=0,
            alpha=1.0, width=0.1, frame_locked=True, 
            quaternion=(0.0, 0.0, 0.0, 1.0)):
        msg = MsgMarker()
        msg.ns = ns
        msg.id = id
        msg.type = msg.LINE_LIST
        msg.action = msg.MODIFY
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 0
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        msg.scale.x = width
        msg.scale.y = 1
        msg.scale.z = 1
        msg.color.r = 1
        msg.color.g = 1
        msg.color.b = 1
        msg.color.a = alpha
        msg.lifetime = lifetime
        msg.frame_locked = frame_locked
        for line in self.lines:
            points, colors = line
            for point, color in zip(points, colors):
                msg.points.append(MsgPoint(*point))
                msg.colors.append(MsgColorRGBA(*color))

        if header is not None: msg.header = header
        if stamp is not None: msg.header.stamp = stamp
        if frame_id is not None: msg.header.frame_id = frame_id
    
        return msg