
from visualization_msgs.msg import Marker as MsgMarker
from geometry_msgs.msg import Point as MsgPoint
from std_msgs.msg import ColorRGBA as MsgColorRGBA

class RVizMesh:
    def __init__(self):
        self.triangles = []

    def add_triangle(self, points, colors):
        self.triangles.append((points, colors))
        # self.triangles.append((list(reversed(points)), list(reversed(colors))))

    def add_quad(self, points, colors):
        p0,p1,p2,p3 = points
        c0,c1,c2,c3 = colors
        self.add_triangle([p0,p1,p2],[c0,c1,c2])
        self.add_triangle([p0,p2,p3],[c0,c2,c3])

    def to_marker(self, 
            header=None, stamp=None, frame_id=None, 
            ns="", id=0, lifetime = 0,
            alpha=1.0, frame_locked=True,
            quaternion=(0.0, 0.0, 0.0, 1.0)):
        msg = MsgMarker()
        msg.ns = ns
        msg.id = id
        msg.type = msg.TRIANGLE_LIST
        msg.action = msg.MODIFY
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 0
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        msg.scale.x = 1
        msg.scale.y = 1
        msg.scale.z = 1
        msg.color.r = 1
        msg.color.g = 1
        msg.color.b = 1
        msg.color.a = alpha
        msg.lifetime = lifetime
        msg.frame_locked = frame_locked
        for triangle in self.triangles:
            points, colors = triangle
            for point, color in zip(points, colors):
                msg.points.append(MsgPoint(*point))
                msg.colors.append(MsgColorRGBA(*color))

        if header is not None: msg.header = header
        if stamp is not None: msg.header.stamp = stamp
        if frame_id is not None: msg.header.frame_id = frame_id
    
        return msg

