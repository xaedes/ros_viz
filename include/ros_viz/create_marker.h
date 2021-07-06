#pragma once

#include "visualization_msgs/Marker.h"
#include <ros/ros.h>


inline visualization_msgs::Marker createMarker(
    const ros::Time& timestamp,
    const std::string& frame_id,
    const std::string& ns,
    int32_t id,
    int32_t marker_type,
    int32_t action,
    float x, float y, float z,
    float qx, float qy, float qz, float qw,
    float sx, float sy, float sz,
    float r, float g, float b, float a
)
{
    visualization_msgs::Marker marker;
    marker.header.stamp = timestamp;
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = marker_type;
    marker.action = action;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = qx;
    marker.pose.orientation.y = qy;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;
    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
}

inline visualization_msgs::Marker createMarker(
    const ros::Time& timestamp,
    const std::string& frame_id,
    const std::string& ns,
    int32_t id,
    int32_t type_,
    int32_t action)
{
    return createMarker(
        timestamp, frame_id, ns, id, type_, action,
        0, 0, 0,    // x, y, z
        0, 0, 0, 1, // qx, qy, qz, qw
        1, 1, 1,    // sx, sy, sz
        0, 0, 0, 1  // r, g, b, a
    );
}

