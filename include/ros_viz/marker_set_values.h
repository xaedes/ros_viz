#pragma once

#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

inline void markerSetPosition(visualization_msgs::Marker& marker, float x, float y, float z)
{
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
}

inline void markerSetScale(visualization_msgs::Marker& marker, float x, float y, float z)
{
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
}

inline void markerSetOrientation(visualization_msgs::Marker& marker, float x, float y, float z, float w)
{
    marker.pose.orientation.x = x;
    marker.pose.orientation.y = y;
    marker.pose.orientation.z = z;
    marker.pose.orientation.w = w;
}

inline void markerSetColor(visualization_msgs::Marker& marker, float r, float g, float b, float a)
{
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

