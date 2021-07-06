#pragma once

#include <vector>
#include <utility>
#include <functional>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "ros_viz/create_marker.h"

class WireFrame
{
public:
    inline void addQuad(const std::vector<cv::Vec3f>& points)
    {
        int id0 = m_points.size();
        addPoints(points);
        std::vector<int> indices;
        indices.resize(m_points.size());
        for (int i = 0; i < m_points.size(); ++i)
            indices[i] = id0 + i;
        addQuad(indices);
    }
    inline void addLine(const std::vector<cv::Vec3f>& points)
    {
        int id0 = m_points.size();
        addPoints(points);
        std::vector<int> indices;
        indices.resize(m_points.size());
        for (int i = 0; i < m_points.size(); ++i)
            indices[i] = id0 + i;
        addLine(indices);
    }
    inline void addQuad(const std::vector<int>& point_indices)
    {
        for (int i = 0; i < point_indices.size(); ++i)
        {
            m_lines.push_back(std::make_pair(
                point_indices[i],
                point_indices[(i+1)%point_indices.size()]
            ));
        }
    }
    inline void addLine(const std::vector<int>& point_indices)
    {
        for (int i = 0; i < point_indices.size()-1; ++i)
        {
            m_lines.push_back(std::make_pair(
                point_indices[i],
                point_indices[i+1]
            ));
        }
    }
    inline void addPoints(const std::vector<cv::Vec3f>& points)
    {
        int old_size = m_points.size();
        m_points.resize(old_size + points.size());
        for (size_t i = 0; i < points.size(); ++i)
        {
            m_points[old_size + i] = points[i];
        }
    }

    inline void insertInMarker(visualization_msgs::Marker& marker)
    {
        int old_size = marker.points.size();
        marker.points.resize(old_size + m_lines.size()*2);
        for (size_t i = 0; i<m_lines.size(); ++i)
        {
            int idx0 = old_size + i*2 + 0;
            int idx1 = old_size + i*2 + 1;
            const auto& point0 = m_points[m_lines[i].first];
            const auto& point1 = m_points[m_lines[i].second];
            marker.points[idx0].x = point0[0];
            marker.points[idx0].y = point0[1];
            marker.points[idx0].z = point0[2];
            marker.points[idx1].x = point1[0];
            marker.points[idx1].y = point1[1];
            marker.points[idx1].z = point1[2];
        }
    }

protected:
    std::vector<cv::Vec3f> m_points;
    std::vector<std::pair<int,int>> m_lines;
};

class WireFrameBox : public WireFrame
{
public:

};

