#ifndef PLANNER_VISUALIZER_H
#define PLANNER_VISUALIZER_H

#pragma once

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "utility.h"

typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;

enum VizColor {
    RED = 0,
    ORANGE = 1,
    BLACK = 2,
    YELLOW = 3,
    BLUE = 4,
    GREEN = 5,
    EMERALD = 6,
    WHITE = 7,
    MAGNA = 8,
    PURPLE = 9
};

class DPVisualizer {
private:
    ros::NodeHandle nh_;
    PointCloudPtr point_cloud_ptr_;
    // rviz publisher
    ros::Publisher viz_node_pub_, viz_path_pub_, viz_poly_pub_, viz_graph_pub_;
    ros::Publisher viz_contour_pub_, viz_map_pub_, viz_view_extend;

public:
    DPVisualizer() = default;
    ~DPVisualizer() = default;
    void Init(const ros::NodeHandle& nh);
    // 发布导航节点可视化
    void VizNodes(const NodePtrStack& node_stack, const std::string& ns, const VizColor& color,
        const float scale = 0.75f, const float alpha = 0.75f);
    // 发布点云可视化
    void VizPointCloud(const ros::Publisher& viz_pub, const PointCloudPtr& pc);  // 可视化点云
    // 发布可视化邻近的格子以及占据的格子
    void VizMapGrids(const PointStack& neighbor_centers, const PointStack& occupancy_centers, const float& ceil_length,
        const float& ceil_height);
    // 发布ct点
    void VizContourGraph(const CTNodeStack& contour_graph);
    // 工具函数
    // 修改传入的scan_marker对象，设置完整属性
    static void SetMarker(const VizColor& color, const std::string& ns, const float& scale, const float& alpha,
        Marker& scan_marker, const float& scale_ratio = FARUtil::kVizRatio);

    static void SetColor(const VizColor& color, const float& alpha, Marker& scan_marker);
};

#endif