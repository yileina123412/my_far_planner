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
    ros::Publisher viz_node_pub_, viz_nav_node_pub_, viz_path_pub_, viz_poly_pub_, viz_graph_pub_;
    ros::Publisher viz_contour_pub_, viz_map_pub_, viz_view_extend;

public:
    DPVisualizer() = default;
    ~DPVisualizer() = default;
    void Init(const ros::NodeHandle& nh);
    // 发布导航节点可视化
    void VizNodes(const NodePtrStack& node_stack, const std::string& ns, const VizColor& color,
        const float scale = 0.75f, const float alpha = 0.75f);
    void VizNavNodes(const NodePtrStack& node_stack, const std::string& ns, const VizColor& color1,
        const VizColor& color2, const float scale = 0.75f, const float alpha = 0.75f);
    /**
     * 节点可视化：
        白色球体：所有导航节点的基础显示
        红色叠加：正在更新的节点
        品红叠加：当前近邻范围内的节点
        蓝色叠加：已被充分探索的节点
        黄色叠加：中间导航点
        橙色叠加：探索前沿节点
        绿色叠加：边界节点
       连接可视化：
        白色细线：基础导航图连接
        翠绿线条：可视性连接和自由空间连接
        红色线条：基于轮廓的连接
        橙色线条：机器人连接和边界连接
        黄色线条：到目标的连接
        绿色粗线：轨迹连接
       方向可视化：
        黄色射线：节点的表面方向向量
        黄色立方体：方向向量的端点标记
        黄色连线：导航节点与轮廓节点的匹配关系
     */
    void VizGraph(const NodePtrStack& graph);
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