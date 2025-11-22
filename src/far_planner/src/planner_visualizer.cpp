#include "far_planner/planner_visualizer.h"

void DPVisualizer::Init(const ros::NodeHandle& nh) {
    nh_ = nh;
    point_cloud_ptr_ = PointCloudPtr(new PointCloud());
    // Rviz Publisher
    viz_node_pub_ = nh_.advertise<Marker>("/viz_node_topic", 5);
    viz_path_pub_ = nh_.advertise<Marker>("/viz_path_topic", 5);
    viz_poly_pub_ = nh_.advertise<MarkerArray>("/viz_poly_topic", 5);
    viz_graph_pub_ = nh_.advertise<MarkerArray>("/viz_graph_topic", 5);
    viz_contour_pub_ = nh_.advertise<MarkerArray>("/viz_contour_topic", 5);
    viz_map_pub_ = nh_.advertise<MarkerArray>("/viz_grid_map_topic", 5);
    viz_view_extend = nh_.advertise<MarkerArray>("/viz_viewpoint_extend_topic", 5);
}
void DPVisualizer::VizNodes(const NodePtrStack& node_stack, const std::string& ns, const VizColor& color,
    const float scale, const float alpha) {
    Marker node_marker;
    node_marker.type = Marker::SPHERE_LIST;
    this->SetMarker(color, ns, scale, alpha, node_marker);
    node_marker.points.resize(node_stack.size());
    std::size_t idx = 0;
    for (const auto& node_ptr : node_stack) {
        if (node_ptr == NULL) continue;
        node_marker.points[idx] = FARUtil::Point3DToGeoMsgPoint(node_ptr->position);
        idx++;
    }
    node_marker.points.resize(idx);
    viz_node_pub_.publish(node_marker);
}

void DPVisualizer::VizPointCloud(const ros::Publisher& viz_pub, const PointCloudPtr& pc) {
    sensor_msgs::PointCloud2 msg_pc;
    pcl::toROSMsg(*pc, msg_pc);
    msg_pc.header.frame_id = pc->header.frame_id;
    msg_pc.header.stamp = ros::Time::now();
    viz_pub.publish(msg_pc);
}

void DPVisualizer::VizMapGrids(const PointStack& neighbor_centers, const PointStack& occupancy_centers,
    const float& ceil_length, const float& ceil_height) {
    MarkerArray map_grid_marker_array;
    Marker neighbor_marker, occupancy_marker;
    neighbor_marker.type = Marker::CUBE_LIST;
    occupancy_marker.type = Marker::CUBE_LIST;
    this->SetMarker(VizColor::GREEN, "neighbor_grids", ceil_length / FARUtil::kVizRatio, 0.3f, neighbor_marker);
    this->SetMarker(VizColor::RED, "occupancy_grids", ceil_length / FARUtil::kVizRatio, 0.2f, occupancy_marker);
    neighbor_marker.scale.z = occupancy_marker.scale.z = ceil_height;
    const std::size_t N1 = neighbor_centers.size();
    const std::size_t N2 = occupancy_centers.size();
    neighbor_marker.points.resize(N1);
    occupancy_marker.points.resize(N2);
    for (std::size_t i = 0; i < N1; i++) {
        geometry_msgs::Point p = FARUtil::Point3DToGeoMsgPoint(neighbor_centers[i]);
        neighbor_marker.points.push_back(p);
    }
    for (std::size_t i = 0; i < N2; i++) {
        geometry_msgs::Point p = FARUtil::Point3DToGeoMsgPoint(occupancy_centers[i]);
        occupancy_marker.points.push_back(p);
    }
    map_grid_marker_array.markers.push_back(neighbor_marker);
    map_grid_marker_array.markers.push_back(occupancy_marker);
    viz_map_pub_.publish(map_grid_marker_array);
}
// 可视化轮廓
void DPVisualizer::VizContourGraph(const CTNodeStack& contour_graph) {
    MarkerArray contour_marker_array;
    Marker contour_vertex_marker, vertex_matched_marker, necessary_vertex_marker;
    Marker contour_marker, contour_surf_marker, contour_helper_marker;
    contour_vertex_marker.type = Marker::SPHERE_LIST;
    vertex_matched_marker.type = Marker::SPHERE_LIST;
    necessary_vertex_marker.type = Marker::SPHERE_LIST;
    contour_marker.type = Marker::LINE_LIST;
    contour_surf_marker.type = Marker::LINE_LIST;
    contour_helper_marker.type = Marker::CUBE_LIST;
    this->SetMarker(VizColor::EMERALD, "polygon_vertex", 0.5f, 0.5f, contour_vertex_marker);
    this->SetMarker(VizColor::RED, "matched_vertex", 0.5f, 0.5f, vertex_matched_marker);
    this->SetMarker(VizColor::GREEN, "necessary_vertex", 0.5f, 0.5f, necessary_vertex_marker);
    this->SetMarker(VizColor::MAGNA, "contour", 0.1f, 0.25f, contour_marker);
    this->SetMarker(VizColor::BLUE, "vertex_angle", 0.15f, 0.75f, contour_surf_marker);
    this->SetMarker(VizColor::BLUE, "angle_direct", 0.25f, 0.75f, contour_helper_marker);
    auto Draw_Contour = [&](const CTNodePtr& ctnode_ptr) {
        geometry_msgs::Point geo_vertex, geo_connect;
        geo_vertex = FARUtil::Point3DToGeoMsgPoint(ctnode_ptr->position);

        // contour_vertex_marker.points.push_back(geo_vertex);
        // if (ctnode_ptr->is_global_match) {
        //     vertex_matched_marker.points.push_back(geo_vertex);
        // }
        // if (ctnode_ptr->is_contour_necessary) {
        //     necessary_vertex_marker.points.push_back(geo_vertex);
        // }

        if (ctnode_ptr->free_direct == NodeFreeDirect::CONVEX) {
            contour_vertex_marker.points.push_back(geo_vertex);
        }
        if (ctnode_ptr->free_direct == NodeFreeDirect::UNKNOW) {
            vertex_matched_marker.points.push_back(geo_vertex);
        }
        if (ctnode_ptr->free_direct == NodeFreeDirect::CONCAVE) {
            necessary_vertex_marker.points.push_back(geo_vertex);
        }

        // if (ctnode_ptr->front == NULL || ctnode_ptr->back == NULL) return;
        // contour_marker.points.push_back(geo_vertex);
        // geo_connect = FARUtil::Point3DToGeoMsgPoint(ctnode_ptr->front->position);
        // contour_marker.points.push_back(geo_connect);
        // contour_marker.points.push_back(geo_vertex);
        // geo_connect = FARUtil::Point3DToGeoMsgPoint(ctnode_ptr->back->position);
        // contour_marker.points.push_back(geo_connect);
    };
    auto Draw_Surf_Dir = [&](const CTNodePtr& ctnode) {
        geometry_msgs::Point p1, p2, p3;
        p1 = FARUtil::Point3DToGeoMsgPoint(ctnode->position);
        Point3D end_p;
        if (ctnode->free_direct != NodeFreeDirect::PILLAR) {
            end_p = ctnode->position + ctnode->surf_dirs.first * FARUtil::kVizRatio;
            p2 = FARUtil::Point3DToGeoMsgPoint(end_p);
            contour_surf_marker.points.push_back(p1);
            contour_surf_marker.points.push_back(p2);
            contour_helper_marker.points.push_back(p2);
            end_p = ctnode->position + ctnode->surf_dirs.second * FARUtil::kVizRatio;
            p3 = FARUtil::Point3DToGeoMsgPoint(end_p);
            contour_surf_marker.points.push_back(p1);
            contour_surf_marker.points.push_back(p3);
            contour_helper_marker.points.push_back(p3);
        }
    };
    for (const auto& ctnode : contour_graph) {
        if (ctnode == NULL) {
            // DEBUG
            // ROS_ERROR("Viz: contour node is NULL.");
            continue;
        }
        Draw_Contour(ctnode);
        // Draw_Surf_Dir(ctnode);
    }
    contour_marker_array.markers.push_back(contour_vertex_marker);
    contour_marker_array.markers.push_back(vertex_matched_marker);
    contour_marker_array.markers.push_back(necessary_vertex_marker);
    contour_marker_array.markers.push_back(contour_marker);
    contour_marker_array.markers.push_back(contour_surf_marker);
    contour_marker_array.markers.push_back(contour_helper_marker);
    viz_contour_pub_.publish(contour_marker_array);
}

void DPVisualizer::SetMarker(const VizColor& color, const std::string& ns, const float& scale, const float& alpha,
    Marker& scan_marker, const float& scale_ratio) {
    scan_marker.header.frame_id = FARUtil::worldFrameId;
    scan_marker.header.stamp = ros::Time::now();
    scan_marker.id = 0;
    scan_marker.ns = ns;
    scan_marker.action = Marker::ADD;
    scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z = scale * scale_ratio;
    scan_marker.pose.orientation.x = 0.0;
    scan_marker.pose.orientation.y = 0.0;
    scan_marker.pose.orientation.z = 0.0;
    scan_marker.pose.orientation.w = 1.0;
    scan_marker.pose.position.x = 0.0;
    scan_marker.pose.position.y = 0.0;
    scan_marker.pose.position.z = 0.0;
    DPVisualizer::SetColor(color, alpha, scan_marker);
}

void DPVisualizer::SetColor(const VizColor& color, const float& alpha, Marker& scan_marker) {
    std_msgs::ColorRGBA c;
    c.a = alpha;
    if (color == VizColor::RED) {
        c.r = 1.0f, c.g = c.b = 0.f;
    } else if (color == VizColor::ORANGE) {
        c.r = 1.0f, c.g = 0.45f, c.b = 0.1f;
    } else if (color == VizColor::BLACK) {
        c.r = c.g = c.b = 0.1f;
    } else if (color == VizColor::YELLOW) {
        c.r = c.g = 0.9f, c.b = 0.1;
    } else if (color == VizColor::BLUE) {
        c.b = 1.0f, c.r = 0.1f, c.g = 0.1f;
    } else if (color == VizColor::GREEN) {
        c.g = 0.9f, c.r = c.b = 0.f;
    } else if (color == VizColor::EMERALD) {
        c.g = c.b = 0.9f, c.r = 0.f;
    } else if (color == VizColor::WHITE) {
        c.r = c.g = c.b = 0.9f;
    } else if (color == VizColor::MAGNA) {
        c.r = c.b = 0.9f, c.g = 0.f;
    } else if (color == VizColor::PURPLE) {
        c.r = c.b = 0.5f, c.g = 0.f;
    }
    scan_marker.color = c;
}
