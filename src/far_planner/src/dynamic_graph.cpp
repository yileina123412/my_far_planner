#include "far_planner/dynamic_graph.h"

void DynamicGraph::Init(const ros::NodeHandle& nh, const DynamicGraphParams& params) {
    dg_params_ = params;
    id_tracker_ = 1;
    last_connect_pos_ = Point3D(0, 0, 0);
}

void DynamicGraph::UpdateRobotPosition(const Point3D& robot_pos) {
    robot_pos_ = robot_pos;
    if (odom_node_ptr_ == NULL) {
        this->CreateNavNodeFromPoint(robot_pos_, odom_node_ptr_, true);
        this->AddNodeToGraph(odom_node_ptr_);
        if (FARUtil::IsDebug) ROS_INFO("DG: Odom node has been initilaized.");
    } else {
        this->UpdateNodePosition(odom_node_ptr_, robot_pos_);
    }
    FARUtil::odom_pos = odom_node_ptr_->position;
}

void DynamicGraph::UpdateNavGraph(
    const NodePtrStack& new_nodes, const bool& is_freeze_vgraph, NodePtrStack& clear_node) {
    clear_node.clear();
}

void DynamicGraph::UpdateGlobalNearNodes() {}

bool DynamicGraph::IsInterNavpointNecessary() {
    if (cur_internav_ptr_ == NULL) {
        last_connect_pos_ = FARUtil::free_odom_p;
        return true;
    }
}

bool DynamicGraph::ExtractGraphNodes(const CTNodeStack& new_ctnodes) {
    if (new_ctnodes.empty()) return false;
    new_nodes_.clear();
    NavNodePtr new_node_ptr = NULL;
    // 导航点
    // if (IsInterNavpointNecessary()) {
    //     if (FARUtil::IsDebug) ROS_INFO("DG: One trajectory node has been created.");
    //     CreateNavNodeFromPoint(last_connect_pos_, new_node_ptr, false, true);
    //     new_nodes_.push_back(new_node_ptr);
    //     last_connect_pos_ = FARUtil::free_odom_p;
    //     if (is_bridge_internav_) is_bridge_internav_ = false;
    // }
    // 轮廓点
    for (const auto& ctnode_ptr : new_ctnodes) {
        bool is_near_new = false;
        if (this->IsAValidNewNode(ctnode_ptr, is_near_new)) {
            CreateNewNavNodeFromContour(ctnode_ptr, new_node_ptr);
            if (!is_near_new) {
                new_node_ptr->is_block_frontier = true;
            }
            new_nodes_.push_back(new_node_ptr);
        }
    }
    if (new_nodes_.empty())
        return false;
    else
        return true;
}

bool DynamicGraph::UpdateNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos) {
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        this->InitNodePosition(node_ptr, new_pos);
        return true;
    }
}

void DynamicGraph::InitNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos) {
    node_ptr->pos_filter_vec.clear();
    node_ptr->position = new_pos;
    node_ptr->pos_filter_vec.push_back(new_pos);
}

bool DynamicGraph::UpdateNodeSurfDirs(const NavNodePtr& node_ptr, PointPair cur_dirs) {
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        node_ptr->surf_dirs = {Point3D(0, 0, -1), Point3D(0, 0, -1)};
        node_ptr->free_direct = NodeFreeDirect::PILLAR;
        return true;
    }
    if (node_ptr->is_finalized = true) {
        return true;
    }
    FARUtil::CorrectDirectOrder(node_ptr->surf_dirs, cur_dirs);
    node_ptr->surf_dirs_vec.push_back(cur_dirs);
    if (node_ptr->surf_dirs_vec.size() > dg_params_.pool_size) {
        node_ptr->surf_dirs_vec.pop_front();
    }
    // calculate the mean surface corner direction using RASCAN
    std::size_t inlier_size = 0;  // 评估质量
    const PointPair mean_dir =
        FARUtil::RANSACSurfDirs(node_ptr->surf_dirs_vec, dg_params_.filter_dirs_margin, inlier_size);
    if (mean_dir.first == Point3D(0, 0, -1) || mean_dir.second == Point3D(0, 0, -1)) {
        node_ptr->surf_dirs.first = Point3D(0, 0, -1);
        node_ptr->surf_dirs.second = Point3D(0, 0, -1);
    } else {
        node_ptr->surf_dirs = mean_dir;
        ReEvaluateConvexity(node_ptr);
    }
    if (inlier_size > dg_params_.finalize_thred) {
        return true;
    }
    return false;
}

void DynamicGraph::ReEvaluateConvexity(const NavNodePtr& node_ptr) {
    if (!node_ptr->is_contour_match || node_ptr->ctnode->poly_ptr->is_pillar) return;
    bool is_wall = false;
    const Point3D topo_dir = FARUtil::SurfTopoDirect(node_ptr->surf_dirs, is_wall);
    if (!is_wall) {
        const Point3D ctnode_p = node_ptr->ctnode->position;
        const Point3D ev_p = ctnode_p + topo_dir * FARUtil::kLeafSize;
        if (FARUtil::IsConvexPoint(node_ptr->ctnode->poly_ptr, ev_p)) {
            node_ptr->free_direct = NodeFreeDirect::CONVEX;
        } else {
            node_ptr->free_direct = NodeFreeDirect::CONCAVE;
        }
    }
}