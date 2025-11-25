#include "far_planner/dynamic_graph.h"

void DynamicGraph::Init(const ros::NodeHandle& nh, const DynamicGraphParams& params) {
    dg_params_ = params;
    id_tracker_ = 1;
    last_connect_pos_ = Point3D(0, 0, 0);
    CONNECT_ANGLE_COS = cos(dg_params_.kConnectAngleThred);
    NOISE_ANGLE_COS = cos(FARUtil::kAngleNoise);
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
    for (const auto& new_node_pr : new_nodes) {
        this->AddNodeToGraph(new_node_pr);
    }
}

void DynamicGraph::UpdateGlobalNearNodes() {
    /*update nearby navigation nodes stack-->near_nav_nodes_*/
    near_nav_nodes_.clear(), wide_near_nodes_.clear(), extend_match_nodes_.clear();
    margin_near_nodes_.clear();
    internav_near_nodes_.clear(), surround_internav_nodes_.clear();
    for (const auto& node_ptr : globalGraphNodes_) {
        node_ptr->is_near_nodes = false;
        node_ptr->is_wide_near = false;
        if (FARUtil::IsNodeInExtendMatchRange(node_ptr) &&
            (!node_ptr->is_active || MapHandler::IsNavPointOnTerrainNeighbor(node_ptr->position, true))) {
            if (FARUtil::IsOutsideGoal(node_ptr)) continue;
            if (this->IsActivateNavNode(node_ptr) || node_ptr->is_boundary) extend_match_nodes_.push_back(node_ptr);
            if (FARUtil::IsNodeInLocalRange(node_ptr) && IsPointOnTerrain(node_ptr->position)) {
                wide_near_nodes_.push_back(node_ptr);
                node_ptr->is_wide_near = true;
                if (node_ptr->is_active || node_ptr->is_boundary) {
                    near_nav_nodes_.push_back(node_ptr);
                    node_ptr->is_near_nodes = true;
                    if (node_ptr->is_navpoint) {
                        node_ptr->position.intensity = node_ptr->fgscore;
                        internav_near_nodes_.push_back(node_ptr);
                        if ((node_ptr->position - odom_node_ptr_->position).norm() < FARUtil::kLocalPlanRange / 2.0f) {
                            surround_internav_nodes_.push_back(node_ptr);
                        }
                    }
                }
            } else if (node_ptr->is_active || node_ptr->is_boundary) {
                margin_near_nodes_.push_back(node_ptr);
            }
        }
    }
    // add additional odom connections to wide near stack
    for (const auto& cnode_ptr : odom_node_ptr_->connect_nodes) {
        if (FARUtil::IsOutsideGoal(cnode_ptr)) continue;
        if (!cnode_ptr->is_wide_near) {
            wide_near_nodes_.push_back(cnode_ptr);
            cnode_ptr->is_wide_near = true;
        }
        for (const auto& c2node_ptr : cnode_ptr->connect_nodes) {
            if (!c2node_ptr->is_wide_near && !FARUtil::IsOutsideGoal(c2node_ptr)) {
                wide_near_nodes_.push_back(c2node_ptr);
                c2node_ptr->is_wide_near = true;
            }
        }
    }
    // find the nearest inter_nav node that connect to odom
    // 中间导航节点cur_internav_ptr_的选择和更新的原理
    // 寻找并更新与里程计节点（机器人当前位置）连通的最优中间导航节点
    if (!internav_near_nodes_.empty()) {
        std::sort(internav_near_nodes_.begin(), internav_near_nodes_.end(), nodeptr_icomp());
        //
        for (std::size_t i = 0; i < internav_near_nodes_.size(); i++) {
            const NavNodePtr temp_internav_ptr = internav_near_nodes_[i];
            // 确保节点和里程计节点有潜在连接
            if (FARUtil::IsTypeInStack(temp_internav_ptr, odom_node_ptr_->potential_edges) &&
                this->IsInternavInRange(temp_internav_ptr)) {
                if (cur_internav_ptr_ == NULL || temp_internav_ptr == cur_internav_ptr_ ||
                    (temp_internav_ptr->position - cur_internav_ptr_->position).norm() < FARUtil::kNearDist ||
                    FARUtil::IsTypeInStack(temp_internav_ptr, cur_internav_ptr_->connect_nodes)) {
                    this->UpdateCurInterNavNode(temp_internav_ptr);
                } else {
                    is_bridge_internav_ = true;
                }
                break;
            }
        }
    }
}

bool DynamicGraph::IsActivateNavNode(const NavNodePtr& node_ptr) {
    if (node_ptr->is_active) return true;
    if (FARUtil::IsPointNearNewPoints(node_ptr->position, true)) {
        node_ptr->is_active;
        return true;
    }
    if (FARUtil::IsFreeNavNode(node_ptr)) {
        const bool is_nearby =
            (node_ptr->position - odom_node_ptr_->position).norm() < FARUtil::kNearDist ? true : false;
        if (is_nearby) {
            node_ptr->is_active = true;
            return true;
        }
        if (FARUtil::IsTypeInStack(node_ptr, odom_node_ptr_->connect_nodes)) {
            node_ptr->is_active = true;
            return true;
        }
        bool is_connects_activate = true;
        for (const auto& cnode_ptr : node_ptr->connect_nodes) {
            if (!cnode_ptr->is_active) {
                is_connects_activate = false;
                break;
            }
        }
        if ((is_connects_activate && !node_ptr->connect_nodes.empty())) {
            node_ptr->is_active = true;
            return true;
        }
    }

    return false;
}

bool DynamicGraph::IsInterNavpointNecessary() {
    ROS_INFO("DG: IsInterNavpointNecessary called");
    if (cur_internav_ptr_ == NULL) {
        ROS_INFO("DG: cur_internav_ptr_ is NULL - creating new internav point");
        last_connect_pos_ = FARUtil::free_odom_p;
        return true;
    }
    const auto it = odom_node_ptr_->edge_votes.find(cur_internav_ptr_->id);
    if (is_bridge_internav_ || it == odom_node_ptr_->edge_votes.end() || !this->IsInternavInRange(cur_internav_ptr_)) {
        float min_dist = FARUtil::kINF;
        for (const auto& internav_ptr : internav_near_nodes_) {
            const float cur_dist = (internav_ptr->position - last_connect_pos_).norm();
            if (cur_dist < min_dist) min_dist = cur_dist;
        }
        ROS_INFO("min_dist:%.2f", min_dist);
        std::cout << "min_dist:" << min_dist << std::endl;
        std::cout << "min_dist < FARUtil::kINF:" << (min_dist < FARUtil::kINF) << std::endl;
        // // 添加检查：如果当前位置距离上次连接位置很近，不要创建新点
        // const float pos_change = (FARUtil::free_odom_p - last_connect_pos_).norm();
        // if (pos_change < FARUtil::kNearDist / 2.0f) {  // 添加位置变化检查
        //     return false;
        // }
        ROS_INFO("start IsInterNavpointNecessary");
        if (min_dist > FARUtil::kNavClearDist && min_dist < FARUtil::kINF) return true;
    }
    // 位置更新检查：机器人移动或投票触发
    if ((FARUtil::free_odom_p - last_connect_pos_).norm() > FARUtil::kNearDist ||
        (it != odom_node_ptr_->edge_votes.end() && it->second.back() == 1)) {
        last_connect_pos_ = FARUtil::free_odom_p;
    }
    return false;
}

bool DynamicGraph::ExtractGraphNodes(const CTNodeStack& new_ctnodes) {
    if (new_ctnodes.empty()) return false;
    new_nodes_.clear();
    NavNodePtr new_node_ptr = NULL;
    // 导航点
    ROS_INFO("start ExtractGraphNodes");
    if (IsInterNavpointNecessary()) {
        // if (false) {
        if (FARUtil::IsDebug) ROS_INFO("DG: One trajectory node has been created.");
        CreateNavNodeFromPoint(last_connect_pos_, new_node_ptr, false, true);
        new_nodes_.push_back(new_node_ptr);
        last_connect_pos_ = FARUtil::free_odom_p;
        if (is_bridge_internav_) is_bridge_internav_ = false;
    }
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
    if (node_ptr->is_finalized == true) {
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