#include "far_planner/dynamic_graph.h"

void DynamicGraph::Init(const ros::NodeHandle& nh, const DynamicGraphParams& params) {
    dg_params_ = params;
    id_tracker_ = 1;
    last_connect_pos_ = Point3D(0, 0, 0);
    CONNECT_ANGLE_COS = cos(dg_params_.kConnectAngleThred);
    NOISE_ANGLE_COS = cos(FARUtil::kAngleNoise);
    /*Initialize Terrain Planner*/
    tp_params_.world_frame = FARUtil::worldFrameId;
    tp_params_.voxel_size = FARUtil::kLeafSize;
    tp_params_.radius = FARUtil::kNearDist * 2.0f;
    tp_params_.inflate_size = FARUtil::kObsInflate;
    terrain_planner_.Init(nh, tp_params_);
}

void DynamicGraph::UpdateRobotPosition(const Point3D& robot_pos) {
    robot_pos_ = robot_pos;
    terrain_planner_.SetLocalTerrainObsCloud(FARUtil::local_terrain_obs_);
    if (odom_node_ptr_ == NULL) {
        this->CreateNavNodeFromPoint(robot_pos_, odom_node_ptr_, true);
        this->AddNodeToGraph(odom_node_ptr_);
        if (FARUtil::IsDebug) ROS_INFO("DG: Odom node has been initilaized.");
    } else {
        this->UpdateNodePosition(odom_node_ptr_, robot_pos_);
    }
    FARUtil::odom_pos = odom_node_ptr_->position;
    terrain_planner_.VisualPaths();
}

void DynamicGraph::UpdateNavGraph(
    const NodePtrStack& new_nodes, const bool& is_freeze_vgraph, NodePtrStack& clear_node) {
    clear_node.clear();
    // 清理无效的节点和验证轨迹连接
    if (!is_freeze_vgraph) {
        // 节点评估和清理
        // 检查节点是否有效
        // 更新节点的位置和表面方向
        // 验证轮廓匹配状态
        for (const auto& node_ptr : extend_match_nodes_) {
            if (FARUtil::IsStaticNode(node_ptr) || node_ptr == cur_internav_ptr_) continue;  // 避免误删重要节点
            if (!ReEvaluateCorner(node_ptr)) {                                               // 如果节点无效
                if (this->SetNodeToClear(node_ptr)) {  // 增加清理投票并检查是否到阈值
                    clear_node.push_back(node_ptr);
                }
            } else {
                this->ReduceDumperCounter(node_ptr);
            }
        }
        // 轨迹连接的地形验证
        // 检查范围surround_internav_nodes_
        // 条件：动态环境且存在当前中间导航点
        if (!FARUtil::IsStaticEnv && cur_internav_ptr_ != NULL) {
            NodePtrStack internav_check_nodes = surround_internav_nodes_;
            if (!FARUtil::IsTypeInStack(cur_internav_ptr_, internav_check_nodes)) {
                internav_check_nodes.push_back(cur_internav_ptr_);
            }
            for (const auto& sur_internav_ptr : internav_check_nodes) {
                const NodePtrStack copy_traj_connects = sur_internav_ptr->trajectory_connects;
                for (const auto& tnode_ptr : copy_traj_connects) {
                    // 地形验证通过，记录为有效
                    if (this->ReEvaluateConnectUsingTerrian(sur_internav_ptr, tnode_ptr)) {
                        this->RecordValidTrajEdge(sur_internav_ptr, tnode_ptr);
                    } else {
                        this->RemoveInValidTrajEdge(sur_internav_ptr, tnode_ptr);
                    }
                }
            }
        }
    }
    /*----------------------------------------    这里尚不明确   ----------------------------------------------*/
    // 清理merged点
    this->ClearMergedNodesInGraph();
    // 将和ct点匹配到的边缘节点margin_near_nodes_节点加入near_nav_nodes_和wide_near_nodes_中
    this->UpdateNearNodesWithMatchedMarginNodes(margin_near_nodes_, near_nav_nodes_, wide_near_nodes_);
    // 在wide_near_nodes_里检查和odom节点连接的点
    NodePtrStack codom_check_list = wide_near_nodes_;
    // 加入new nodes
    codom_check_list.insert(codom_check_list.end(), new_nodes.begin(), new_nodes.end());
    // 更新机器人当前位置（里程计节点）odom_node_ptr_与周围节点的连接关系
    for (const auto& conode_ptr : codom_check_list) {
        if (conode_ptr->is_odom) continue;
        // 根据连接的判断，加入poly_connects和connect_nodes
        if (this->IsValidConnect(odom_node_ptr_, conode_ptr, false)) {
            // 添加poly_connects                               添加connect_nodes
            this->AddPolyEdge(odom_node_ptr_, conode_ptr), this->AddEdge(odom_node_ptr_, conode_ptr);
        } else {
            ErasePolyEdge(odom_node_ptr_, conode_ptr), EraseEdge(odom_node_ptr_, conode_ptr);
        }
    }

    if (!is_freeze_vgraph) {
        // Adding new nodes to near nodes stack
        for (const auto& new_node_ptr : new_nodes) {
            this->AddNodeToGraph(new_node_ptr);
            new_node_ptr->is_near_nodes = true;
            near_nav_nodes_.push_back(new_node_ptr);
            // 在new_nodes中，is_navpoint基本一次只有一个，就是只会更新最多一次
            // 将新创建的is_navpoint点设为cur_internav_ptr_
            if (new_node_ptr->is_navpoint) this->UpdateCurInterNavNode(new_node_ptr);
            if (new_node_ptr->ctnode != NULL) {
                ContourGraph::MatchCTNodeWithNavNode(new_node_ptr->ctnode, new_node_ptr);
            }
        }
        // connect outrange contour nodes
        // 超出范围轮廓节点的动态重连管理
        for (const auto& out_node_ptr : out_contour_nodes_) {
            // 寻找匹配的近邻节点
            const NavNodePtr matched_node = ContourGraph::MatchOutrangeNodeWithCTNode(out_node_ptr, near_nav_nodes_);
            const auto it = out_contour_nodes_map_.find(out_node_ptr);
            if (matched_node != NULL) {
                // contour_votes投票
                this->RecordContourVote(out_node_ptr, matched_node);
                it->second.second.insert(matched_node);
            }
            for (const auto& reached_node_ptr : it->second.second) {
                if (reached_node_ptr != matched_node) {
                    // contour_votes的投票-1
                    this->DeleteContourVote(out_node_ptr, reached_node_ptr);
                }
            }
        }
        // near_nav_nodes_之间的连接
        NodePtrStack out_side_break_nodes;  // 记录断开的外部节点
        for (std::size_t i = 0; i < near_nav_nodes_.size(); i++) {
            const NavNodePtr nav_ptr1 = near_nav_nodes_[i];
            if (nav_ptr1->is_odom) continue;
            // 重新评估near_nav_nodes_的connect_nodes的连接
            const NodePtrStack copy_connect_nodes = nav_ptr1->connect_nodes;
            for (const auto& cnode : copy_connect_nodes) {
                // 过滤这些点，一个是机器人移动的点更新频繁，一个是跳过轮廓连接的点，防止重复验证
                if (cnode->is_odom || cnode->is_near_nodes || FARUtil::IsOutsideGoal(cnode) ||
                    FARUtil::IsTypeInStack(cnode, nav_ptr1->contour_connects))
                    continue;
                // 判断是否符合多边形连接
                if (this->IsValidConnect(nav_ptr1, cnode, false)) {
                    this->AddPolyEdge(nav_ptr1, cnode), this->AddEdge(nav_ptr1, cnode);
                } else {
                    this->ErasePolyEdge(nav_ptr1, cnode), this->EraseEdge(nav_ptr1, cnode);
                    out_side_break_nodes.push_back(cnode);
                }
            }
            // 评估near_nav_nodes_之间点的连接
            for (std::size_t j = 0; j < near_nav_nodes_.size(); j++) {
                const NavNodePtr nav_ptr2 = near_nav_nodes_[j];
                if (i == j || j > i || nav_ptr2->is_odom) continue;
                if (this->IsValidConnect(nav_ptr1, nav_ptr2, true)) {
                    this->AddPolyEdge(nav_ptr1, nav_ptr2), this->AddEdge(nav_ptr1, nav_ptr2);
                } else {
                    this->ErasePolyEdge(nav_ptr1, nav_ptr2), this->EraseEdge(nav_ptr1, nav_ptr2);
                }
            }
            // 对于断开的外部节点,判断是否通过轮廓连接
            for (const auto& oc_node_ptr : out_contour_nodes_) {
                // 如果两个都没匹配ct点
                if (!oc_node_ptr->is_contour_match || !nav_ptr1->is_contour_match) continue;
                if (ContourGraph::IsNavNodesConnectFromContour(nav_ptr1, oc_node_ptr)) {
                    // contour_votes的投票 + 1
                    this->RecordContourVote(nav_ptr1, oc_node_ptr);
                } else {
                    this->DeleteContourVote(nav_ptr1, oc_node_ptr);
                }
            }
            // 轮廓连接优化
            this->TopTwoContourConnector(nav_ptr1);
        }
        // update out range break nodes connects
        // 断开节点的恢复机制
        for (const auto& node_ptr : near_nav_nodes_) {
            for (const auto& ob_node_ptr : out_side_break_nodes) {
                if (this->IsValidConnect(node_ptr, ob_node_ptr, false)) {
                    this->AddPolyEdge(node_ptr, ob_node_ptr), this->AddEdge(node_ptr, ob_node_ptr);
                } else {
                    this->ErasePolyEdge(node_ptr, ob_node_ptr), this->EraseEdge(node_ptr, ob_node_ptr);
                }
            }
        }
        // Analysisig frontier nodes
        for (const auto& node_ptr : near_nav_nodes_) {
            if (this->IsNodeFullyCovered(node_ptr)) {
                node_ptr->is_covered = true;
            } else {
                node_ptr->is_covered = false;
            }

            if (this->IsFrontierNode(node_ptr)) {
                node_ptr->is_frontier = true;
            } else {
                node_ptr->is_frontier = false;
            }
        }
    }
}
/*
1. 基础几何检查
2. 轮廓连接验证 (如果is_check_contour=true)
3. 普通边连接验证
4. 投票结果评估
5. 轨迹连接检查
6. 紧密区域轮廓连接
 */
bool DynamicGraph::IsValidConnect(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& is_check_contour) {
    // 如果两个点的距离很近，可以连接
    const float dist = (node_ptr1->position - node_ptr2->position).norm();
    if (dist < FARUtil::kEpsilon) return true;
    // 特殊情况：里程计节点与导航点的近距离连接  ------------- 这里的逻辑貌似有问题？？？？？------------------------
    if ((node_ptr1->is_odom || node_ptr2->is_odom) && (node_ptr1->is_navpoint || node_ptr2->is_navpoint)) {
        if (dist < FARUtil::kNavClearDist) return true;
    }
    // 轮廓连接验证
    // 轮廓连接是基于环境的几何结构，想对宽松  普通的连接是根据几何的碰撞检测，方向约束，多边形碰撞等，非常严格
    if (is_check_contour) {
        // 判断两个导航节点是否可以通过它们对应的轮廓节点进行连接。
        // 检查边界节点间的特殊连接
        // 基于环境轮廓的拓扑连接性
        // 轮廓路径的地形可行性
        if (this->IsBoundaryConnect(node_ptr1, node_ptr2) ||
            (ContourGraph::IsNavNodesConnectFromContour(node_ptr1, node_ptr2) &&
                IsOnTerrainConnect(node_ptr1, node_ptr2, true))) {
            // contour_votes的投票+1
            this->RecordContourVote(node_ptr1, node_ptr2);
        } else if (node_ptr1->is_contour_match && node_ptr2->is_contour_match) {
            // contour_votes的投票-1
            this->DeleteContourVote(node_ptr1, node_ptr2);
        }
    }
    bool is_connect = false;
    /* check polygon connections */
    // 投票队列的大小
    // 里程计节点快速向应   普通的谨慎响应
    const int vote_queue_size =
        (node_ptr1->is_odom || node_ptr2->is_odom) ? std::ceil(dg_params_.votes_size / 3.0f) : dg_params_.votes_size;
    // 四重验证“
    // 确保连接的不是凹点    确保连接方向符合节点的表面方向约束  进行多边形碰撞检测，检查两点的连线是否和多边形碰撞
    // 地形坡度和高度验证,确保路径地形可行
    if (IsConvexConnect(node_ptr1, node_ptr2) && this->IsInDirectConstraint(node_ptr1, node_ptr2) &&
        ContourGraph::IsNavNodesConnectFreePolygon(node_ptr1, node_ptr2) &&
        IsOnTerrainConnect(node_ptr1, node_ptr2, false)) {
        if (IsPolyMatchedForConnect(node_ptr1, node_ptr2)) {  // 过滤稳定节点  基于节点状态而非几何条件
            // edge_votes投票加1，并添加两节点间的potential_edges
            RecordPolygonVote(node_ptr1, node_ptr2, vote_queue_size);
        }
    } else {
        DeletePolygonVote(node_ptr1, node_ptr2, vote_queue_size);  // 清除edge_votes
    }
    // edge_votes如果达到投票阈值
    if (this->IsPolygonEdgeVoteTrue(node_ptr1, node_ptr2)) {
        if (!this->IsSimilarConnectInDiection(node_ptr1, node_ptr2)) is_connect = true;
        // 对于导航节点要快速连接和排出连接
    } else if (node_ptr1->is_odom || node_ptr2->is_odom) {
        node_ptr1->edge_votes.erase(node_ptr2->id);
        node_ptr2->edge_votes.erase(node_ptr1->id);
        FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->potential_edges);
        FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->potential_edges);
    }
    // 在连接失败后给一个额外的机会
    /* check if exsiting trajectory connection exist */
    if (!is_connect) {
        // 如果已有轨迹连接，直接认定为可连接
        if (FARUtil::IsTypeInStack(node_ptr1, node_ptr2->trajectory_connects)) is_connect = true;
        // 机器人轨迹连接  检查目标节点是否在当前中间导航点的轨迹连接列表中 基于历史轨迹的可达性验证
        if ((node_ptr1->is_odom || node_ptr2->is_odom) && cur_internav_ptr_ != NULL) {
            if (node_ptr1->is_odom && FARUtil::IsTypeInStack(node_ptr2, cur_internav_ptr_->trajectory_connects)) {
                if (FARUtil::IsInCylinder(
                        cur_internav_ptr_->position, node_ptr2->position, node_ptr1->position, FARUtil::kNearDist)) {
                    is_connect = true;
                }
            } else if (node_ptr2->is_odom &&
                       FARUtil::IsTypeInStack(node_ptr1, cur_internav_ptr_->trajectory_connects)) {
                if (FARUtil::IsInCylinder(
                        cur_internav_ptr_->position, node_ptr1->position, node_ptr2->position, FARUtil::kNearDist)) {
                    is_connect = true;
                }
            }
        }
    }
    /* check for additional contour connection through tight area from current robot position */
    // 紧密区域轮廓连接 如果前面连接全都失效  通过轮廓节点实现间接连接
    // 条件：1 设计is_odom；  2 都不是凹点；  3 方向约束通过
    if (!is_connect && (node_ptr1->is_odom || node_ptr2->is_odom) && IsConvexConnect(node_ptr1, node_ptr2) &&
        IsInDirectConstraint(node_ptr1, node_ptr2)) {
        // 遍历轮廓节点
        if (node_ptr1->is_odom && !node_ptr2->contour_connects.empty()) {
            for (const auto& ctnode_ptr : node_ptr2->contour_connects) {
                if (FARUtil::IsInCylinder(
                        ctnode_ptr->position, node_ptr2->position, node_ptr1->position, FARUtil::kNavClearDist)) {
                    is_connect = true;
                }
            }
        } else if (node_ptr2->is_odom && !node_ptr1->contour_connects.empty()) {
            for (const auto& ctnode_ptr : node_ptr1->contour_connects) {
                if (FARUtil::IsInCylinder(
                        ctnode_ptr->position, node_ptr1->position, node_ptr2->position, FARUtil::kNavClearDist)) {
                    is_connect = true;
                }
            }
        }
    }
    return is_connect;
}

bool DynamicGraph::IsNodeFullyCovered(const NavNodePtr& node_ptr) {
    if (FARUtil::IsFreeNavNode(node_ptr) || node_ptr->is_covered) return true;
    NodePtrStack check_odom_list = internav_near_nodes_;
    check_odom_list.push_back(odom_node_ptr_);
    for (const auto& near_optr : check_odom_list) {
        const float cur_dist = (node_ptr->position - near_optr->position).norm();
        if (cur_dist < FARUtil::kMatchDist) return true;
        // 视线覆盖检查
        // 非柱状节点；与机器人/导航点有有效连接；机器人位置在节点的覆盖方向范围内
        if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            // TODO: concave nodes will not be marked as covered based on current implementation
            const auto it = near_optr->edge_votes.find(node_ptr->id);
            if (it != near_optr->edge_votes.end() && FARUtil::IsVoteTrue(it->second)) {
                const Point3D diff_p = near_optr->position - node_ptr->position;
                if (FARUtil::IsInCoverageDirPairs(diff_p, node_ptr)) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool DynamicGraph::IsFrontierNode(const NavNodePtr& node_ptr) {
    if (node_ptr->is_contour_match) {
        // 如果不是被阻塞的边界，不被覆盖，是凸点，轮廓周长长
        if (node_ptr->is_block_frontier || node_ptr->is_covered || node_ptr->free_direct != NodeFreeDirect::CONVEX ||
            node_ptr->ctnode->poly_ptr->perimeter < dg_params_.frontier_perimeter_thred) {
            node_ptr->frontier_votes.push_back(0);  // non convex frontier or too small
        } else {
            node_ptr->frontier_votes.push_back(1);  // convex frontier
        }
        // 超出边界范围的节点不会被删除，但不作为边界节点
    } else if (!FARUtil::IsPointInMarginRange(
                   node_ptr->position)) {       // if not in margin range, the node won't be deleted
        node_ptr->frontier_votes.push_back(0);  // non convex frontier
    }
    // 投票队列管理
    if (node_ptr->frontier_votes.size() > dg_params_.finalize_thred) {
        node_ptr->frontier_votes.pop_front();
    }
    // 最终边界判断
    bool is_frontier = FARUtil::IsVoteTrue(node_ptr->frontier_votes);
    if (!node_ptr->is_frontier && is_frontier && node_ptr->frontier_votes.size() == dg_params_.finalize_thred) {
        if (!FARUtil::IsPointNearNewPoints(node_ptr->position, true)) {
            is_frontier = false;
        }
    }
    return is_frontier;
}

bool DynamicGraph::IsOnTerrainConnect(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& is_contour) {
    // 验证两个节点间连接的地形可行性，包括坡度检查、高度验证和地形匹配
    if (!node_ptr1->is_active || !node_ptr2->is_active) return true;
    // 坡度安全检查  是否超过45度  距离足够大才进行坡度检测，防止噪声干扰
    Point3D mid_p = (node_ptr1->position + node_ptr2->position) / 2.0f;
    const Point3D diff_p = node_ptr2->position - node_ptr1->position;
    if (diff_p.norm() > FARUtil::kMatchDist && abs(diff_p.z) / std::hypotf(diff_p.x, diff_p.y) > 1) {
        // 记录无效的地形连接  terrain_votes +1 投一票
        if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
        return false;  // slope is too steep > 45 degree
    }
    // 如果轮廓连接而且已经有了投票记录直接通过
    if (is_contour && node_ptr1->contour_votes.find(node_ptr2->id) !=
                          node_ptr1->contour_votes.end()) {  // recorded contour terrain connection
        return true;
    }
    // 地形高度匹配验证
    bool is_match;
    float minH, maxH;
    const float avg_h = MapHandler::NearestHeightOfRadius(mid_p, FARUtil::kMatchDist, minH, maxH, is_match);
    // 匹配失败  如果地形数据不匹配且不是边界节点间连接，则失败
    // 边界节点（frontier nodes）在地形边缘，允许一定的地形数据缺失
    if (!is_match && (is_contour || !node_ptr1->is_frontier || !node_ptr2->is_frontier)) {
        // 记录无效的地形连接  terrain_votes +1 投一票
        if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
        return false;
    }
    // 如果匹配成功，但是高度差太大 还是不通过
    if (is_match && (maxH - minH > FARUtil::kMarginHeight ||
                        abs(minH + FARUtil::vehicle_height - mid_p.z) > FARUtil::kTolerZ / 2.0f)) {
        // 记录无效的地形连接  terrain_votes +1 投一票
        if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
        return false;
    }
    // 地形连接投票管理（非轮廓连接）
    if (!is_contour) {
        // 如果匹配成功，直接terrain_votes -1 记录有效的地形连接，减少失效计数
        if (is_match) RecordVaildTerrainConnect(node_ptr1, node_ptr2);
        const auto it = node_ptr1->terrain_votes.find(node_ptr2->id);
        if (it != node_ptr1->terrain_votes.end() && it->second > dg_params_.finalize_thred) {
            return false;
        }
    }
    return true;
}

bool DynamicGraph::IsInDirectConstraint(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    // check for odom -> frontier connections
    if ((node_ptr1->is_odom && node_ptr2->is_frontier) || (node_ptr2->is_odom && node_ptr1->is_frontier)) return true;
    // check node1 -> node2
    if (node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_1to2 = (node_ptr2->position - node_ptr1->position);
        if (!FARUtil::IsOutReducedDirs(diff_1to2, node_ptr1->surf_dirs)) {
            return false;
        }
    }
    // check node1 -> node2
    if (node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_2to1 = (node_ptr1->position - node_ptr2->position);
        if (!FARUtil::IsOutReducedDirs(diff_2to1, node_ptr2->surf_dirs)) {
            return false;
        }
    }
    return true;
}

void DynamicGraph::TopTwoContourConnector(const NavNodePtr& node_ptr) {
    // 收集有效的投票分数，计算每个连接的总投票分数
    std::vector<int> votesc;
    for (const auto& vote : node_ptr->contour_votes) {
        if (FARUtil::IsVoteTrue(vote.second, false)) {
            votesc.push_back(std::accumulate(vote.second.begin(), vote.second.end(), 0));
        }
    }
    // 对连击分数排序
    std::sort(votesc.begin(), votesc.end(), std::greater<int>());
    // 筛选所有的潜在的轮廓连接
    for (const auto& cnode_ptr : node_ptr->potential_contours) {
        const auto it = node_ptr->contour_votes.find(cnode_ptr->id);
        // DEBUG
        if (it == node_ptr->contour_votes.end()) ROS_ERROR("DG: contour potential node matching error");
        const int itc = std::accumulate(it->second.begin(), it->second.end(), 0);
        // 如果排前两名且投票过阈值，就增加contour_connects连接，添加connect_nodes
        if (FARUtil::VoteRankInVotes(itc, votesc) < 2 && FARUtil::IsVoteTrue(it->second, false)) {
            DynamicGraph::AddContourConnect(node_ptr, cnode_ptr);
            this->AddEdge(node_ptr, cnode_ptr);
            // 否则从contour_connects里删除连接，如果还不是poly_connects，则从connect_nodes清除
        } else if (DynamicGraph::DeleteContourConnect(node_ptr, cnode_ptr) &&
                   !FARUtil::IsTypeInStack(cnode_ptr, node_ptr->poly_connects)) {
            this->EraseEdge(node_ptr, cnode_ptr);
        }
    }
}

void DynamicGraph::RecordContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    if (FARUtil::IsDebug) {
        if ((it1 == node_ptr1->contour_votes.end()) != (it2 == node_ptr2->contour_votes.end())) {
            ROS_ERROR_THROTTLE(1.0, "DG: Critical! Contour edge votes queue error.");
        }
    }
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) {
        // init contour connection votes
        std::deque<int> vote_queue1, vote_queue2;
        vote_queue1.push_back(1), vote_queue2.push_back(1);
        node_ptr1->contour_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->contour_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_contours) &&
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_contours)) {
            node_ptr1->potential_contours.push_back(node_ptr2);
            node_ptr2->potential_contours.push_back(node_ptr1);
        }
    } else {
        if (FARUtil::IsDebug) {
            if (it1->second.size() != it2->second.size())
                ROS_ERROR_THROTTLE(1.0, "DG: contour connection votes are not equal.");
        }
        it1->second.push_back(1), it2->second.push_back(1);
        if (it1->second.size() > dg_params_.votes_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
    }
}

void DynamicGraph::DeleteContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end())
        return;  // no connection (not counter init) in the first place
    it1->second.push_back(0), it2->second.push_back(0);
    if (it1->second.size() > dg_params_.votes_size) {
        it1->second.pop_front(), it2->second.pop_front();
    }
}

bool DynamicGraph::IsSimilarConnectInDiection(const NavNodePtr& node_ptr_from, const NavNodePtr& node_ptr_to) {
    // TODO: check for connection loss
    // 特殊节点豁免  里程计节点（机器人位置）总是允许建立新连接
    if (node_ptr_from->is_odom || node_ptr_to->is_odom) return false;
    // 轮廓连接不受冗余检查限制
    if (FARUtil::IsTypeInStack(node_ptr_to, node_ptr_from->contour_connects)) {
        return false;
    }
    // 双向冗余检查
    if (this->IsAShorterConnectInDir(node_ptr_from, node_ptr_to)) {
        return true;
    }
    if (this->IsAShorterConnectInDir(node_ptr_to, node_ptr_from)) {
        return true;
    }
    return false;
}

bool DynamicGraph::IsAShorterConnectInDir(const NavNodePtr& node_ptr_from, const NavNodePtr& node_ptr_to) {
    bool is_nav_connect = false;
    bool is_cover_connect = false;
    // 导航点只和导航点比较，完全覆盖点只和完全覆盖点比较
    if (node_ptr_from->is_navpoint && node_ptr_to->is_navpoint) is_nav_connect = true;
    if (node_ptr_from->is_covered && node_ptr_to->is_covered) is_cover_connect = true;
    // 如果这个点没有连接过其他点，就不冗余
    if (node_ptr_from->connect_nodes.empty()) return false;
    Point3D ref_dir, ref_diff;
    const Point3D diff_p = node_ptr_to->position - node_ptr_from->position;
    const Point3D connect_dir = diff_p.normalize();
    const float dist = diff_p.norm();
    for (const auto& cnode : node_ptr_from->connect_nodes) {
        if (is_nav_connect && !cnode->is_navpoint) continue;
        if (is_cover_connect && !cnode->is_covered) continue;
        if (FARUtil::IsTypeInStack(cnode, node_ptr_from->contour_connects))
            continue;  // 轮廓连接基于环境结构，有特殊价值
        ref_diff = cnode->position - node_ptr_from->position;
        if (cnode->is_odom || ref_diff.norm() < FARUtil::kEpsilon) continue;
        ref_dir = ref_diff.normalize();
        // 方向相似性检测  如果方向相同，已经存在同方向的连接
        if ((connect_dir * ref_dir) > CONNECT_ANGLE_COS && dist > ref_diff.norm()) {
            return true;
        }
    }
    // 允许不同方向的连接
    return false;
}

// 重新评估导航节点的有效性和更新状态，如果无效就清理
// 返回true表示将诶点依然有效，返回false表示无效，多次要被清理
bool DynamicGraph::ReEvaluateCorner(const NavNodePtr node_ptr) {
    if (node_ptr->is_boundary) return true;
    // 导航点的处理：如果在障碍物内就无效，清理
    if (node_ptr->is_navpoint) {
        if (FARUtil::IsTypeInStack(node_ptr, surround_internav_nodes_) && this->IsNodeInTerrainOccupy(node_ptr)) {
            return false;
        }
        return true;
    }
    // 节点附近是否有环境变化响应,如果有环境变化，就重置滤波器和连接投票   出发重新评估机制
    const bool is_near_new = FARUtil::IsPointNearNewPoints(node_ptr->position, false);
    if (is_near_new) {
        this->ResetNodeFilters(node_ptr);
        if (!node_ptr->is_contour_match) ResetNodeConnectVotes(node_ptr);
    }
    // 对非轮廓匹配节点的处理  在传感器范围内未被匹配到的无效或附近环境变化则无效
    if (!node_ptr->is_contour_match) {
        if (FARUtil::IsPointInMarginRange(node_ptr->position) || is_near_new) return false;
        return true;
    }
    // 已完成节点直接返回
    if (node_ptr->is_finalized) return true;

    // 对于本次匹配到ct点的导航点
    bool is_pos_cov = false;
    bool is_dirs_cov = false;
    if (node_ptr->is_contour_match) {
        is_pos_cov = this->UpdateNodePosition(node_ptr, node_ptr->ctnode->position);
        is_dirs_cov = this->UpdateNodeSurfDirs(node_ptr, node_ptr->ctnode->surf_dirs);
        if (FARUtil::IsDebug)
            ROS_ERROR_COND(node_ptr->free_direct == NodeFreeDirect::UNKNOW, "DG: node free space is unknown.");
    }
    if (is_pos_cov && is_dirs_cov) node_ptr->is_finalized = true;
    return true;
}

void DynamicGraph::DeletePolygonVote(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const int& queue_size, const bool& is_reset) {
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) return;
    if (is_reset) it1->second.clear(), it2->second.clear();
    it1->second.push_back(0), it2->second.push_back(0);
    if (it1->second.size() > queue_size) {
        it1->second.pop_front(), it2->second.pop_front();
    }
}

void DynamicGraph::RecordPolygonVote(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const int& queue_size, const bool& is_reset) {
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (FARUtil::IsDebug) {
        if ((it1 == node_ptr1->edge_votes.end()) != (it2 == node_ptr2->edge_votes.end())) {
            ROS_ERROR_THROTTLE(1.0, "DG: Critical! Polygon edge votes queue error.");
        }
    }
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) {
        // init polygon edge votes
        std::deque<int> vote_queue1, vote_queue2;
        vote_queue1.push_back(1), vote_queue2.push_back(1);
        node_ptr1->edge_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->edge_votes.insert({node_ptr1->id, vote_queue2});
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_edges) &&
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_edges)) {
            node_ptr1->potential_edges.push_back(node_ptr2);
            node_ptr2->potential_edges.push_back(node_ptr1);
        }
    } else {
        if (FARUtil::IsDebug) {
            if (it1->second.size() != it2->second.size())
                ROS_ERROR_THROTTLE(1.0, "DG: Polygon edge votes are not equal.");
        }
        if (is_reset) it1->second.clear(), it2->second.clear();
        it1->second.push_back(1), it2->second.push_back(1);
        if (it1->second.size() > queue_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
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
    // ROS_INFO("DG: IsInterNavpointNecessary called");
    if (cur_internav_ptr_ == NULL) {
        // ROS_INFO("DG: cur_internav_ptr_ is NULL - creating new internav point");
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

        // // 添加检查：如果当前位置距离上次连接位置很近，不要创建新点
        // const float pos_change = (FARUtil::free_odom_p - last_connect_pos_).norm();
        // if (pos_change < FARUtil::kNearDist / 2.0f) {  // 添加位置变化检查
        //     return false;
        // }

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
    // ROS_INFO("start ExtractGraphNodes");
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
    if (node_ptr->is_finalized) return true;
    node_ptr->pos_filter_vec.push_back(new_pos);
    if (node_ptr->pos_filter_vec.size() > dg_params_.pool_size) {
        node_ptr->pos_filter_vec.pop_front();
    }
    // 用RANSACS计算平均的位置
    std::size_t inlier_size = 0;
    Point3D mean_p = FARUtil::RANSACPoisiton(node_ptr->pos_filter_vec, dg_params_.filter_pos_margin, inlier_size);
    if (node_ptr->pos_filter_vec.size() > 1) mean_p.z = node_ptr->position.z;  // keep z value with terrain updates
    node_ptr->position = mean_p;
    if (inlier_size > dg_params_.finalize_thred) {
        return true;
    }
    return false;
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

bool DynamicGraph::ReEvaluateConnectUsingTerrian(const NavNodePtr& node_ptr1, const NavNodePtr node_ptr2) {
    PointStack terrain_path;
    if (terrain_planner_.PlanPathFromNodeToNode(node_ptr1, node_ptr2, terrain_path)) {
        return true;
    }
    return false;
}
