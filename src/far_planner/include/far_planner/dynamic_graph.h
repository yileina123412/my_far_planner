#ifndef DYNAMIC_GRAPH_H
#define DYNAMIC_GRAPH_H
#pragma once
#include "contour_graph.h"
#include "map_handler.h"
#include "terrain_planner.h"
#include "utility.h"
struct DynamicGraphParams {
    DynamicGraphParams() = default;
    int dumper_thred;                // 节点清理阈值
    int finalize_thred;              // 确认被里那接的阈值
    int pool_size;                   // RASCAN滤波池大小
    int votes_size;                  // 投票队列大小
    float kConnectAngleThred;        // 连接角度阈值
    float filter_pos_margin;         // 位置滤波容差
    float filter_dirs_margin;        // 方向滤波容差
    float frontier_perimeter_thred;  // 边界周长阈值
};

class DynamicGraph {
private:
    Point3D robot_pos_;
    NavNodePtr odom_node_ptr_ = NULL;
    NavNodePtr cur_internav_ptr_ = NULL;   // 中间导航节点 表示正在导航到的目标节点
    NavNodePtr last_internav_ptr_ = NULL;  // 上一个中间导航节点
    NodePtrStack new_nodes_;  // ExtractGraphNodes中每帧新提取的导航点，还未和全剧导航点连接的导航点
    // 不同的局部节点集合
    /**
     * near_nav_nodes_:近邻导航节点：在局部范围内且已激活或成为边界节点  核心的局部导航节点，用于路径规划和连接性检查
     * wide_near_nodes_:宽近邻近节点：比邻近节点更大的范围
     * 在局部范围内且在地形上的所有节点,范围大于near_nav_nodes_，实际边连接和导航 需要地形约束
     * extend_match_nodes_：扩展匹配节点：最大的处理范围  传感器范围，用于重新评估和角点检测
     * margin_near_nodes_：边缘节点：不再局部范围但仍需要处理的节点，is_active || is_boundary 但不在局部范围内
     * 处理边界情况
     * internav_near_nodes_：中间导航节点：近邻节点中的导航点     长距离路径规划
     * surround_internav_nodes_：周围导航节点：机器人局部规划范围内的最近的导航节点   近距离轨迹规划
     */
    NodePtrStack near_nav_nodes_, wide_near_nodes_, extend_match_nodes_, margin_near_nodes_;
    NodePtrStack internav_near_nodes_, surround_internav_nodes_;
    // 处理超出处理范围但仍有价值的点，实现延迟删除和智能重连 存储当前超出处理范围但仍然活跃的轮廓节点的快速访问列表。
    NodePtrStack out_contour_nodes_;
    float CONNECT_ANGLE_COS, NOISE_ANGLE_COS;
    // 是否需要创建一个临时的桥接导航点，来连接当前位置和心的目标导航点
    bool is_bridge_internav_ = false;
    Point3D last_connect_pos_;  // 上次创建轨迹连接时机器人的位置

    static DynamicGraphParams dg_params_;
    static std::size_t id_tracker_;
    static NodePtrStack globalGraphNodes_;  // 存储全部的导航节点
    static std::unordered_map<std::size_t, NavNodePtr> idx_node_map_;
    // 处理超出处理范围但仍有价值的点，实现延迟删除和智能重连
    //                             超范围节点    生存计数器    历史连接节点集合
    static std::unordered_map<NavNodePtr, std::pair<int, std::unordered_set<NavNodePtr>>> out_contour_nodes_map_;

    TerrainPlanner terrain_planner_;
    TerrainPlannerParams tp_params_;
    // 采用多重验证判断两个导航点之间是否可以建立连接
    // is_check_contour
    bool IsValidConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& is_check_contour);
    // 验证连接方向符合节点的表面方向约束
    // 就是如果节点的连接方向在表面方向的允许的范围内，就是不指向障碍物内部返回true
    bool IsInDirectConstraint(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    // 是否需要创建一个中间导航节点
    bool IsInterNavpointNecessary();
    // 判断一个导航点是否应该被激活
    bool IsActivateNavNode(const NavNodePtr& node_ptr);
    // 更新节点的位置 pos_filter_vec 通过多次观测，使用RANSAC算法逐步收敛节点位置到最准确的估计值
    bool UpdateNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos);

    static void InitNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos);
    // 更新导航节点的表面方向信息
    bool UpdateNodeSurfDirs(const NavNodePtr& node_ptr, PointPair cur_dirs);
    // 重新评估凹凸性
    void ReEvaluateConvexity(const NavNodePtr& node_ptr);
    // 检测和避免冗余
    // 判断两个节点间是否已经存在更短或更优的连接路径，避免建立冗余连接。
    bool IsSimilarConnectInDiection(const NavNodePtr& node_ptr_from, const NavNodePtr& node_ptr_to);
    // 判断两个节点连接是否冗余，通过检查是否已有更短的路径连接相同方向的节点来优化图结构
    // 检查是否存在更短的同方向连接
    bool IsAShorterConnectInDir(const NavNodePtr& node_ptr_from, const NavNodePtr& node_ptr_to);
    // 重新评估导航节点的有效性和更新状态，如果无效就清理
    // 返回true表示将诶点依然有效，返回false表示无效，多次要被清理
    bool ReEvaluateCorner(const NavNodePtr node_ptr);
    /**
     * 记录并建立连接投票
     * contour_votes的投票+1，创建投票
     * 清理多的投票记录
     */
    void RecordContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);
    /**
     * contour_votes的投票-1
     * 记录这一次没有连接，投一票未连接
     */
    void DeleteContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);
    // 优化轮廓连接数量，通过投票机制只保留每个节点最优的前2个轮廓连接，避免过度连接导致的性能问题
    void TopTwoContourConnector(const NavNodePtr& node_ptr);
    // 判断节点是否被完全覆盖is_covered
    bool IsNodeFullyCovered(const NavNodePtr& node_ptr);
    // 判断是不是前沿点
    bool IsFrontierNode(const NavNodePtr& node_ptr);
    // 使用Terrian评估连通性
    bool ReEvaluateConnectUsingTerrian(const NavNodePtr& node_ptr1, const NavNodePtr node_ptr2);

    inline bool IsNodeInTerrainOccupy(const NavNodePtr& node_ptr) {
        if (!FARUtil::IsStaticEnv && terrain_planner_.IsPointOccupy(node_ptr->position)) return true;
        return false;
    }

    // 更新cur_internav_ptr_，并建立轨迹连接
    inline void UpdateCurInterNavNode(const NavNodePtr& internav_node_ptr) {
        if (internav_node_ptr == NULL || !internav_node_ptr->is_navpoint) return;
        cur_internav_ptr_ = internav_node_ptr;
        if (last_internav_ptr_ == NULL) {  // init inter navigation nodes
            terrain_planner_.UpdateCenterNode(cur_internav_ptr_);
            last_internav_ptr_ = cur_internav_ptr_;
        } else if (last_internav_ptr_ != cur_internav_ptr_) {
            terrain_planner_.UpdateCenterNode(cur_internav_ptr_);
            this->AddTrajectoryConnect(cur_internav_ptr_, last_internav_ptr_);
            last_internav_ptr_ = cur_internav_ptr_;
        }
    }
    // 重置节点的pos_filter_vec和surf_dirs_vec
    inline void ResetNodeFilters(const NavNodePtr& node_ptr) {
        node_ptr->is_finalized;
        node_ptr->pos_filter_vec.clear();
        node_ptr->surf_dirs_vec.clear();
    }
    // 重置轮廓投票 候选边
    inline void ResetContourVotes(const NavNodePtr& node_ptr) {
        for (const auto& pcnode : node_ptr->potential_contours) {
            const auto it1 = node_ptr->contour_votes.find(pcnode->id);
            const auto it2 = pcnode->contour_votes.find(node_ptr->id);
            if (FARUtil::IsVoteTrue(it1->second, false)) {
                it1->second.clear(), it1->second.push_back(1);
                it2->second.clear(), it2->second.push_back(1);
            } else {
                it1->second.clear(), it1->second.push_back(0);
                it2->second.clear(), it2->second.push_back(0);
            }
        }
    }
    // 重置多边形投票  贴墙的边
    inline void ResetPolygonVotes(const NavNodePtr& node_ptr) {
        for (const auto& pcnode : node_ptr->potential_edges) {
            const auto it1 = node_ptr->edge_votes.find(pcnode->id);
            const auto it2 = pcnode->edge_votes.find(node_ptr->id);
            if (FARUtil::IsVoteTrue(it1->second)) {
                it1->second.clear(), it1->second.push_back(1);
                it2->second.clear(), it2->second.push_back(1);
            } else {
                it1->second.clear(), it1->second.push_back(0);
                it2->second.clear(), it2->second.push_back(0);
            }
        }
    }
    // 重置potential_contours和potential_edges及其投票
    inline void ResetNodeConnectVotes(const NavNodePtr& node_ptr) {
        ResetContourVotes(node_ptr);
        ResetPolygonVotes(node_ptr);
    }
    // 在两个导航节点之间建立双向轨迹连接trajectory_connects 以及投票记录
    inline void AddTrajectoryConnect(const NavNodePtr& cur_node, const NavNodePtr& last_node) {
        if (last_node == NULL || cur_node == NULL) return;
        if (!FARUtil::IsTypeInStack(last_node, cur_node->trajectory_connects) &&
            !FARUtil::IsTypeInStack(cur_node, last_node->trajectory_connects)) {
            cur_node->trajectory_votes.insert({last_node->id, 0});
            last_node->trajectory_votes.insert({cur_node->id, 0});
            cur_node->trajectory_connects.push_back(last_node);
            last_node->trajectory_connects.push_back(cur_node);
        }
    }
    // trajectory_votes验证有效进行投票，就是减一次无效次数
    inline void RecordValidTrajEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->trajectory_votes.find(node_ptr2->id);
        const auto it2 = node_ptr2->trajectory_votes.find(node_ptr1->id);

        if (it1->second > 0) {
            it1->second--, it2->second--;
        }
    }
    // 轨迹连接无小，增加trajectory_votes并判断阈值是否移除轨迹连接
    inline void RemoveInValidTrajEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->trajectory_votes.find(node_ptr2->id);
        const auto it2 = node_ptr2->trajectory_votes.find(node_ptr1->id);
        if (FARUtil::IsDebug) {
            if (it1 == node_ptr1->trajectory_votes.end() || it2 == node_ptr2->trajectory_votes.end() ||
                it1->second != it2->second) {
                ROS_ERROR("DG: Trajectory votes queue error.");
                return;
            }
        }
        it1->second++, it2->second++;
        if (it1->second > dg_params_.finalize_thred) {  // clear trajectory connections and votes
            if (FARUtil::IsDebug) ROS_WARN("DG: Current trajectory edge disconnected, no traversable path found.");
            node_ptr1->trajectory_votes.erase(node_ptr2->id);
            FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->trajectory_connects);

            node_ptr2->trajectory_votes.erase(node_ptr1->id);
            FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->trajectory_connects);
        }
    }
    // 判断两个节点是否适合建立多边形连接
    // 如果任一节点满足以下三个条件同时成立，则拒绝连接
    // 保护稳定的节点，减少对稳定节点的频繁修改
    inline bool IsPolyMatchedForConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if ((node_ptr1->is_finalized && !node_ptr1->is_contour_match && !FARUtil::IsFreeNavNode(node_ptr1)) ||
            (node_ptr2->is_finalized && !node_ptr2->is_contour_match && !FARUtil::IsFreeNavNode(node_ptr2))) {
            return false;
        }
        return true;
    }
    // 基于历史投票记录判断两个节点之间的多边形连接edge_votes是否可靠
    // 对edge_votes的投票结果评估，如果edge_votes达到阈值返回true
    inline bool IsPolygonEdgeVoteTrue(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
        const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
        if (it1 != node_ptr1->edge_votes.end() && it2 != node_ptr2->edge_votes.end()) {
            if (FARUtil::IsVoteTrue(it1->second)) {
                if (FARUtil::IsDebug && !FARUtil::IsVoteTrue(it2->second))
                    ROS_ERROR_THROTTLE(1.0, "DG: Polygon edge vote result are not matched.");
                if (IsNodeDirectConnect(node_ptr1, node_ptr2) || it1->second.size() > 2) {
                    return true;
                }
            }
        }
        return false;
    }
    // 判断两个点是否属于可以直接连接的特殊类型
    // 特权节点无需复杂的严格的投票可以快速连接
    inline bool IsNodeDirectConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (FARUtil::IsFreeNavNode(node_ptr1) || FARUtil::IsFreeNavNode(node_ptr2)) return true;
        if (node_ptr1->is_contour_match || node_ptr2->is_contour_match) return true;
        return false;
    }
    // 增加contour_connects连接
    static inline void AddContourConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->contour_connects) &&
            !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->contour_connects)) {
            node_ptr1->contour_connects.push_back(node_ptr2);
            node_ptr2->contour_connects.push_back(node_ptr1);
            ContourGraph::AddContourToSets(node_ptr1, node_ptr2);
        }
    }
    // 从contour_connects里删除连接
    static inline bool DeleteContourConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->contour_connects)) return false;

        FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->contour_connects);
        FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->contour_connects);
        ContourGraph::DeleteContourFromSets(node_ptr1, node_ptr2);
        return true;
    }

    // clear_dumper_count 清理的投票增加并判断是否需要清理
    // 返回true达到清理的阈值且is_merged = true
    inline bool SetNodeToClear(const NavNodePtr& node_ptr) {
        if (FARUtil::IsStaticNode(node_ptr)) return false;
        node_ptr->clear_dumper_count++;
        const int N = node_ptr->is_navpoint ? dg_params_.dumper_thred * 2 : dg_params_.dumper_thred;
        if (node_ptr->clear_dumper_count > N) {
            node_ptr->is_merged = true;
            return true;
        }
        return false;
    }
    // clear_dumper_count 清理的投票减少
    inline void ReduceDumperCounter(const NavNodePtr& node_ptr) {
        if (FARUtil::IsStaticNode(node_ptr)) return;
        node_ptr->clear_dumper_count--;
        if (node_ptr->clear_dumper_count < 0) {
            node_ptr->clear_dumper_count = 0;
        }
    }

    inline void CreateNewNavNodeFromContour(const CTNodePtr& ctnode_ptr, NavNodePtr& node_ptr) {
        CreateNavNodeFromPoint(ctnode_ptr->position, node_ptr, false);
        node_ptr->is_contour_match = true;
        node_ptr->ctnode = ctnode_ptr;
        node_ptr->free_direct = ctnode_ptr->free_direct;
        UpdateNodeSurfDirs(node_ptr, ctnode_ptr->surf_dirs);
    }
    // 检查两节点是否是contour_connects
    inline bool IsBoundaryConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
            if (FARUtil::IsTypeInStack(node_ptr2, node_ptr1->contour_connects)) {
                return true;
            }
        }
        return false;
    }

    // 是否值的作为新导航点
    inline bool IsAValidNewNode(const CTNodePtr ctnode_ptr, bool& is_near_new) {
        is_near_new = FARUtil::IsPointNearNewPoints(ctnode_ptr->position, true);
        if (ctnode_ptr->is_contour_necessary || is_near_new) {
            if (MapHandler::IsNavPointOnTerrainNeighbor(ctnode_ptr->position, false) &&
                IsPointOnTerrain(ctnode_ptr->position)) {
                return true;
            } else if (ctnode_ptr->is_contour_necessary) {
                ctnode_ptr->is_contour_necessary = false;
            }
        }
        return false;
    }
    // 检查当前的中间导航节点是否在有效导航范围
    inline bool IsInternavInRange(const NavNodePtr& cur_inter_ptr) {
        if (cur_inter_ptr == NULL) return false;
        // 节点的总代价是否超过规划范围或者导航点高度是否在可接受范围
        if (cur_inter_ptr->fgscore > FARUtil::kLocalPlanRange ||
            !FARUtil::IsPointInToleratedHeight(cur_inter_ptr->position, FARUtil::kMarginHeight)) {
            return false;
        }
        return true;
    }
    // 将有效但超出当前处理范围的轮廓节点添加到特殊映射中，延迟处理。
    inline void AddNodeToOutrangeContourMap(const NavNodePtr& node_ptr) {
        const auto it = out_contour_nodes_map_.find(node_ptr);
        if (it != out_contour_nodes_map_.end())
            it->second.first = dg_params_.finalize_thred;
        else {
            std::unordered_set<NavNodePtr> empty_set;
            const auto init_pair = std::make_pair(dg_params_.finalize_thred, empty_set);
            out_contour_nodes_map_.insert({node_ptr, init_pair});
        }
    }
    // 重置被阻塞的轮廓对
    // 当一个节点被删除时，它原本连接的轮廓节点对之间可能会产生连接状态的混乱
    // 系统认为A→B的直接连接会穿过X，所以投票失败，A和B因为历史失败记录不会重新尝试连接
    // 处理节点删除时可能产生的轮廓连接冲突，重置相关的投票状态。
    static inline void ResetBlockedContourPairs(const NavNodePtr& node_ptr) {
        if (node_ptr->contour_connects.empty()) return;
        const std::size_t N = node_ptr->contour_connects.size();
        for (std::size_t i = 0; i < N; i++) {
            for (std::size_t j = 0; j < N; j++) {
                if (i == j || j > i) continue;
                const NavNodePtr cnode1 = node_ptr->contour_connects[i];
                const NavNodePtr cnode2 = node_ptr->contour_connects[j];
                const auto it1 = cnode1->contour_votes.find(cnode2->id);
                if (it1 != cnode1->contour_votes.end()) {
                    if (!FARUtil::IsVoteTrue(it1->second, false)) {  // 如果连接被阻塞
                        const auto it2 = cnode2->contour_votes.find(cnode1->id);
                        // 重置为阻塞状态
                        it1->second.clear(), it1->second.push_back(0);
                        it2->second.clear(), it2->second.push_back(0);
                    }
                }
            }
        }
    }
    // 完全清理一个节点的所有轮廓相关连接
    inline void ClearContourConnectionInGraph(const NavNodePtr& node_ptr) {
        // reset connected contour pairs if exists
        ResetBlockedContourPairs(node_ptr);
        // 实际清理轮廓连接
        for (const auto& ct_cnode_ptr : node_ptr->contour_connects) {
            FARUtil::EraseNodeFromStack(node_ptr, ct_cnode_ptr->contour_connects);
            ContourGraph::DeleteContourFromSets(ct_cnode_ptr, node_ptr);
        }
        // 3. 处理潜在轮廓连接
        for (const auto& pt_cnode_ptr : node_ptr->potential_contours) {
            const auto it = pt_cnode_ptr->contour_votes.find(node_ptr->id);
            if (pt_cnode_ptr->is_active && !pt_cnode_ptr->is_near_nodes && FARUtil::IsVoteTrue(it->second, false)) {
                AddNodeToOutrangeContourMap(pt_cnode_ptr);
            }
            FARUtil::EraseNodeFromStack(node_ptr, pt_cnode_ptr->potential_contours);
            pt_cnode_ptr->contour_votes.erase(it);
        }
        // 4. 清空节点自身的轮廓数据
        node_ptr->contour_connects.clear();
        node_ptr->contour_votes.clear();
        node_ptr->potential_contours.clear();
    }

    // 点是否要被清理is_merged
    static inline bool IsMergedNode(const NavNodePtr& node_ptr) {
        if (FARUtil::IsStaticNode(node_ptr)) {
            node_ptr->is_merged = false;
            return false;
        }
        if (node_ptr->is_merged) return true;
        return false;
    }
    // 清理节点连接的边 connect_nodes，poly_connects，potential_edges，以及投票edge_votes
    static inline void ClearNodeConnectInGraph(const NavNodePtr& node_ptr) {
        if (node_ptr == NULL) return;
        // clear navigation connections
        for (const auto& cnode_ptr : node_ptr->connect_nodes) {
            FARUtil::EraseNodeFromStack(node_ptr, cnode_ptr->connect_nodes);
        }
        for (const auto& pnode_ptr : node_ptr->poly_connects) {
            FARUtil::EraseNodeFromStack(node_ptr, pnode_ptr->poly_connects);
        }
        for (const auto& pt_cnode_ptr : node_ptr->potential_edges) {
            FARUtil::EraseNodeFromStack(node_ptr, pt_cnode_ptr->potential_edges);
            pt_cnode_ptr->edge_votes.erase(node_ptr->id);
        }
        node_ptr->connect_nodes.clear();
        node_ptr->poly_connects.clear();
        node_ptr->edge_votes.clear();
        node_ptr->potential_edges.clear();
    }
    // 清理轨迹连接trajectory_connects和投票trajectory_votes
    static inline void ClearTrajectoryConnectInGraph(const NavNodePtr& node_ptr) {
        for (const auto& tjnode_ptr : node_ptr->trajectory_connects) {
            tjnode_ptr->trajectory_votes.erase(node_ptr->id);
            FARUtil::EraseNodeFromStack(node_ptr, tjnode_ptr->trajectory_connects);
        }
        node_ptr->trajectory_connects.clear();
        node_ptr->trajectory_votes.clear();
    }
    // 清理在near_nav_nodes_，wide_near_nodes_，margin_near_nodes_，internav_near_nodes_，surround_internav_nodes_中的点
    inline void ClearNodeFromInternalStack(const NavNodePtr& node_ptr) {
        FARUtil::EraseNodeFromStack(node_ptr, near_nav_nodes_);
        FARUtil::EraseNodeFromStack(node_ptr, wide_near_nodes_);
        FARUtil::EraseNodeFromStack(node_ptr, margin_near_nodes_);
        if (node_ptr->is_navpoint) {
            FARUtil::EraseNodeFromStack(node_ptr, internav_near_nodes_);
            FARUtil::EraseNodeFromStack(node_ptr, surround_internav_nodes_);
        }
    }

    /* 清理is_merged的点 */
    inline void ClearMergedNodesInGraph() {
        // remove nodes
        for (auto it = globalGraphNodes_.begin(); it != globalGraphNodes_.end(); it++) {
            if (IsMergedNode(*it)) {
                ClearNodeConnectInGraph(*it);
                ClearContourConnectionInGraph(*it);
                ClearTrajectoryConnectInGraph(*it);
                RemoveNodeIdFromMap(*it);
                ClearNodeFromInternalStack(*it);
                globalGraphNodes_.erase(it--);
            }
        }
        // clean outrange contour nodes
        // 管理超出当前处理范围但仍有价值的轮廓节点
        out_contour_nodes_.clear();
        for (auto it = out_contour_nodes_map_.begin(); it != out_contour_nodes_map_.end();) {
            it->second.first--;
            if (it->second.first <= 0 || it->first->is_near_nodes || IsMergedNode(it->first))
                it = out_contour_nodes_map_.erase(it);
            else {
                out_contour_nodes_.push_back(it->first);
                ++it;
            }
        }
    }
    // 将和ct点匹配到的边缘节点margin_near_nodes_节点加入near_nav_nodes_和wide_near_nodes_中
    inline void UpdateNearNodesWithMatchedMarginNodes(
        const NodePtrStack& margin_nodes, NodePtrStack& near_nodes, NodePtrStack& wide_nodes) {
        for (const auto& node_ptr : margin_nodes) {
            if (node_ptr->is_contour_match) {
                node_ptr->is_wide_near = true, wide_nodes.push_back(node_ptr);
                node_ptr->is_near_nodes = true, near_nodes.push_back(node_ptr);
            }
        }
    }

public:
    DynamicGraph() = default;
    ~DynamicGraph() = default;
    void Init(const ros::NodeHandle& nh, const DynamicGraphParams& params);
    void UpdateRobotPosition(const Point3D& robot_pos);
    // 从轮廓中提取ct点  创造导航点
    bool ExtractGraphNodes(const CTNodeStack& new_ctnodes);
    /**
     * 使用给定的导航节点更新整个导航图
     * 1.清除假阳性节点检测
     * 2.更新现有节点的边
     * 3.使用新提取的点在现有节点间添加边
     * new_nodes：为图提供了新的导航节点
     * is_freeze_vgraph：停止可见性图更新（除了机器人节点）
     * clear_node：现有节点现在被识别为误报
     */
    void UpdateNavGraph(const NodePtrStack& new_nodes, const bool& is_freeze_vgraph, NodePtrStack& clear_node);

    /**
     *  更新局部范围内节点栈: near nodes, wide near nodes etc.,
     */
    void UpdateGlobalNearNodes();
    // 清除两节点之间的edge_votes连接
    static void DeletePolygonVote(
        const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const int& queue_size, const bool& is_reset = false);
    // 给两节点之间的edge_votes投票加1，并添加两节点间的potential_edges
    static void RecordPolygonVote(
        const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const int& queue_size, const bool& is_reset = false);

    // 地形连接验证  验证两个导航节点之间的连接是否在地形上可行
    // 包括坡度检查、高度验证和地形匹配。
    // is_contour：用于区分连接类型：  true：轮廓连接验证    false：多边形连接验证，类似几何碰撞
    // 轮廓连接允许一定的地形数据不匹配
    // 多边形连接必须通过严格的地形验证，使用投票机制防止因噪声导致的连接不稳定
    static bool IsOnTerrainConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& is_contour);

    // 是否能找到在terrain_height上的高的信息
    static inline bool IsPointOnTerrain(const Point3D& p) {
        bool UNUSE_match = false;
        const float terrain_h = MapHandler::TerrainHeightOfPoint(p, UNUSE_match, true);
        if (abs(p.z - terrain_h - FARUtil::vehicle_height) < FARUtil::kTolerZ) {
            return true;
        }

        return false;
    }
    // 判断两个点是不是凹点。
    static inline bool IsConvexConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1->free_direct != NodeFreeDirect::CONCAVE && node_ptr2->free_direct != NodeFreeDirect::CONCAVE) {
            return true;
        }
        return false;
    }
    // 记录无效的地形连接  terrain_votes +1 投一票
    static inline void RemoveInvaildTerrainConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->terrain_votes.find(node_ptr2->id);
        if (it1 == node_ptr1->terrain_votes.end()) {
            node_ptr1->terrain_votes.insert({node_ptr2->id, 1});
            node_ptr2->terrain_votes.insert({node_ptr1->id, 1});
        } else if (it1->second < dg_params_.finalize_thred * 2) {
            const auto it2 = node_ptr2->terrain_votes.find(node_ptr1->id);
            it1->second++, it2->second++;
        }
    }
    // 记录有效的地形连接  terrain_votes -1 减少一票
    static inline void RecordVaildTerrainConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->terrain_votes.find(node_ptr2->id);
        if (it1 == node_ptr1->terrain_votes.end()) return;
        if (it1->second > 0) {
            const auto it2 = node_ptr2->terrain_votes.find(node_ptr1->id);
            it1->second--, it2->second--;
        }
    }
    // 创建导航点
    static inline void CreateNavNodeFromPoint(const Point3D& point, NavNodePtr& node_ptr, const bool& is_odom,
        const bool& is_navpoint = false, const bool& is_goal = false, const bool& is_boundary = false) {
        node_ptr = std::make_shared<NavNode>();
        node_ptr->pos_filter_vec.clear();
        node_ptr->surf_dirs_vec.clear();
        node_ptr->ctnode = NULL;
        node_ptr->is_active = true;
        node_ptr->is_block_frontier = false;
        node_ptr->is_contour_match = false;
        node_ptr->is_odom = is_odom;
        node_ptr->is_near_nodes = true;
        node_ptr->is_wide_near = true;
        node_ptr->is_merged = false;
        node_ptr->is_covered = (is_odom || is_navpoint || is_goal) ? true : false;
        node_ptr->is_frontier = false;
        node_ptr->is_finalized = is_navpoint ? true : false;
        node_ptr->is_traversable = is_odom;
        node_ptr->is_navpoint = is_navpoint;
        node_ptr->is_boundary = is_boundary;
        node_ptr->is_goal = is_goal;
        node_ptr->clear_dumper_count = 0;
        node_ptr->frontier_votes.clear();
        node_ptr->invalid_boundary.clear();
        node_ptr->connect_nodes.clear();
        node_ptr->poly_connects.clear();
        node_ptr->contour_connects.clear();
        node_ptr->contour_votes.clear();
        node_ptr->potential_contours.clear();
        node_ptr->trajectory_connects.clear();
        node_ptr->trajectory_votes.clear();
        node_ptr->terrain_votes.clear();
        node_ptr->free_direct = (is_odom || is_navpoint) ? NodeFreeDirect::PILLAR : NodeFreeDirect::UNKNOW;
        // planner members
        node_ptr->is_block_to_goal = false;
        node_ptr->gscore = FARUtil::kINF;
        node_ptr->fgscore = FARUtil::kINF;
        node_ptr->is_traversable = true;
        node_ptr->is_free_traversable = true;
        node_ptr->parent = NULL;
        node_ptr->free_parent = NULL;
        InitNodePosition(node_ptr, point);
        // Assign Global ID
        AssignGlobalNodeID(node_ptr);
    }
    static inline void AssignGlobalNodeID(const NavNodePtr& node_ptr) {
        node_ptr->id = id_tracker_;
        idx_node_map_.insert({id_tracker_, node_ptr});
        id_tracker_++;
    }
    static inline void RemoveNodeIdFromMap(const NavNodePtr& node_ptr) {
        idx_node_map_.erase(node_ptr->id);
    }
    static inline void AddNodeToGraph(const NavNodePtr& node_ptr) {
        if (node_ptr != NULL) {
            globalGraphNodes_.push_back(node_ptr);
        } else if (FARUtil::IsDebug) {
            ROS_WARN_THROTTLE(1.0, "DG: exist new node pointer is NULL, fails to add into graph");
        }
    }
    // 两个导航点之间添加poly_connects
    static inline void AddPolyEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1 == node_ptr2) return;
        if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->poly_connects) &&
            !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->poly_connects)) {
            node_ptr1->poly_connects.push_back(node_ptr2);
            node_ptr2->poly_connects.push_back(node_ptr1);
        }
    }
    // 删除两个点之间的poly_connects
    static inline void ErasePolyEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        // clear node2 in node1's connection
        FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->poly_connects);
        // clear node1 in node2's connection
        FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->poly_connects);
    }

    // 两个导航点之间添加connect_nodes
    static inline void AddEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1 == node_ptr2) return;
        if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->connect_nodes) &&
            !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->connect_nodes)) {
            node_ptr1->connect_nodes.push_back(node_ptr2);
            node_ptr2->connect_nodes.push_back(node_ptr1);
        }
    }

    // 删除两个点之间的connect_nodes
    static inline void EraseEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        // clear node2 in node1's connection
        FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->connect_nodes);
        // clear node1 in node2's connection
        FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->connect_nodes);
    }
    inline void ResetCurrentGraph() {
        odom_node_ptr_ = NULL;
        cur_internav_ptr_ = NULL;
        last_internav_ptr_ = NULL;

        id_tracker_ = 0;
        is_bridge_internav_ = false;
        last_connect_pos_ = Point3D(0, 0, 0);

        idx_node_map_.clear();
        near_nav_nodes_.clear();
        wide_near_nodes_.clear();
        extend_match_nodes_.clear();
        margin_near_nodes_.clear();
        internav_near_nodes_.clear();
        surround_internav_nodes_.clear();
        out_contour_nodes_.clear();
        out_contour_nodes_map_.clear();

        globalGraphNodes_.clear();
        new_nodes_.clear();
    }

    /* Get Internal Values */
    const NavNodePtr GetOdomNode() const {
        return odom_node_ptr_;
    };
    const NodePtrStack& GetNavGraph() const {
        return globalGraphNodes_;
    };
    const NodePtrStack& GetExtendLocalNode() const {
        return extend_match_nodes_;
    }
    const NodePtrStack& GetOutContourNodes() const {
        return out_contour_nodes_;
    };
    const NodePtrStack& GetNewNodes() const {
        return new_nodes_;
    };
    const NavNodePtr& GetLastInterNavNode() const {
        return last_internav_ptr_;
    };
};

#endif