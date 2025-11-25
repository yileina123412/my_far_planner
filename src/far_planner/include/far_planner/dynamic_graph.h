#ifndef DYNAMIC_GRAPH_H
#define DYNAMIC_GRAPH_H
#pragma once
#include "map_handler.h"
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
    NodePtrStack out_contour_nodes_;
    float CONNECT_ANGLE_COS, NOISE_ANGLE_COS;
    // 是否需要创建一个临时的桥接导航点，来连接当前位置和心的目标导航点
    bool is_bridge_internav_ = false;
    Point3D last_connect_pos_;  // 上次创建轨迹连接时机器人的位置

    static DynamicGraphParams dg_params_;
    static std::size_t id_tracker_;
    static NodePtrStack globalGraphNodes_;  // 存储全部的导航节点
    static std::unordered_map<std::size_t, NavNodePtr> idx_node_map_;

    // 是否需要创建一个中间导航节点
    bool IsInterNavpointNecessary();
    // 判断一个导航点是否应该被激活
    bool IsActivateNavNode(const NavNodePtr& node_ptr);

    bool UpdateNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos);

    static void InitNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos);
    // 更新导航节点的表面方向信息
    bool UpdateNodeSurfDirs(const NavNodePtr& node_ptr, PointPair cur_dirs);
    // 重新评估凹凸性
    void ReEvaluateConvexity(const NavNodePtr& node_ptr);
    // 更新cur_internav_ptr_，并建立轨迹连接
    inline void UpdateCurInterNavNode(const NavNodePtr& internav_node_ptr) {
        if (internav_node_ptr == NULL || !internav_node_ptr->is_navpoint) return;
        cur_internav_ptr_ = internav_node_ptr;
        if (last_internav_ptr_ == NULL) {  // init inter navigation nodes
            // terrain_planner_.UpdateCenterNode(cur_internav_ptr_);
            last_internav_ptr_ = cur_internav_ptr_;
        } else if (last_internav_ptr_ != cur_internav_ptr_) {
            // terrain_planner_.UpdateCenterNode(cur_internav_ptr_);
            this->AddTrajectoryConnect(cur_internav_ptr_, last_internav_ptr_);
            last_internav_ptr_ = cur_internav_ptr_;
        }
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

    inline void CreateNewNavNodeFromContour(const CTNodePtr& ctnode_ptr, NavNodePtr& node_ptr) {
        CreateNavNodeFromPoint(ctnode_ptr->position, node_ptr, false);
        node_ptr->is_contour_match = true;
        node_ptr->ctnode = ctnode_ptr;
        node_ptr->free_direct = ctnode_ptr->free_direct;
        UpdateNodeSurfDirs(node_ptr, ctnode_ptr->surf_dirs);
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

    // 是否能找到在terrain_height上的高的信息
    static inline bool IsPointOnTerrain(const Point3D& p) {
        bool UNUSE_match = false;
        const float terrain_h = MapHandler::TerrainHeightOfPoint(p, UNUSE_match, true);
        if (abs(p.z - terrain_h - FARUtil::vehicle_height) < FARUtil::kTolerZ) {
            return true;
        }

        return false;
    }

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
    inline void ResetCurrentGraph() {
        odom_node_ptr_ = NULL;
        cur_internav_ptr_ = NULL;
        last_internav_ptr_ = NULL;

        id_tracker_ = 0;
        is_bridge_internav_ = false;
        last_connect_pos_ = Point3D(0, 0, 0);

        near_nav_nodes_.clear();
        wide_near_nodes_.clear();
        extend_match_nodes_.clear();
        margin_near_nodes_.clear();
        internav_near_nodes_.clear();
        surround_internav_nodes_.clear();

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
    const NodePtrStack& GetNewNodes() const {
        return new_nodes_;
    };
    const NavNodePtr& GetLastInterNavNode() const {
        return last_internav_ptr_;
    };
};

#endif