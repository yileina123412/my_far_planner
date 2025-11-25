#ifndef CONTOUR_GRAPH_H
#define CONTOUR_GRAPH_H

#pragma once

#include "utility.h"
struct ConnectPair {
    cv::Point2f start_p;
    cv::Point2f end_p;

    ConnectPair() = default;
    ConnectPair(const cv::Point2f& p1, const cv::Point2f& p2) : start_p(p1), end_p(p2) {}
    ConnectPair(const Point3D& p1, const Point3D& p2) {
        this->start_p.x = p1.x;
        this->start_p.y = p1.y;
        this->end_p.x = p2.x;
        this->end_p.y = p2.y;
    }

    bool operator==(const ConnectPair& pt) const {
        return (this->start_p == pt.start_p && this->end_p == pt.end_p) ||
               (this->start_p == pt.end_p && this->end_p == pt.start_p);
    }
};

struct HeightPair {
    float minH;
    float maxH;
    HeightPair() = default;
    HeightPair(const float& minV, const float& maxV) : minH(minV), maxH(maxV) {}
    HeightPair(const Point3D& p1, const Point3D p2) {
        this->minH = std::min(p1.z, p2.z);
        this->maxH = std::max(p1.z, p2.z);
    }
};

struct ContourGraphParams {
    ContourGraphParams() = default;
    float kPillarPerimeter;
};
class ContourGraph {
public:
    ContourGraph() = default;
    ~ContourGraph() = default;

    // ct点的图,存储从轮廓转化的ctnode
    static CTNodeStack contour_graph_;
    static std::vector<PointPair> global_contour_;
    static std::vector<PointPair> inactive_contour_;
    static std::vector<PointPair> unmatched_contour_;
    static std::vector<PointPair> boundary_contour_;
    static std::vector<PointPair> local_boundary_;

    void Init(const ContourGraphParams& params);
    void UpdateContourGraph(
        const NavNodePtr& odom_node_ptr, const std::vector<std::vector<Point3D>>& filtered_contours);

    void MatchContourWithNavGraph(
        const NodePtrStack& global_nodes, const NodePtrStack& near_nodes, CTNodeStack& new_convex_vertices);
    void ResetCurrentContour();
    // 检查两点之间的连接线是否和多边形发生碰撞
    static bool IsPointsConnectFreePolygon(
        const ConnectPair& cedge, const ConnectPair& bd_cedge, const HeightPair h_pair, const bool& is_global_check);

    static inline void MatchCTNodeWithNavNode(const CTNodePtr& ctnode_ptr, const NavNodePtr& node_ptr) {
        if (ctnode_ptr == nullptr || node_ptr == nullptr) {
            return;
        }
        ctnode_ptr->is_global_match = true;
        ctnode_ptr->nav_node_id = node_ptr->id;
        node_ptr->is_contour_match = true;
        node_ptr->ctnode = ctnode_ptr;
    }

private:
    // 存储结果

    // 存储每个多边形第一个ctnode
    static CTNodeStack polys_ctnodes_;
    // 存储多边形
    static PolygonStack contour_polygons_;
    ContourGraphParams ctgraph_params_;
    float ALIGN_ANGLE_COS;
    NavNodePtr odom_node_ptr_;
    bool is_robot_inside_poly_ = false;

    // global contour set
    static std::unordered_set<NavEdge, navedge_hash> global_contour_set_;
    static std::unordered_set<NavEdge, navedge_hash> boundary_contour_set_;

    /*static private functions*/

    inline void AddCTNodeToGraph(const CTNodePtr& ctnode_ptr) {
        if (ctnode_ptr == nullptr && ctnode_ptr->free_direct == NodeFreeDirect::UNKNOW) {
            if (FARUtil::IsDebug) ROS_ERROR_THROTTLE(1.0, "CG: Add ctnode to contour graph fails, ctnode is invaild.");
            return;
        }
        ContourGraph::contour_graph_.push_back(ctnode_ptr);
    }

    inline void AddPolyToContourPolygon(const PolygonPtr& poly_ptr) {
        if (poly_ptr == nullptr || poly_ptr->vertices.empty()) {
            if (FARUtil::IsDebug) ROS_ERROR_THROTTLE(1.0, "CG: Add polygon fails, polygon is invaild.");
            return;
        }
        ContourGraph::contour_polygons_.push_back(poly_ptr);  // 存储
    }

    template <typename NodeType1, typename NodeType2>
    static inline bool IsInMatchHeight(const NodeType1& node_ptr1, const NodeType2& node_ptr2) {
        if (abs(node_ptr1->position.z - node_ptr2->position.z) < FARUtil::kTolerZ) {
            return true;
        }
        return false;
    }

    inline void ClearContourGraph() {
        ContourGraph::contour_graph_.clear();
        ContourGraph::polys_ctnodes_.clear();
        ContourGraph::contour_polygons_.clear();
    }

    static inline void RemoveMatchWithNavNode(const NavNodePtr& node_ptr) {
        if (node_ptr == NULL) return;
        if (!node_ptr->is_contour_match) return;
        if (node_ptr->ctnode == NULL) {
            node_ptr->is_contour_match = false;
            return;
        }
        node_ptr->ctnode->is_global_match = false;
        node_ptr->ctnode->nav_node_id = 0;

        node_ptr->ctnode = NULL;
    }

    void UpdateOdomFreePosition(const NavNodePtr& odom_ptr, Point3D& global_free_p);
    bool IsAPillarPolygon(const PointStack& vertex_points, float& perimeter);
    void CreateCTNode(const Point3D& pos, CTNodePtr& ctnode_ptr, const PolygonPtr& poly_ptr, const bool& is_pillar);
    void CreatePolygon(const std::vector<Point3D>& poly_points, PolygonPtr& poly_ptr);
    // 分析ctnode的凹凸性
    void AnalysisConvexityOfCTNode(const CTNodePtr& ctnode_ptr);
    // 分析ctnode表面的方向的
    void AnalysisSurfAngleAndConvexity(const CTNodeStack& contour_graph);

    /**
     * @brief extract necessary ctnodes that are essential for contour construction
     */
    // 检查匹配到导航点的ct点的完整性和合理性，防止过渡简化导致轮廓确实
    void EnclosePolygonsCheck();
    CTNodePtr FirstMatchedCTNode(const CTNodePtr& ctnode_ptr);
    static bool IsCTNodesConnectWithinOrder(
        const CTNodePtr& ctnode1, const CTNodePtr& ctnode2, CTNodePtr& block_vertex);

    /*匹配ct点和导航点辅助函数*/
    // 找到和当前给出的ct点对近的导航点
    NavNodePtr NearestNavNodeForCTNode(const CTNodePtr& ctnode_ptr, const NodePtrStack& near_nodes);

    // 检查两线是否相交
    static bool IsEdgeCollidePoly(const PointStack& poly, const ConnectPair& edge);

    static bool IsEdgeCollideSegment(const PointPair& line, const ConnectPair& edge);

    // 用于检查ct点和nav点之间的匹配连线是否畅通无阻
    static bool IsCTMatchLineFreePolygon(
        const CTNodePtr& matched_ctnode, const NavNodePtr& matched_navnode, const bool& is_global_check);

    template <typename NodeType>
    static inline cv::Point2f NodeProjectDir(const NodeType& node) {
        cv::Point2f project_dir(0, 0);
        if (node->free_direct != NodeFreeDirect::PILLAR && node->free_direct != NodeFreeDirect::UNKNOW) {
            const Point3D topo_dir = FARUtil::SurfTopoDirect(node->surf_dirs);
            if (node->free_direct == NodeFreeDirect::CONCAVE) {
                project_dir = cv::Point2f(topo_dir.x, topo_dir.y);
            } else {
                project_dir = cv::Point2f(-topo_dir.x, -topo_dir.y);
            }
        }
        return project_dir;
    }

    template <typename NodeType>
    static inline cv::Point2f ProjectNode(const NodeType& node, const float& dist) {
        const cv::Point2f node_cv = cv::Point2f(node->position.x, node->position.y);
        const cv::Point2f dir = NodeProjectDir(node);
        return node_cv + dist * dir;
    }
    // 重投影后的连接边  生成远离障碍物表面的连接边
    static ConnectPair ReprojectEdge(
        const NavNodePtr& node1, const NavNodePtr& node2, const float& dist, const bool& is_global_check);
    // 生成远离障碍物表面的连接边
    static ConnectPair ReprojectEdge(const CTNodePtr& node1, const NavNodePtr& node2, const float& dist);
};

#endif