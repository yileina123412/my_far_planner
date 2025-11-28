#include "far_planner/contour_graph.h"

#include "far_planner/intersection.h"

void ContourGraph::Init(const ContourGraphParams& params) {
    ctgraph_params_ = params;
    ContourGraph::contour_polygons_.clear();
    ContourGraph::contour_graph_.clear();
    ALIGN_ANGLE_COS = cos(M_PI - FARUtil::kAcceptAlign / 2.0f);
    is_robot_inside_poly_ = false;
    ContourGraph::global_contour_set_.clear();
    ContourGraph::boundary_contour_set_.clear();
}
// 更新轮廓图，将轮廓点转化为ct点
void ContourGraph::UpdateContourGraph(
    const NavNodePtr& odom_node_ptr, const std::vector<std::vector<Point3D>>& filtered_contours) {
    // 1. 清空上一帧的结果
    this->ClearContourGraph();
    odom_node_ptr_ = odom_node_ptr;
    for (const auto& poly_points : filtered_contours) {
        // 把每一个poly_points做成一个polygon对象
        PolygonPtr new_poly_ptr = nullptr;
        this->CreatePolygon(poly_points, new_poly_ptr);
        this->AddPolyToContourPolygon(new_poly_ptr);
    }
    UpdateOdomFreePosition(odom_node_ptr_, FARUtil::free_odom_p);
    for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
        poly_ptr->is_robot_inside = FARUtil::PointInsideAPoly(poly_ptr->vertices, FARUtil::free_odom_p);
        CTNodePtr new_ctnode_ptr = nullptr;
        if (poly_ptr->is_pillar) {
            Point3D meanp = FARUtil::AveragePoints(poly_ptr->vertices);
            this->CreateCTNode(meanp, new_ctnode_ptr, poly_ptr, true);
            this->AddCTNodeToGraph(new_ctnode_ptr);
        } else {
            CTNodeStack ctnode_stack;
            ctnode_stack.clear();
            const int N = poly_ptr->vertices.size();
            for (std::size_t idx = 0; idx < N; idx++) {
                this->CreateCTNode(poly_ptr->vertices[idx], new_ctnode_ptr, poly_ptr, false);
                ctnode_stack.push_back(new_ctnode_ptr);
            }
            for (std::size_t idx = 0; idx < N; idx++) {
                int ref_index = FARUtil::Mod(idx - 1, N);
                ctnode_stack[idx]->front = ctnode_stack[ref_index];
                ref_index = FARUtil::Mod(idx + 1, N);
                ctnode_stack[idx]->back = ctnode_stack[ref_index];
                this->AddCTNodeToGraph(ctnode_stack[idx]);
            }
            if (!ctnode_stack.empty()) ContourGraph::polys_ctnodes_.push_back(ctnode_stack.front());
        }
    }

    this->AnalysisSurfAngleAndConvexity(ContourGraph::contour_graph_);
}

void ContourGraph::UpdateOdomFreePosition(const NavNodePtr& odom_ptr, Point3D& global_free_p) {
    Point3D free_p = odom_ptr->position;
    bool is_free_p = true;
    PointStack free_sample_points;
    for (const auto& poly_ptr : contour_polygons_) {
        if (!poly_ptr->is_pillar && poly_ptr->is_robot_inside) {
            is_free_p = false;
            FARUtil::CreatePointsAroundCenter(free_p, FARUtil::kNavClearDist, FARUtil::kLeafSize, free_sample_points);
            break;
        }
    }
    if (is_free_p) is_robot_inside_poly_ = false;
    global_free_p = free_p;
    // 上次是自由的，这次是不自由的。可能是噪声问题
    if (!is_free_p && !is_robot_inside_poly_) {
        bool is_free_pos_found = false;
        for (const auto& p : free_sample_points) {
            bool is_sample_free = true;
            for (const auto& poly_ptr : contour_polygons_) {
                if (!poly_ptr->is_pillar && FARUtil::PointInsideAPoly(poly_ptr->vertices, p)) {
                    is_sample_free = false;
                    break;
                }
            }
            if (is_sample_free) {
                global_free_p = p;
                is_free_pos_found = true;
                break;
            }
        }
        if (!is_free_pos_found) is_robot_inside_poly_ = true;
    }
}

void ContourGraph::CreatePolygon(const std::vector<Point3D>& poly_points, PolygonPtr& poly_ptr) {
    poly_ptr = std::make_shared<Polygon>();
    poly_ptr->N = poly_points.size();
    poly_ptr->vertices = poly_points;
    poly_ptr->is_robot_inside = FARUtil::PointInsideAPoly(poly_points, odom_node_ptr_->position);
    float perimeter = 0.0f;
    poly_ptr->is_pillar = this->IsAPillarPolygon(poly_points, perimeter);
    poly_ptr->perimeter = perimeter;
}
void ContourGraph::CreateCTNode(
    const Point3D& pos, CTNodePtr& ctnode_ptr, const PolygonPtr& poly_ptr, const bool& is_pillar) {
    ctnode_ptr = std::make_shared<CTNode>();
    ctnode_ptr->position = pos;
    ctnode_ptr->front = NULL;
    ctnode_ptr->back = NULL;
    ctnode_ptr->is_global_match = false;
    ctnode_ptr->is_contour_necessary = false;
    ctnode_ptr->is_ground_associate = false;
    ctnode_ptr->nav_node_id = 0;
    ctnode_ptr->poly_ptr = poly_ptr;
    ctnode_ptr->free_direct = is_pillar ? NodeFreeDirect::PILLAR : NodeFreeDirect::UNKNOW;
    ctnode_ptr->connect_nodes.clear();
}
bool ContourGraph::IsAPillarPolygon(const PointStack& vertex_points, float& perimeter) {
    perimeter = 0.0f;
    if (vertex_points.size() < 3) return true;
    Point3D pre_node = vertex_points[0];
    for (int i = 1; i < vertex_points.size(); i++) {
        const Point3D cur_p(vertex_points[i]);
        const float dist = std::hypotf(cur_p.x - pre_node.x, cur_p.y - pre_node.y);
        perimeter += dist;
        pre_node = cur_p;
    }
    return perimeter > ctgraph_params_.kPillarPerimeter ? false : true;
}
// 分析ct点的表面方向和凹凸性
void ContourGraph::AnalysisSurfAngleAndConvexity(const CTNodeStack& contour_graph) {
    for (const auto& ctnode_ptr : contour_graph) {
        if (ctnode_ptr == NULL) continue;
        if (ctnode_ptr->free_direct == NodeFreeDirect::PILLAR || ctnode_ptr->poly_ptr->is_pillar) {
            ctnode_ptr->surf_dirs = {Point3D(0, 0, -1), Point3D(0, 0, -1)};
            ctnode_ptr->poly_ptr->is_pillar = true;
            ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
        } else {
            if (ctnode_ptr->front == NULL || ctnode_ptr->back == NULL) {
                ctnode_ptr->free_direct = NodeFreeDirect::UNKNOW;
                continue;
            }
            CTNodePtr next_ctnode;
            // front direction
            next_ctnode = ctnode_ptr->front;
            Point3D start_p = ctnode_ptr->position;
            Point3D end_p = next_ctnode->position;
            float edis = (end_p - ctnode_ptr->position).norm_flat();
            while (next_ctnode != NULL && next_ctnode != ctnode_ptr && edis < FARUtil::kNavClearDist) {
                next_ctnode = next_ctnode->front;
                start_p = end_p;
                end_p = next_ctnode->position;
                edis = (end_p - ctnode_ptr->position).norm_flat();
            }
            if (edis < FARUtil::kNavClearDist) {
                ctnode_ptr->surf_dirs = {Point3D(0, 0, -1), Point3D(0, 0, -1)};
                ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
                ctnode_ptr->poly_ptr->is_pillar = true;
                continue;
            } else {
                ctnode_ptr->surf_dirs.first =
                    FARUtil::ContourSurfDirs(end_p, start_p, ctnode_ptr->position, FARUtil::kNavClearDist);
            }
            // back direction
            next_ctnode = ctnode_ptr->back;
            start_p = ctnode_ptr->position;
            end_p = next_ctnode->position;
            edis = (end_p - ctnode_ptr->position).norm_flat();
            while (next_ctnode != NULL && next_ctnode != ctnode_ptr && edis < FARUtil::kNavClearDist) {
                next_ctnode = next_ctnode->back;
                start_p = end_p;
                end_p = next_ctnode->position;
                edis = (end_p - ctnode_ptr->position).norm_flat();
            }
            if (edis < FARUtil::kNavClearDist) {
                ctnode_ptr->surf_dirs = {Point3D(0, 0, -1), Point3D(0, 0, -1)};
                ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
                ctnode_ptr->poly_ptr->is_pillar = true;
                continue;
            } else {
                ctnode_ptr->surf_dirs.second =
                    FARUtil::ContourSurfDirs(end_p, start_p, ctnode_ptr->position, FARUtil::kNavClearDist);
            }
        }
        this->AnalysisConvexityOfCTNode(ctnode_ptr);
    }
}
// 分析ct点的凹凸性
void ContourGraph::AnalysisConvexityOfCTNode(const CTNodePtr& ctnode_ptr) {
    if (ctnode_ptr->surf_dirs.first == Point3D(0, 0, -1) || ctnode_ptr->surf_dirs.second == Point3D(0, 0, -1) ||
        ctnode_ptr->poly_ptr->is_pillar) {
        ctnode_ptr->surf_dirs.first = Point3D(0, 0, -1), ctnode_ptr->surf_dirs.second == Point3D(0, 0, -1);
        ctnode_ptr->poly_ptr->is_pillar = true;
        ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
        return;
    }
    // 增加调试输出

    // ROS_INFO("CG: Analyzing node at (%.2f, %.2f), surf_dirs: (%.3f, %.3f) + (%.3f, %.3f)", ctnode_ptr->position.x,
    //     ctnode_ptr->position.y, ctnode_ptr->surf_dirs.first.x, ctnode_ptr->surf_dirs.first.y,
    //     ctnode_ptr->surf_dirs.second.x, ctnode_ptr->surf_dirs.second.y);

    bool is_wall = false;
    const Point3D topo_dir = FARUtil::SurfTopoDirect(ctnode_ptr->surf_dirs, is_wall);

    if (is_wall) {
        ctnode_ptr->free_direct = NodeFreeDirect::UNKNOW;
        return;
    }
    const Point3D ev_p = ctnode_ptr->position + topo_dir * FARUtil::kLeafSize;
    if (FARUtil::IsConvexPoint(ctnode_ptr->poly_ptr, ev_p)) {
        ctnode_ptr->free_direct = NodeFreeDirect::CONVEX;
    } else {
        ctnode_ptr->free_direct = NodeFreeDirect::CONCAVE;
    }
}

void ContourGraph::ResetCurrentContour() {
    this->ClearContourGraph();
    // clear contour sets
    ContourGraph::global_contour_set_.clear();
    ContourGraph::boundary_contour_set_.clear();

    odom_node_ptr_ = NULL;
    is_robot_inside_poly_ = false;
}

/*将ct点转化为导航点*/
void ContourGraph::MatchContourWithNavGraph(
    const NodePtrStack& global_nodes, const NodePtrStack& near_nodes, CTNodeStack& new_convex_vertices) {
    for (const auto& node_ptr : global_nodes) {
        node_ptr->is_contour_match = false;
        node_ptr->ctnode = NULL;
    }
    for (const auto& ctnode_ptr : ContourGraph::contour_graph_) {
        ctnode_ptr->is_global_match = false;
        ctnode_ptr->nav_node_id = 0;
        if (ctnode_ptr->free_direct != NodeFreeDirect::UNKNOW) {
            const NavNodePtr matched_node = NearestNavNodeForCTNode(ctnode_ptr, near_nodes);
            // 验证匹配的有效性
            if (matched_node != NULL && IsCTMatchLineFreePolygon(ctnode_ptr, matched_node, false)) {
                this->MatchCTNodeWithNavNode(ctnode_ptr, matched_node);
            }
        }
    }
    EnclosePolygonsCheck();
    // 处理未匹配到的ct点
    new_convex_vertices.clear();
    for (const auto& ctnode_ptr : ContourGraph::contour_graph_) {
        if (!ctnode_ptr->is_global_match && ctnode_ptr->free_direct != NodeFreeDirect::UNKNOW) {
            // if (!ctnode_ptr->is_global_match) {
            if (ctnode_ptr->free_direct != NodeFreeDirect::PILLAR) {  // 排出墙
                const float dot_value = ctnode_ptr->surf_dirs.first * ctnode_ptr->surf_dirs.second;
                if (dot_value < ALIGN_ANGLE_COS) continue;
            }
            new_convex_vertices.push_back(ctnode_ptr);
        }
    }
}
bool ContourGraph::IsCTNodesConnectWithinOrder(
    const CTNodePtr& ctnode1, const CTNodePtr& ctnode2, CTNodePtr& block_vertex) {
    block_vertex = NULL;
    if (ctnode1 == ctnode2 || ctnode1->poly_ptr != ctnode2->poly_ptr) return false;
    CTNodePtr next_ctnode = ctnode1->front;  // forward search
    while (next_ctnode != NULL && next_ctnode != ctnode2) {
        if (!FARUtil::IsInCylinder(
                ctnode1->position, ctnode2->position, next_ctnode->position, FARUtil::kNearDist, true)) {
            block_vertex = next_ctnode;
            return false;
        }
        next_ctnode = next_ctnode->front;
    }
    return true;
}
void ContourGraph::EnclosePolygonsCheck() {
    for (const auto& ctnode_ptr : ContourGraph::polys_ctnodes_) {
        if (ctnode_ptr->poly_ptr->is_pillar) continue;
        const CTNodePtr start_ctnode_ptr = FirstMatchedCTNode(ctnode_ptr);
        if (start_ctnode_ptr == NULL) continue;
        CTNodePtr pre_ctnode_ptr = start_ctnode_ptr;
        CTNodePtr cur_ctnode_ptr = start_ctnode_ptr->front;
        while (cur_ctnode_ptr != start_ctnode_ptr) {
            if (!cur_ctnode_ptr->is_global_match) {
                cur_ctnode_ptr = cur_ctnode_ptr->front;
                continue;
            }
            // 检查两个匹配节点的连接
            CTNodePtr block_vertex = NULL;
            if (!IsCTNodesConnectWithinOrder(pre_ctnode_ptr, cur_ctnode_ptr, block_vertex) && block_vertex != NULL) {
                if (block_vertex->is_ground_associate && FARUtil::IsPointInMarginRange(block_vertex->position)) {
                    block_vertex->is_contour_necessary = true;
                }
            }

            pre_ctnode_ptr = cur_ctnode_ptr;
            cur_ctnode_ptr = cur_ctnode_ptr->front;
        }
    }
}
CTNodePtr ContourGraph::FirstMatchedCTNode(const CTNodePtr& ctnode_ptr) {
    if (ctnode_ptr->is_global_match) return ctnode_ptr;
    CTNodePtr cur_ctnode_ptr = ctnode_ptr->front;
    while (cur_ctnode_ptr != ctnode_ptr) {
        if (cur_ctnode_ptr->is_global_match) return cur_ctnode_ptr;
        cur_ctnode_ptr = cur_ctnode_ptr->front;
    }
    return NULL;
}
NavNodePtr ContourGraph::NearestNavNodeForCTNode(const CTNodePtr& ctnode_ptr, const NodePtrStack& near_nodes) {
    float nearest_dist = FARUtil::kINF;
    NavNodePtr nearest_node = NULL;
    float min_edist = FARUtil::kINF;
    const float dir_thred = 0.5f;
    for (const auto& node_ptr : near_nodes) {
        if (node_ptr == NULL) continue;
        if (node_ptr->is_odom || node_ptr->is_navpoint || FARUtil::IsOutsideGoal(node_ptr) ||
            !IsInMatchHeight(ctnode_ptr, node_ptr))
            continue;
        // pillar只和pillar匹配

        bool is_node_pillar = (node_ptr->free_direct == NodeFreeDirect::PILLAR);
        bool is_ct_pillar = (ctnode_ptr->free_direct == NodeFreeDirect::PILLAR);
        if (is_node_pillar != is_ct_pillar) continue;
        // if ((node_ptr->free_direct == NodeFreeDirect::PILLAR && ctnode_ptr->free_direct != NodeFreeDirect::PILLAR) ||
        //     (node_ptr->free_direct != NodeFreeDirect::PILLAR && ctnode_ptr->free_direct == NodeFreeDirect::PILLAR)) {
        //     continue;
        // }
        float dist_thread = FARUtil::kMatchDist;
        float match_score = 0.0f;
        float dir_score = 0.2f;
        if (ctnode_ptr->free_direct != NodeFreeDirect::PILLAR && node_ptr->free_direct != NodeFreeDirect::PILLAR &&
            node_ptr->free_direct != NodeFreeDirect::UNKNOW) {
            if (ctnode_ptr->free_direct == node_ptr->free_direct) {
                const Point3D topo_dir1 = FARUtil::SurfTopoDirect(node_ptr->surf_dirs);
                const Point3D topo_dir2 = FARUtil::SurfTopoDirect(ctnode_ptr->surf_dirs);
                dir_score = (topo_dir1 * topo_dir2 - dir_thred) / (1.0f - dir_thred);
            }
        } else if (node_ptr->free_direct == NodeFreeDirect::PILLAR &&
                   ctnode_ptr->free_direct == NodeFreeDirect::PILLAR) {
            dir_score = 0.5f;
        }
        dist_thread *= dir_score;
        const float edist = (ctnode_ptr->position - node_ptr->position).norm_flat();
        if (edist < dist_thread && edist < min_edist) {
            min_edist = edist;
            nearest_node = node_ptr;
        }
    }

    if (nearest_node != NULL && nearest_node->is_contour_match) {
        const float pre_dis = (nearest_node->position - nearest_node->ctnode->position).norm_flat();
        if (min_edist < pre_dis) {
            RemoveMatchWithNavNode(nearest_node);
        } else {
            nearest_node = NULL;
        }
    }

    return nearest_node;
}

bool ContourGraph::IsCTMatchLineFreePolygon(
    const CTNodePtr& matched_ctnode, const NavNodePtr& matched_navnode, const bool& is_global_check) {
    if ((matched_ctnode->position - matched_navnode->position).norm() < FARUtil::kNavClearDist) return true;
    const HeightPair h_pair(matched_ctnode->position, matched_navnode->position);
    const ConnectPair bd_cedge = ConnectPair(matched_ctnode->position, matched_navnode->position);
    const ConnectPair cedge = ContourGraph::ReprojectEdge(matched_ctnode, matched_navnode, FARUtil::kProjectDist);
    return IsPointsConnectFreePolygon(cedge, bd_cedge, h_pair, is_global_check);
}

// 检查两点之间的连接线是否和多边形发生碰撞
bool ContourGraph::IsPointsConnectFreePolygon(
    const ConnectPair& cedge, const ConnectPair& bd_cedge, const HeightPair h_pair, const bool& is_global_check) {
    // check for boundaries edges
    for (const auto& contour : ContourGraph::boundary_contour_) {
        if (!ContourGraph::IsEdgeOverlapInHeight(h_pair, HeightPair(contour.first, contour.second))) continue;
        if (ContourGraph::IsEdgeCollideSegment(contour, bd_cedge)) {
            return false;
        }
    }

    if (!is_global_check) {
        // 检测是否和当前的检测到的障碍物碰撞
        const Point3D center_p =
            Point3D((cedge.start_p.x + cedge.end_p.x) / 2.0f, (cedge.start_p.y + cedge.end_p.y) / 2.0f, 0.0f);
        for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
            if (poly_ptr->is_pillar) continue;
            if ((poly_ptr->is_robot_inside != FARUtil::PointInsideAPoly(poly_ptr->vertices, center_p)) ||
                IsEdgeCollidePoly(poly_ptr->vertices, cedge)) {
                return false;
            }
        }
        // check for unmatched local contours
        for (const auto& contour : ContourGraph::unmatched_contour_) {
            if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                return false;
            }
        }
        // check for any inactive local contours
        for (const auto& contour : ContourGraph::inactive_contour_) {
            if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                return false;
            }
        }
    } else {
        for (const auto& contour : ContourGraph::global_contour_) {
            if (!ContourGraph::IsEdgeOverlapInHeight(h_pair, HeightPair(contour.first, contour.second))) continue;
            if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                return false;
            }
        }
        for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
            if (poly_ptr->is_pillar) continue;
            if (ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, cedge)) {
                return false;
            }
        }
    }
    return true;
}

ConnectPair ContourGraph::ReprojectEdge(const CTNodePtr& ctnode_ptr1, const NavNodePtr& node_ptr2, const float& dist) {
    ConnectPair edgeOut;
    const float ndist = (ctnode_ptr1->position - node_ptr2->position).norm_flat();
    const float ref_dist = std::min(ndist * 0.4f, dist);

    edgeOut.start_p = ProjectNode(ctnode_ptr1, ref_dist);  // node 1
    edgeOut.end_p = ProjectNode(node_ptr2, ref_dist);      // node 2

    return edgeOut;
}

ConnectPair ContourGraph::ReprojectEdge(
    const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const float& dist, const bool& is_global_check) {
    ConnectPair edgeOut;
    const float ndist = (node_ptr1->position - node_ptr2->position).norm_flat();
    const float ref_dist = std::min(ndist * 0.4f, dist);
    // node 1
    if (!is_global_check && node_ptr1->is_contour_match &&
        node_ptr1->ctnode->free_direct == node_ptr1->free_direct) {  // node 1
        const auto ctnode1 = node_ptr1->ctnode;
        edgeOut.start_p = ProjectNode(ctnode1, ref_dist);  // node 1
    } else {
        edgeOut.start_p = ProjectNode(node_ptr1, ref_dist);  // node 1
    }
    // node 2
    if (!is_global_check && node_ptr2->is_contour_match &&
        node_ptr2->ctnode->free_direct == node_ptr2->free_direct) {  // node 2
        const auto ctnode2 = node_ptr2->ctnode;
        edgeOut.end_p = ProjectNode(ctnode2, ref_dist);  // node 1
    } else {
        edgeOut.end_p = ProjectNode(node_ptr2, ref_dist);
    }
    return edgeOut;
}

bool ContourGraph::IsEdgeCollideSegment(const PointPair& line, const ConnectPair& edge) {
    const cv::Point2f start_p(line.first.x, line.first.y);
    const cv::Point2f end_p(line.second.x, line.second.y);
    if (POLYOPS::doIntersect(start_p, end_p, edge.start_p, edge.end_p)) {
        return true;
    }
    return false;
}

bool ContourGraph::IsEdgeCollidePoly(const PointStack& poly, const ConnectPair& edge) {
    const int N = poly.size();
    if (N < 3) cout << "Poly vertex size less than 3." << endl;
    for (int i = 0; i < N; i++) {
        const PointPair line(poly[i], poly[FARUtil::Mod(i + 1, N)]);
        if (ContourGraph::IsEdgeCollideSegment(line, edge)) {
            return true;
        }
    }
    return false;
}

void ContourGraph::AddContourToSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    NavEdge edge(node_ptr1, node_ptr2);
    // force to form pair id1 < id2
    if (node_ptr1->id > node_ptr2->id) edge = NavEdge(node_ptr2, node_ptr1);

    ContourGraph::global_contour_set_.insert(edge);
    if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
        ContourGraph::boundary_contour_set_.insert(edge);
    }
}

void ContourGraph::DeleteContourFromSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    NavEdge edge(node_ptr1, node_ptr2);
    // force to form pair id1 < id2
    if (node_ptr1->id > node_ptr2->id) edge = NavEdge(node_ptr2, node_ptr1);

    ContourGraph::global_contour_set_.erase(edge);
    if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
        ContourGraph::boundary_contour_set_.erase(edge);
    }
}

NavNodePtr ContourGraph::MatchOutrangeNodeWithCTNode(const NavNodePtr& out_node_ptr, const NodePtrStack& near_nodes) {
    if (near_nodes.empty()) return NULL;
    float min_dist = FARUtil::kINF;
    NavNodePtr min_matched_node = NULL;
    for (const auto& node_ptr : near_nodes) {
        if (!node_ptr->is_contour_match) continue;  // 只考虑轮廓匹配的节点
        CTNodePtr matched_ctnode = NULL;
        // 轮廓线匹配验证
        if (IsContourLineMatch(node_ptr, out_node_ptr, matched_ctnode)) {
            // 计算候选节点到参考线（近邻节点-轮廓节点-超范围节点）的垂直距离
            const float dist =
                FARUtil::VerticalDistToLine2D(node_ptr->position, matched_ctnode->position, out_node_ptr->position);
            if (dist < min_dist) {
                min_dist = dist;
                min_matched_node = node_ptr;
            }
        }
    }
    if (min_dist < FARUtil::kNavClearDist) {
        return min_matched_node;
    }
    return NULL;
}

bool ContourGraph::IsContourLineMatch(
    const NavNodePtr& inNode_ptr, const NavNodePtr& outNode_ptr, CTNodePtr& matched_ctnode) {
    // 关键思想：如果两个节点应该连接，那么它们的连线方向应该与环境轮廓的某个段落高度对齐。
    // 近邻导航节点 ←→ 轮廓多边形上的某个点 ←→ 超范围导航节点
    // 由于outNode_ptr是超出范围的轮廓点，如果可以连接，那outNode_ptr应该也是轮廓匹配的点，inNode_ptr应该如果是一个轮廓就能连上
    // 轮廓多边形可能很大，跨越多个处理范围
    const CTNodePtr ctnode_ptr = inNode_ptr->ctnode;
    if (ctnode_ptr == NULL || ctnode_ptr->poly_ptr->is_pillar) return false;
    // cheak forward
    const PointPair line1(inNode_ptr->position, outNode_ptr->position);
    CTNodePtr next_ctnode = ctnode_ptr->front;
    CTNodePtr prev_ctnode = ctnode_ptr;
    while (!next_ctnode->is_global_match && next_ctnode != ctnode_ptr &&
           FARUtil::IsInCylinder(
               ctnode_ptr->position, next_ctnode->position, prev_ctnode->position, FARUtil::kNearDist, true)) {
        if (!FARUtil::IsPointInMarginRange(next_ctnode->position) &&
            (ctnode_ptr->position - next_ctnode->position).norm_flat() > FARUtil::kMatchDist) {
            const PointPair line2(ctnode_ptr->position, next_ctnode->position);
            if (FARUtil::LineMatchPercentage(line1, line2) > 0.99f) {
                if (IsCTMatchLineFreePolygon(next_ctnode, outNode_ptr, true)) {
                    matched_ctnode = next_ctnode;
                    return true;
                }
            }
        }
        prev_ctnode = next_ctnode;
        next_ctnode = next_ctnode->front;
    }
    // check backward
    next_ctnode = ctnode_ptr->back;
    prev_ctnode = ctnode_ptr;
    while (!next_ctnode->is_global_match && next_ctnode != ctnode_ptr &&
           FARUtil::IsInCylinder(
               ctnode_ptr->position, next_ctnode->position, prev_ctnode->position, FARUtil::kNearDist, true)) {
        if (!FARUtil::IsPointInMarginRange(next_ctnode->position) &&
            (ctnode_ptr->position - next_ctnode->position).norm_flat() > FARUtil::kMatchDist) {
            const PointPair line2(ctnode_ptr->position, next_ctnode->position);
            if (FARUtil::LineMatchPercentage(line1, line2) > 0.99f) {
                if (IsCTMatchLineFreePolygon(next_ctnode, outNode_ptr, true)) {
                    matched_ctnode = next_ctnode;
                    return true;
                }
            }
        }
        prev_ctnode = next_ctnode;
        next_ctnode = next_ctnode->back;
    }
    return false;
}

bool ContourGraph::IsNavNodesConnectFromContour(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1->is_odom || node_ptr2->is_odom) return false;
    const CTNodePtr ctnode1 = node_ptr1->ctnode;
    const CTNodePtr ctnode2 = node_ptr2->ctnode;
    if (ctnode1 == NULL || ctnode2 == NULL || ctnode1 == ctnode2) return false;
    return ContourGraph::IsCTNodesConnectFromContour(ctnode1, ctnode2);
}

bool ContourGraph::IsCTNodesConnectFromContour(const CTNodePtr& ctnode1, const CTNodePtr& ctnode2) {
    if (ctnode1 == ctnode2 || ctnode1->poly_ptr != ctnode2->poly_ptr) return false;
    // check for boundary collision
    // 判断是否和传入的障碍相交
    const ConnectPair cedge = ConnectPair(ctnode1->position, ctnode2->position);
    for (const auto& contour : ContourGraph::boundary_contour_) {
        if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
            return false;
        }
    }
    // forward search
    CTNodePtr next_ctnode = ctnode1->front;
    while (next_ctnode != NULL && next_ctnode != ctnode1) {
        if (next_ctnode == ctnode2) {
            return true;
        }
        // 遇到已匹配节点：is_global_match → 被其他导航节点占用，终止
        // 超出合理范围： !IsInCylinder(...) → 偏离直线路径太远，终止
        if (next_ctnode->is_global_match || !FARUtil::IsInCylinder(ctnode1->position, ctnode2->position,
                                                next_ctnode->position, FARUtil::kNearDist, true)) {
            break;
        } else {
            next_ctnode = next_ctnode->front;
        }
    }
    // backward search
    next_ctnode = ctnode1->back;
    while (next_ctnode != NULL && next_ctnode != ctnode1) {
        if (next_ctnode == ctnode2) {
            return true;
        }
        if (next_ctnode->is_global_match || !FARUtil::IsInCylinder(ctnode1->position, ctnode2->position,
                                                next_ctnode->position, FARUtil::kNearDist, true)) {
            break;
        } else {
            next_ctnode = next_ctnode->back;
        }
    }
    return false;
}

bool ContourGraph::IsNavNodesConnectFreePolygon(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1->is_navpoint || node_ptr2->is_navpoint) {
        if ((node_ptr1->position - node_ptr2->position).norm() < FARUtil::kNavClearDist) {  // connect to internav node
            return true;
        }
    }
    const bool is_global_check = ContourGraph::IsNeedGlobalCheck(node_ptr1->position, node_ptr2->position);
    ConnectPair cedge = ContourGraph::ReprojectEdge(node_ptr1, node_ptr2, FARUtil::kProjectDist, is_global_check);
    if (node_ptr1->is_odom) {
        cedge.start_p = cv::Point2f(FARUtil::free_odom_p.x, FARUtil::free_odom_p.y);
    } else if (node_ptr2->is_odom) {
        cedge.end_p = cv::Point2f(FARUtil::free_odom_p.x, FARUtil::free_odom_p.y);
    }
    ConnectPair bd_cedge = cedge;
    const HeightPair h_pair(node_ptr1->position, node_ptr2->position);
    if (!node_ptr1->is_boundary) bd_cedge.start_p = cv::Point2f(node_ptr1->position.x, node_ptr1->position.y);
    if (!node_ptr2->is_boundary) bd_cedge.end_p = cv::Point2f(node_ptr2->position.x, node_ptr2->position.y);
    return ContourGraph::IsPointsConnectFreePolygon(cedge, bd_cedge, h_pair, is_global_check);
}

void ContourGraph::ExtractGlobalContours() {
    ContourGraph::global_contour_.clear();
    ContourGraph::inactive_contour_.clear();
    ContourGraph::unmatched_contour_.clear();
    ContourGraph::boundary_contour_.clear();
    ContourGraph::local_boundary_.clear();
    // 2. 从集合中提取全局轮廓边
    for (const auto& edge : ContourGraph::global_contour_set_) {
        ContourGraph::global_contour_.push_back({edge.first->position, edge.second->position});
        if (IsEdgeInLocalRange(edge.first, edge.second)) {
            if (!this->IsActiveEdge(edge.first, edge.second)) {
                ContourGraph::inactive_contour_.push_back({edge.first->position, edge.second->position});
            } else if (!edge.first->is_near_nodes || !edge.second->is_near_nodes) {
                PointPair unmatched_pair = std::make_pair(edge.first->position, edge.second->position);
                if (edge.first->is_contour_match) {
                    unmatched_pair.first = edge.first->ctnode->position;
                } else if (edge.second->is_contour_match) {
                    unmatched_pair.second = edge.second->ctnode->position;
                }
                ContourGraph::unmatched_contour_.push_back(unmatched_pair);
            }
        }
    }
    for (const auto& edge : ContourGraph::boundary_contour_set_) {
        ContourGraph::boundary_contour_.push_back({edge.first->position, edge.second->position});
        if (IsEdgeInLocalRange(edge.first, edge.second)) {
            ContourGraph::local_boundary_.push_back({edge.first->position, edge.second->position});
            bool is_new_invalid = false;
            if (!IsValidBoundary(edge.first, edge.second, is_new_invalid) && is_new_invalid) {
                edge.first->invalid_boundary.insert(edge.second->id);
                edge.second->invalid_boundary.insert(edge.first->id);
            }
        }
    }
}

bool ContourGraph::IsValidBoundary(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, bool& is_new) {
    is_new = true;
    if (node_ptr1->invalid_boundary.find(node_ptr2->id) != node_ptr1->invalid_boundary.end()) {  // already invalid
        is_new = false;
        return false;
    }
    // check against local polygon
    const ConnectPair cedge = ConnectPair(node_ptr1->position, node_ptr2->position);
    for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
        if (poly_ptr->is_pillar) continue;
        if (ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, cedge)) {
            return false;
        }
    }
    return true;
}