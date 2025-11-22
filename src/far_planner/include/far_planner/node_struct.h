#pragma once

#include <unordered_set>

#include "point_struct.h"
//节点类型：未定义，地面上，空中    （没怎么用到）
enum NodeType { NOT_DEFINED = 0, GROUND = 1, AIR = 2 };
//节点的自由方向：对轮廓点进行分类：
//未知，凸点，凹点，柱子（孤立小型障碍物，通常一个点表示）
enum NodeFreeDirect { UNKNOW = 0, CONVEX = 1, CONCAVE = 2, PILLAR = 3 };
typedef std::pair<Point3D, Point3D> PointPair;
//激光雷达的物理参数
namespace LiDARMODEL {
constexpr int kHorizontalFOV = 360;
constexpr int kVerticalFOV = 31;
constexpr float kAngleResX = 0.2;
constexpr float kAngleResY = 360;

}  // namespace LiDARMODEL
//多边形
struct Polygon {
    Polygon() = default;
    std::size_t N;                  //顶点数量
    std::vector<Point3D> vertices;  //顶点集合
    bool is_robot_inside;           //机器人是否在多边形内部  判断凹凸的
    bool is_pillar;
    float perimeter;  //周长
};

typedef std::shared_ptr<Polygon> PolygonPtr;
typedef std::vector<PolygonPtr> PolygonStack;
//轮廓点
struct CTNode {
    CTNode() = default;
    Point3D position;
    bool is_global_match;        //  是否被NavNode认领
    bool is_contour_necessary;   //  是否十分重要,即使不是清晰的凸点也假如到NavNode中
    bool is_ground_associate;    //  是否和地形关联
    std::size_t nav_node_id;     //  如果匹配到了NavNode,这个导航点的id
    NodeFreeDirect free_direct;  //  顶点的几何分类

    PointPair surf_dirs;            //  存储构成这个角的两条边的方向向量
    PolygonPtr poly_ptr;            //  它所在的Polygon
    std::shared_ptr<CTNode> front;  //  同一个轮廓的前一个节点
    std::shared_ptr<CTNode> back;   //  同一个轮廓上的后一个节点

    std::vector<std::shared_ptr<CTNode>> connect_nodes;  //  几乎没用到
};
typedef std::shared_ptr<CTNode> CTNodePtr;
typedef std::vector<CTNodePtr> CTNodeStack;

//导航节点
struct NavNode {
    NavNode() = default;
    std::size_t id;
    Point3D position;                    //  滤波后的位置
    PointPair surf_dirs;                 //  存储构成这个角的两条边的方向向量
    std::deque<Point3D> pos_filter_vec;  //  一个滑动窗口，储存最近几次匹配到的CTNode的位置，用于确认position
    std::deque<PointPair> surf_dirs_vec;  //  用于滤波surf_diirs
    CTNodePtr ctnode;                     //  匹配的CTNode
    bool is_active;                       //  是否被激活，就是是否在传感器范围或被机器人访问过
    bool is_block_frontier;               //  是否阻止成为前沿   去噪的，不让孤立点称为前沿
    bool is_contour_match;                //  ctnode节点指针是否有效
    bool is_odom;                         //  是否为当前位置
    bool is_goal;                         //  是否是目标节点
    bool is_near_nodes;                   //  是否在机器人处理范围内
    bool is_wide_near;                    //  是否在机器人扩展处理范围内
    bool is_merged;                       //  标记是否应被删除
    bool is_covered;                      //  是否位于已探索的自由空间
    //  定义前沿点：一个**“值得探索的”、“稳定的”、“非噪声的”、位于感知边界的凸点**
    bool is_frontier;  //  是否是前沿点 位于已探索区域边缘，具有潜在探索价值的导航节点。  基于探索价值和几何特征
    bool is_finalized;       //  节点的位置方向是否稳定（pos_filter_vec），一般稳定后不再更新
    bool is_navpoint;        //  是否是机器人走过的轨迹点
    bool is_boundary;        //  边界点  节点是否位于探索边界（已知/未知分界线）
    int clear_dumper_count;  //  计数器，如果一个节点多帧未被扫描会增加，结合is_merged
    std::deque<int> frontier_votes;  //  投票决定is_frontier
    std::unordered_set<std::size_t> invalid_boundary;  //  存储与我相连但被当前障碍物阻挡的其他边接节点id 动态障碍物
    std::vector<std::shared_ptr<NavNode>> connect_nodes;  //  图的边  最终可通行的图，是A*算法在这个搜索路径的
    //  这些的区别是边的不同来源，是connect_nodes的子集
    std::vector<std::shared_ptr<NavNode>>
        poly_connects;  //  通过视线检查存储的边  DynamicGraph 中的 IsValidConnect 函数会调用
                        //  ContourGraph::IsNavNodesConnectFreePolygon 来做碰撞检测。   核心 edge_votes成功的点
    std::vector<std::shared_ptr<NavNode>> contour_connects;  //  轮廓连接，存储贴着墙走的边。沿着障碍物边缘的连接
                                                             //  ContourGraph::IsNavNodesConnectFromContour
    std::vector<std::shared_ptr<NavNode>> trajectory_connects;  //  机器人走过的连接，置信度最高

    std::unordered_map<std::size_t, std::deque<int>> contour_votes;
    std::unordered_map<std::size_t, std::deque<int>> edge_votes;
    //  候选的边  目的是为了优化，防止节点太多计算量太大。
    std::vector<std::shared_ptr<NavNode>>
        potential_contours;  //  候选可见的边，存储所有曾经行过可见行投票edge_votes的节点
    std::vector<std::shared_ptr<NavNode>> potential_edges;  //  存储候选的贴墙边
    //  负面投票，在动态或复杂环境中断开已建立的连接
    std::unordered_map<std::size_t, std::size_t> trajectory_votes;  //  用于断开trajectory_connects的
    std::unordered_map<std::size_t, std::size_t> terrain_votes;     //  用于断开地形不适合的poly_connects边
    NodeType node_type;
    NodeFreeDirect free_direct;
    // planner members
    bool is_block_to_goal;  // 用于优化连接目标点这个操作。  这个节点是否被阻止连接到当前目标点
    bool is_traversable;                   //  节点是否可达
    bool is_free_traversable;              //  节点是否可达
    float gscore, fgscore;                 //  A*算法的代价
    std::shared_ptr<NavNode> parent;       //  A*算法找到的路径中上一个节点
    std::shared_ptr<NavNode> free_parent;  //  A*算法找到的路径中上一个节点
};
typedef std::shared_ptr<NavNode> NavNodePtr;
typedef std::pair<NavNodePtr, NavNodePtr> NavEdge;

struct nodeptr_equal {
    bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const {
        return n1->id == n2->id;
    }
};

struct navedge_hash {
    std::size_t operator()(const NavEdge& nav_edge) {
        return boost::hash<std::pair<std::size_t, std::size_t>>()({nav_edge.first->id, nav_edge.second->id});
    }
};

struct nodeptr_hash {
    std::size_t operator()(const NavNodePtr& n_ptr) {
        return std::hash<std::size_t>()(n_ptr->id);
    }
};

struct nodeptr_gcomp {
    bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const {
        return n1->gscore > n2->gscore;
    }
};

struct nodeptr_fgcomp {
    bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const {
        return n1->fgscore > n2->fgscore;
    }
};

struct nodeptr_icomp {
    bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const {
        return n1->position.intensity < n2->position.intensity;
    }
};