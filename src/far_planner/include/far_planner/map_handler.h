#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include "utility.h"
enum CloudType { FREE_CLOUD = 0, OBS_CLOUD = 1 };

struct MapHandlerParams {
    MapHandlerParams() = default;
    float sensor_range;
    float floor_height;
    float cell_length;
    float cell_height;
    float grid_max_length;  // 网格的最大长度（m）
    float grid_max_height;
    // local terrain height map
    float height_voxel_dim;
};

class MapHandler {
public:
    MapHandler() = default;
    ~MapHandler() = default;
    // 主要函数：初始化，设置原点，写入读取格子内点云
    void Init(const MapHandlerParams& params);                  // comp
    void SetMapOrigin(const Point3D& robot_pos);                // comp
    void SetTerrainHeightGridOrigin(const Point3D& robot_pos);  // comp
    /**
     * 作用：
     * 初始化的时候更新地图原点；初始化后更新局部格子索引
     */
    void UpdateRobotPosition(const Point3D& odom_pos);  // comp

    void UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut);  // comp
    void UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn);   // comp
    /**
     * 更新terrain_height_grid_
     * 输出可通行的地形点云的位置
     */
    void UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut);

    void GetSurroundObsCloud(const PointCloudPtr& obsCloudOut);    // comp
    void GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut);  // comp

    // 重置
    void ResetGridMapCloud();  // comp

    void AdjustNodesHeight(const NodePtrStack& nodes);

    void AdjustCTNodeHeight(const CTNodeStack& ctnodes);
    // 判断导航点是不是在地形相关的领域网络中  就是是否在neighbor_obs_indices_索引范围内
    static bool IsNavPointOnTerrainNeighbor(const Point3D& p, const bool& is_extend);

    static float NearestTerrainHeightofNavPoint(const Point3D& point, bool& is_associated);
    /**
     * 查询指定点位置的地形高度
     * p:位置
     * is_matched:是否找到匹配的地形高度
     * is_seatch:是否使用扩展搜索模式
     */
    static float TerrainHeightOfPoint(const Point3D& p, bool& is_matched, const bool& is_search);
    // 在指定半径范围内搜索地形点，返回该区域的平均高度，同时提供最小、最大高度统计信息。 分析地形特征
    template <typename Position>
    static inline float NearestHeightOfRadius(
        const Position& p, const float& radius, float& minH, float& maxH, bool& is_matched) {
        std::vector<int> pIdxK;
        std::vector<float> pdDistK;
        PCLPoint pcl_p;
        pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = 0.0f, pcl_p.intensity = 0.0f;
        minH = maxH = p.z;
        is_matched = false;
        if (kdtree_terrain_clould_->radiusSearch(pcl_p, radius, pIdxK, pdDistK) > 0) {
            float avgH = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[0]].intensity;
            minH = maxH = avgH;
            for (int i = 1; i < pIdxK.size(); i++) {
                const float temp = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[i]].intensity;
                if (temp < minH) minH = temp;
                if (temp > maxH) maxH = temp;
                avgH += temp;
            }
            avgH /= (float)pIdxK.size();
            is_matched = true;
            return avgH;
        }
        return p.z;
    }

    // 次级函数
    /**
     * 提取指定中心点周围领域内的点云数据，根据点云类型（障碍或自由）和领域大小返回合并后的点云
     */
    void GetCloudOfPoint(
        const Point3D& center, const PointCloudPtr& CloudOut, const CloudType& type, const bool& is_large);  // comp

    /**
     * Get neihbor cells center positions
     * @param neighbor_centers[out] neighbor centers stack
     */
    void GetNeighborCeilsCenters(PointStack& neighbor_centers);

    /**
     * Get neihbor cells center positions
     * @param occupancy_centers[out] occupanied cells center stack
     */
    void GetOccupancyCeilsCenters(PointStack& occupancy_centers);

    /**
     * Remove pointcloud from grid map
     * @param obsCloud obstacle cloud points that need to be removed
     */
    void RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud);
    /**
     * 清除机器人位置到指定点之间射线路径上的障碍物点云
     */
    void ClearObsCellThroughPosition(const Point3D& point);  // comp

private:
    MapHandlerParams map_params_;
    int neighbor_Lnum_, neighbor_Hnum_;  // 决定哪些格子是邻居
    Eigen::Vector3i robot_cell_sub_;     // 机器人在全局的图的索引位置
    int INFLATE_N;                       // 障碍物膨胀半径
    bool is_init_ = false;
    // 存储平坦化点云，作为kd数的输入  indensity表示XY位置的地形高度
    PointCloudPtr flat_terrain_cloud_;
    // kd树，用于查询高度
    static PointKdTreePtr kdtree_terrain_clould_;

    // 在地形点云中搜索距离给定位置最近的地形点，并返回高度信息
    template <typename Position>
    static inline float NearestHeightOfPoint(const Position& p, float& dist_square) {
        // Find the nearest node in graph
        std::vector<int> pIdxK(1);
        std::vector<float> pdDistK(1);
        PCLPoint pcl_p;
        dist_square = FARUtil::kINF;
        pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = 0.0f, pcl_p.intensity = 0.0f;
        if (kdtree_terrain_clould_->nearestKSearch(pcl_p, 1, pIdxK, pdDistK) > 0) {
            pcl_p = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[0]];
            dist_square = pdDistK[0];
            return pcl_p.intensity;
        }
        return p.z;
    }

    // 从机器人当前位置出发，向周围扩散寻找高度差在允许范围内的可通行地形，并将结果存储到terrainHeightOut
    void TraversableAnalysis(const PointCloudPtr& terrainHeightOut);
    // 将点云转化为flat点云，z点清空，indensity表示XY位置的地形高度
    inline void AssignFlatTerrainCloud(const PointCloudPtr& terrainRef, PointCloudPtr& terrainFlatOut) {
        const int N = terrainRef->size();
        terrainFlatOut->resize(N);
        for (int i = 0; i < N; i++) {
            PCLPoint pcl_p = terrainRef->points[i];
            pcl_p.intensity = pcl_p.z, pcl_p.z = 0.0f;
            terrainFlatOut->points[i] = pcl_p;
        }
    }
    // 扩展2D平面的sub索引
    inline void Expansion2D(const Eigen::Vector3i& csub, std::vector<Eigen::Vector3i>& subs, const int& n) {
        subs.clear();
        for (int ix = -n; ix <= n; ix++) {
            for (int iy = -n; iy <= n; iy++) {
                Eigen::Vector3i sub = csub;
                sub.x() += ix, sub.y() += iy;
                subs.push_back(sub);
            }
        }
    }
    // 根据地形信息过滤和调整障碍物网格索引。  过滤掉高度与地形不匹配的障碍物网格
    // 为有效的障碍物网格生成向下扩展的索引集合
    // 形成经过高度地形验证的标准领域，以及垂直扩展领域
    void ObsNeighborCloudWithTerrain(
        std::unordered_set<int>& neighbor_obs, std::unordered_set<int>& extend_terrain_obs);

    // 机器人附近的栅格索引
    std::unordered_set<int> neighbor_free_indices_;
    static std::unordered_set<int> neighbor_obs_indices_;
    static std::unordered_set<int> extend_obs_indices_;  // 邻居索引向下扩展一层 包含垂直扩展的扩展邻域

    // 标记数组
    std::vector<int> global_visited_induces_;   // 全局访问，哪些格子被修改过
    std::vector<int> util_obs_modified_list_;   // 临时访问，记录本次哪些障碍格子被访问过
    std::vector<int> util_free_modified_list_;  // 临时访问，记录本次哪些自由格子被访问过
    std::vector<int> util_remove_check_list_;  // 临时访问，记录哪些格子需要检查并移除重叠的障碍点
    // 标记数组 --- terrain_height_grid_
    static std::vector<int> terrain_grid_occupy_list_;    // 标记地形高度数据 被填充
    static std::vector<int> terrain_grid_traverse_list_;  // 标记可通行数据
    // 全局地图
    static std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_free_cloud_grid_;  // 存储自由空间
    static std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_obs_cloud_grid_;   // 存储障碍物
    // 以机器人为重心的局部地图，存储2.5D高度
    static std::unique_ptr<grid_ns::Grid<std::vector<float>>> terrain_height_grid_;
};
#endif