#include "far_planner/map_handler.h"

void MapHandler::Init(const MapHandlerParams& params) {
    map_params_ = params;
    const int row_num = std::ceil(map_params_.grid_max_length / map_params_.cell_length);
    const int col_num = row_num;
    int level_num = std::ceil(map_params_.grid_max_height / map_params_.cell_height);
    neighbor_Lnum_ = std::ceil(map_params_.sensor_range * 2.0f / map_params_.cell_length) + 1;
    neighbor_Hnum_ = 5;
    if (level_num % 2 == 0) level_num++;            // force to odd number, robot will be at center
    if (neighbor_Lnum_ % 2 == 0) neighbor_Lnum_++;  // force to odd number

    // initialize grid
    Eigen::Vector3i pointcloud_grid_size(row_num, col_num, level_num);
    Eigen::Vector3d pointcloud_grid_origin(0, 0, 0);
    Eigen::Vector3d pointcloud_grid_resolution(
        map_params_.cell_length, map_params_.cell_length, map_params_.cell_height);
    PointCloudPtr cloud_ptr_tmp;
    world_free_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);
    world_obs_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);
    const int n_cell = world_free_cloud_grid_->GetCellNumber();
    for (int i = 0; i < n_cell; i++) {
        world_free_cloud_grid_->GetCell(i) = PointCloudPtr(new PointCloud);
        world_obs_cloud_grid_->GetCell(i) = PointCloudPtr(new PointCloud);
    }
    global_visited_induces_.resize(n_cell), util_remove_check_list_.resize(n_cell);
    util_obs_modified_list_.resize(n_cell), util_free_modified_list_.resize(n_cell);
    std::fill(global_visited_induces_.begin(), global_visited_induces_.end(), 0);
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);

    // init terrain height map
    int height_dim = std::ceil((map_params_.sensor_range + map_params_.cell_height) * 2.0f / FARUtil::robot_dim);
    if (height_dim % 2 == 0) height_dim++;
    Eigen::Vector3i height_grid_size(height_dim, height_dim, 1);
    Eigen::Vector3d height_grid_origin(0, 0, 0);
    Eigen::Vector3d height_grid_resolution(FARUtil::robot_dim, FARUtil::robot_dim, FARUtil::kLeafSize);
    std::vector<float> temp_vec;
    terrain_height_grid_ = std::make_unique<grid_ns::Grid<std::vector<float>>>(
        height_grid_size, temp_vec, height_grid_origin, height_grid_resolution, 3);
    const int n_terrain_cell = terrain_height_grid_->GetCellNumber();
    terrain_grid_occupy_list_.resize(n_terrain_cell), terrain_grid_traverse_list_.resize(n_terrain_cell);
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0.0f);
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0.0f);
    // init others param
    INFLATE_N = 1;
    flat_terrain_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    kdtree_terrain_clould_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
    kdtree_terrain_clould_->setSortedResults(false);  // 搜索结果不按距离排序
}

void MapHandler::ResetGridMapCloud() {
    const int n_cell = world_free_cloud_grid_->GetCellNumber();
    for (int i = 0; i < n_cell; i++) {
        world_free_cloud_grid_->GetCell(i)->clear();
        world_obs_cloud_grid_->GetCell(i)->clear();
    }
    std::fill(global_visited_induces_.begin(), global_visited_induces_.end(), 0);
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0.0f);
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0.0f);
}

void MapHandler::SetMapOrigin(const Point3D& robot_pos) {
    Point3D map_origen;
    const Eigen::Vector3i dim = world_free_cloud_grid_->GetSize();
    map_origen.x = robot_pos.x - (map_params_.cell_length * dim.x()) / 2.0f;
    map_origen.y = robot_pos.y - (map_params_.cell_length * dim.y()) / 2.0f;
    map_origen.z =
        robot_pos.z - (map_params_.cell_height * dim.z()) / 2.0f - FARUtil::vehicle_height;  // From Ground Level
    Eigen::Vector3d origen(map_origen.x, map_origen.y, map_origen.z);
    world_free_cloud_grid_->SetOrigin(origen);
    world_obs_cloud_grid_->SetOrigin(origen);
    is_init_ = true;
    if (FARUtil::IsDebug) ROS_INFO("MH: Global Cloud Map Grid Initialized.");
}
void MapHandler::UpdateRobotPosition(const Point3D& odom_pos) {
    if (!is_init_) this->SetMapOrigin(odom_pos);
    robot_cell_sub_ = world_free_cloud_grid_->Pos2Sub(Eigen::Vector3d(odom_pos.x, odom_pos.y, odom_pos.z));
    neighbor_free_indices_.clear(), neighbor_obs_indices_.clear();
    int N = neighbor_Lnum_ / 2;
    int H = neighbor_Hnum_ / 2;
    Eigen::Vector3i neighbor_sub;
    for (int i = -N; i <= N; i++) {
        neighbor_sub.x() = robot_cell_sub_.x() + i;
        for (int j = -N; j <= N; j++) {
            neighbor_sub.y() = robot_cell_sub_.y() + j;
            neighbor_sub.z() = robot_cell_sub_.z() - H - 1;
            if (world_free_cloud_grid_->InRange(neighbor_sub)) {
                int ind = world_free_cloud_grid_->Sub2Ind(neighbor_sub);
                neighbor_free_indices_.insert(ind);
            }
            for (int k = -H; k <= H; k++) {
                neighbor_sub.z() = robot_cell_sub_.z() + k;
                if (world_free_cloud_grid_->InRange(neighbor_sub)) {
                    int ind = world_free_cloud_grid_->Sub2Ind(neighbor_sub);
                    neighbor_obs_indices_.insert(ind);
                    neighbor_free_indices_.insert(ind);
                }
            }
        }
    }
    this->SetTerrainHeightGridOrigin(odom_pos);
}
void MapHandler::SetTerrainHeightGridOrigin(const Point3D& robot_pos) {
    const Eigen::Vector3d res = terrain_height_grid_->GetResolution();
    const Eigen::Vector3i dim = terrain_height_grid_->GetSize();
    Point3D map_origin;
    map_origin.x = robot_pos.x - res.x() * dim.x() / 2.0f;
    map_origin.y = robot_pos.y - res.y() * dim.y() / 2.0f;
    map_origin.z = 0.0 - res.z() * dim.z() / 2.0f;
    terrain_height_grid_->SetOrigin(Eigen::Vector3d(map_origin.x, map_origin.y, map_origin.z));
}

void MapHandler::GetSurroundObsCloud(const PointCloudPtr& obsCloudOut) {
    if (!is_init_) return;
    obsCloudOut->clear();
    for (const auto& neighbor_ind : neighbor_obs_indices_) {
        if (world_obs_cloud_grid_->GetCell(neighbor_ind)->empty()) continue;
        *obsCloudOut += *(world_obs_cloud_grid_->GetCell(neighbor_ind));
    }
    obsCloudOut->header.frame_id = FARUtil::worldFrameId;  // 设置为世界坐标系
    obsCloudOut->header.stamp = pcl_conversions::toPCL(ros::Time::now());
}
void MapHandler::GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut) {
    if (!is_init_) return;
    freeCloudOut->clear();
    for (const auto& neighbor_ind : neighbor_free_indices_) {
        if (world_free_cloud_grid_->GetCell(neighbor_ind)->empty()) continue;
        *freeCloudOut += *(world_free_cloud_grid_->GetCell(neighbor_ind));
    }
    freeCloudOut->header.frame_id = FARUtil::worldFrameId;  // 设置为世界坐标系
    freeCloudOut->header.stamp = pcl_conversions::toPCL(ros::Time::now());
}

void MapHandler::UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut) {
    if (!is_init_ || obsCloudInOut->empty()) return;
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    PointCloudPtr obs_valid_ptr(new PointCloud());
    for (const auto& point : obsCloudInOut->points) {
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_obs_cloud_grid_->InRange(sub)) continue;
        const int ind = world_obs_cloud_grid_->Sub2Ind(sub);
        if (neighbor_obs_indices_.find(ind) != neighbor_obs_indices_.end()) {
            world_obs_cloud_grid_->GetCell(ind)->push_back(point);
            obs_valid_ptr->push_back(point);
            util_obs_modified_list_[ind] = 1;
            global_visited_induces_[ind] = 1;
        }
    }
    *obsCloudInOut = *obs_valid_ptr;
    // 降采样
    for (int i = 0; i < world_obs_cloud_grid_->GetCellNumber(); i++) {
        if (util_obs_modified_list_[i] == 1)
            FARUtil::FilterCloud(world_obs_cloud_grid_->GetCell(i), FARUtil::kLeafSize);
    }
}
void MapHandler::UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn) {
    if (!is_init_ || freeCloudIn->empty()) return;
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    for (const auto& point : freeCloudIn->points) {
        Eigen::Vector3i sub = world_free_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_free_cloud_grid_->InRange(sub)) continue;
        const int ind = world_free_cloud_grid_->Sub2Ind(sub);
        world_free_cloud_grid_->GetCell(ind)->points.push_back(point);
        util_free_modified_list_[ind] = 1;
        global_visited_induces_[ind] = 1;
    }
    // Filter Modified Ceils
    for (int i = 0; i < world_free_cloud_grid_->GetCellNumber(); ++i) {
        if (util_free_modified_list_[i] == 1)
            FARUtil::FilterCloud(world_free_cloud_grid_->GetCell(i), FARUtil::kLeafSize);
    }
}

void MapHandler::GetCloudOfPoint(
    const Point3D& center, const PointCloudPtr& cloudOut, const CloudType& type, const bool& is_large) {
    cloudOut->clear();
    Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(center.x, center.y, center.z);
    int N = is_large ? 1 : 0;
    int H = neighbor_Hnum_ / 2;
    for (int i = -N; i <= N; i++) {
        for (int j = -N; j <= N; j++) {
            for (int k = -H; k <= H; k++) {
                Eigen::Vector3i csub = sub;
                csub.x() += i, csub.y() += j, csub.z() += k;
                if (!world_obs_cloud_grid_->InRange(csub)) continue;
                if (type == CloudType::FREE_CLOUD) {
                    *cloudOut += *(world_free_cloud_grid_->GetCell(csub));
                } else if (type == CloudType::OBS_CLOUD) {
                    *cloudOut += *(world_obs_cloud_grid_->GetCell(csub));
                } else {
                    if (FARUtil::IsDebug) ROS_ERROR("MH: Assigned cloud type invalid.");
                    return;
                }
            }
        }
    }
}
// 通过位置清除点云
void MapHandler::ClearObsCellThroughPosition(const Point3D& point) {
    const Eigen::Vector3i psub = world_obs_cloud_grid_->Pos2Sub(point.x, point.y, point.z);
    std::vector<Eigen::Vector3i> ray_subs;
    world_obs_cloud_grid_->RayTraceSubs(robot_cell_sub_, psub, ray_subs);
    const int H = neighbor_Hnum_ / 2;
    for (const auto& sub : ray_subs) {
        for (int k = -H; k <= H; k++) {
            Eigen::Vector3i csub = sub;
            csub.z() += k;
            const int ind = world_obs_cloud_grid_->Sub2Ind(csub);
            if (!world_obs_cloud_grid_->InRange(csub) || neighbor_obs_indices_.find(ind) == neighbor_obs_indices_.end())
                continue;
            world_obs_cloud_grid_->GetCell(ind)->clear();
            if (world_free_cloud_grid_->GetCell(ind)->empty()) {
                global_visited_induces_[ind] = 0;
            }
        }
    }
}

/*Terrain点云相关 */

void MapHandler::UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut) {
    if (freeCloudIn->empty()) return;
    PointCloudPtr copy_free_ptr(new pcl::PointCloud<PCLPoint>());
    pcl::copyPointCloud(*freeCloudIn, *copy_free_ptr);
    FARUtil::FilterCloud(copy_free_ptr, terrain_height_grid_->GetResolution());
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0);
    for (const auto& point : copy_free_ptr->points) {
        Eigen::Vector3i csub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, 0.0f));
        std::vector<Eigen::Vector3i> subs;
        this->Expansion2D(csub, subs, INFLATE_N);
        for (const auto& sub : subs) {
            if (!terrain_height_grid_->InRange(sub)) continue;
            const int ind = terrain_height_grid_->Sub2Ind(sub);
            if (terrain_grid_occupy_list_[ind] == 0) {
                terrain_height_grid_->GetCell(ind).resize(1);
                terrain_height_grid_->GetCell(ind)[0] = point.z;
            } else {
                terrain_height_grid_->GetCell(ind).push_back(point.z);
            }
            terrain_grid_occupy_list_[ind] = 1;
        }
    }
    const int N = terrain_grid_occupy_list_.size();
    this->TraversableAnalysis(terrainHeightOut);
    terrainHeightOut->header.frame_id = FARUtil::worldFrameId;  // 设置为世界坐标系
    terrainHeightOut->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    if (terrainHeightOut->empty()) {
        FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
    } else {
        this->AssignFlatTerrainCloud(terrainHeightOut, flat_terrain_cloud_);
        kdtree_terrain_clould_->setInputCloud(flat_terrain_cloud_);
    }
    this->ObsNeighborCloudWithTerrain(neighbor_obs_indices_, extend_obs_indices_);
}
void MapHandler::TraversableAnalysis(const PointCloudPtr& terrainHeightOut) {
    const Eigen::Vector3i robot_sub =
        terrain_height_grid_->Pos2Sub(Eigen::Vector3d(FARUtil::robot_pos.x, FARUtil::robot_pos.y, 0.0f));
    terrainHeightOut->clear();
    if (!terrain_height_grid_->InRange(robot_sub)) {
        ROS_ERROR("MH: terrain height analysis error: robot position is not in range");
        return;
    }
    const float H_THREAD = map_params_.height_voxel_dim;
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0);
    // Lambda Function
    // 判断相邻网格是否可通行  所谓的可通行就是查看点云是否是和车的位置接近，就是是不是地面点
    auto IsTraversableNeighbor = [&](const int& cur_id, const int& ref_id) {
        if (terrain_grid_occupy_list_[ref_id == 0]) return false;
        const float cur_h = terrain_height_grid_->GetCell(cur_id)[0];
        float ref_h = 0.0f;
        int counter = 0;
        for (const auto& e : terrain_height_grid_->GetCell(ref_id)) {
            if (abs(e - cur_h) > H_THREAD) continue;
            ref_h += e, counter++;
        }
        if (counter > 0) {
            terrain_height_grid_->GetCell(ref_h).resize(1);
            terrain_height_grid_->GetCell(ref_h)[0] = ref_h / counter;
            return true;
        }
        return false;
    };
    // 将可通行的地形点添加到输出点云中
    auto AddTraversePoint = [&](const int& idx) {
        Eigen::Vector3d cpos = terrain_height_grid_->Ind2Pos(idx);
        cpos.z() = terrain_height_grid_->GetCell(idx)[0];
        const PCLPoint p = FARUtil::Point3DToPCLPoint(Point3D(cpos));
        terrainHeightOut->points.push_back(p);
        terrain_grid_traverse_list_[idx] = 1;
    };
    // BFS搜索初始化
    const int robot_idx = terrain_height_grid_->Sub2Ind(robot_sub);
    const std::array<int, 4> dx = {-1, 0, 1, 0};
    const std::array<int, 4> dy = {0, 1, 0, -1};
    std::deque<int> q;
    bool is_robot_terrain_init = false;
    std::unordered_set<int> visited_set;
    q.push_back(robot_idx), visited_set.insert(robot_idx);
    while (!q.empty()) {
        const int cur_id = q.front();
        q.pop_front();
        if (terrain_grid_occupy_list_[cur_id] != 0) {
            if (!is_robot_terrain_init) {
                float avg_h = 0.0f;
                int counter = 0;
                for (const auto& e : terrain_height_grid_->GetCell(cur_id)) {
                    if (abs(e - FARUtil::robot_pos.z + FARUtil::vehicle_height) > H_THREAD) continue;
                    avg_h += e, counter++;
                }
                if (counter > 0) {
                    avg_h /= counter;
                    terrain_height_grid_->GetCell(cur_id).resize(1);
                    terrain_height_grid_->GetCell(cur_id)[0] = avg_h;
                    AddTraversePoint(cur_id);
                    is_robot_terrain_init = true;
                    q.clear();
                }
            } else {
                AddTraversePoint(cur_id);
            }
        } else if (is_robot_terrain_init) {
            continue;
        }
        const Eigen::Vector3i csub = terrain_height_grid_->Ind2Sub(cur_id);
        for (int i = 0; i < 4; i++) {
            Eigen::Vector3i ref_sub = csub;
            ref_sub.x() += dx[i], ref_sub.y() += dy[i];
            if (!terrain_height_grid_->InRange(ref_sub)) continue;
            const int ref_id = terrain_height_grid_->Sub2Ind(ref_sub);
            if (!visited_set.count(ref_id) && (!is_robot_terrain_init || IsTraversableNeighbor(cur_id, ref_id))) {
                q.push_back(ref_id);
                visited_set.insert(ref_id);
            }
        }
    }
}

void MapHandler::ObsNeighborCloudWithTerrain(
    std::unordered_set<int>& neighbor_obs, std::unordered_set<int>& extend_terrain_obs) {
    std::unordered_set<int> neighbor_copy = neighbor_obs;
    neighbor_obs.clear();
    const float R = map_params_.cell_length * 0.7071f;
    for (const auto& idx : neighbor_copy) {
        const Point3D pos = Point3D(world_obs_cloud_grid_->Ind2Pos(idx));
        const Eigen::Vector3i sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(pos.x, pos.y, 0.0f));
        const int terrain_ind = terrain_height_grid_->Sub2Ind(sub);
        bool inRange = false;
        float minH, maxH;
        const float avgH = NearestHeightOfRadius(pos, R, minH, maxH, inRange);
        if (inRange && pos.z + map_params_.cell_height > minH &&
            pos.z - map_params_.cell_height < maxH + FARUtil::kTolerZ) {
            neighbor_obs.insert(idx);
        }
    }

    extend_terrain_obs.clear();
    const std::vector<int> inflate_vec{-1, 0};
    for (const int& idx : neighbor_obs) {
        const Eigen::Vector3i csub = world_obs_cloud_grid_->Ind2Sub(idx);
        for (const int& plus : inflate_vec) {
            Eigen::Vector3i sub = csub;
            sub.z() += plus;
            if (!world_obs_cloud_grid_->InRange(sub)) continue;
            const int plus_idx = world_obs_cloud_grid_->Sub2Ind(sub);
            extend_terrain_obs.insert(plus_idx);
        }
    }
}

void MapHandler::AdjustNodesHeight(const NodePtrStack& nodes) {}

void MapHandler::AdjustCTNodeHeight(const CTNodeStack& ctnodes) {}

bool MapHandler::IsNavPointOnTerrainNeighbor(const Point3D& p, const bool& is_extend) {
    const float h = p.z - FARUtil::vehicle_height;
    const Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(p.x, p.y, h));
    if (!world_obs_cloud_grid_->InRange(sub)) return false;
    const int ind = world_obs_cloud_grid_->Sub2Ind(sub);
    if (is_extend && extend_obs_indices_.find(ind) != extend_obs_indices_.end()) {
        return true;
    }
    if (!is_extend && neighbor_obs_indices_.find(ind) != neighbor_obs_indices_.end()) {
        return true;
    }
    return false;
}

float MapHandler::NearestTerrainHeightofNavPoint(const Point3D& point, bool& is_associated) {
    return 0.0f;
}

float MapHandler::TerrainHeightOfPoint(const Point3D& p, bool& is_matched, const bool& is_search) {
    is_matched = false;
    const Eigen::Vector3i sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(p.x, p.y, 0.0f));
    if (terrain_height_grid_->InRange(sub)) {
        const int ind = terrain_height_grid_->Sub2Ind(sub);
        if (terrain_grid_traverse_list_[ind] != 0) {
            is_matched = true;
            return terrain_height_grid_->GetCell(ind)[0];
        }
    }
    if (is_search) {
        float matched_dist_squre;
        const float terrain_h = NearestHeightOfPoint(p, matched_dist_squre);
        return terrain_h;
    }

    return 0.0f;
}
void MapHandler::GetNeighborCeilsCenters(PointStack& neighbor_centers) {
    if (!is_init_) return;
    neighbor_centers.clear();
    for (const auto& ind : neighbor_obs_indices_) {
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        neighbor_centers.push_back(center_p);
    }
}
void MapHandler::GetOccupancyCeilsCenters(PointStack& occupancy_centers) {
    if (!is_init_) return;
    occupancy_centers.clear();
    const int N = world_obs_cloud_grid_->GetCellNumber();
    for (int ind = 0; ind < N; ind++) {
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        occupancy_centers.push_back(center_p);
    }
}
void MapHandler::RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud) {}
