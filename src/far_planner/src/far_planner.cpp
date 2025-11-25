#include "far_planner/far_planner.h"

void FARMaster::LoadROSParams() {
    const std::string master_prefix = "/far_planner/";
    const std::string map_prefix = master_prefix + "MapHandler/";
    const std::string scan_prefix = master_prefix + "ScanHandler/";
    const std::string cdetect_prefix = master_prefix + "CDetector/";
    const std::string graph_prefix = master_prefix + "Graph/";
    const std::string viz_prefix = master_prefix + "Viz/";
    const std::string utility_prefix = master_prefix + "Util/";
    const std::string planner_prefix = master_prefix + "GPlanner/";
    const std::string contour_prefix = master_prefix + "ContourGraph/";
    const std::string msger_prefix = master_prefix + "GraphMsger/";

    // master params
    nh.param<float>(master_prefix + "main_run_freq", master_params_.main_run_freq, 5.0);
    nh.param<float>(master_prefix + "voxel_dim", master_params_.voxel_dim, 0.2);
    nh.param<float>(master_prefix + "robot_dim", master_params_.robot_dim, 0.8);
    nh.param<float>(master_prefix + "vehicle_height", master_params_.vehicle_height, 0.75);
    nh.param<float>(master_prefix + "sensor_range", master_params_.sensor_range, 50.0);
    nh.param<float>(master_prefix + "terrain_range", master_params_.terrain_range, 15.0);
    nh.param<float>(master_prefix + "local_planner_range", master_params_.local_planner_range, 5.0);
    nh.param<float>(master_prefix + "visualize_ratio", master_params_.viz_ratio, 1.0);
    nh.param<bool>(master_prefix + "is_viewpoint_extend", master_params_.is_viewpoint_extend, true);
    nh.param<bool>(master_prefix + "is_multi_layer", master_params_.is_multi_layer, false);
    nh.param<bool>(master_prefix + "is_opencv_visual", master_params_.is_visual_opencv, true);
    nh.param<bool>(master_prefix + "is_static_env", master_params_.is_static_env, true);
    nh.param<bool>(master_prefix + "is_pub_boundary", master_params_.is_pub_boundary, true);
    nh.param<bool>(master_prefix + "is_debug_output", master_params_.is_debug_output, false);
    nh.param<bool>(master_prefix + "is_attempt_autoswitch", master_params_.is_attempt_autoswitch, true);
    nh.param<std::string>(master_prefix + "world_frame", master_params_.world_frame, "map");
    master_params_.terrain_range = std::min(master_params_.terrain_range, master_params_.sensor_range);

    // map handler params
    nh.param<float>(map_prefix + "floor_height", map_params_.floor_height, 2.0);
    nh.param<float>(map_prefix + "cell_length", map_params_.cell_length, 5.0);
    nh.param<float>(map_prefix + "map_grid_max_length", map_params_.grid_max_length, 5000.0);
    nh.param<float>(map_prefix + "map_grad_max_height", map_params_.grid_max_height, 100.0);
    map_params_.height_voxel_dim = master_params_.voxel_dim * 2.0f;
    map_params_.cell_height = map_params_.floor_height / 2.5f;
    map_params_.sensor_range = master_params_.sensor_range;

    // utility params
    nh.param<float>(utility_prefix + "angle_noise", FARUtil::kAngleNoise, 15.0);
    nh.param<float>(utility_prefix + "accept_max_align_angle", FARUtil::kAcceptAlign, 15.0);
    nh.param<float>(utility_prefix + "new_intensity_thred", FARUtil::kNewPIThred, 2.0);
    nh.param<float>(utility_prefix + "nav_clear_dist", FARUtil::kNavClearDist, 0.5);
    nh.param<float>(utility_prefix + "terrain_free_Z", FARUtil::kFreeZ, 0.1);
    nh.param<int>(utility_prefix + "dyosb_update_thred", FARUtil::kDyObsThred, 4);
    nh.param<int>(utility_prefix + "new_point_counter", FARUtil::KNewPointC, 10);
    nh.param<float>(utility_prefix + "dynamic_obs_dacay_time", FARUtil::kObsDecayTime, 10.0);
    nh.param<float>(utility_prefix + "new_points_decay_time", FARUtil::kNewDecayTime, 2.0);
    nh.param<int>(utility_prefix + "obs_inflate_size", FARUtil::kObsInflate, 2);
    FARUtil::kLeafSize = master_params_.voxel_dim;
    FARUtil::kNearDist = master_params_.robot_dim;
    FARUtil::kHeightVoxel = map_params_.height_voxel_dim;
    FARUtil::kMatchDist = master_params_.robot_dim * 2.0f + FARUtil::kLeafSize;
    FARUtil::kNavClearDist = master_params_.robot_dim / 2.0f + FARUtil::kLeafSize;
    FARUtil::kProjectDist = master_params_.voxel_dim;
    FARUtil::worldFrameId = master_params_.world_frame;
    FARUtil::kVizRatio = master_params_.viz_ratio;
    FARUtil::kTolerZ = map_params_.floor_height - FARUtil::kHeightVoxel;
    FARUtil::kCellLength = map_params_.cell_length;
    FARUtil::kCellHeight = map_params_.cell_height;
    FARUtil::kAcceptAlign = FARUtil::kAcceptAlign / 180.0f * M_PI;
    FARUtil::kAngleNoise = FARUtil::kAngleNoise / 180.0f * M_PI;
    FARUtil::robot_dim = master_params_.robot_dim;
    FARUtil::IsStaticEnv = master_params_.is_static_env;
    FARUtil::IsDebug = master_params_.is_debug_output;
    FARUtil::IsMultiLayer = master_params_.is_multi_layer;
    FARUtil::vehicle_height = master_params_.vehicle_height;
    FARUtil::kSensorRange = master_params_.sensor_range;
    FARUtil::kMarginDist = master_params_.sensor_range - FARUtil::kMatchDist;
    FARUtil::kMarginHeight = FARUtil::kTolerZ - FARUtil::kCellHeight / 2.0f;
    FARUtil::kTerrainRange = master_params_.terrain_range;
    FARUtil::kLocalPlanRange = master_params_.local_planner_range;

    // contour detector params
    nh.param<float>(cdetect_prefix + "resize_ratio", cdetect_params_.kRatio, 5.0);
    nh.param<int>(cdetect_prefix + "filter_count_value", cdetect_params_.kThredValue, 5);
    nh.param<bool>(cdetect_prefix + "is_save_img", cdetect_params_.is_save_img, false);
    nh.param<std::string>(cdetect_prefix + "img_folder_path", cdetect_params_.img_path, "");
    cdetect_params_.kBlurSize = (int)std::round(FARUtil::kNavClearDist / master_params_.voxel_dim);
    cdetect_params_.sensor_range = master_params_.sensor_range;
    cdetect_params_.voxel_dim = master_params_.voxel_dim;

    // dynamic graph params
    nh.param<int>(graph_prefix + "connect_votes_size", graph_params_.votes_size, 10);
    nh.param<int>(graph_prefix + "clear_dumper_thred", graph_params_.dumper_thred, 3);
    nh.param<int>(graph_prefix + "node_finalize_thred", graph_params_.finalize_thred, 3);
    nh.param<int>(graph_prefix + "filter_pool_size", graph_params_.pool_size, 12);
    nh.param<float>(graph_prefix + "connect_angle_thred", graph_params_.kConnectAngleThred, 10.0);
    nh.param<float>(graph_prefix + "dirs_filter_margin", graph_params_.filter_dirs_margin, 10.0);
    graph_params_.filter_pos_margin = FARUtil::kNavClearDist;
    graph_params_.filter_dirs_margin = FARUtil::kAngleNoise;
    graph_params_.kConnectAngleThred = FARUtil::kAcceptAlign;
    graph_params_.frontier_perimeter_thred = FARUtil::kMatchDist * 4.0f;

    // contour graph params
    cg_params_.kPillarPerimeter = master_params_.robot_dim * 4.0f;
}

/* allocate static utility PointCloud pointer memory */
PointCloudPtr FARUtil::surround_obs_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::surround_free_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::stack_new_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::cur_new_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::cur_dyobs_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::stack_dyobs_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::cur_scan_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::local_terrain_obs_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr FARUtil::local_terrain_free_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_new_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_filter_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
/* init static utility values */
const float FARUtil::kEpsilon = 1e-7;
const float FARUtil::kINF = std::numeric_limits<float>::max();
std::string FARUtil::worldFrameId;
float FARUtil::kAngleNoise;
Point3D FARUtil::robot_pos;
Point3D FARUtil::odom_pos;
Point3D FARUtil::map_origin;
Point3D FARUtil::free_odom_p;
float FARUtil::robot_dim;
float FARUtil::vehicle_height;
float FARUtil::kLeafSize;
float FARUtil::kHeightVoxel;
float FARUtil::kNavClearDist;
float FARUtil::kCellLength;
float FARUtil::kCellHeight;
float FARUtil::kNewPIThred;
float FARUtil::kSensorRange;
float FARUtil::kMarginDist;
float FARUtil::kMarginHeight;
float FARUtil::kTerrainRange;
float FARUtil::kLocalPlanRange;
float FARUtil::kFreeZ;
float FARUtil::kVizRatio;
double FARUtil::systemStartTime;
float FARUtil::kObsDecayTime;
float FARUtil::kNewDecayTime;
float FARUtil::kNearDist;
float FARUtil::kMatchDist;
float FARUtil::kProjectDist;
int FARUtil::kDyObsThred;
int FARUtil::KNewPointC;
int FARUtil::kObsInflate;
float FARUtil::kTolerZ;
float FARUtil::kAcceptAlign;
bool FARUtil::IsStaticEnv;
bool FARUtil::IsDebug;
bool FARUtil::IsMultiLayer;
TimeMeasure FARUtil::Timer;

/* init terrain map values */
PointKdTreePtr MapHandler::kdtree_terrain_clould_;
std::vector<int> MapHandler::terrain_grid_occupy_list_;
std::vector<int> MapHandler::terrain_grid_traverse_list_;
std::unordered_set<int> MapHandler::neighbor_obs_indices_;
std::unordered_set<int> MapHandler::extend_obs_indices_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_free_cloud_grid_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_obs_cloud_grid_;
std::unique_ptr<grid_ns::Grid<std::vector<float>>> MapHandler::terrain_height_grid_;

/* Global Graph */
DynamicGraphParams DynamicGraph::dg_params_;
NodePtrStack DynamicGraph::globalGraphNodes_;
std::size_t DynamicGraph::id_tracker_;
std::unordered_map<std::size_t, NavNodePtr> DynamicGraph::idx_node_map_;

/* init static contour graph values */
CTNodeStack ContourGraph::polys_ctnodes_;
CTNodeStack ContourGraph::contour_graph_;
PolygonStack ContourGraph::contour_polygons_;
std::vector<PointPair> ContourGraph::global_contour_;
std::vector<PointPair> ContourGraph::unmatched_contour_;
std::vector<PointPair> ContourGraph::inactive_contour_;
std::vector<PointPair> ContourGraph::boundary_contour_;
std::vector<PointPair> ContourGraph::local_boundary_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::global_contour_set_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::boundary_contour_set_;

int main(int argc, char** argv) {
    ros::init(argc, argv, "far_planner_node");
    FARMaster dp_node;
    dp_node.Init();
    dp_node.Loop();
}

void FARMaster::Init() {
    /* initialize subscriber and publisher */
    odom_sub_ = nh.subscribe("/odom_world", 5, &FARMaster::OdomCallBack, this);
    terrain_sub_ = nh.subscribe("/terrain_cloud", 1, &FARMaster::TerrainCallBack, this);

    // DEBUG Publisher
    surround_free_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_free_debug", 1);
    surround_obs_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_obs_debug", 1);

    terrain_height_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_terrain_height_debug", 1);
    this->LoadROSParams();

    /* init Dynamic Planner Processing Objects */
    map_handler_.Init(map_params_);
    planner_viz_.Init(nh);
    contour_detector_.Init(cdetect_params_);
    contour_graph_.Init(cg_params_);
    graph_manager_.Init(nh, graph_params_);
    /* init internal params */
    odom_node_ptr_ = NULL;
    is_cloud_init_ = false;
    is_odom_init_ = false;
    is_reset_env_ = false;
    is_stop_update_ = false;

    // init TF listener
    tf_listener_ = new tf::TransformListener();

    // init global utility cloud
    FARUtil::stack_new_cloud_->clear();
    FARUtil::stack_dyobs_cloud_->clear();

    // allocate memory to pointers
    new_vertices_ptr_ = PointCloudPtr(new PointCloud());
    temp_obs_ptr_ = PointCloudPtr(new PointCloud());
    temp_free_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    temp_cloud_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    terrain_height_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());

    // clear temp vectors and memory
    this->ClearTempMemory();
    FARUtil::robot_pos = Point3D(0, 0, 0);
    FARUtil::free_odom_p = Point3D(0, 0, 0);

    robot_pos_ = Point3D(0, 0, 0);
    nav_heading_ = Point3D(0, 0, 0);
}
void FARMaster::Loop() {
    ros::Rate loop_rate(master_params_.main_run_freq);
    while (ros::ok()) {
        if (is_reset_env_) {
            is_reset_env_ = false;
            continue;
        }
        /* Process callback functions */
        ros::spinOnce();  // 处理回调函数
        if (!this->PreconditionCheck()) {
            loop_rate.sleep();
            continue;
        }
        /*add main process after this line*/
        // 创建/更新odom_node_ptr_
        graph_manager_.UpdateRobotPosition(robot_pos_);
        odom_node_ptr_ = graph_manager_.GetOdomNode();
        if (odom_node_ptr_ == NULL) {
            ROS_WARN("FAR: Waiting for Odometry...");
            loop_rate.sleep();
            continue;
        }
        /* Extract Vertices and new nodes */
        ros::Time start_time = ros::Time::now();
        // 提取轮廓点
        contour_detector_.BuildTerrainImgAndExtractContour(
            odom_node_ptr_, FARUtil::surround_obs_cloud_, realworld_contour_);
        // 将轮廓点转化为ct点
        contour_graph_.UpdateContourGraph(odom_node_ptr_, realworld_contour_);

        map_handler_.AdjustCTNodeHeight(ContourGraph::contour_graph_);
        map_handler_.AdjustNodesHeight(nav_graph_);
        graph_manager_.UpdateGlobalNearNodes();
        near_nav_graph_ = graph_manager_.GetExtendLocalNode();

        // 匹配当前的导航点和ct点，筛选出没有匹配到导航点的新ct点
        contour_graph_.MatchContourWithNavGraph(nav_graph_, near_nav_graph_, new_ctnodes_);
        ROS_INFO("new_ctnodes_: %ld ", new_ctnodes_.size());

        // 相机的可视化
        // if (master_params_.is_visual_opencv) {
        //     FARUtil::ConvertCTNodeStackToPCL(new_ctnodes_, new_vertices_ptr_);
        //     cv::Mat cloud_img = contour_detector_.GetCloudImgMat();
        //     contour_detector_.ShowCornerImage(cloud_img, new_vertices_ptr_);
        // }
        /* update planner graph */
        new_nodes_.clear();
        if (!is_stop_update_ && graph_manager_.ExtractGraphNodes(new_ctnodes_)) {
            new_nodes_ = graph_manager_.GetNewNodes();
        }

        /* Graph Updating */
        graph_manager_.UpdateNavGraph(new_nodes_, is_stop_update_, clear_nodes_);

        nav_graph_ = graph_manager_.GetNavGraph();
        std::cout << "    "
                  << "Number of all vertices adding to global V-Graph: " << nav_graph_.size() << std::endl;

        planner_viz_.VizNodes(nav_graph_, "new_nodes", VizColor::ORANGE);
        planner_viz_.VizContourGraph(ContourGraph::contour_graph_);

        ros::Duration elapsed_time = ros::Time::now() - start_time;
        // ROS_INFO("Elapsed time: %f seconds", elapsed_time.toSec());

        loop_rate.sleep();
    }
}

void FARMaster::OdomCallBack(const nav_msgs::OdometryConstPtr& msg) {
    // transform from odom frame to mapping frame 坐标系转换
    std::string odom_frame = msg->header.frame_id;
    tf::Pose tf_odom_pose;
    tf::poseMsgToTF(msg->pose.pose, tf_odom_pose);
    if (!FARUtil::IsSameFrameID(odom_frame, master_params_.world_frame)) {
        if (FARUtil::IsDebug) ROS_WARN_ONCE("FARMaster: odom frame does NOT match with world frame!");
        tf::StampedTransform odom_to_world_tf_stamp;
        try {
            tf_listener_->waitForTransform(master_params_.world_frame, odom_frame, ros::Time(0), ros::Duration(1.0));
            tf_listener_->lookupTransform(master_params_.world_frame, odom_frame, ros::Time(0), odom_to_world_tf_stamp);
            tf_odom_pose = odom_to_world_tf_stamp * tf_odom_pose;
        } catch (tf::TransformException ex) {
            ROS_ERROR("Tracking odom TF lookup: %s", ex.what());
            return;
        }
    }
    // 提取机器人的位置
    robot_pos_.x = tf_odom_pose.getOrigin().getX();
    robot_pos_.y = tf_odom_pose.getOrigin().getY();
    robot_pos_.z = tf_odom_pose.getOrigin().getZ();
    // extract robot heading
    FARUtil::robot_pos = robot_pos_;
    double roll, pitch, yaw;
    tf_odom_pose.getBasis().getRPY(roll, pitch, yaw);  // 从里程计中提取朝向
    robot_heading_ = Point3D(cos(yaw), sin(yaw), 0);

    if (!is_odom_init_) {
        // system start time
        FARUtil::systemStartTime = ros::Time::now().toSec();
        FARUtil::map_origin = robot_pos_;
        map_handler_.UpdateRobotPosition(robot_pos_);
    }

    is_odom_init_ = true;
}
void FARMaster::PrcocessCloud(const sensor_msgs::PointCloud2ConstPtr& pc, const PointCloudPtr& cloudOut) {
    PointCloud temp_cloud;
    pcl::fromROSMsg(*pc, temp_cloud);
    cloudOut->clear(), *cloudOut = temp_cloud;
    if (cloudOut->empty()) return;
    FARUtil::FilterCloud(cloudOut, master_params_.voxel_dim);
    // transform cloud frame
    std::string cloud_frame = pc->header.frame_id;
    FARUtil::RemoveNanInfPoints(cloudOut);
    if (!FARUtil::IsSameFrameID(cloud_frame, master_params_.world_frame)) {
        if (FARUtil::IsDebug) ROS_WARN_ONCE("FARMaster: cloud frame does NOT match with world frame!");
        try {
            FARUtil::TransformPCLFrame(cloud_frame, master_params_.world_frame, tf_listener_, cloudOut);
        } catch (tf::TransformException ex) {
            ROS_ERROR("Tracking cloud TF lookup: %s", ex.what());
            return;
        }
    }
}

void FARMaster::TerrainCallBack(const sensor_msgs::PointCloud2ConstPtr& pc) {
    if (!is_odom_init_) return;
    map_handler_.UpdateRobotPosition(FARUtil::robot_pos);
    if (!is_stop_update_) {
        this->PrcocessCloud(pc, temp_cloud_ptr_);
        FARUtil::CropBoxCloud(temp_cloud_ptr_, robot_pos_,
            Point3D(master_params_.terrain_range, master_params_.terrain_range, FARUtil::kTolerZ));
        FARUtil::ExtractFreeAndObsCloud(temp_cloud_ptr_, temp_free_ptr_, temp_obs_ptr_);
        map_handler_.UpdateObsCloudGrid(temp_obs_ptr_);
        map_handler_.UpdateFreeCloudGrid(temp_free_ptr_);
        FARUtil::ExtractNewObsPointCloud(temp_obs_ptr_, FARUtil::surround_obs_cloud_, FARUtil::cur_new_cloud_);
    }

    map_handler_.GetSurroundFreeCloud(FARUtil::surround_free_cloud_);
    map_handler_.UpdateTerrainHeightGrid(FARUtil::surround_free_cloud_, terrain_height_ptr_);
    map_handler_.GetSurroundObsCloud(FARUtil::surround_obs_cloud_);

    // create and update kdtrees  和flat点云相关
    FARUtil::StackCloudByTime(FARUtil::cur_new_cloud_, FARUtil::stack_new_cloud_, FARUtil::kNewDecayTime);
    FARUtil::UpdateKdTrees(FARUtil::stack_new_cloud_);

    if (!FARUtil::surround_obs_cloud_->empty()) is_cloud_init_ = true;
    planner_viz_.VizPointCloud(surround_free_debug_, FARUtil::surround_free_cloud_);
    planner_viz_.VizPointCloud(surround_obs_debug_, FARUtil::surround_obs_cloud_);
    planner_viz_.VizPointCloud(terrain_height_pub_, terrain_height_ptr_);
}