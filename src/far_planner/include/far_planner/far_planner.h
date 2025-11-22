#pragma once
#include <ros/ros.h>

#include "far_planner/contour_detector.h"
#include "far_planner/contour_graph.h"
#include "far_planner/dynamic_graph.h"
#include "far_planner/map_handler.h"
#include "far_planner/planner_visualizer.h"
#include "utility.h"

struct FARMasterParams {
    FARMasterParams() = default;
    float robot_dim;            // 机器人的半径物理尺寸 米
    float vehicle_height;       // 机器人的高度
    float voxel_dim;            // 体素地图的分辨率，用于降采样
    float sensor_range;         // 雷达的感知半径
    float terrain_range;        // 处理地形点云的半径
    float local_planner_range;  // 底层局部规划器的工作范围
    float main_run_freq;        // 规划器主循环运行频率
    float viz_ratio;            // 可视化比例因子 缩放标记大小
    bool is_multi_layer;        // 是否使用多层规划
    bool is_viewpoint_extend;
    bool is_visual_opencv;  // 是否使用opencv的可视化窗口
    bool is_static_env;     // 环境是否静态的
    bool is_pub_boundary;   // 是否发布局部边界
    bool is_debug_output;
    bool is_attempt_autoswitch;
    std::string world_frame;  // 世界坐标系的名字
};

class FARMaster {
public:
    FARMaster() = default;
    ~FARMaster() = default;
    void Init();
    void Loop();

private:
    ros::NodeHandle nh;

    // ROS订阅者
    ros::Subscriber odom_sub_, terrain_sub_;

    ros::Publisher surround_free_debug_, surround_obs_debug_;
    ros::Publisher terrain_height_pub_;

    // 状态变量
    Point3D robot_pos_, robot_heading_, nav_heading_;
    bool is_reset_env_, is_stop_update_;
    bool is_odom_init_ = false;
    bool is_cloud_init_ = false;  // 点云是否初始化
    // 临时点云，用于TerrainCallBack中的数据中转
    PointCloudPtr new_vertices_ptr_;
    PointCloudPtr temp_obs_ptr_;        //
    PointCloudPtr temp_free_ptr_;       //
    PointCloudPtr temp_cloud_ptr_;      //
    PointCloudPtr terrain_height_ptr_;  // 存储可通行地形的体素位置

    /* veiwpoint extension clouds */
    NavNodePtr odom_node_ptr_ = NULL;
    NavNodePtr nav_node_ptr = NULL;
    NodePtrStack new_nodes_;
    NodePtrStack nav_graph_;
    NodePtrStack near_nav_graph_;
    NodePtrStack clear_nodes_;

    CTNodeStack new_ctnodes_;
    std::vector<PointStack> realworld_contour_;

    // (tf_listener_ 最好也加上，OdomCallBack 会用)
    tf::TransformListener* tf_listener_;

    // 其他模块实例
    MapHandler map_handler_;
    DPVisualizer planner_viz_;
    ContourDetector contour_detector_;
    ContourGraph contour_graph_;
    DynamicGraph graph_manager_;

    /*ROS param*/
    FARMasterParams master_params_;
    MapHandlerParams map_params_;
    ContourDetectParams cdetect_params_;
    DynamicGraphParams graph_params_;
    ContourGraphParams cg_params_;

    // 回调函数
    void OdomCallBack(const nav_msgs::OdometryConstPtr& msg);
    void TerrainCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);

    /*-------辅助函数-------*/
    void LoadROSParams();
    // 点云的TF变换以及降采样
    void PrcocessCloud(const sensor_msgs::PointCloud2ConstPtr& pc, const PointCloudPtr& cloudOut);

    /* define inline functions */
    inline bool PreconditionCheck() {
        if (is_odom_init_ && is_cloud_init_) {
            return true;
        }
        return false;
    }

    inline void ClearTempMemory() {
        new_nodes_.clear();
        clear_nodes_.clear();
        near_nav_graph_.clear();
        realworld_contour_.clear();
    }
};