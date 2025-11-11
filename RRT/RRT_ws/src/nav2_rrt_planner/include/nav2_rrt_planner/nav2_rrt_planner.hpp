#ifndef NAV2_RRT_PLANNER__NAV2_RRT_PLANNER_HPP_
#define NAV2_RRT_PLANNER__NAV2_RRT_PLANNER_HPP_

#include <string>
#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace nav2_rrt_planner
{
    struct Node
    {
        double x;
        double y;
        int parent_index;
    };
    class RRT : public nav2_core::GlobalPlanner
    {
    public:
        RRT() = default;
        ~RRT() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        /**
         * @brief 清理规划器资源
         */
        void cleanup() override;

        /**
         * @brief 激活规划器
         */
        void activate() override;

        /**
         * @brief 停用规划器
         */
        void deactivate() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal) override;

    private:
        int findNearNode(std::vector<Node> tree, double x, double y);

        geometry_msgs::msg::PoseStamped generateRandomPoint(
            const nav2_costmap_2d::Costmap2D *costmap,
            std::default_random_engine &rng);

        Node expandNode(Node start_node, geometry_msgs::msg::PoseStamped random_point);
        void PathSmooth(std::vector<geometry_msgs::msg::PoseStamped> &Path, int iterations);

        bool isNodeSafe(double x, double y);

        std::shared_ptr<tf2_ros::Buffer> tf_;      // TF缓冲器，用于坐标变换
        nav2_util::LifecycleNode::SharedPtr node_; // 生命周期节点指针
        nav2_costmap_2d::Costmap2D *costmap_;      // 代价地图指针
        std::string global_frame_;                 // 全局坐标系名称
        std::string name_;                         // 规划器名称
        double interpolation_resolution_;          // 插值分辨率
        bool use_smoothing;
    };
}

#endif