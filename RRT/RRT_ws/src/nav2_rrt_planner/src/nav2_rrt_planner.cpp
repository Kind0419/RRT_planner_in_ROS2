#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>
#include "tf2/utils.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_rrt_planner/nav2_rrt_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <random> 

#define max_iteration 10000
#define step_size 1.0
namespace nav2_rrt_planner
{
    void RRT::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        tf_ = tf;
        name_ = name;
        node_ = parent.lock();
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        // 参数初始化
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
        node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".use_smoothing", rclcpp::ParameterValue(true));
        node_->get_parameter(name_ + ".use_smoothing", use_smoothing);
    }
    void RRT::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "正在清理 RRT 规划器: %s", name_.c_str());
    }

    void RRT::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在激活 RRT 规划器: %s", name_.c_str());
    }

    void RRT::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在停用 RRT 规划器: %s", name_.c_str());
    }
    nav_msgs::msg::Path RRT::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
        // 1. 初始化路径
        nav_msgs::msg::Path global_path;
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;
        // 2. 检查 start 和 goal 的坐标系是否与 global_frame_ 一致
        if (start.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "RRT: 起点坐标系错误！期望: %s, 实际: %s",
                         global_frame_.c_str(), start.header.frame_id.c_str());
            return global_path; // 坐标系不对，直接返回空路径
        }

        if (goal.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "RRT: 目标坐标系错误！期望: %s, 实际: %s",
                         global_frame_.c_str(), goal.header.frame_id.c_str());
            return global_path; // 坐标系不对，直接返回空路径
        }

        RCLCPP_INFO(node_->get_logger(), "RRT: 开始规划，从 (%.2f, %.2f) 到 (%.2f, %.2f)",
                    start.pose.position.x, start.pose.position.y,
                    goal.pose.position.x, goal.pose.position.y);
        // 3. 检查起点和目标点是否在合法位置
        Node start_node{
            start.pose.position.x,
            start.pose.position.y,
            -1};
        Node goal_node{
            goal.pose.position.x,
            goal.pose.position.y,
            -1};
        if (!isNodeSafe(start_node.x, start_node.y))
        {
            RCLCPP_ERROR(node_->get_logger(), "RRT: 起点位置不安全！");

            return global_path;
        }
        if (!isNodeSafe(goal_node.x, goal_node.y))
        {
            RCLCPP_ERROR(node_->get_logger(), "RRT: 终点位置不安全！");
            return global_path;
        }
        // 4. 相关数据结构初始化
        std::default_random_engine rng(std::chrono::system_clock::now().time_since_epoch().count());
        std::vector<Node> tree;
        tree.push_back(start_node);
        int Count = 0;
        // 5.核心算法内容
        for (int i = 0; i < max_iteration; i++)
        {

            geometry_msgs::msg::PoseStamped random_point = generateRandomPoint(costmap_,rng);
            int nearIndex = findNearNode(tree, random_point.pose.position.x, random_point.pose.position.y);
            Node new_node = expandNode(tree[nearIndex], random_point);
            if (isNodeSafe(new_node.x, new_node.y))
            {
                new_node.parent_index = nearIndex;
                tree.push_back(new_node);
                Count++;
            }
            if ((((goal_node.x - new_node.x) * (goal_node.x - new_node.x) 
            + (goal_node.y - new_node.y) * (goal_node.y - new_node.y)) < 0.01)
            &&isNodeSafe(new_node.x, new_node.y))
            {
                RCLCPP_INFO(node_->get_logger(), "RRT: 成功生成路径，共 %d 个点", Count);
                std::vector<Node> path_nodes;
                int current_index = tree.size() - 1;
                while (current_index != -1)
                {
                    Node currentPoint=tree[current_index];
                    current_index = currentPoint.parent_index;
                    path_nodes.push_back(currentPoint);
                }
                std::reverse(path_nodes.begin(), path_nodes.end());
                RCLCPP_INFO(node_->get_logger(), " 搜索到的路径节点数 = %lu", path_nodes.size());
                ////

                for (const auto &p : path_nodes)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = global_path.header;
                    //
                    pose.pose.position.x = p.x;
                    pose.pose.position.y = p.y;
                    //
                    pose.pose.position.z = 0.0;

                    tf2::Quaternion q;
                    q.setRPY(0, 0, 0);
                    pose.pose.orientation = tf2::toMsg(q);

                    global_path.poses.push_back(pose);
                }
                if (use_smoothing)
                {
                    PathSmooth(global_path.poses, 10);
                }
                RCLCPP_INFO(node_->get_logger(), " RRT: 最终生成的路径包含 %lu 个位姿 (poses)", global_path.poses.size());

                return global_path;
            }
        }
        // 如果循环结束仍未找到路径，返回空路径
        RCLCPP_WARN(node_->get_logger(), "RRT: 未能找到有效路径");
        throw nav2_core::PlannerException("RRT: 未找到可行路径到目标");
    }
    // 生成随机点
    geometry_msgs::msg::PoseStamped RRT::generateRandomPoint(
        const nav2_costmap_2d::Costmap2D *costmap,
        std::default_random_engine &rng)
    {
        unsigned int wide = costmap->getSizeInCellsX();
        unsigned int high = costmap->getSizeInCellsY();
        double resolution = costmap->getResolution();

        double map_wide = wide * resolution;
        double map_high = high * resolution;
        // 生成随机数的方法，但是要搭配随机数生成引擎使用
        std::uniform_real_distribution<double> ran_x(0.0, map_wide);
        std::uniform_real_distribution<double> ran_y(0.0, map_high);

        double random_x = ran_x(rng);
        double random_y = ran_y(rng);

        geometry_msgs::msg::PoseStamped random_pose;
        random_pose.header.frame_id = "global_frame";
        random_pose.header.stamp = tf2_ros::toMsg(tf2::TimePointZero);

        random_pose.pose.position.x = random_x;
        random_pose.pose.position.y = random_y;
        random_pose.pose.orientation.w = 1.0;
        return random_pose;
    }
    // 平滑算法
    void RRT::PathSmooth(std::vector<geometry_msgs::msg::PoseStamped> &Path, int iterations)
    {
        if (Path.empty())
            return; // 空路径直接返回
        // if (Path.size() < 3)
        // {
        //     return;
        // }
        // 主迭代优化循环
        for (int i = 0; i < iterations; ++i)
        {
            std::vector<geometry_msgs::msg::PoseStamped> Smoothed;
            // 起点不变
            Smoothed.push_back(Path.front());
            for (unsigned j = 1; j < Path.size() - 1; ++j)
            {
                geometry_msgs::msg::PoseStamped current = Path[j];
                geometry_msgs::msg::PoseStamped pre = Path[j - 1];
                geometry_msgs::msg::PoseStamped next = Path[j + 1];

                geometry_msgs::msg::PoseStamped SmoothPoint;
                SmoothPoint.pose.position.x = (pre.pose.position.x + current.pose.position.x + next.pose.position.x) / 3;
                SmoothPoint.pose.position.y = (pre.pose.position.y + current.pose.position.y + next.pose.position.y) / 3;

                if (!isNodeSafe(SmoothPoint.pose.position.x, SmoothPoint.pose.position.y))
                {
                    SmoothPoint = current;
                }
                Smoothed.push_back(SmoothPoint);
            }
            Smoothed.push_back(Path.back());
            Path = Smoothed;
        }
    }
    // 扩展节点路径
    Node RRT::expandNode(Node start_node, geometry_msgs::msg::PoseStamped random_point)
    {
        double dist = std::sqrt((start_node.x - random_point.pose.position.x) * (start_node.x - random_point.pose.position.x) + (start_node.y - random_point.pose.position.y) * (start_node.y - random_point.pose.position.y));
        if (dist < 1e-6)
        {
            return start_node; // 几乎已经在目标点上了
        }
        double new_x = ((random_point.pose.position.x - start_node.x) / dist) * step_size;
        double new_y = ((random_point.pose.position.y - start_node.y) / dist) * step_size;
        Node newNode;
        newNode.x = new_x + start_node.x;
        newNode.y = new_y + start_node.y;
        newNode.parent_index = -1;
        return newNode;
    }
    // 查找最近节点
    int RRT::findNearNode(std::vector<Node> tree, double x, double y)
    {
        if (tree.empty())
        {
            //RCLCPP_ERROR(node_->get_logger(), "错误！RRT扩展节点时传入树为空！");
            return -1;
        }
        double min_dist = std::numeric_limits<double>::max();
        int nearIndex = 0;
        for (size_t i = 0; i < tree.size(); ++i)
        {
            Node current_node = tree[i];
            double dx = current_node.x - x;
            double dy = current_node.y - y;
            double current_dist = dx * dx + dy * dy;
            if (current_dist < min_dist)
            {
                min_dist = current_dist;
                nearIndex = i;
            }
        }
        return nearIndex;
    }
    // 检查节点是否安全（不在障碍物上）
    bool RRT::isNodeSafe(double x, double y)
    {
        unsigned int mx, my;
        if (!costmap_->worldToMap(x, y, mx, my))
        {
            RCLCPP_DEBUG(node_->get_logger(), " isNodeSafe: 坐标 (%.2f, %.2f) 超出地图范围！", x, y);
            return false;
        }

        unsigned char cost = costmap_->getCost(mx, my);
        RCLCPP_DEBUG(node_->get_logger(), "  检查点 (%.2f, %.2f) 的 cost = %d", x, y, cost);

        bool is_safe = (cost < nav2_costmap_2d::LETHAL_OBSTACLE);
        RCLCPP_DEBUG(node_->get_logger(), "  isNodeSafe(%.2f, %.2f) = %d", x, y, is_safe);
        return is_safe;
    }

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrt_planner::RRT, nav2_core::GlobalPlanner)