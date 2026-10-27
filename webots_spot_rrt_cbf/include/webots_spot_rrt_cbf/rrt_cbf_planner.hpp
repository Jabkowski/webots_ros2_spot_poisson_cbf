#pragma once

#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"
#include "webots_spot_msgs/msg/safety_field.hpp"
#include "webots_spot_rrt_cbf/field_adapter.hpp"

namespace webots_spot_rrt_cbf {

/// Nav2 global planner wrapping Safe CBF-RRT* (rrt_star_cbf). The PINN safety
/// field is received once (latched topic), cached, and reused by every plan;
/// the planner never triggers inference.
class RrtCbfPlanner : public nav2_core::GlobalPlanner {
 public:
  RrtCbfPlanner() = default;
  ~RrtCbfPlanner() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                 const geometry_msgs::msg::PoseStamped& goal) override;

 private:
  struct Settings {
    int max_iter = 10000;
    double step_size_m = 0.5;
    double neighbor_radius_m = 1.5;
    int samples_per_edge = 20;
    double kappa = 2.0;
    double v = 0.5;
    double c = 0.2;
    double goal_bias = 0.05;
    bool do_cbf = true;
    std::string seed_mode = "fixed";  // fixed | per_request
    int seed = 1;
    double h_scale = 0.0;  // 0 = auto (max of received h)
    bool require_field = true;
    bool lock_field = true;
    bool allow_unknown = true;
    double max_planning_time = 2.0;  // seconds
    double start_clear_radius_m = 0.4;
    double goal_clear_radius_m = 0.0;
    bool shortcut = true;
    double shortcut_h_tolerance = 0.9;
    double interpolation_resolution = 0.05;
    bool publish_tree = true;
    std::string field_topic = "/pinn_safety/field";
    std::string stats_csv_path;
  };

  void declareAndLoadParameters();
  rcl_interfaces::msg::SetParametersResult onParametersSet(
      const std::vector<rclcpp::Parameter>& parameters);
  void onField(const webots_spot_msgs::msg::SafetyField::SharedPtr msg);
  void prepareFields(const GridGeometry& geom);
  void appendStatsCsv(const std::string& line, bool is_header);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("RrtCbfPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  Settings cfg_;
  std::mutex cfg_mutex_;  // cfg_ is updated by the parameter callback; createPlan copies it
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Cached safety field (set once when lock_field is true).
  std::mutex field_mutex_;
  std::shared_ptr<const FieldData> field_;
  unsigned long field_version_ = 0;
  unsigned int field_messages_seen_ = 0;

  // Field resampled/scaled onto the costmap grid (rebuilt only when field or geometry changes).
  unsigned long cached_version_ = 0;
  GridGeometry cached_geom_;
  double cached_h_scale_ = 1.0;
  safe_cbf_rrt::Grid2D cached_h_, cached_dhdx_, cached_dhdy_;

  rclcpp::Subscription<webots_spot_msgs::msg::SafetyField>::SharedPtr field_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr tree_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr stats_pub_;

  std::mutex plan_mutex_;
  std::ofstream stats_file_;
  bool csv_header_written_ = false;
};

}  // namespace webots_spot_rrt_cbf
