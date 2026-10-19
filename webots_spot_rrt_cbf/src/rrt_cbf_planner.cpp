#include "webots_spot_rrt_cbf/rrt_cbf_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "safe_cbf_rrt/planner.hpp"

namespace webots_spot_rrt_cbf {

namespace {

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

double pathLengthMetres(const std::vector<Point>& path, double resolution) {
  double len = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    len += std::hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]);
  }
  return len * resolution;
}

std::uint32_t requestSeed(double sx, double sy, double gx, double gy) {
  // Quantise to 0.1 m so tiny pose noise maps to the same seed.
  const auto q = [](double v) { return static_cast<long long>(std::llround(v * 10.0)); };
  std::size_t h = std::hash<long long>{}(q(sx));
  const auto mix = [&h](long long v) {
    h ^= std::hash<long long>{}(v) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  };
  mix(q(sy));
  mix(q(gx));
  mix(q(gy));
  return static_cast<std::uint32_t>(h & 0xffffffffu);
}

}  // namespace

void RrtCbfPlanner::declareAndLoadParameters() {
  auto node = node_.lock();
  const auto p = [this](const std::string& key) { return name_ + "." + key; };

  nav2_util::declare_parameter_if_not_declared(node, p("max_iter"),
                                               rclcpp::ParameterValue(cfg_.max_iter));
  nav2_util::declare_parameter_if_not_declared(node, p("step_size_m"),
                                               rclcpp::ParameterValue(cfg_.step_size_m));
  nav2_util::declare_parameter_if_not_declared(node, p("neighbor_radius_m"),
                                               rclcpp::ParameterValue(cfg_.neighbor_radius_m));
  nav2_util::declare_parameter_if_not_declared(node, p("samples_per_edge"),
                                               rclcpp::ParameterValue(cfg_.samples_per_edge));
  nav2_util::declare_parameter_if_not_declared(node, p("kappa"), rclcpp::ParameterValue(cfg_.kappa));
  nav2_util::declare_parameter_if_not_declared(node, p("v"), rclcpp::ParameterValue(cfg_.v));
  nav2_util::declare_parameter_if_not_declared(node, p("c"), rclcpp::ParameterValue(cfg_.c));
  nav2_util::declare_parameter_if_not_declared(node, p("goal_bias"),
                                               rclcpp::ParameterValue(cfg_.goal_bias));
  nav2_util::declare_parameter_if_not_declared(node, p("do_cbf"),
                                               rclcpp::ParameterValue(cfg_.do_cbf));
  nav2_util::declare_parameter_if_not_declared(node, p("seed_mode"),
                                               rclcpp::ParameterValue(cfg_.seed_mode));
  nav2_util::declare_parameter_if_not_declared(node, p("seed"), rclcpp::ParameterValue(cfg_.seed));
  nav2_util::declare_parameter_if_not_declared(node, p("h_scale"),
                                               rclcpp::ParameterValue(cfg_.h_scale));
  nav2_util::declare_parameter_if_not_declared(node, p("require_field"),
                                               rclcpp::ParameterValue(cfg_.require_field));
  nav2_util::declare_parameter_if_not_declared(node, p("lock_field"),
                                               rclcpp::ParameterValue(cfg_.lock_field));
  nav2_util::declare_parameter_if_not_declared(node, p("allow_unknown"),
                                               rclcpp::ParameterValue(cfg_.allow_unknown));
  nav2_util::declare_parameter_if_not_declared(node, p("max_planning_time"),
                                               rclcpp::ParameterValue(cfg_.max_planning_time));
  nav2_util::declare_parameter_if_not_declared(node, p("start_clear_radius_m"),
                                               rclcpp::ParameterValue(cfg_.start_clear_radius_m));
  nav2_util::declare_parameter_if_not_declared(node, p("goal_clear_radius_m"),
                                               rclcpp::ParameterValue(cfg_.goal_clear_radius_m));
  nav2_util::declare_parameter_if_not_declared(node, p("shortcut"),
                                               rclcpp::ParameterValue(cfg_.shortcut));
  nav2_util::declare_parameter_if_not_declared(node, p("shortcut_h_tolerance"),
                                               rclcpp::ParameterValue(cfg_.shortcut_h_tolerance));
  nav2_util::declare_parameter_if_not_declared(
      node, p("interpolation_resolution"), rclcpp::ParameterValue(cfg_.interpolation_resolution));
  nav2_util::declare_parameter_if_not_declared(node, p("publish_tree"),
                                               rclcpp::ParameterValue(cfg_.publish_tree));
  nav2_util::declare_parameter_if_not_declared(node, p("field_topic"),
                                               rclcpp::ParameterValue(cfg_.field_topic));
  nav2_util::declare_parameter_if_not_declared(node, p("stats_csv_path"),
                                               rclcpp::ParameterValue(cfg_.stats_csv_path));

  node->get_parameter(p("max_iter"), cfg_.max_iter);
  node->get_parameter(p("step_size_m"), cfg_.step_size_m);
  node->get_parameter(p("neighbor_radius_m"), cfg_.neighbor_radius_m);
  node->get_parameter(p("samples_per_edge"), cfg_.samples_per_edge);
  node->get_parameter(p("kappa"), cfg_.kappa);
  node->get_parameter(p("v"), cfg_.v);
  node->get_parameter(p("c"), cfg_.c);
  node->get_parameter(p("goal_bias"), cfg_.goal_bias);
  node->get_parameter(p("do_cbf"), cfg_.do_cbf);
  node->get_parameter(p("seed_mode"), cfg_.seed_mode);
  node->get_parameter(p("seed"), cfg_.seed);
  node->get_parameter(p("h_scale"), cfg_.h_scale);
  node->get_parameter(p("require_field"), cfg_.require_field);
  node->get_parameter(p("lock_field"), cfg_.lock_field);
  node->get_parameter(p("allow_unknown"), cfg_.allow_unknown);
  node->get_parameter(p("max_planning_time"), cfg_.max_planning_time);
  node->get_parameter(p("start_clear_radius_m"), cfg_.start_clear_radius_m);
  node->get_parameter(p("goal_clear_radius_m"), cfg_.goal_clear_radius_m);
  node->get_parameter(p("shortcut"), cfg_.shortcut);
  node->get_parameter(p("shortcut_h_tolerance"), cfg_.shortcut_h_tolerance);
  node->get_parameter(p("interpolation_resolution"), cfg_.interpolation_resolution);
  node->get_parameter(p("publish_tree"), cfg_.publish_tree);
  node->get_parameter(p("field_topic"), cfg_.field_topic);
  node->get_parameter(p("stats_csv_path"), cfg_.stats_csv_path);
}

void RrtCbfPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                              std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;
  auto node = node_.lock();
  if (!node) throw std::runtime_error("RrtCbfPlanner: parent node expired");
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  global_frame_ = costmap_ros_->getGlobalFrameID();

  declareAndLoadParameters();

  // Latched: one inference result is delivered to late joiners; no inference is ever requested here.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  field_sub_ = node->create_subscription<webots_spot_msgs::msg::SafetyField>(
      cfg_.field_topic, qos,
      std::bind(&RrtCbfPlanner::onField, this, std::placeholders::_1));

  reset_srv_ = node->create_service<std_srvs::srv::Trigger>(
      name_ + "/reset_field",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lock(field_mutex_);
        field_.reset();
        res->success = true;
        res->message = "safety field cleared; next /pinn_safety/field message will be cached";
        RCLCPP_INFO(logger_, "%s", res->message.c_str());
      });

  raw_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/plan_raw", 1);
  tree_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(name_ + "/tree", 1);
  stats_pub_ = node->create_publisher<std_msgs::msg::String>(name_ + "/stats", 10);

  if (!cfg_.stats_csv_path.empty()) {
    std::error_code ec;
    csv_header_written_ = std::filesystem::exists(cfg_.stats_csv_path, ec) &&
                          std::filesystem::file_size(cfg_.stats_csv_path, ec) > 0;
    stats_file_.open(cfg_.stats_csv_path, std::ios::app);
    if (!stats_file_.is_open()) {
      RCLCPP_WARN(logger_, "Cannot open stats_csv_path '%s'", cfg_.stats_csv_path.c_str());
    }
  }

  RCLCPP_INFO(logger_,
              "%s configured: do_cbf=%d kappa=%.2f v=%.2f c=%.2f step=%.2fm radius=%.2fm "
              "max_iter=%d field_topic=%s lock_field=%d",
              name_.c_str(), cfg_.do_cbf, cfg_.kappa, cfg_.v, cfg_.c, cfg_.step_size_m,
              cfg_.neighbor_radius_m, cfg_.max_iter, cfg_.field_topic.c_str(), cfg_.lock_field);
}

void RrtCbfPlanner::cleanup() {
  RCLCPP_INFO(logger_, "Cleaning up %s", name_.c_str());
  field_sub_.reset();
  reset_srv_.reset();
  raw_path_pub_.reset();
  tree_pub_.reset();
  stats_pub_.reset();
  if (stats_file_.is_open()) stats_file_.close();
  std::lock_guard<std::mutex> lock(field_mutex_);
  field_.reset();
  cached_version_ = 0;
}

void RrtCbfPlanner::activate() {
  raw_path_pub_->on_activate();
  tree_pub_->on_activate();
  stats_pub_->on_activate();
}

void RrtCbfPlanner::deactivate() {
  raw_path_pub_->on_deactivate();
  tree_pub_->on_deactivate();
  stats_pub_->on_deactivate();
}

void RrtCbfPlanner::onField(const webots_spot_msgs::msg::SafetyField::SharedPtr msg) {
  auto field = std::make_shared<FieldData>();
  field->geometry.nx = static_cast<int>(msg->info.width);
  field->geometry.ny = static_cast<int>(msg->info.height);
  field->geometry.resolution = msg->info.resolution;
  field->geometry.origin_x = msg->info.origin.position.x;
  field->geometry.origin_y = msg->info.origin.position.y;
  field->h = msg->h;
  field->dhdx = msg->dhdx;
  field->dhdy = msg->dhdy;
  if (!field->valid() || field->geometry.resolution <= 0.0f) {
    RCLCPP_ERROR(logger_, "Ignoring malformed SafetyField (%ux%u, h=%zu)", msg->info.width,
                 msg->info.height, msg->h.size());
    return;
  }
  field->h_max = *std::max_element(field->h.begin(), field->h.end());
  if (!(field->h_max > 0.0)) {
    RCLCPP_ERROR(logger_, "Ignoring SafetyField with h_max <= 0");
    return;
  }

  std::lock_guard<std::mutex> lock(field_mutex_);
  ++field_messages_seen_;
  if (field_ && cfg_.lock_field) {
    RCLCPP_WARN(logger_,
                "lock_field: ignoring SafetyField update #%u, keeping the cached field "
                "(call %s/reset_field to accept a new one)",
                field_messages_seen_, name_.c_str());
    return;
  }
  field_ = field;
  ++field_version_;
  RCLCPP_INFO(logger_, "Cached SafetyField %dx%d res=%.3f h_max=%.5f (version %lu)",
              field->geometry.nx, field->geometry.ny, field->geometry.resolution, field->h_max,
              field_version_);
}

void RrtCbfPlanner::prepareFields(const GridGeometry& geom) {
  std::shared_ptr<const FieldData> field;
  unsigned long version;
  {
    std::lock_guard<std::mutex> lock(field_mutex_);
    field = field_;
    version = field_version_;
  }

  const bool same_geom = cached_geom_.nx == geom.nx && cached_geom_.ny == geom.ny &&
                         std::abs(cached_geom_.resolution - geom.resolution) < 1e-9 &&
                         std::abs(cached_geom_.origin_x - geom.origin_x) < 1e-9 &&
                         std::abs(cached_geom_.origin_y - geom.origin_y) < 1e-9;
  if (field && cached_version_ == version && same_geom) return;

  if (!field) {
    cached_h_ = Grid2D(geom.ny, geom.nx, 0.0);
    cached_dhdx_ = Grid2D(geom.ny, geom.nx, 0.0);
    cached_dhdy_ = Grid2D(geom.ny, geom.nx, 0.0);
    cached_version_ = 0;
    cached_geom_ = geom;
    return;
  }
  cached_h_scale_ = cfg_.h_scale > 0.0 ? cfg_.h_scale : field->h_max;
  resampleField(*field, geom, cached_h_scale_, cached_h_, cached_dhdx_, cached_dhdy_);
  cached_geom_ = geom;
  cached_version_ = version;
  RCLCPP_INFO(logger_, "Resampled safety field to costmap grid %dx%d (h_scale=%.5f)", geom.nx,
              geom.ny, cached_h_scale_);
}

void RrtCbfPlanner::appendStatsCsv(const std::string& line, bool is_header) {
  if (!stats_file_.is_open()) return;
  if (is_header) {
    if (csv_header_written_) return;
    csv_header_written_ = true;
  }
  stats_file_ << line << std::endl;
}

nav_msgs::msg::Path RrtCbfPlanner::createPlan(const geometry_msgs::msg::PoseStamped& start,
                                              const geometry_msgs::msg::PoseStamped& goal) {
  std::lock_guard<std::mutex> plan_lock(plan_mutex_);
  const auto t_begin = std::chrono::steady_clock::now();

  if (start.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException("Planner will only accept start position from " +
                                      global_frame_ + " frame");
  }
  if (goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException("Planner will only accept goal position from " +
                                      global_frame_ + " frame");
  }

  // 1. Grids on the costmap geometry.
  nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  GridGeometry geom;
  Grid2D occupancy;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    geom.nx = static_cast<int>(costmap->getSizeInCellsX());
    geom.ny = static_cast<int>(costmap->getSizeInCellsY());
    geom.resolution = costmap->getResolution();
    geom.origin_x = costmap->getOriginX();
    geom.origin_y = costmap->getOriginY();
    occupancy = buildOccupancy(costmap->getCharMap(), geom, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
                               cfg_.allow_unknown);
  }

  bool have_field;
  {
    std::lock_guard<std::mutex> lock(field_mutex_);
    have_field = static_cast<bool>(field_);
  }
  if (!have_field && cfg_.do_cbf && cfg_.require_field) {
    throw nav2_core::PlannerException(
        "No safety field received on " + cfg_.field_topic +
        " yet. Run: ros2 service call /pinn_safety/run std_srvs/srv/Trigger "
        "(or set require_field: false to plan without CBF)");
  }
  prepareFields(geom);
  const bool use_cbf = cfg_.do_cbf && have_field;
  if (cfg_.do_cbf && !have_field) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
                         "No safety field available: planning without CBF (require_field=false)");
  }

  const double sx = start.pose.position.x;
  const double sy = start.pose.position.y;
  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  clearDisc(occupancy, geom, sx, sy, cfg_.start_clear_radius_m);
  clearDisc(occupancy, geom, gx, gy, cfg_.goal_clear_radius_m);

  Point p_start = worldToPixel(geom, sx, sy);
  Point p_goal = worldToPixel(geom, gx, gy);
  const auto in_bounds = [&](const Point& p) {
    return p[0] >= 1.0 && p[1] >= 1.0 && p[0] <= geom.nx && p[1] <= geom.ny;
  };
  if (!in_bounds(p_start)) {
    throw nav2_core::PlannerException("Start pose is outside of the global costmap");
  }
  if (!in_bounds(p_goal)) {
    throw nav2_core::PlannerException("Goal pose is outside of the global costmap");
  }
  if (safe_cbf_rrt::BilinearInterpolant(occupancy)(p_goal[0], p_goal[1]) > 0.5) {
    throw nav2_core::PlannerException("Goal pose is in an occupied or inflated costmap cell");
  }

  // 2. Trivial case.
  const double direct_m = std::hypot(gx - sx, gy - sy);
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;
  if (direct_m < 1e-3) {
    geometry_msgs::msg::PoseStamped pose = start;
    pose.pose.orientation = goal.pose.orientation;
    path.poses.push_back(pose);
    return path;
  }

  // 3. Planner parameters in pixel units.
  safe_cbf_rrt::Params params;
  params.c = cfg_.c;
  params.maxIter = cfg_.max_iter;
  params.stepSize = cfg_.step_size_m / geom.resolution;
  params.neighborRadius = cfg_.neighbor_radius_m / geom.resolution;
  params.samplesPerEdge = cfg_.samples_per_edge;
  params.kappa = cfg_.kappa;
  params.v = cfg_.v;
  params.doCbf = use_cbf;
  params.goalBias = cfg_.goal_bias;
  params.maxTimeMs = cfg_.max_planning_time * 1000.0;
  params.recordTree = cfg_.publish_tree && tree_pub_ && tree_pub_->is_activated() &&
                      tree_pub_->get_subscription_count() > 0;
  params.seed = cfg_.seed_mode == "per_request"
                    ? requestSeed(sx, sy, gx, gy)
                    : static_cast<std::uint32_t>(cfg_.seed);

  safe_cbf_rrt::Result result;
  try {
    result = safe_cbf_rrt::safeCbfRrtStar(p_start, p_goal, occupancy, cached_h_, cached_dhdx_,
                                          cached_dhdy_, params);
  } catch (const std::exception& e) {
    throw nav2_core::PlannerException(std::string("RRT*-CBF failed: ") + e.what());
  }

  if (params.recordTree && !result.treeEdges.empty()) {
    visualization_msgs::msg::Marker m;
    m.header = path.header;
    m.ns = "rrt_cbf_tree";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.01;
    m.color.r = 0.2f;
    m.color.g = 0.6f;
    m.color.b = 1.0f;
    m.color.a = 0.6f;
    m.pose.orientation.w = 1.0;
    m.points.reserve(result.treeEdges.size() * 2);
    for (const auto& e : result.treeEdges) {
      geometry_msgs::msg::Point a, b;
      pixelToWorld(geom, e.first, a.x, a.y);
      pixelToWorld(geom, e.second, b.x, b.y);
      m.points.push_back(a);
      m.points.push_back(b);
    }
    tree_pub_->publish(m);
  }

  const double runtime_ms = result.runtimeMs;
  if (!result.found) {
    std::ostringstream os;
    os << "RRT*-CBF found no path after " << result.iterations << " iterations ("
       << runtime_ms << " ms" << (result.timedOut ? ", time budget exhausted" : "") << ")";
    RCLCPP_WARN(logger_, "%s", os.str().c_str());
    throw nav2_core::PlannerException(os.str());
  }

  // 4. Post-process: shortcut (same validity rules), resample, headings.
  const safe_cbf_rrt::EdgeChecker checker(occupancy, cached_h_, cached_dhdx_, cached_dhdy_, params);
  std::vector<Point> raw_path = result.path;
  std::vector<Point> final_px = raw_path;
  if (cfg_.shortcut) {
    final_px = shortcutPath(raw_path, checker, use_cbf, cfg_.shortcut_h_tolerance);
  }
  const double raw_len_m = pathLengthMetres(raw_path, geom.resolution);
  const double final_len_m = pathLengthMetres(final_px, geom.resolution);
  const auto dense_px = resamplePath(final_px, cfg_.interpolation_resolution / geom.resolution);

  double h_sum = 0.0;
  double h_min = std::numeric_limits<double>::infinity();
  for (const auto& p : dense_px) {
    const double hv = checker.rawH(p[0], p[1]);
    h_sum += hv;
    h_min = std::min(h_min, hv);
  }
  const double h_mean = dense_px.empty() ? 0.0 : h_sum / static_cast<double>(dense_px.size());

  path.poses.reserve(dense_px.size());
  for (std::size_t i = 0; i < dense_px.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    double wx, wy;
    pixelToWorld(geom, dense_px[i], wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    if (i + 1 < dense_px.size()) {
      const double dx = dense_px[i + 1][0] - dense_px[i][0];
      const double dy = dense_px[i + 1][1] - dense_px[i][1];
      pose.pose.orientation = yawToQuaternion(std::atan2(dy, dx));
    } else {
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
  }
  // Start and goal are reproduced exactly (pixel round trip is exact up to float error).
  path.poses.front().pose.position.x = sx;
  path.poses.front().pose.position.y = sy;
  path.poses.back().pose.position.x = gx;
  path.poses.back().pose.position.y = gy;

  if (raw_path_pub_ && raw_path_pub_->is_activated()) {
    nav_msgs::msg::Path raw;
    raw.header = path.header;
    for (const auto& p : raw_path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      pixelToWorld(geom, p, ps.pose.position.x, ps.pose.position.y);
      ps.pose.orientation.w = 1.0;
      raw.poses.push_back(ps);
    }
    raw_path_pub_->publish(raw);
  }

  const double total_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_begin).count();
  std::ostringstream stats;
  stats << "cbf=" << use_cbf << " iterations=" << result.iterations
        << " nodes=" << result.numNodes << " rejection_ratio=" << result.rejectionRatio
        << " raw_len_m=" << raw_len_m << " final_len_m=" << final_len_m
        << " mean_h_norm=" << h_mean << " min_h_norm=" << h_min << " search_ms=" << runtime_ms
        << " total_ms=" << total_ms;
  RCLCPP_INFO(logger_, "RRT*-CBF plan: %s", stats.str().c_str());
  if (stats_pub_ && stats_pub_->is_activated()) {
    std_msgs::msg::String s;
    s.data = stats.str();
    stats_pub_->publish(s);
  }
  appendStatsCsv(
      "stamp,start_x,start_y,goal_x,goal_y,cbf,iterations,nodes,rejection_ratio,raw_len_m,"
      "final_len_m,mean_h_norm,min_h_norm,search_ms,total_ms",
      true);
  std::ostringstream row;
  row << clock_->now().seconds() << "," << sx << "," << sy << "," << gx << "," << gy << ","
      << use_cbf << "," << result.iterations << "," << result.numNodes << ","
      << result.rejectionRatio << "," << raw_len_m << "," << final_len_m << "," << h_mean << ","
      << h_min << "," << runtime_ms << "," << total_ms;
  appendStatsCsv(row.str(), false);

  return path;
}

}  // namespace webots_spot_rrt_cbf

PLUGINLIB_EXPORT_CLASS(webots_spot_rrt_cbf::RrtCbfPlanner, nav2_core::GlobalPlanner)
