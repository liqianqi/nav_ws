/**
 * @brief 先验点云地图重定位节点 (small_gicp 持续跟踪)
 *
 * @details
 * 开机时本节点用 C++ 多高度层 FFT 做全局搜索，生成 top-K 候选，
 * 再逐个做 3D GICP、最近邻 RMSE、内点率和歧义验证，只采纳唯一可靠解。
 * 也可以在 RViz 里手动给 /initialpose。
 *
 * 本节点收到初始位姿后, 用 small_gicp 以上一帧结果为初值做
 * scan-to-map 精配准, 持续修正并发布 map -> odom TF。
 * GICP 连续失败时自动切回 C++ 全局搜索。
 *
 * 历史: 此前用 KISS-Matcher(FPFH+ROBIN)做无初值全局初始化, 但在货架
 * 密集的场景单视角扫描特征内点只有个位数, 基本配不上, 已弃用并删除。
 */

#include "gicp_relocalization/gicp_relocalization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "pcl/filters/voxel_grid.h"

namespace gicp_relocalization
{

GicpRelocalizationNode::GicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declareParameters();

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);
  global_search_ = std::make_unique<GlobalSearch2D>(*global_map_, global_search_config_);

  // 发布先验地图到 /prior_map_cloud,latched(QoS TRANSIENT_LOCAL + RELIABLE),
  // RViz 订阅后能用不同颜色和 /cloud_registered 对比配准情况。
  // 下采样到 0.1m 降低数据量(281 万点 -> 几十万点),视觉对比足够
  prior_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "prior_map_cloud", rclcpp::QoS(1).transient_local().reliable());
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr viz_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(global_map_);
    voxel.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel.filter(*viz_map);
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*viz_map, map_msg);
    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = this->now();
    prior_map_pub_->publish(map_msg);
    RCLCPP_INFO(this->get_logger(),
      "Published prior map (%zu pts after 0.1m voxel) to /prior_map_cloud in '%s' frame",
      viz_map->size(), map_frame_.c_str());
  }

  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&GicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&GicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  // GICP 配准频率。
  // 0.5 秒一次(2Hz):在跟踪精度和 CPU 负载之间平衡。
  // 之前试过 2 秒(0.5Hz),但间隔太长导致 odom 漂移累积,GICP 初始猜测偏差过大,
  // 配准完全失败(inliers=0.000)。0.5 秒间隔下 odom 漂移小,GICP 能稳定跟踪。
  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&GicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&GicpRelocalizationNode::publishTransform, this));
}

void GicpRelocalizationNode::declareParameters()
{
  global_leaf_size_ = this->declare_parameter<double>("global_leaf_size", 0.25);
  registered_leaf_size_ = this->declare_parameter<double>("registered_leaf_size", 0.25);
  num_threads_ = this->declare_parameter<int>("num_threads", 4);
  num_neighbors_ = this->declare_parameter<int>("num_neighbors", 10);
  max_dist_sq_ = this->declare_parameter<double>("max_dist_sq", 1.0);
  // 初始位姿有 10cm/几度误差时迭代次数太少会被误判失败
  gicp_max_iterations_ = this->declare_parameter<int>("gicp_max_iterations", 50);
  gicp_max_consecutive_failures_ =
    this->declare_parameter<int>("gicp_max_consecutive_failures", 3);
  global_min_accum_sec_ = this->declare_parameter<double>("global.min_accum_sec", 8.0);
  global_retry_period_sec_ =
    this->declare_parameter<double>("global.retry_period_sec", 5.0);
  global_search_config_.z_layers =
    this->declare_parameter<std::vector<double>>(
    "global.z_layers", {0.3, 0.8, 1.3, 1.8, 2.3});
  global_search_config_.resolution =
    this->declare_parameter<double>("global.grid_resolution", 0.1);
  global_search_config_.yaw_step_deg =
    this->declare_parameter<double>("global.yaw_step_deg", 2.0);
  global_search_config_.match_threshold =
    this->declare_parameter<double>("global.match_threshold", 0.85);
  global_search_config_.top_k =
    this->declare_parameter<int>("global.top_k", 12);
  global_search_config_.peaks_per_yaw =
    this->declare_parameter<int>("global.peaks_per_yaw", 3);
  global_search_config_.peak_window_cells =
    this->declare_parameter<int>("global.peak_window_cells", 11);
  global_search_config_.candidate_min_translation =
    this->declare_parameter<double>("global.candidate_min_translation", 0.7);
  global_search_config_.candidate_min_yaw_deg =
    this->declare_parameter<double>("global.candidate_min_yaw_deg", 8.0);
  global_search_config_.min_occupied_cells =
    this->declare_parameter<int>("global.min_occupied_cells", 1200);
  global_search_config_.num_threads =
    this->declare_parameter<int>("global.num_threads", num_threads_);
  validation_max_dist_ = this->declare_parameter<double>("validation_max_dist", 0.30);
  validation_max_points_ = this->declare_parameter<int>("validation_max_points", 10000);
  candidate_min_inlier_ratio_ =
    this->declare_parameter<double>("candidate_min_inlier_ratio", 0.65);
  candidate_max_rmse_ = this->declare_parameter<double>("candidate_max_rmse", 0.22);
  // best_score / second_score 必须小于该值；越小越严格
  candidate_ambiguity_ratio_ =
    this->declare_parameter<double>("candidate_ambiguity_ratio", 0.95);
  candidate_max_correction_translation_ =
    this->declare_parameter<double>("candidate_max_correction_translation", 0.75);
  candidate_max_correction_yaw_ =
    this->declare_parameter<double>("candidate_max_correction_yaw_deg", 10.0) *
    M_PI / 180.0;
  // 去重阈值:精配准后位姿距离/角度小于此值的候选视为同一解,合并后再判歧义。
  // 默认 1.0m / 10° —— GICP 在同一真实位置附近的收敛抖动通常 <1m,合并掉避免假歧义
  candidate_dedup_distance_ =
    this->declare_parameter<double>("candidate_dedup_distance", 1.0);
  candidate_dedup_yaw_deg_ =
    this->declare_parameter<double>("candidate_dedup_yaw_deg", 10.0);
  tracking_min_inlier_ratio_ =
    this->declare_parameter<double>("tracking_min_inlier_ratio", 0.35);
  tracking_max_rmse_ = this->declare_parameter<double>("tracking_max_rmse", 0.30);
  tracking_max_jump_translation_ =
    this->declare_parameter<double>("tracking_max_jump_translation", 0.25);
  tracking_max_jump_yaw_ =
    this->declare_parameter<double>("tracking_max_jump_yaw_deg", 3.0) *
    M_PI / 180.0;
  force_planar_ = this->declare_parameter<bool>("force_planar", true);

  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame", "base_link");
  prior_pcd_file_ = this->declare_parameter<std::string>("prior_pcd_file", "");
  input_cloud_topic_ =
    this->declare_parameter<std::string>("input_cloud_topic", "cloud_registered");
}

void GicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  // map_cloud.pcd 已扶正(地面 z=0), /cloud_registered 在直立的 odom 系,
  // 两者同姿态, 无需任何预变换
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());
}

void GicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;
  if (first_accumulation_time_.nanoseconds() == 0) {
    first_accumulation_time_ = this->now();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  // 等待初始化期间累积不清空, 防止内存无限增长
  constexpr size_t kMaxAccumPoints = 5000000;
  if (accumulated_cloud_->size() > kMaxAccumPoints) {
    accumulated_cloud_->clear();
  }
  *accumulated_cloud_ += *scan;
}

void GicpRelocalizationNode::runAutomaticInitialization()
{
  if (!global_search_ || accumulated_cloud_->empty()) {
    return;
  }
  const auto candidates = global_search_->search(*accumulated_cloud_);
  if (candidates.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "C++ multi-layer global search found no reliable candidate; keep accumulating.");
    return;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "C++ global search generated %zu candidates; best=(%.3f, %.3f, %.2fdeg), score=%.3f",
    candidates.size(), candidates.front().x, candidates.front().y,
    candidates.front().yaw * 180.0 / M_PI, candidates.front().score);
  validateCandidates(candidates);
}

bool GicpRelocalizationNode::validateCandidates(
  const std::vector<SearchCandidate> & candidates)
{
  if (!prepareSource(accumulated_cloud_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to prepare source cloud for candidate validation.");
    return false;
  }

  struct CandidateResult
  {
    size_t index;
    Eigen::Isometry3d pose;
    double inlier_ratio;
    double rmse;
    double score;
  };
  std::vector<CandidateResult> valid_results;

  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto & candidate = candidates[i];
    // 自动候选直接表示 T_map_odom; source 点云也在 odom 系。
    Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
    initial_guess.translation() << candidate.x, candidate.y, 0.0;
    initial_guess.linear() =
      Eigen::AngleAxisd(candidate.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Isometry3d refined_pose = Eigen::Isometry3d::Identity();
    double inlier_ratio = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
    if (!alignPreparedSource(initial_guess, refined_pose, inlier_ratio, rmse)) {
      RCLCPP_INFO(this->get_logger(), "Candidate %zu: GICP did not converge", i);
      continue;
    }

    const double score = rmse / std::max(inlier_ratio, 1e-6);
    const double correction_translation =
      (refined_pose.translation() - initial_guess.translation()).head<2>().norm();
    const double initial_yaw =
      std::atan2(initial_guess.linear()(1, 0), initial_guess.linear()(0, 0));
    const double refined_yaw =
      std::atan2(refined_pose.linear()(1, 0), refined_pose.linear()(0, 0));
    const double correction_yaw = std::abs(
      std::atan2(
        std::sin(refined_yaw - initial_yaw), std::cos(refined_yaw - initial_yaw)));
    RCLCPP_INFO(
      this->get_logger(),
      "Candidate %zu: pose=(%.3f, %.3f, %.2fdeg) inliers=%.3f rmse=%.3f score=%.3f "
      "correction=(%.3fm, %.2fdeg)",
      i, refined_pose.translation().x(), refined_pose.translation().y(),
      refined_yaw * 180.0 / M_PI,
      inlier_ratio, rmse, score, correction_translation,
      correction_yaw * 180.0 / M_PI);
    if (
      inlier_ratio >= candidate_min_inlier_ratio_ && rmse <= candidate_max_rmse_ &&
      correction_translation <= candidate_max_correction_translation_ &&
      correction_yaw <= candidate_max_correction_yaw_)
    {
      valid_results.push_back({i, refined_pose, inlier_ratio, rmse, score});
    }
  }

  if (valid_results.empty()) {
    RCLCPP_WARN(this->get_logger(), "All automatic localization candidates were rejected.");
    return false;
  }

  std::sort(
    valid_results.begin(), valid_results.end(),
    [](const CandidateResult & lhs, const CandidateResult & rhs) {
      return lhs.score < rhs.score;
    });

  // 多个粗候选可能被 GICP 拉进同一局部极小值，先按最终位姿去重再判断歧义。
  std::vector<CandidateResult> distinct_results;
  for (const auto & result : valid_results) {
    const double yaw = std::atan2(result.pose.linear()(1, 0), result.pose.linear()(0, 0));
    bool distinct = true;
    for (const auto & kept : distinct_results) {
      const double distance =
        (result.pose.translation().head<2>() - kept.pose.translation().head<2>()).norm();
      const double kept_yaw =
        std::atan2(kept.pose.linear()(1, 0), kept.pose.linear()(0, 0));
      const double yaw_distance =
        std::abs(std::atan2(std::sin(yaw - kept_yaw), std::cos(yaw - kept_yaw)));
      if (distance < candidate_dedup_distance_ &&
          yaw_distance < candidate_dedup_yaw_deg_ * M_PI / 180.0) {
        distinct = false;
        break;
      }
    }
    if (distinct) {
      distinct_results.push_back(result);
    }
  }

  const auto & best = distinct_results.front();
  if (distinct_results.size() > 1) {
    const auto & second = distinct_results[1];
    const double ratio = best.score / std::max(second.score, 1e-9);
    // 打印所有 distinct 候选的坐标和两两距离,用于判断歧义是"近距离重复"还是"真歧义"
    std::string distinct_summary;
    for (size_t k = 0; k < distinct_results.size(); ++k) {
      const double kx = distinct_results[k].pose.translation().x();
      const double ky = distinct_results[k].pose.translation().y();
      const double kyaw = std::atan2(
        distinct_results[k].pose.linear()(1, 0), distinct_results[k].pose.linear()(0, 0)) * 180.0 / M_PI;
      char buf[128];
      snprintf(buf, sizeof(buf), "[%zu](%.2f,%.2f,%.0fdeg,s=%.3f) ", k, kx, ky, kyaw, distinct_results[k].score);
      distinct_summary += buf;
    }
    RCLCPP_INFO(this->get_logger(), "Distinct candidates after dedup: %s", distinct_summary.c_str());
    if (distinct_results.size() == 2) {
      const double dx = best.pose.translation().x() - second.pose.translation().x();
      const double dy = best.pose.translation().y() - second.pose.translation().y();
      RCLCPP_INFO(this->get_logger(), "Best vs second distance: %.3f m", std::hypot(dx, dy));
    }
    if (ratio > candidate_ambiguity_ratio_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Automatic localization ambiguous: best/second score ratio=%.3f > %.3f. "
        "Accumulate more scans and retry.",
        ratio, candidate_ambiguity_ratio_);
      return false;
    }
  }

  result_t_ = previous_result_t_ = best.pose;
  reloc_state_ = RelocState::TRACKING;
  gicp_fail_count_ = 0;
  accumulated_cloud_->clear();
  // 初始化智能跟踪的 odom 基准
  try {
    auto tf = tf_buffer_->lookupTransform(
      odom_frame_, robot_base_frame_, tf2::TimePointZero);
    last_track_odom_pos_ = tf2::transformToEigen(tf.transform).translation();
    last_successful_track_time_ = this->now();
    last_track_odom_valid_ = true;
  } catch (tf2::TransformException &) {
    last_track_odom_valid_ = false;
  }
  const double accepted_yaw =
    std::atan2(best.pose.linear()(1, 0), best.pose.linear()(0, 0));
  RCLCPP_INFO(
    this->get_logger(),
    "Automatic localization accepted candidate %zu: pose=(%.3f, %.3f, %.2f deg) "
    "inliers=%.3f rmse=%.3f",
    best.index, best.pose.translation().x(), best.pose.translation().y(),
    accepted_yaw * 180.0 / M_PI, best.inlier_ratio, best.rmse);
  return true;
}

void GicpRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000, "No accumulated points to process.");
    return;
  }

  switch (reloc_state_) {
    case RelocState::WAIT_INIT: {
      const auto now = this->now();
      const double accumulated_sec =
        first_accumulation_time_.nanoseconds() == 0 ? 0.0 :
        (now - first_accumulation_time_).seconds();
      const bool cooldown_elapsed =
        last_global_search_time_.nanoseconds() == 0 ||
        (now - last_global_search_time_).seconds() >= global_retry_period_sec_;
      if (accumulated_sec < global_min_accum_sec_ || !cooldown_elapsed) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "C++ global localization accumulating: %.1f/%.1fs, points=%zu",
          accumulated_sec, global_min_accum_sec_, accumulated_cloud_->size());
        return;
      }
      last_global_search_time_ = now;
      runAutomaticInitialization();
      return;
    }

    case RelocState::TRACKING: {
      // === 智能跟踪策略 ===
      // 1. 检查自上次成功跟踪以来的 odom 位移/旋转
      // 2. 位移 < 0.3m 且旋转 < 5° 且 < 10 秒 → 跳过本次配准(省 CPU)
      // 3. 有明显位移 或 超过 10 秒 → 执行 GICP 配准
      Eigen::Isometry3d current_odom_pose = Eigen::Isometry3d::Identity();
      bool got_odom = false;
      try {
        auto tf = tf_buffer_->lookupTransform(
          odom_frame_, robot_base_frame_, tf2::TimePointZero);
        current_odom_pose = tf2::transformToEigen(tf.transform);
        got_odom = true;
      } catch (tf2::TransformException &) {
        // odom TF 不可用,强制配准(用 GICP 结果兜底)
      }

      // 判断是否需要配准
      bool need_align = true;
      if (got_odom && last_track_odom_valid_) {
        double dx = current_odom_pose.translation().x() - last_track_odom_pos_.x();
        double dy = current_odom_pose.translation().y() - last_track_odom_pos_.y();
        double trans_moved = std::hypot(dx, dy);
        double time_since = last_successful_track_time_.nanoseconds() == 0 ? 999.0 :
          (this->now() - last_successful_track_time_).seconds();
        if (trans_moved < TRACK_MIN_TRANSLATION && time_since < TRACK_FORCE_INTERVAL_SEC) {
          need_align = false;  // 没怎么动,跳过配准
        }
      }

      if (!need_align) {
        // 跳过配准,清空累积点云(下次从新累积)
        accumulated_cloud_->clear();
        return;
      }

      // 执行 GICP 配准
      Eigen::Isometry3d gicp_pose = Eigen::Isometry3d::Identity();
      double inlier_ratio = 0.0;
      double rmse = std::numeric_limits<double>::infinity();
      const bool aligned = tryGicpAlignment(
        accumulated_cloud_, previous_result_t_, gicp_pose, inlier_ratio, rmse);
      const double translation_jump =
        (gicp_pose.translation() - previous_result_t_.translation()).head<2>().norm();
      const double previous_yaw =
        std::atan2(previous_result_t_.linear()(1, 0), previous_result_t_.linear()(0, 0));
      const double current_yaw =
        std::atan2(gicp_pose.linear()(1, 0), gicp_pose.linear()(0, 0));
      const double yaw_jump = std::abs(
        std::atan2(
          std::sin(current_yaw - previous_yaw), std::cos(current_yaw - previous_yaw)));
      if (
        aligned && inlier_ratio >= tracking_min_inlier_ratio_ &&
        rmse <= tracking_max_rmse_ &&
        translation_jump <= tracking_max_jump_translation_ &&
        yaw_jump <= tracking_max_jump_yaw_)
      {
        result_t_ = previous_result_t_ = gicp_pose;
        gicp_fail_count_ = 0;
        accumulated_cloud_->clear();
        // 记录本次跟踪时的 odom 位姿,用于下次判断是否需要配准
        if (got_odom) {
          last_track_odom_pos_ = current_odom_pose.translation();
          last_successful_track_time_ = this->now();
          last_track_odom_valid_ = true;
        }
        return;
      }

      ++gicp_fail_count_;
      // 跟踪失败时清空累积点云,避免点云越来越大导致后续 GICP 更不稳定
      accumulated_cloud_->clear();
      RCLCPP_WARN(
        this->get_logger(),
        "GICP tracking rejected: inliers=%.3f rmse=%.3f jump=(%.3fm, %.2fdeg) "
        "fail_count=%d/%d",
        inlier_ratio, rmse, translation_jump, yaw_jump * 180.0 / M_PI,
        gicp_fail_count_, gicp_max_consecutive_failures_);

      if (gicp_fail_count_ >= gicp_max_consecutive_failures_) {
        reloc_state_ = RelocState::LOST;
        RCLCPP_WARN(this->get_logger(), "GICP lost. Switching to C++ global re-initialization.");
      }
      return;
    }

    case RelocState::LOST: {
      reloc_state_ = RelocState::WAIT_INIT;
      first_accumulation_time_ = this->now();
      last_global_search_time_ =
        rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      return;
    }
  }
}

bool GicpRelocalizationNode::prepareSource(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud)
{
  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *source_cloud, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  return source_ && !source_->empty();
}

Eigen::Isometry3d GicpRelocalizationNode::projectToPlanar(
  const Eigen::Isometry3d & pose) const
{
  if (!force_planar_) {
    return pose;
  }
  Eigen::Isometry3d planar = Eigen::Isometry3d::Identity();
  const double yaw = std::atan2(pose.linear()(1, 0), pose.linear()(0, 0));
  planar.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  planar.translation().x() = pose.translation().x();
  planar.translation().y() = pose.translation().y();
  // 地面机器人 map 与 odom 都以地面为 z=0，不让 GICP 把 TF 拉出平面。
  planar.translation().z() = 0.0;
  return planar;
}

bool GicpRelocalizationNode::alignPreparedSource(
  const Eigen::Isometry3d & initial_guess,
  Eigen::Isometry3d & pose_out,
  double & inlier_ratio_out,
  double & rmse_out)
{
  if (!source_ || source_->empty()) {
    return false;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = gicp_max_iterations_;

  auto result = register_->align(*target_, *source_, *target_tree_, initial_guess);
  if (!result.converged) {
    return false;
  }

  pose_out = projectToPlanar(result.T_target_source);

  const size_t stride =
    std::max<size_t>(1, source_->size() / static_cast<size_t>(validation_max_points_));
  const double max_dist_sq = validation_max_dist_ * validation_max_dist_;
  size_t tested = 0;
  size_t inliers = 0;
  double sum_sq = 0.0;
  for (size_t i = 0; i < source_->size(); i += stride) {
    const Eigen::Vector4d point_source =
      source_->points[i].getVector4fMap().cast<double>();
    const Eigen::Vector4d point_target = pose_out.matrix() * point_source;
    size_t index = 0;
    double sq_dist = 0.0;
    ++tested;
    if (
      target_tree_->nearest_neighbor_search(point_target, &index, &sq_dist) > 0 &&
      sq_dist <= max_dist_sq)
    {
      ++inliers;
      sum_sq += sq_dist;
    }
  }

  inlier_ratio_out =
    tested > 0 ? static_cast<double>(inliers) / static_cast<double>(tested) : 0.0;
  rmse_out =
    inliers > 0 ? std::sqrt(sum_sq / static_cast<double>(inliers)) :
    std::numeric_limits<double>::infinity();
  return true;
}

bool GicpRelocalizationNode::tryGicpAlignment(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source_cloud,
  const Eigen::Isometry3d & initial_guess,
  Eigen::Isometry3d & pose_out,
  double & inlier_ratio_out,
  double & rmse_out)
{
  if (!prepareSource(source_cloud)) {
    return false;
  }
  return alignPreparedSource(
    initial_guess, pose_out, inlier_ratio_out, rmse_out);
}

void GicpRelocalizationNode::publishTransform()
{
  if (reloc_state_ != RelocState::TRACKING) {
    return;
  }
  if (result_t_.matrix().isZero()) {
    return;
  }
  // 还没收到扫描时 last_scan_time_ 为 0, 别往 TF 树里发时间戳为 0 的变换
  if (last_scan_time_.nanoseconds() == 0) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void GicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    // T_map_odom = T_map_base * T_base_odom (扫描 frame 即 odom)
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
    reloc_state_ = RelocState::TRACKING;
    gicp_fail_count_ = 0;
    accumulated_cloud_->clear();
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gicp_relocalization::GicpRelocalizationNode)
