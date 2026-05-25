#include <mutex>
#include <memory>
#include <iostream>
#include <atomic>
#include <chrono>
#include <future>
#include <functional>
#include <stdexcept>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>

#include <hdl_localization/msg/scan_matching_status.hpp>
#include <hdl_localization/msg/hdl_reloc_status.hpp>
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/srv/query_global_localization.hpp>

namespace hdl_localization {

class HdlLocalizationNodelet : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNodelet(const rclcpp::NodeOptions& options)
  : Node("hdl_localization", options)
  {
    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    robot_odom_frame_id = declare_parameter<std::string>("robot_odom_frame_id", "robot_odom");
    odom_child_frame_id = declare_parameter<std::string>("odom_child_frame_id", "base_link");
    send_tf_transforms = declare_parameter<bool>("send_tf_transforms", true);
    cool_time_duration = declare_parameter<double>("cool_time_duration", 0.5);
    reg_method = declare_parameter<std::string>("reg_method", "NDT_OMP");
    ndt_neighbor_search_method = declare_parameter<std::string>("ndt_neighbor_search_method", "DIRECT7");
    ndt_neighbor_search_radius = declare_parameter<double>("ndt_neighbor_search_radius", 2.0);
    ndt_resolution = declare_parameter<double>("ndt_resolution", 1.0);
    enable_robot_odometry_prediction = declare_parameter<bool>("enable_robot_odometry_prediction", false);

    enable_auto_relocalize_monitor = declare_parameter<bool>("enable_auto_relocalize_monitor", false);
    auto_relocalize_error_threshold = declare_parameter<double>("auto_relocalize_error_threshold", 0.2);
    auto_relocalize_cooldown = declare_parameter<double>("auto_relocalize_cooldown", 5.0);

    use_imu = declare_parameter<bool>("use_imu", true);
    invert_acc = declare_parameter<bool>("invert_acc", false);
    invert_gyro = declare_parameter<bool>("invert_gyro", false);

    if (use_imu) {
      RCLCPP_INFO(get_logger(), "enable imu-based prediction");
      imu_sub = create_subscription<sensor_msgs::msg::Imu>(
        "/gpsimu_driver/imu_data",
        256,
        std::bind(&HdlLocalizationNodelet::imu_callback, this, std::placeholders::_1));
    }

    points_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points",
      5,
      std::bind(&HdlLocalizationNodelet::points_callback, this, std::placeholders::_1));

    auto latch_qos = rclcpp::QoS(1).transient_local();
    globalmap_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/globalmap",
      latch_qos,
      std::bind(&HdlLocalizationNodelet::globalmap_callback, this, std::placeholders::_1));

    initialpose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",
      8,
      std::bind(&HdlLocalizationNodelet::initialpose_callback, this, std::placeholders::_1));

    pose_pub = create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
    aligned_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", 5);
    status_pub = create_publisher<hdl_localization::msg::ScanMatchingStatus>("/status", 5);
    reloc_status_pub = create_publisher<hdl_localization::msg::HdlRelocStatus>(
      "/hdl_localization/reloc_status", 5);

    initialize_params();

    use_global_localization = declare_parameter<bool>("use_global_localization", true);
    if (use_global_localization) {
      global_loc_client_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      set_global_map_service = create_client<hdl_global_localization::srv::SetGlobalMap>(
        "/hdl_global_localization/set_global_map",
        rmw_qos_profile_services_default,
        global_loc_client_cb_group);
      query_global_localization_service = create_client<hdl_global_localization::srv::QueryGlobalLocalization>(
        "/hdl_global_localization/query",
        rmw_qos_profile_services_default,
        global_loc_client_cb_group);

      relocalize_cb_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      relocalize_server = create_service<std_srvs::srv::Empty>(
        "/relocalize",
        std::bind(&HdlLocalizationNodelet::relocalize, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        relocalize_cb_group);

      global_loc_init_timer = create_wall_timer(
        std::chrono::milliseconds(0),
        std::bind(&HdlLocalizationNodelet::wait_for_global_localization_services, this));

      RCLCPP_INFO(get_logger(), "/relocalize service advertised");
    }

    if (enable_auto_relocalize_monitor) {
      RCLCPP_INFO(
        get_logger(),
        "auto relocalize monitor enabled (threshold=%.3f m, cooldown=%.1f s)",
        auto_relocalize_error_threshold,
        auto_relocalize_cooldown);
    }
  }

private:
  pcl::Registration<PointT, PointT>::Ptr create_registration() {
    if (reg_method == "NDT_OMP") {
      RCLCPP_INFO(get_logger(), "NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(
        new pclomp::NormalDistributionsTransform<PointT, PointT>());

      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);

      if (ndt_neighbor_search_method == "DIRECT1") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (ndt_neighbor_search_method == "KDTREE") {
          RCLCPP_INFO(get_logger(), "search_method KDTREE is selected");
        } else {
          RCLCPP_WARN(get_logger(), "invalid search method was given");
          RCLCPP_WARN(get_logger(), "default method is selected (KDTREE)");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    } else if (reg_method.find("NDT_CUDA") != std::string::npos) {
      RCLCPP_ERROR(get_logger(),
        "NDT_CUDA is not available (fast_gicp built with BUILD_VGICP_CUDA=OFF). Use NDT_OMP.");
    }

    RCLCPP_ERROR_STREAM(get_logger(), "unknown registration method:" << reg_method);
    return nullptr;
  }

  void initialize_params() {
    downsample_resolution = declare_parameter<double>("downsample_resolution", 0.1);
    auto voxelgrid = std::make_shared<pcl::VoxelGrid<PointT>>();
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    RCLCPP_INFO(get_logger(), "create registration method for localization");
    registration = create_registration();

    RCLCPP_INFO(get_logger(), "create registration method for fallback during relocalization");
    relocalizing = false;
    auto_relocalize_in_progress = false;
    delta_estimater.reset(new DeltaEstimater(create_registration()));

    bool specify_init_pose = declare_parameter<bool>("specify_init_pose", true);
    if (specify_init_pose) {
      const double init_pos_x = declare_parameter<double>("init_pos_x", 0.0);
      const double init_pos_y = declare_parameter<double>("init_pos_y", 0.0);
      const double init_pos_z = declare_parameter<double>("init_pos_z", 0.0);
      const double init_ori_w = declare_parameter<double>("init_ori_w", 1.0);
      const double init_ori_x = declare_parameter<double>("init_ori_x", 0.0);
      const double init_ori_y = declare_parameter<double>("init_ori_y", 0.0);
      const double init_ori_z = declare_parameter<double>("init_ori_z", 0.0);

      RCLCPP_INFO(get_logger(), "initialize pose estimator with specified parameters");
      RCLCPP_INFO(
        get_logger(),
        "  init position (map): %.3f, %.3f, %.3f",
        init_pos_x, init_pos_y, init_pos_z);
      RCLCPP_INFO(
        get_logger(),
        "  init orientation (wxyz): %.4f, %.4f, %.4f, %.4f",
        init_ori_w, init_ori_x, init_ori_y, init_ori_z);

      pose_estimator.reset(new hdl_localization::PoseEstimator(
        registration,
        get_clock()->now(),
        Eigen::Vector3f(
          static_cast<float>(init_pos_x),
          static_cast<float>(init_pos_y),
          static_cast<float>(init_pos_z)),
        Eigen::Quaternionf(
          static_cast<float>(init_ori_w),
          static_cast<float>(init_ori_x),
          static_cast<float>(init_ori_y),
          static_cast<float>(init_ori_z)),
        cool_time_duration
      ));
    } else {
      RCLCPP_INFO(get_logger(),
        "specify_init_pose=false: waiting for RViz /initialpose (2D Pose Estimate)");
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu_msg);
  }

  void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    if (!globalmap) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "globalmap has not been received!!");
      return;
    }

    const auto& stamp = points_msg->header.stamp;

    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    if (pcl_cloud->empty()) {
      RCLCPP_ERROR(get_logger(), "cloud is empty!!");
      return;
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    const rclcpp::Time cloud_stamp(stamp);
    if (!tf_buffer->canTransform(
          odom_child_frame_id,
          points_msg->header.frame_id,
          cloud_stamp,
          rclcpp::Duration(std::chrono::milliseconds(100)))) {
      RCLCPP_ERROR(
        get_logger(),
        "cannot transform %s -> %s at stamp %.3f",
        points_msg->header.frame_id.c_str(),
        odom_child_frame_id.c_str(),
        cloud_stamp.seconds());
      return;
    }

    try {
      geometry_msgs::msg::TransformStamped tf = tf_buffer->lookupTransform(
        odom_child_frame_id,
        points_msg->header.frame_id,
        cloud_stamp,
        rclcpp::Duration(std::chrono::milliseconds(100)));

      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*pcl_cloud, cloud_msg);
      cloud_msg.header = points_msg->header;

      sensor_msgs::msg::PointCloud2 transformed_msg;
      tf2::doTransform(cloud_msg, transformed_msg, tf);
      pcl::fromROSMsg(transformed_msg, *cloud);
      cloud->header.frame_id = odom_child_frame_id;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(get_logger(), "point cloud transform failed: %s", ex.what());
      return;
    }

    auto filtered = downsample(cloud);
    last_scan = filtered;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5.0,
      "scan points: raw=%zu downsampled=%zu (leaf=%.2f m)",
      cloud->size(),
      filtered->size(),
      downsample_resolution);

    if (relocalizing) {
      delta_estimater->add_frame(filtered);
    }

    Eigen::Matrix4f pose_matrix = Eigen::Matrix4f::Identity();
    double linear_matching_error = 0.0;
    bool converged = false;
    double matching_time_ms = 0.0;
    double fitness_score = 0.0;
    double cov_trace = 0.0;

    {
      std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);

      if (!pose_estimator) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "waiting for initial pose input!!");
        return;
      }

      if (!use_imu) {
        pose_estimator->predict(stamp);
      } else {
        std::lock_guard<std::mutex> lock(imu_data_mutex);

        auto imu_iter = imu_data.begin();
        for (; imu_iter != imu_data.end(); ++imu_iter) {
          if (rclcpp::Time(stamp) < rclcpp::Time((*imu_iter)->header.stamp)) {
            break;
          }

          const auto& acc = (*imu_iter)->linear_acceleration;
          const auto& gyro = (*imu_iter)->angular_velocity;

          double acc_sign = invert_acc ? -1.0 : 1.0;
          double gyro_sign = invert_gyro ? -1.0 : 1.0;

          pose_estimator->predict(
            (*imu_iter)->header.stamp,
            acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z),
            gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
        }
        imu_data.erase(imu_data.begin(), imu_iter);
      }

      rclcpp::Time last_correction_time = pose_estimator->last_correction_time();
      if (enable_robot_odometry_prediction &&
          last_correction_time != rclcpp::Time((int64_t)0, get_clock()->get_clock_type())) {
        geometry_msgs::msg::TransformStamped odom_delta;

        if (tf_buffer->canTransform(
              odom_child_frame_id,
              last_correction_time,
              odom_child_frame_id,
              stamp,
              robot_odom_frame_id,
              rclcpp::Duration(std::chrono::milliseconds(100)))) {
          odom_delta = tf_buffer->lookupTransform(
            odom_child_frame_id,
            last_correction_time,
            odom_child_frame_id,
            stamp,
            robot_odom_frame_id,
            rclcpp::Duration(std::chrono::milliseconds(0)));
        } else if (tf_buffer->canTransform(
                     odom_child_frame_id,
                     last_correction_time,
                     odom_child_frame_id,
                     rclcpp::Time((int64_t)0, get_clock()->get_clock_type()),
                     robot_odom_frame_id,
                     rclcpp::Duration(std::chrono::milliseconds(0)))) {
          odom_delta = tf_buffer->lookupTransform(
            odom_child_frame_id,
            last_correction_time,
            odom_child_frame_id,
            rclcpp::Time((int64_t)0, get_clock()->get_clock_type()),
            robot_odom_frame_id,
            rclcpp::Duration(std::chrono::milliseconds(0)));
        }

        if (odom_delta.header.stamp == rclcpp::Time((int64_t)0, get_clock()->get_clock_type())) {
          RCLCPP_WARN_STREAM(
            get_logger(),
            "failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
        } else {
          Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
          pose_estimator->predict_odom(delta.cast<float>().matrix());
        }
      }

      const auto t_start = get_clock()->now();
      auto aligned = pose_estimator->correct(stamp, filtered);
      const auto t_end = get_clock()->now();
      matching_time_ms = (t_end - t_start).seconds() * 1000.0;

      fitness_score = registration->getFitnessScore();
      converged = registration->hasConverged();
      cov_trace = pose_estimator->get_cov_trace();
      linear_matching_error = std::sqrt(std::max(0.0, fitness_score));

      hdl_localization::msg::HdlRelocStatus reloc_msg;
      reloc_msg.header.stamp = stamp;
      reloc_msg.header.frame_id = "map";
      reloc_msg.fitness_score = fitness_score;
      reloc_msg.matching_time_ms = matching_time_ms;
      reloc_msg.covariance_trace = cov_trace;
      reloc_msg.is_converged = converged;
      reloc_status_pub->publish(reloc_msg);

      RCLCPP_INFO_STREAM(
        get_logger(),
        "Metrics -> Error: " << fitness_score
        << " (RMSE ~" << linear_matching_error << " m)"
        << " | Time: " << matching_time_ms << " ms"
        << " | CovTrace: " << cov_trace
        << " | Converged: " << (converged ? "True" : "False"));

      if (aligned_pub->get_subscription_count()) {
        aligned->header.frame_id = "map";
        aligned->header.stamp = cloud->header.stamp;
        sensor_msgs::msg::PointCloud2 aligned_msg;
        pcl::toROSMsg(*aligned, aligned_msg);
        aligned_pub->publish(aligned_msg);
      }

      if (status_pub->get_subscription_count()) {
        publish_scan_matching_status(points_msg->header, aligned);
      }

      pose_matrix = pose_estimator->matrix();
    }

    publish_odometry(points_msg->header.stamp, pose_matrix);

    if (enable_auto_relocalize_monitor && use_global_localization) {
      maybe_trigger_auto_relocalize(linear_matching_error, converged);
    }
  }

  void wait_for_global_localization_services() {
    global_loc_init_timer.reset();

    RCLCPP_INFO(get_logger(), "wait for global localization services");
    constexpr auto k_wait_timeout = std::chrono::seconds(120);
    if (!set_global_map_service->wait_for_service(k_wait_timeout)) {
      RCLCPP_ERROR(get_logger(), "SetGlobalMap service not available (is hdl_global_localization running?)");
      return;
    }
    if (!query_global_localization_service->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "QueryGlobalLocalization service not available");
      return;
    }

    global_loc_services_ready = true;
    RCLCPP_INFO(get_logger(), "global localization services are ready");
    send_global_map_to_engine();
  }

  bool send_global_map_to_engine() {
    if (!use_global_localization || !set_global_map_service || !globalmap) {
      return false;
    }
    if (!set_global_map_service->service_is_ready()) {
      return false;
    }

    auto req = std::make_shared<hdl_global_localization::srv::SetGlobalMap::Request>();
    pcl::toROSMsg(*globalmap, req->global_map);
    req->global_map.header.frame_id = "map";

    auto future = set_global_map_service->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "SetGlobalMap timeout");
      return false;
    }

    try {
      future.get();
      RCLCPP_INFO(get_logger(), "SetGlobalMap finished");
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "SetGlobalMap failed: %s", e.what());
      return false;
    }
  }

  void globalmap_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    RCLCPP_INFO(get_logger(), "globalmap received!");

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap = cloud;

    registration->setInputTarget(globalmap);

    if (use_global_localization && set_global_map_service) {
      if (!global_loc_services_ready && !set_global_map_service->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "SetGlobalMap service not ready yet, will retry when services are up");
        return;
      }
      send_global_map_to_engine();
    }
  }

  void maybe_trigger_auto_relocalize(double linear_matching_error, bool /*converged*/) {
    if (!enable_auto_relocalize_monitor || !use_global_localization) {
      return;
    }

    if (relocalizing || auto_relocalize_in_progress) {
      return;
    }

    if (linear_matching_error <= auto_relocalize_error_threshold) {
      return;
    }

    const rclcpp::Time now = get_clock()->now();
    if (last_auto_relocalize_time_.nanoseconds() != 0 &&
        (now - last_auto_relocalize_time_).seconds() < auto_relocalize_cooldown) {
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "auto relocalize triggered: RMSE=%.3f m (threshold=%.3f m)",
      linear_matching_error,
      auto_relocalize_error_threshold);

    auto_relocalize_in_progress = true;
    auto_relocalize_timer = create_wall_timer(
      std::chrono::milliseconds(0),
      std::bind(&HdlLocalizationNodelet::auto_relocalize_timer_callback, this),
      relocalize_cb_group);
  }

  void auto_relocalize_timer_callback() {
    auto_relocalize_timer.reset();

    const bool ok = execute_relocalize("auto_monitor");
    auto_relocalize_in_progress = false;
    if (ok) {
      last_auto_relocalize_time_ = get_clock()->now();
    }
  }

  bool execute_relocalize(const char* trigger_source) {
    if (last_scan == nullptr) {
      RCLCPP_WARN(get_logger(), "[%s] no scan has been received", trigger_source);
      return false;
    }

    if (!query_global_localization_service) {
      RCLCPP_WARN(get_logger(), "[%s] global localization client is not available", trigger_source);
      return false;
    }

    if (!query_global_localization_service->service_is_ready()) {
      RCLCPP_ERROR(get_logger(),
        "[%s] QueryGlobalLocalization service not available "
        "(ensure hdl_global_localization is running and services are ready)",
        trigger_source);
      return false;
    }

    relocalizing = true;
    delta_estimater->reset();

    pcl::PointCloud<PointT>::ConstPtr scan = last_scan;

    RCLCPP_INFO(
      get_logger(),
      "[%s] relocalize query cloud: %zu points, frame=%s",
      trigger_source,
      scan->size(),
      scan->header.frame_id.c_str());

    auto query_req = std::make_shared<hdl_global_localization::srv::QueryGlobalLocalization::Request>();
    pcl::toROSMsg(*scan, query_req->cloud);
    query_req->cloud.header.frame_id = scan->header.frame_id;
    query_req->max_num_candidates = 1;

    auto future = query_global_localization_service->async_send_request(query_req);
    if (future.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "[%s] QueryGlobalLocalization timeout", trigger_source);
      relocalizing = false;
      return false;
    }

    auto query_result = future.get();
    if (query_result->poses.empty()) {
      RCLCPP_ERROR(get_logger(), "[%s] global localization failed", trigger_source);
      relocalizing = false;
      return false;
    }

    const auto& result = query_result->poses[0];

    RCLCPP_INFO_STREAM(get_logger(), "--- Global localization result (" << trigger_source << ") ---");
    RCLCPP_INFO_STREAM(get_logger(),
      "Trans :" << result.position.x << " " << result.position.y << " " << result.position.z);
    RCLCPP_INFO_STREAM(get_logger(),
      "Quat  :" << result.orientation.x << " " << result.orientation.y << " "
                << result.orientation.z << " " << result.orientation.w);
    RCLCPP_INFO_STREAM(get_logger(), "Error :" << query_result->errors[0]);
    RCLCPP_INFO_STREAM(get_logger(), "Inlier:" << query_result->inlier_fractions[0]);

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() = Eigen::Quaternionf(
      result.orientation.w,
      result.orientation.x,
      result.orientation.y,
      result.orientation.z).toRotationMatrix();
    pose.translation() = Eigen::Vector3f(
      result.position.x,
      result.position.y,
      result.position.z);

    pose = pose * delta_estimater->estimated_delta();

    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex);
      pose_estimator.reset(new hdl_localization::PoseEstimator(
        registration,
        get_clock()->now(),
        pose.translation(),
        Eigen::Quaternionf(pose.linear()),
        cool_time_duration));
    }

    relocalizing = false;
    return true;
  }

  bool relocalize(
    std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
  {
    return execute_relocalize("service");
  }

  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg) {
    RCLCPP_INFO(get_logger(), "initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);

    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;

    pose_estimator.reset(
      new hdl_localization::PoseEstimator(
        registration,
        get_clock()->now(),
        Eigen::Vector3f(p.x, p.y, p.z),
        Eigen::Quaternionf(q.w, q.x, q.y, q.z),
        cool_time_duration));
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(
    const pcl::PointCloud<PointT>::ConstPtr& cloud) const
  {
    if (!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  void publish_odometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
    if (send_tf_transforms) {
      if (tf_buffer->canTransform(
            robot_odom_frame_id,
            odom_child_frame_id,
            rclcpp::Time((int64_t)0, get_clock()->get_clock_type()))) {
        geometry_msgs::msg::TransformStamped map_wrt_frame =
          tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
        map_wrt_frame.header.stamp = stamp;
        map_wrt_frame.header.frame_id = odom_child_frame_id;
        map_wrt_frame.child_frame_id = "map";

        geometry_msgs::msg::TransformStamped frame_wrt_odom =
          tf_buffer->lookupTransform(
            robot_odom_frame_id,
            odom_child_frame_id,
            rclcpp::Time((int64_t)0, get_clock()->get_clock_type()),
            rclcpp::Duration(std::chrono::milliseconds(100)));

        geometry_msgs::msg::TransformStamped map_wrt_odom;
        tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

        tf2::Transform odom_wrt_map;
        tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
        odom_wrt_map = odom_wrt_map.inverse();

        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.transform = tf2::toMsg(odom_wrt_map);
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = robot_odom_frame_id;

        tf_broadcaster->sendTransform(odom_trans);
      } else {
        geometry_msgs::msg::TransformStamped odom_trans =
          tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = odom_child_frame_id;
        tf_broadcaster->sendTransform(odom_trans);
      }
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.pose.pose = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
    odom.child_frame_id = odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub->publish(odom);
  }

  void publish_scan_matching_status(
    const std_msgs::msg::Header& header,
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned)
  {
    hdl_localization::msg::ScanMatchingStatus status;
    status.header = header;

    status.matching_error = 0.0;

    const double max_correspondence_dist = 0.5;
    const double max_valid_point_dist = 25.0;

    int num_inliers = 0;
    int num_valid_points = 0;

    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;

    for (int i = 0; i < static_cast<int>(aligned->size()); i++) {
      const auto& pt = aligned->at(i);

      if (pt.getVector3fMap().norm() > max_valid_point_dist) {
        continue;
      }

      num_valid_points++;

      registration->getSearchMethodTarget()->nearestKSearch(
        pt,
        1,
        k_indices,
        k_sq_dists);

      if (k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        status.matching_error += k_sq_dists[0];
        num_inliers++;
      }
    }

    status.matching_error /= std::max(1, num_inliers);
    status.inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);
    status.relative_pose = tf2::eigenToTransform(
      Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;

    status_pub->publish(status);
  }

private:
  std::string robot_odom_frame_id;
  std::string odom_child_frame_id;
  bool send_tf_transforms;

  bool use_imu;
  bool invert_acc;
  bool invert_gyro;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub;
  rclcpp::Publisher<hdl_localization::msg::ScanMatchingStatus>::SharedPtr status_pub;
  rclcpp::Publisher<hdl_localization::msg::HdlRelocStatus>::SharedPtr reloc_status_pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_data;

  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  bool use_global_localization;
  std::atomic_bool relocalizing;
  std::atomic_bool auto_relocalize_in_progress;
  std::unique_ptr<DeltaEstimater> delta_estimater;

  bool enable_auto_relocalize_monitor;
  double auto_relocalize_error_threshold;
  double auto_relocalize_cooldown;
  rclcpp::Time last_auto_relocalize_time_;
  rclcpp::TimerBase::SharedPtr auto_relocalize_timer;

  pcl::PointCloud<PointT>::ConstPtr last_scan;
  rclcpp::Client<hdl_global_localization::srv::SetGlobalMap>::SharedPtr set_global_map_service;
  rclcpp::Client<hdl_global_localization::srv::QueryGlobalLocalization>::SharedPtr query_global_localization_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr relocalize_server;
  rclcpp::CallbackGroup::SharedPtr relocalize_cb_group;
  rclcpp::CallbackGroup::SharedPtr global_loc_client_cb_group;
  rclcpp::TimerBase::SharedPtr global_loc_init_timer;
  std::atomic_bool global_loc_services_ready{false};

  double cool_time_duration;
  std::string reg_method;
  std::string ndt_neighbor_search_method;
  double ndt_neighbor_search_radius;
  double ndt_resolution;
  double downsample_resolution;
  bool enable_robot_odometry_prediction;
};

}  // namespace hdl_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::HdlLocalizationNodelet)
