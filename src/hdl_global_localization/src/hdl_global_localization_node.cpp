#include <mutex>
#include <memory>
#include <functional>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/srv/query_global_localization.hpp>
#include <hdl_global_localization/srv/set_global_localization_engine.hpp>

#include <hdl_global_localization/engines/global_localization_bbs.hpp>
#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>

namespace hdl_global_localization {

class GlobalLocalizationNode : public rclcpp::Node {
public:
  GlobalLocalizationNode(const rclcpp::NodeOptions& options)
  : Node("hdl_global_localization", options)
  {
    globalmap_downsample_resolution = declare_parameter<double>("globalmap_downsample_resolution", 0.1);
    query_downsample_resolution = declare_parameter<double>("query_downsample_resolution", 0.1);
    engine_name = declare_parameter<std::string>("global_localization_engine", "BBS");

    set_engine_srv = create_service<srv::SetGlobalLocalizationEngine>(
      "~/set_engine",
      std::bind(&GlobalLocalizationNode::set_engine_cb, this, std::placeholders::_1, std::placeholders::_2));

    set_map_srv = create_service<srv::SetGlobalMap>(
      "~/set_global_map",
      std::bind(&GlobalLocalizationNode::set_global_map, this, std::placeholders::_1, std::placeholders::_2));

    query_localization_srv = create_service<srv::QueryGlobalLocalization>(
      "~/query",
      std::bind(&GlobalLocalizationNode::query, this, std::placeholders::_1, std::placeholders::_2));

    // set_engine() uses shared_from_this(); defer until the node is owned by shared_ptr
    // (required for composable nodes loaded into component_container).
    init_timer = create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() {
        init_timer.reset();

        RCLCPP_INFO(get_logger(), "global_localization_engine = %s", engine_name.c_str());
        if (!set_engine(engine_name)) {
          RCLCPP_ERROR(get_logger(), "Failed to initialize global localization engine: %s", engine_name.c_str());
          return;
        }

        RCLCPP_INFO(get_logger(), "Start Global Localization Node");
        RCLCPP_INFO(get_logger(),
          "hdl_global_localization ready (/hdl_global_localization/set_global_map, "
          "/hdl_global_localization/query, /hdl_global_localization/set_engine)");
      });
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    double resolution)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(resolution, resolution, resolution);
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*filtered);
    return filtered;
  }

  bool set_engine(const std::string& engine_name) {
    std::lock_guard<std::mutex> lock(engine_mutex);

    if (engine_name == "BBS") {
      engine.reset(new GlobalLocalizationBBS(shared_from_this()));
    } else if (engine_name == "FPFH_RANSAC") {
      engine.reset(new GlobalLocalizationEngineFPFH_RANSAC(shared_from_this()));
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown Global Localization Engine: %s", engine_name.c_str());
      return false;
    }

    if (global_map) {
      engine->set_global_map(global_map);
    }

    return true;
  }

  bool set_engine_cb(
    srv::SetGlobalLocalizationEngine::Request::SharedPtr req,
    srv::SetGlobalLocalizationEngine::Response::SharedPtr /*res*/)
  {
    RCLCPP_INFO(get_logger(), "Set Global Localization Engine");
    return set_engine(req->engine_name.data);
  }

  bool set_global_map(
    srv::SetGlobalMap::Request::SharedPtr req,
    srv::SetGlobalMap::Response::SharedPtr /*res*/)
  {
    RCLCPP_INFO(get_logger(), "Global Map Received");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req->global_map, *cloud);

    cloud = downsample(cloud, globalmap_downsample_resolution);

    {
      std::lock_guard<std::mutex> lock(engine_mutex);
      globalmap_header = req->global_map.header;
      global_map = cloud;

      if (engine) {
        engine->set_global_map(global_map);
      }
    }

    RCLCPP_INFO(get_logger(), "DONE");
    return true;
  }

  bool query(
    srv::QueryGlobalLocalization::Request::SharedPtr req,
    srv::QueryGlobalLocalization::Response::SharedPtr res)
  {
    RCLCPP_INFO(get_logger(), "Query Global Localization");

    if (!global_map) {
      RCLCPP_WARN(get_logger(), "No Globalmap");
      return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req->cloud, *cloud);
    cloud = downsample(cloud, query_downsample_resolution);

    auto results = [&]() {
      std::lock_guard<std::mutex> lock(engine_mutex);
      if (!engine) {
        throw std::runtime_error("Engine is null");
      }
      return engine->query(cloud, req->max_num_candidates);
    }();

    res->inlier_fractions.resize(results.results.size());
    res->errors.resize(results.results.size());
    res->poses.resize(results.results.size());
    res->header = req->cloud.header;
    res->globalmap_header = globalmap_header;

    for (size_t i = 0; i < results.results.size(); i++) {
      const auto& result = results.results[i];
      Eigen::Quaternionf quat(result->pose.linear());
      Eigen::Vector3f trans(result->pose.translation());

      res->inlier_fractions[i] = result->inlier_fraction;
      res->errors[i] = result->error;
      res->poses[i].orientation.x = quat.x();
      res->poses[i].orientation.y = quat.y();
      res->poses[i].orientation.z = quat.z();
      res->poses[i].orientation.w = quat.w();
      res->poses[i].position.x = trans.x();
      res->poses[i].position.y = trans.y();
      res->poses[i].position.z = trans.z();
    }

    return !results.results.empty();
  }

private:
  rclcpp::Service<srv::SetGlobalLocalizationEngine>::SharedPtr set_engine_srv;
  rclcpp::Service<srv::SetGlobalMap>::SharedPtr set_map_srv;
  rclcpp::Service<srv::QueryGlobalLocalization>::SharedPtr query_localization_srv;

  std_msgs::msg::Header globalmap_header;
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
  std::unique_ptr<GlobalLocalizationEngine> engine;

  std::mutex engine_mutex;

  double query_downsample_resolution;
  double globalmap_downsample_resolution;
  std::string engine_name;
  rclcpp::TimerBase::SharedPtr init_timer;
};

}  // namespace hdl_global_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_global_localization::GlobalLocalizationNode)
