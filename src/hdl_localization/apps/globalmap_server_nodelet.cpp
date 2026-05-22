#include <fstream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace hdl_localization {

class GlobalmapServerNodelet : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNodelet(const rclcpp::NodeOptions& options)
  : Node("globalmap_server", options) {
    initialize_params();

    auto latch_qos = rclcpp::QoS(1).transient_local();
    globalmap_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/globalmap", latch_qos);
    map_update_sub = create_subscription<std_msgs::msg::String>(
      "/map_request/pcd", 10,
      std::bind(&GlobalmapServerNodelet::map_update_callback, this, std::placeholders::_1));

    globalmap_pub_timer = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GlobalmapServerNodelet::pub_once_cb, this));
  }

private:
  void initialize_params() {
    std::string globalmap_pcd = declare_parameter<std::string>("globalmap_pcd", "");
    if (globalmap_pcd.empty()) {
      RCLCPP_ERROR(get_logger(), "globalmap_pcd parameter is empty");
      return;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Loading global map: " << globalmap_pcd);

    globalmap.reset(new pcl::PointCloud<PointT>());
    if (pcl::io::loadPCDFile(globalmap_pcd, *globalmap) < 0) {
      RCLCPP_ERROR_STREAM(get_logger(),
        "Failed to load global map PCD: " << globalmap_pcd
        << " (check path exists; after editing launch run: colcon build --packages-select hdl_localization)");
      globalmap->clear();
      return;
    }
    RCLCPP_INFO_STREAM(get_logger(),
      "Global map loaded: " << globalmap->size() << " points");
    globalmap->header.frame_id = "map";

    bool convert_utm_to_local = declare_parameter<bool>("convert_utm_to_local", true);
    std::ifstream utm_file(globalmap_pcd + ".utm");
    if (utm_file.is_open() && convert_utm_to_local) {
      double utm_easting, utm_northing, altitude;
      utm_file >> utm_easting >> utm_northing >> altitude;
      for (auto& pt : globalmap->points) {
        pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
      }
      RCLCPP_INFO_STREAM(get_logger(), "Global map offset by UTM (x=" << utm_easting
                        << ", y=" << utm_northing << ", z=" << altitude << ")");
    }

    double downsample_resolution = declare_parameter<double>("downsample_resolution", 0.1);
    pcl::VoxelGrid<PointT> voxelgrid;
    voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid.setInputCloud(globalmap);
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid.filter(*filtered);
    globalmap = filtered;
  }

  void pub_once_cb() {
    if (!globalmap || globalmap->empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Global map is empty, skip publishing /globalmap");
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*globalmap, msg);
    msg.header.frame_id = "map";
    globalmap_pub->publish(msg);
    globalmap_pub_timer.reset();
  }

  void map_update_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO_STREAM(get_logger(), "Received map request: " << msg->data);
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(msg->data, *globalmap);
    globalmap->header.frame_id = "map";

    double downsample_resolution = get_parameter("downsample_resolution").as_double();
    pcl::VoxelGrid<PointT> voxelgrid;
    voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid.setInputCloud(globalmap);
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid.filter(*filtered);
    globalmap = filtered;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*globalmap, cloud_msg);
    cloud_msg.header.frame_id = "map";
    globalmap_pub->publish(cloud_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_update_sub;
  rclcpp::TimerBase::SharedPtr globalmap_pub_timer;
  pcl::PointCloud<PointT>::Ptr globalmap;
};

}  // namespace hdl_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::GlobalmapServerNodelet)
