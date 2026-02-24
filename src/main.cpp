#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

class MultiLidarMerger : public rclcpp::Node {
public:
  MultiLidarMerger()
  : Node("multi_lidar_merger"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    load_parameters();

    if (lidar_topics_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'lidar_topics' is empty. Nothing to subscribe to.");
      return;
    }

    if (enabled_lidars_.size() != lidar_topics_.size()) {
      RCLCPP_WARN(get_logger(),
                  "enabled_lidars size (%zu) != lidar_topics size (%zu). Resizing enabled_lidars to match.",
                  enabled_lidars_.size(), lidar_topics_.size());
      enabled_lidars_.assign(lidar_topics_.size(), true);
    }

    // Dynamic parameter updates
    setup_parameter_callbacks();

    // Subscribers (created for all topics; runtime gating uses enabled_lidars_)
    for (size_t i = 0; i < lidar_topics_.size(); ++i) {
      auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topics_[i],
        rclcpp::SensorDataQoS(),
        [this, i](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->cloudCallback(msg, i);
        }
      );
      cloud_subscribers_.push_back(sub);
    }

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_out_, 10);

    RCLCPP_INFO(get_logger(),
                "MultiLidarMerger started. target_frame='%s', out='%s', sync=%s, per_cloud_filter=%s, ground_filter=%s",
                target_frame_.c_str(),
                pointcloud_topic_out_.c_str(),
                sync_ ? "true" : "false",
                enable_filter_cloud_ ? "true" : "false",
                ground_filter_enable_ ? "true" : "false");
  }

private:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  void declare_parameters() {
    this->declare_parameter("lidar_topics", std::vector<std::string>{});
    this->declare_parameter("target_frame", "base_link");
    this->declare_parameter("publish_merged_pointcloud", true);
    this->declare_parameter("pointCloudTopic_out", "/merged_pointcloud");
    this->declare_parameter("sync", false);

    // Per-cloud reduction
    this->declare_parameter("enable_filter_cloud", true);
    this->declare_parameter("voxel_size", 0.10);

    // Robot footprint cropbox (removal)
    this->declare_parameter("cropbox_x", 0.91);
    this->declare_parameter("cropbox_y", 0.82);
    this->declare_parameter("cropbox_z", 3.0);

    // Lidar enable + per-lidar filter params
    // NOTE: per-lidar range/angle filtering is currently not applied
    this->declare_parameter("enabled_lidars", std::vector<bool>{});
    this->declare_parameter("min_ranges", std::vector<double>{});
    this->declare_parameter("max_ranges", std::vector<double>{});
    this->declare_parameter("min_angles", std::vector<double>{});
    this->declare_parameter("max_angles", std::vector<double>{});

    // Ground removal (merged cloud)
    this->declare_parameter("ground_filter_enable", true);
    this->declare_parameter("ground_axis_eps_deg", 15.0);
    this->declare_parameter("ground_dist_thresh", 0.05);
    this->declare_parameter("ground_z_prefilter_min", -2.0);
    this->declare_parameter("ground_z_prefilter_max",  1.0);
    this->declare_parameter("ground_min_inliers", 200);
    this->declare_parameter("ground_max_iterations", 150);
  }

  void load_parameters() {
    lidar_topics_ = this->get_parameter("lidar_topics").as_string_array();

    target_frame_ = this->get_parameter("target_frame").as_string();
    publish_merged_pointcloud_ = this->get_parameter("publish_merged_pointcloud").as_bool();
    pointcloud_topic_out_ = this->get_parameter("pointCloudTopic_out").as_string();
    sync_ = this->get_parameter("sync").as_bool();

    enable_filter_cloud_ = this->get_parameter("enable_filter_cloud").as_bool();
    voxel_size_ = this->get_parameter("voxel_size").as_double();

    // Cropbox symmetric about origin
    const double cx = this->get_parameter("cropbox_x").as_double();
    const double cy = this->get_parameter("cropbox_y").as_double();
    const double cz = this->get_parameter("cropbox_z").as_double();
    min_cropbox_point_ = Eigen::Vector4f(-static_cast<float>(cx), -static_cast<float>(cy), -static_cast<float>(cz), 1.0f);
    max_cropbox_point_ = Eigen::Vector4f( static_cast<float>(cx),  static_cast<float>(cy),  static_cast<float>(cz), 1.0f);

    // These may be empty; we’ll size-check later
    enabled_lidars_ = this->get_parameter("enabled_lidars").as_bool_array();
    min_ranges_ = this->get_parameter("min_ranges").as_double_array();
    max_ranges_ = this->get_parameter("max_ranges").as_double_array();
    min_angles_ = this->get_parameter("min_angles").as_double_array();
    max_angles_ = this->get_parameter("max_angles").as_double_array();

    // Ground filter
    ground_filter_enable_ = this->get_parameter("ground_filter_enable").as_bool();
    ground_axis_eps_deg_ = this->get_parameter("ground_axis_eps_deg").as_double();
    ground_dist_thresh_ = this->get_parameter("ground_dist_thresh").as_double();
    ground_z_prefilter_min_ = this->get_parameter("ground_z_prefilter_min").as_double();
    ground_z_prefilter_max_ = this->get_parameter("ground_z_prefilter_max").as_double();
    ground_min_inliers_ = static_cast<int>(this->get_parameter("ground_min_inliers").as_int());
    ground_max_iterations_ = static_cast<int>(this->get_parameter("ground_max_iterations").as_int());
  }

  void setup_parameter_callbacks() {
    parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto param_callback = [this](const rclcpp::Parameter &p) {
      const std::string &name = p.get_name();

      if (name == "enabled_lidars") {
        enabled_lidars_ = p.as_bool_array();
        RCLCPP_INFO(get_logger(), "Updated enabled_lidars");
      } else if (name == "min_ranges") {
        min_ranges_ = p.as_double_array();
        RCLCPP_INFO(get_logger(), "Updated min_ranges");
      } else if (name == "max_ranges") {
        max_ranges_ = p.as_double_array();
        RCLCPP_INFO(get_logger(), "Updated max_ranges");
      } else if (name == "min_angles") {
        min_angles_ = p.as_double_array();
        RCLCPP_INFO(get_logger(), "Updated min_angles");
      } else if (name == "max_angles") {
        max_angles_ = p.as_double_array();
        RCLCPP_INFO(get_logger(), "Updated max_angles");
      } else if (name == "voxel_size") {
        voxel_size_ = p.as_double();
        RCLCPP_INFO(get_logger(), "Updated voxel_size=%.3f", voxel_size_);
      } else if (name == "sync") {
        sync_ = p.as_bool();
        RCLCPP_INFO(get_logger(), "Updated sync=%s", sync_ ? "true" : "false");
      } else if (name == "enable_filter_cloud") {
        enable_filter_cloud_ = p.as_bool();
        RCLCPP_INFO(get_logger(), "Updated enable_filter_cloud=%s", enable_filter_cloud_ ? "true" : "false");
      } else if (name == "cropbox_x") {
        min_cropbox_point_.x() = -static_cast<float>(p.as_double());
        max_cropbox_point_.x() =  static_cast<float>(p.as_double());
        RCLCPP_INFO(get_logger(), "Updated cropbox_x=%.3f", p.as_double());
      } else if (name == "cropbox_y") {
        min_cropbox_point_.y() = -static_cast<float>(p.as_double());
        max_cropbox_point_.y() =  static_cast<float>(p.as_double());
        RCLCPP_INFO(get_logger(), "Updated cropbox_y=%.3f", p.as_double());
      } else if (name == "cropbox_z") {
        min_cropbox_point_.z() = -static_cast<float>(p.as_double());
        max_cropbox_point_.z() =  static_cast<float>(p.as_double());
        RCLCPP_INFO(get_logger(), "Updated cropbox_z=%.3f", p.as_double());
      }

      // Ground filter params
      else if (name == "ground_filter_enable") {
        ground_filter_enable_ = p.as_bool();
        RCLCPP_INFO(get_logger(), "Updated ground_filter_enable=%s", ground_filter_enable_ ? "true" : "false");
      } else if (name == "ground_axis_eps_deg") {
        ground_axis_eps_deg_ = p.as_double();
        RCLCPP_INFO(get_logger(), "Updated ground_axis_eps_deg=%.2f", ground_axis_eps_deg_);
      } else if (name == "ground_dist_thresh") {
        ground_dist_thresh_ = p.as_double();
        RCLCPP_INFO(get_logger(), "Updated ground_dist_thresh=%.3f", ground_dist_thresh_);
      } else if (name == "ground_z_prefilter_min") {
        ground_z_prefilter_min_ = p.as_double();
        RCLCPP_INFO(get_logger(), "Updated ground_z_prefilter_min=%.3f", ground_z_prefilter_min_);
      } else if (name == "ground_z_prefilter_max") {
        ground_z_prefilter_max_ = p.as_double();
        RCLCPP_INFO(get_logger(), "Updated ground_z_prefilter_max=%.3f", ground_z_prefilter_max_);
      } else if (name == "ground_min_inliers") {
        ground_min_inliers_ = static_cast<int>(p.as_int());
        RCLCPP_INFO(get_logger(), "Updated ground_min_inliers=%d", ground_min_inliers_);
      } else if (name == "ground_max_iterations") {
        ground_max_iterations_ = static_cast<int>(p.as_int());
        RCLCPP_INFO(get_logger(), "Updated ground_max_iterations=%d", ground_max_iterations_);
      } else {
        RCLCPP_WARN(get_logger(), "Unknown parameter updated: %s", name.c_str());
      }
    };

    // Register callbacks and keep handles alive
    auto add_cb = [&](const std::string &param) {
      parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback(param, param_callback));
    };

    add_cb("enabled_lidars");
    add_cb("min_ranges");
    add_cb("max_ranges");
    add_cb("min_angles");
    add_cb("max_angles");
    add_cb("voxel_size");
    add_cb("sync");
    add_cb("enable_filter_cloud");
    add_cb("cropbox_x");
    add_cb("cropbox_y");
    add_cb("cropbox_z");

    add_cb("ground_filter_enable");
    add_cb("ground_axis_eps_deg");
    add_cb("ground_dist_thresh");
    add_cb("ground_z_prefilter_min");
    add_cb("ground_z_prefilter_max");
    add_cb("ground_min_inliers");
    add_cb("ground_max_iterations");
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, size_t index) {
    if (index >= enabled_lidars_.size() || !enabled_lidars_[index]) {
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed;
    try {
      const auto tf = tf_buffer_.lookupTransform(
        target_frame_, cloud_msg->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(*cloud_msg, transformed, tf);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Transform error: %s", ex.what());
      return;
    }

    CloudT::Ptr pcl_cloud(new CloudT());
    pcl::fromROSMsg(transformed, *pcl_cloud);

    if (enable_filter_cloud_) {
      pcl_cloud = filterCloudPerInput(pcl_cloud);
    }

    clouds_buffer_.push_back(pcl_cloud);

    if (publish_merged_pointcloud_ && (!sync_ || clouds_buffer_.size() == active_input_count())) {
      publishMergedCloud();
    }
  }

  size_t active_input_count() const {
    size_t cnt = 0;
    for (bool en : enabled_lidars_) if (en) cnt++;
    return std::max<size_t>(1, cnt);
  }

  CloudT::Ptr filterCloudPerInput(const CloudT::Ptr &in) const {
    if (!in || in->empty()) return in;

    // Crop robot footprint
    CloudT::Ptr cropped(new CloudT());
    pcl::CropBox<PointT> crop;
    crop.setMin(min_cropbox_point_);
    crop.setMax(max_cropbox_point_);
    crop.setInputCloud(in);
    crop.setNegative(true);
    crop.filter(*cropped);

    if (cropped->empty()) return cropped;

    // Voxel downsample
    if (voxel_size_ > 1e-6) {
      CloudT::Ptr down(new CloudT());
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cropped);
      vg.setLeafSize(static_cast<float>(voxel_size_),
                     static_cast<float>(voxel_size_),
                     static_cast<float>(voxel_size_));
      vg.filter(*down);
      return down;
    }

    return cropped;
  }

  CloudT::Ptr removeGroundPlane(const CloudT::Ptr &in) const {
    if (!in || in->empty()) return in;

    CloudT::Ptr zf(new CloudT());
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(ground_z_prefilter_min_),
                         static_cast<float>(ground_z_prefilter_max_));
    pass.filter(*zf);

    if (zf->size() < static_cast<size_t>(ground_min_inliers_)) {
      return in;
    }

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ground_max_iterations_);
    seg.setDistanceThreshold(ground_dist_thresh_);
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(static_cast<float>(ground_axis_eps_deg_ * M_PI / 180.0));

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    seg.setInputCloud(zf);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.size() < static_cast<size_t>(ground_min_inliers_)) {
      return in;
    }

    CloudT::Ptr out(new CloudT());
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(zf);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*out);

    return out;
  }

  void publishMergedCloud() {
    if (clouds_buffer_.empty()) return;

    CloudT::Ptr merged(new CloudT());
    merged->reserve(estimateReserve());

    for (const auto &c : clouds_buffer_) {
      if (c && !c->empty()) {
        *merged += *c;
      }
    }
    clouds_buffer_.clear();

    if (!merged || merged->empty()) return;

    if (ground_filter_enable_) {
      merged = removeGroundPlane(merged);
      if (!merged || merged->empty()) return;
    }

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*merged, out_msg);
    out_msg.header.frame_id = target_frame_;
    out_msg.header.stamp = this->now();
    cloud_pub_->publish(out_msg);
  }

  size_t estimateReserve() const {
    size_t total = 0;
    for (const auto &c : clouds_buffer_) if (c) total += c->size();
    return total;
  }

  // Topics / subs
  std::vector<std::string> lidar_topics_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subscribers_;

  // Per-lidar enable & filters
  std::vector<bool> enabled_lidars_;
  std::vector<double> min_ranges_, max_ranges_, min_angles_, max_angles_;  // currently unused unless re-enabled

  // Output config
  std::string target_frame_;
  std::string pointcloud_topic_out_;
  bool publish_merged_pointcloud_{true};
  bool sync_{false};

  // Per-cloud filtering
  bool enable_filter_cloud_{true};
  double voxel_size_{0.10};
  Eigen::Vector4f min_cropbox_point_{Eigen::Vector4f::Zero()};
  Eigen::Vector4f max_cropbox_point_{Eigen::Vector4f::Zero()};

  // Ground filtering
  bool ground_filter_enable_{true};
  double ground_axis_eps_deg_{15.0};
  double ground_dist_thresh_{0.05};
  double ground_z_prefilter_min_{-2.0};
  double ground_z_prefilter_max_{1.0};
  int ground_min_inliers_{200};
  int ground_max_iterations_{150};

  // Buffer
  std::vector<CloudT::Ptr> clouds_buffer_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Dynamic param handling
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> parameter_callback_handles_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiLidarMerger>());
  rclcpp::shutdown();
  return 0;
}