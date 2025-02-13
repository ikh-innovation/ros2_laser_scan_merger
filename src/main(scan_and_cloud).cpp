#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <unordered_map>
#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class MultiLidarMerger : public rclcpp::Node {
public:
    MultiLidarMerger() : Node("multi_lidar_merger"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        declare_parameters();
        load_parameters();

        if (lidar_topics_.size() != enabled_lidars_.size()) {
            RCLCPP_ERROR(this->get_logger(), "lidar_topics and enabled_lidars must have the same size!");
            return;
        }

        parameter_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        parameter_callback_handle_ = parameter_event_handler_->add_parameter_callback("enabled_lidars", [this](const rclcpp::Parameter &p) {
            enabled_lidars_ = p.as_bool_array();
            RCLCPP_INFO(this->get_logger(), "Updated enabled_lidars");
        });

        for (size_t i = 0; i < lidar_topics_.size(); ++i) {
            if (enabled_lidars_[i]) {
                auto sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    lidar_topics_[i], rclcpp::SensorDataQoS(),
                    [this, i](std::shared_ptr<sensor_msgs::msg::LaserScan> msg) {
                        this->scanCallback(msg, i);
                    });
                scan_subscribers_.emplace_back(sub);
            }
        }
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("merged_pointcloud", 10);
        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/merged_scan", 10);     
    }

private:

    void declare_parameters() {
        this->declare_parameter("lidar_topics", std::vector<std::string>{});
        this->declare_parameter("enabled_lidars", std::vector<bool>{});
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("publish_merged_pointcloud", true);
        this->declare_parameter("publish_merged_scan", false);
        this->declare_parameter("sync", false);
        this->declare_parameter("min_range", 0.1);
        this->declare_parameter("max_range", 10.0);
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("angle_min", -1.57);
        this->declare_parameter("angle_max", 1.57);
        this->declare_parameter("enable_filter_scan", false);
        this->declare_parameter("enable_filter_cloud", false);
    }

    void load_parameters() {
        lidar_topics_ = this->get_parameter("lidar_topics").as_string_array();
        enabled_lidars_ = this->get_parameter("enabled_lidars").as_bool_array();
        target_frame_ = this->get_parameter("target_frame").as_string();
        publish_merged_pointcloud_ = get_parameter("publish_merged_pointcloud").as_bool();
        publish_merged_scan_ = get_parameter("publish_merged_scan").as_bool();
        sync_ = this->get_parameter("sync").as_bool();
        min_range_ = get_parameter("min_range").as_double();
        max_range_ = get_parameter("max_range").as_double();
        voxel_size_ = get_parameter("voxel_size").as_double();
        angle_min_ = get_parameter("angle_min").as_double();
        angle_max_ = get_parameter("angle_max").as_double();
        enable_filter_scan_ = get_parameter("enable_filter_scan").as_bool();
        enable_filter_cloud_ = get_parameter("enable_filter_cloud").as_bool();
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, size_t index) {
        if (!enabled_lidars_[index]) return;

        if (enable_filter_scan_) {
          filterScan(*scan);
        }

        sensor_msgs::msg::PointCloud2 cloud;
        projector_.projectLaser(*scan, cloud);

        try {
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(cloud, transformed_cloud, tf_buffer_.lookupTransform(
                target_frame_, scan->header.frame_id, scan->header.stamp, tf2::TimePointZero)); // rclcpp::Duration::from_seconds(0.1)

            if (enable_filter_cloud_) {
              filter_cloud(transformed_cloud);
            }    
            merged_cloud_.push_back(transformed_cloud);
            merged_scan_.push_back(*scan);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }

        if (publish_merged_pointcloud_ && (!sync_ || merged_cloud_.size() == scan_subscribers_.size())) {
            publishMergedCloud();
        }

        if (publish_merged_scan_ && (!sync_ || merged_scan_.size() == scan_subscribers_.size())) {
            publishMergedScan();
        }

    }

    void publishMergedCloud() {
        if (merged_cloud_.empty()) return;
        sensor_msgs::msg::PointCloud2 final_cloud = merged_cloud_.front();
        for (size_t i = 1; i < merged_cloud_.size(); ++i) {
            pcl::concatenatePointCloud(final_cloud, merged_cloud_[i], final_cloud);
        }
        final_cloud.header.frame_id = target_frame_;
        final_cloud.header.stamp = this->now();
        cloud_pub_->publish(final_cloud);
        merged_cloud_.clear();
    }
    
    void publishMergedScan() {
        if (merged_scan_.empty()) return;
        sensor_msgs::msg::LaserScan merged_scan = merged_scan_.front();
        for (size_t i = 1; i < merged_scan_.size(); ++i) {
            merged_scan.ranges.insert(merged_scan.ranges.end(), merged_scan_[i].ranges.begin(), merged_scan_[i].ranges.end());
        }
        merged_scan.header.frame_id = target_frame_;
        merged_scan.header.stamp = this->now();
        scan_pub_->publish(merged_scan);
        merged_scan_.clear();
    }

    void filterScan(sensor_msgs::msg::LaserScan &scan) {
        size_t num_ranges = scan.ranges.size();
        std::vector<float> filtered_ranges(num_ranges, std::numeric_limits<float>::quiet_NaN());

        for (size_t i = 0; i < num_ranges; ++i) {
            float angle = scan.angle_min + i * scan.angle_increment;
            if (angle >= angle_min_ && angle <= angle_max_ &&
                scan.ranges[i] >= min_range_ && scan.ranges[i] <= max_range_) {
                filtered_ranges[i] = scan.ranges[i];
            }
        }

        scan.ranges = filtered_ranges;
    }

    void filter_cloud(sensor_msgs::msg::PointCloud2 &cloud) {
        pcl::PCLPointCloud2 pcl_cloud;
        pcl_conversions::toPCL(cloud, pcl_cloud);

        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(std::make_shared<pcl::PCLPointCloud2>(pcl_cloud));
        sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        sor.filter(pcl_cloud);

        pcl_conversions::fromPCL(pcl_cloud, cloud);
    }

    std::vector<std::string> lidar_topics_;
    std::vector<bool> enabled_lidars_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subscribers_;
    std::vector<sensor_msgs::msg::PointCloud2> merged_cloud_;
    std::vector<sensor_msgs::msg::LaserScan> merged_scan_;

    std::string target_frame_;
    bool publish_merged_pointcloud_;
    bool publish_merged_scan_;
    double min_range_, max_range_, voxel_size_;
    double angle_min_, angle_max_;    
    bool sync_;
    bool enable_filter_scan_;
    bool enable_filter_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiLidarMerger>());
    rclcpp::shutdown();
    return 0;
}
