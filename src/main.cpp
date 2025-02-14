#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
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
                auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    lidar_topics_[i], rclcpp::SensorDataQoS(),
                    [this, i](std::shared_ptr<sensor_msgs::msg::PointCloud2> msg) {
                        this->cloudCallback(msg, i);
                    });
                cloud_subscribers_.emplace_back(sub);
            }
        }
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_out_, 10);
    }

private:

    void declare_parameters() {
        this->declare_parameter("lidar_topics", std::vector<std::string>{});
        this->declare_parameter("enabled_lidars", std::vector<bool>{});
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("publish_merged_pointcloud", true);
        this->declare_parameter("pointCloudTopic_out", "/merged_pointcloud");
        this->declare_parameter("sync", false);
        this->declare_parameter("min_range", 0.1);
        this->declare_parameter("max_range", 10.0);
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("enable_filter_cloud", false);
    }

    void load_parameters() {
        lidar_topics_ = this->get_parameter("lidar_topics").as_string_array();
        enabled_lidars_ = this->get_parameter("enabled_lidars").as_bool_array();
        target_frame_ = this->get_parameter("target_frame").as_string();
        publish_merged_pointcloud_ = get_parameter("publish_merged_pointcloud").as_bool();
        pointcloud_topic_out_ = get_parameter("pointCloudTopic_out").as_string();
        sync_ = this->get_parameter("sync").as_bool();
        min_range_ = get_parameter("min_range").as_double();
        max_range_ = get_parameter("max_range").as_double();
        voxel_size_ = get_parameter("voxel_size").as_double();
        enable_filter_cloud_ = get_parameter("enable_filter_cloud").as_bool();
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud, size_t index) {
        if (!enabled_lidars_[index]) return;

        try {
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*cloud, transformed_cloud, tf_buffer_.lookupTransform(
                target_frame_, cloud->header.frame_id, tf2::TimePointZero));

            if (enable_filter_cloud_) {
                filter_cloud(transformed_cloud);
            }

            merged_cloud_.push_back(transformed_cloud);

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }

        if (publish_merged_pointcloud_ && (!sync_ || merged_cloud_.size() == cloud_subscribers_.size())) {
            publishMergedCloud();
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

    void filter_cloud(sensor_msgs::msg::PointCloud2 &cloud) {
        // Convert ROS2 PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud, *pcl_cloud);

        // Create a new cloud to store filtered points
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        for (const auto &point : pcl_cloud->points) {
            float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

            if (range >= min_range_ && range <= max_range_) {
                filtered_cloud->points.push_back(point);
            }
        }

        // Apply voxel grid filter (optional)
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        sor.filter(*filtered_cloud);

        // Convert back to ROS2 PointCloud2 message
        pcl::toROSMsg(*filtered_cloud, cloud);
    }

    std::vector<std::string> lidar_topics_;
    std::vector<bool> enabled_lidars_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subscribers_;
    std::vector<sensor_msgs::msg::PointCloud2> merged_cloud_;

    std::string target_frame_;
    std::string pointcloud_topic_out_;
    bool publish_merged_pointcloud_;
    double min_range_, max_range_, voxel_size_;
    bool sync_;
    bool enable_filter_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
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
