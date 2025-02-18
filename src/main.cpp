#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <unordered_map>
#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
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
        auto param_callback = [this](const rclcpp::Parameter &p) {
            const std::string &param_name = p.get_name();

            if (param_name == "enabled_lidars") {
                enabled_lidars_ = p.as_bool_array();
                RCLCPP_INFO(this->get_logger(), "Updated enabled_lidars");
            } else if (param_name == "min_ranges") {
                min_ranges_ = p.as_double_array();
                RCLCPP_INFO(this->get_logger(), "Updated min_ranges");
            } else if (param_name == "max_ranges") {
                max_ranges_ = p.as_double_array();
                RCLCPP_INFO(this->get_logger(), "Updated max_ranges");
            } else if (param_name == "min_angles") {
                min_angles_ = p.as_double_array();
                RCLCPP_INFO(this->get_logger(), "Updated min_angles");
            } else if (param_name == "max_angles") {
                max_angles_ = p.as_double_array();
                RCLCPP_INFO(this->get_logger(), "Updated max_angles");
            } else if (param_name == "voxel_size") {
                voxel_size_ = p.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated voxel_size");
            } else if (param_name == "sync") {
                sync_ = p.as_bool();
                RCLCPP_INFO(this->get_logger(), "Updated sync");
            } else if (param_name == "cropbox_x") {
                min_cropbox_point_.x() = -float(p.as_double());
                max_cropbox_point_.x() = float(p.as_double());
                RCLCPP_INFO(this->get_logger(), "Updated cropbox_x");
            } else if (param_name == "cropbox_y") {
                min_cropbox_point_.y() = -float(p.as_double());
                max_cropbox_point_.y() = float(p.as_double());
                RCLCPP_INFO(this->get_logger(), "Updated cropbox_y");      
            } else if (param_name == "cropbox_z") {
                min_cropbox_point_.z() = -float(p.as_double());
                max_cropbox_point_.z() = float(p.as_double());
                RCLCPP_INFO(this->get_logger(), "Updated cropbox_z");                       
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown parameter updated: %s", param_name.c_str());
            }
        };

        // Register parameter callbacks and store handles
        parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback("enabled_lidars", param_callback));
        parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback("min_ranges", param_callback));
        parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback("max_ranges", param_callback));
        parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback("min_angles", param_callback));
        parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback("max_angles", param_callback));
        parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback("voxel_size", param_callback));
        parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback("sync", param_callback));
        parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback("cropbox_x", param_callback));
        parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback("cropbox_y", param_callback));
        parameter_callback_handles_.push_back(parameter_event_handler_->add_parameter_callback("cropbox_z", param_callback));

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
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("publish_merged_pointcloud", true);
        this->declare_parameter("pointCloudTopic_out", "/merged_pointcloud");
        this->declare_parameter("sync", false);
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("enable_filter_cloud", false);
        
        this->declare_parameter("cropbox_x", 0.91);
        this->declare_parameter("cropbox_y", 0.82);
        this->declare_parameter("cropbox_z", 3.0);

        size_t num_lidars = lidar_topics_.size();            

        this->declare_parameter("enabled_lidars", std::vector<bool>(num_lidars, true));
        // Per Lidar filtering parameters
        this->declare_parameter("min_ranges", std::vector<double>(num_lidars, 0.1));
        this->declare_parameter("max_ranges", std::vector<double>(num_lidars, 10.0));
        this->declare_parameter("min_angles", std::vector<double>(num_lidars, -M_PI));
        this->declare_parameter("max_angles", std::vector<double>(num_lidars, M_PI));   
    }

    void load_parameters() {
        lidar_topics_ = this->get_parameter("lidar_topics").as_string_array();
        enabled_lidars_ = this->get_parameter("enabled_lidars").as_bool_array();
        target_frame_ = this->get_parameter("target_frame").as_string();
        publish_merged_pointcloud_ = this->get_parameter("publish_merged_pointcloud").as_bool();
        pointcloud_topic_out_ = this->get_parameter("pointCloudTopic_out").as_string();
        sync_ = this->get_parameter("sync").as_bool();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        enable_filter_cloud_ = this->get_parameter("enable_filter_cloud").as_bool();

        min_cropbox_point_.x() = -this->get_parameter("cropbox_x").as_double();
        min_cropbox_point_.y() = -this->get_parameter("cropbox_y").as_double();
        min_cropbox_point_.z() = -this->get_parameter("cropbox_z").as_double();
        min_cropbox_point_.w() = 1.0;

        max_cropbox_point_.x() = this->get_parameter("cropbox_x").as_double();
        max_cropbox_point_.y() = this->get_parameter("cropbox_y").as_double();
        max_cropbox_point_.z() = this->get_parameter("cropbox_z").as_double();
        max_cropbox_point_.w() = 1.0;       

        min_ranges_ = this->get_parameter("min_ranges").as_double_array();
        max_ranges_ = this->get_parameter("max_ranges").as_double_array();
        min_angles_ = this->get_parameter("min_angles").as_double_array();
        max_angles_ = this->get_parameter("max_angles").as_double_array();
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud, size_t index) {
        if (!enabled_lidars_[index]) return;

        try {
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            tf2::doTransform(*cloud, transformed_cloud, tf_buffer_.lookupTransform(
                target_frame_, cloud->header.frame_id, tf2::TimePointZero));

            if (enable_filter_cloud_) {
                filter_cloud(transformed_cloud, index);
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

    void filter_cloud(sensor_msgs::msg::PointCloud2 &cloud, size_t index) {
        // Convert ROS2 PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud, *pcl_cloud);

        // Create a new cloud to store filtered points
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        /*
        double min_range = min_ranges_[index];
        double max_range = max_ranges_[index];
        double min_angle = min_angles_[index];
        double max_angle = max_angles_[index];

        for (const auto &point : pcl_cloud->points) {
            double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            double angle = std::atan2(point.y, point.x);
            // double elevation_angle = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y)); // for filtering vertically, e.g. excluding points above or below a certain elevation

            if (range >= min_range && range <= max_range && angle >= min_angle && angle <= max_angle) {
                filtered_cloud->points.push_back(point);
            }
        }
        */

        // Apply CropBox filter to remove points within the robot's footprint
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setMin(min_cropbox_point_);
        crop_box_filter.setMax(max_cropbox_point_);
        crop_box_filter.setInputCloud(pcl_cloud);
        crop_box_filter.setNegative(true);
        crop_box_filter.filter(*filtered_cloud);        

        // Apply voxel grid filter for downsampling
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
    double voxel_size_;
    std::vector<double> min_ranges_, max_ranges_, min_angles_, max_angles_;
    bool sync_;
    bool enable_filter_cloud_;

    Eigen::Vector4f min_cropbox_point_;
    Eigen::Vector4f max_cropbox_point_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Event handler and vector for storing callback handles
    std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_;
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> parameter_callback_handles_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiLidarMerger>());
    rclcpp::shutdown();
    return 0;
}
