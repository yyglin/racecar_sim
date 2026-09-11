#include "fast_ground_segmenter/ground_segmenter_node.hpp"

#include <functional>
#include <cstdint>

#include <pcl_conversions/pcl_conversions.h>

namespace fast_ground_segmenter{
    GroundSegmenterNode::GroundSegmenterNode() : Node("ground_segmenter_node"){
        const auto input_topic = this->declare_parameter<std::string>(
            "input_topic", "/point");
        
        const float min_range = static_cast<float>(this->declare_parameter<double>("min_range", 1.0));
        const float max_range = static_cast<float>(this->declare_parameter<double>("max_range", 80.0));
        const float min_z = static_cast<float>(this->declare_parameter<double>("min_z", -3.0));
        const float max_z =static_cast<float>(this->declare_parameter<double>("max_z", 2.0));
        
        FilterConfig filter_config;
        filter_config.min_range = min_range;
        filter_config.max_range = max_range;
        filter_config.min_z = min_z;
        filter_config.max_z = max_z;
        
        filter_ = std::make_unique<PointCloudFilter>(filter_config);

        PolarGridConfig grid_config;
        grid_config.min_range = min_range;
        grid_config.max_range = max_range;
        grid_config.num_segments = this->declare_parameter<int>("num_segments", 360);
        grid_config.num_bins = this->declare_parameter<int>("num_bins", 120);

        polar_grid_ =std::make_unique<PolarGrid>(grid_config);

        GroundLineFitterConfig fitter_config;

        fitter_config.max_slope_deg = static_cast<float>(this->declare_parameter<double>("max_slope_deg", 10.0));
        fitter_config.max_fit_error = static_cast<float>(this->declare_parameter<double>("max_fit_error", 0.15));
        fitter_config.max_start_height_error = static_cast<float>(this->declare_parameter<double>("max_start_height_error", 0.30));
        fitter_config.long_threshold = static_cast<float>(this->declare_parameter<double>("long_threshold", 2.0));
        fitter_config.max_long_height_error = static_cast<float>(this->declare_parameter<double>("max_long_height_error", 0.10));
        fitter_config.max_point_to_line_distance = static_cast<float>(this->declare_parameter<double>("max_point_to_line_distance", 0.20));

        ground_line_fitter_ = std::make_unique<GroundLineFitter>(fitter_config);

        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic,
            rclcpp::SensorDataQoS(),
            std::bind(
                &GroundSegmenterNode::pointCloudCallback,
                this,
                std::placeholders::_1)
            );
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/debug/filtered_points",
            rclcpp::SensorDataQoS()
        );

        bin_min_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/debug/bin_min_points",
            rclcpp::SensorDataQoS()
        );

        ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_points", rclcpp::SensorDataQoS() );
        nonground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/nonground_points", rclcpp::SensorDataQoS());


        RCLCPP_INFO(
            this->get_logger(),
            "Subscribed to '%s', publishing to '/debug/filtered_points'",
            input_topic.c_str()
        );
    }

    void GroundSegmenterNode::pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr message ){
            const auto point_count = 
               static_cast<std::size_t>(message->width) * static_cast<std::size_t>(message->height);
        
        pcl::PointCloud<pcl::PointXYZI> pcl_input;
        pcl::fromROSMsg(*message, pcl_input);
        
        const auto pcl_filtered = filter_->filter(pcl_input);

        polar_grid_->build(pcl_filtered);

        ground_lines_ = ground_line_fitter_->fit(polar_grid_->bins());

        const auto classification = ground_line_fitter_->classify(pcl_filtered, polar_grid_->bins(), ground_lines_);

        pcl::PointCloud<pcl::PointXYZI> ground_cloud;
        pcl::PointCloud<pcl::PointXYZI> nonground_cloud;

        ground_cloud.points.reserve(classification.ground_indices.size());
        nonground_cloud.points.reserve(classification.nonground_indices.size());

        for (const std::size_t point_index : classification.ground_indices){
            ground_cloud.points.push_back(pcl_filtered.points[point_index]);
        }

        for (const std::size_t point_index : classification.nonground_indices){
            nonground_cloud.points.push_back(pcl_filtered.points[point_index]);
        }

        ground_cloud.width = static_cast<std::uint32_t>(ground_cloud.points.size());
        ground_cloud.height = 1U;
        ground_cloud.is_dense = true;

        nonground_cloud.width = static_cast<std::uint32_t>(nonground_cloud.points.size());
        nonground_cloud.height = 1U;
        nonground_cloud.is_dense = true;

        const auto bin_min_points = polar_grid_->getBinMinimumPoints(pcl_filtered);

        sensor_msgs::msg::PointCloud2 bin_min_message;
        pcl::toROSMsg(bin_min_points, bin_min_message);
        bin_min_message.header = message->header;

        bin_min_publisher_->publish(bin_min_message);

        sensor_msgs::msg::PointCloud2 output_message;
        pcl::toROSMsg(pcl_filtered, output_message);

        output_message.header = message->header;

        publisher_->publish(output_message);

        sensor_msgs::msg::PointCloud2 ground_message;
        sensor_msgs::msg::PointCloud2 nonground_message;

        pcl::toROSMsg(nonground_cloud, nonground_message);
        pcl::toROSMsg(ground_cloud, ground_message);

        ground_message.header = message->header;
        nonground_message.header = message->header;

        ground_publisher_->publish(ground_message);
        nonground_publisher_->publish(nonground_message);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Processed cloud: input=%zu, filtered=%zu, ground=%zu, nonground=%zu, "
            "stamp=%d.%09u, frame_id='%s'",
            point_count,
            pcl_filtered.points.size(),
            ground_cloud.points.size(),
            nonground_cloud.points.size(),
            message->header.stamp.sec,
            message->header.stamp.nanosec,
            message->header.frame_id.c_str()
        );
    }
}
