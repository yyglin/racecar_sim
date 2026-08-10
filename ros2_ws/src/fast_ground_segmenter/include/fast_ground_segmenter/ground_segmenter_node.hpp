#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "fast_ground_segmenter/point_cloud_filter.hpp"
#include "fast_ground_segmenter/polar_grid.hpp"
#include "fast_ground_segmenter/ground_line_fitter.hpp"

namespace fast_ground_segmenter {
    class GroundSegmenterNode : public rclcpp::Node{
        public:
         GroundSegmenterNode();
        
        private:
         void pointCloudCallback(
            const sensor_msgs::msg::PointCloud2::SharedPtr message
         );

         rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
         rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
         std::unique_ptr<PointCloudFilter> filter_;
         std::unique_ptr<PolarGrid> polar_grid_;
         rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bin_min_publisher_;
         std::unique_ptr<GroundLineFitter> ground_line_fitter_;
         GroundLinesBySegment ground_lines_;
         rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
         rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;

    };
} //namespace fast_ground_segmenter
