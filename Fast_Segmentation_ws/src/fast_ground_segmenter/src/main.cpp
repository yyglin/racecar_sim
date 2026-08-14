#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "fast_ground_segmenter/ground_segmenter_node.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<fast_ground_segmenter::GroundSegmenterNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}