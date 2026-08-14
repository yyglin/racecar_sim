#include <cmath>
#include <vector>
#include <cstdint>

#include <pcl/filters/filter.h>

#include "fast_ground_segmenter/point_cloud_filter.hpp"

namespace fast_ground_segmenter{
    PointCloudFilter::PointCloudFilter(const FilterConfig & config) : config_(config){
    }

    pcl::PointCloud<pcl::PointXYZI> PointCloudFilter::filter(const pcl::PointCloud<pcl::PointXYZI> & input) const
    {
        pcl::PointCloud<pcl::PointXYZI> valid_points;
        std::vector<int> valid_indices;

        pcl::removeNaNFromPointCloud(input, valid_points, valid_indices);

        pcl::PointCloud<pcl::PointXYZI> output;
        output.points.reserve(valid_points.points.size());

        for (const auto & point : valid_points.points) {
            const float range = std::hypot(point.x, point.y);

            const bool in_range = range >= config_.min_range && range <= config_.max_range;
            const bool in_height = point.z >= config_.min_z && point.z <= config_.max_z;

            if (in_range && in_height){
                output.points.push_back(point);
            }
        }

        output.width = static_cast<std::uint32_t>(output.points.size());
        output.height = 1U;
        output.is_dense = true;

        return output;
    }
}