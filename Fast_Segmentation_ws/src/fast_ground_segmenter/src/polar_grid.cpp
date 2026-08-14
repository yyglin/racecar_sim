#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "fast_ground_segmenter/polar_grid.hpp"

namespace fast_ground_segmenter{

    namespace {
        constexpr float kPi =  3.14159265358979323846F;
    }

    PolarGrid::PolarGrid(const PolarGridConfig & config) : config_(config){
        if (config_.min_range < 0.0F){
            throw std::invalid_argument("min_range must not be negative");
        }

        if (config_.max_range <= config_.min_range){
            throw std::invalid_argument("max_range must be greater than min_range");
        }

        if (config_.num_segments <= 0 || config_.num_bins <= 0){
            throw std::invalid_argument("num_segments and num_bins must be positive");
        }
        reset();
    }

    void PolarGrid::reset()
    {
        grid_.assign(
            static_cast<std::size_t>(config_.num_segments),
            std::vector<Bin>(static_cast<std::size_t>(config_.num_bins))
        );
    }

        void PolarGrid::build(const pcl::PointCloud<pcl::PointXYZI> & cloud){
            reset();

            const float range_span = config_.max_range - config_.min_range;

            for (std::size_t point_index = 0; point_index < cloud.points.size(); ++point_index){
                const auto & point = cloud.points[point_index];

                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)){
                    continue;
                }

                const float range = std::hypot(point.x, point.y);

                if (range < config_.min_range || range > config_.max_range){
                    continue;
                }

                const float angle = std::atan2(point.y, point.x);

                int segment_index = static_cast<int>(std::floor((angle + kPi) / (2.0F * kPi) * static_cast<float>(config_.num_segments)));

                int bin_index = static_cast<int>(std::floor((range - config_.min_range) / range_span * static_cast<float>(config_.num_bins)));

                segment_index = std::clamp(segment_index, 0, config_.num_segments - 1);

                bin_index = std::clamp(bin_index, 0, config_.num_bins - 1);

                Bin & bin = grid_[static_cast<std::size_t>(segment_index)] [static_cast<std::size_t>(bin_index)];

                bin.point_indices.push_back(point_index);

                if (!bin.occupied || point.z < bin.min_z) {
                    bin.min_z = point.z;
                    bin.min_range = range;
                    bin.min_point_index = point_index;
                }

                bin.occupied = true;
        }
    }

    pcl::PointCloud<pcl::PointXYZI> PolarGrid::getBinMinimumPoints(
        const pcl::PointCloud<pcl::PointXYZI> & cloud
    ) const
    {
        pcl::PointCloud<pcl::PointXYZI> output;

        for (const auto & segment : grid_) {
            for (const auto & bin : segment) {
                if (!bin.occupied) {
                    continue;
                }

                if (bin.min_point_index >= cloud.points.size()) {
                    continue;
                }

                output.points.push_back(cloud.points[bin.min_point_index]);
            }
        }

        output.width = static_cast<std::uint32_t>(output.points.size());
        output.height = 1U;
        output.is_dense = true;

        return output;
    }

    const PolarGridData & PolarGrid::bins() const{
        return grid_;
    }
   
} // namespace fast_ground_segmenter


