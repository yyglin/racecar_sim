#include "cone_detection/adaptive_clusterer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include <pcl/common/centroid.h>
#include <pcl/common/point_tests.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace cone_detection
{
    AdaptiveClusterer::AdaptiveClusterer(
        const AdaptiveClusterConfig & config
    ) : config_(config)
    {
        validateConfig(config_);
    }
    
    void AdaptiveClusterer::setConfig(const AdaptiveClusterConfig & config)
    {
        validateConfig(config);
        config_ = config;
    }

    const AdaptiveClusterConfig & AdaptiveClusterer::getConfig() const noexcept
    {
        return config_;
    }

    void AdaptiveClusterer::validateConfig(const AdaptiveClusterConfig & config)
    {
        const bool finite_values = 
            std::isfinite(config.min_range) &&
            std::isfinite(config.max_range) &&
            std::isfinite(config.range_bin_size) &&
            std::isfinite(config.base_tolerance) &&
            std::isfinite(config.tolerance_gain) &&
            std::isfinite(config.max_tolerance);
        
        if (!finite_values) {
            throw std::invalid_argument("AdaptiveClusterConfig contains non-finite values");
        }

        if (config.min_range < 0.0F) {
            throw std::invalid_argument("min_range must be greater than or equal to zero");
        }

        if (config.max_range <= config.min_range) {
            throw std::invalid_argument("max_range must be greater than min_range");
        }

        if (config.range_bin_size <= 0.0F) {
           throw std::invalid_argument("range_bin_size must be greater than zero");
        }

        if (config.base_tolerance <= 0.0F) {
            throw std::invalid_argument("base_tolerance must be greater than zero");
        }

        if (config.tolerance_gain < 0.0F) {
            throw std::invalid_argument(
            "tolerance_gain must be greater than or equal to zero");
        }

        if (config.max_tolerance < config.base_tolerance) {
            throw std::invalid_argument(
            "max_tolerance must be greater than or equal to "
            "base_tolerance");
        }

        if (config.min_cluster_size <= 0) {
            throw std::invalid_argument(
            "min_cluster_size must be greater than zero");
        }

        if (config.max_cluster_size < config.min_cluster_size) {
            throw std::invalid_argument(
            "max_cluster_size must be greater than or equal to "
            "min_cluster_size");
        }
    }

    float AdaptiveClusterer::toleranceForDistance(const float distance) const noexcept
    {
        const float adaptive_tolerance = config_.base_tolerance + config_.tolerance_gain * distance;

        return std::min(config_.max_tolerance, adaptive_tolerance);
    }

    std::vector<ClusterCandidate> AdaptiveClusterer::cluster(const CloudConsPtr & non_ground_cloud) const
    {
        std::vector<ClusterCandidate> candidates;

        if (!non_ground_cloud || non_ground_cloud->empty()){
            return candidates;
        }

        const float detection_range = config_.max_range - config_.min_range;

        const auto bin_count = static_cast<std::size_t>(std::ceil(detection_range / config_.range_bin_size));

        for (std::size_t bin_index = 0U; bin_index < bin_count; ++ bin_index)
        {
            const float core_min = config_.min_range + static_cast<float>(bin_index) * config_.range_bin_size;

            const float core_max = std::min(core_min + config_.range_bin_size, config_.max_range);

            if (core_min >= config_.max_range){
                break;
            }

            const float representative_distnace = 0.5F * (core_min + core_max);
            const float cluster_tolerance = toleranceForDistance(representative_distnace);

            const float extened_min = std::max(config_.min_range, core_min - cluster_tolerance);
            const float extened_max = std::min(config_.max_range, core_max + cluster_tolerance);

            Cloud::Ptr bin_cloud(new Cloud);
            bin_cloud->header = non_ground_cloud->header;
            bin_cloud->sensor_origin_ = non_ground_cloud->sensor_origin_;
            bin_cloud->sensor_orientation_ = non_ground_cloud->sensor_orientation_;

            bin_cloud->points.reserve(non_ground_cloud->size());

            for (const auto & point : non_ground_cloud->points){
                if (!pcl::isFinite(point)){
                    continue;
                }

                const float point_distance = std::hypot(point.x, point.y);

                if (point_distance < extened_min || point_distance > extened_max){
                    continue;
                }

                bin_cloud->points.push_back(point);
            }

            bin_cloud->width = static_cast<std::uint32_t>(bin_cloud->points.size());
            bin_cloud->height = 1U;
            bin_cloud->is_dense = true;

            if (bin_cloud->size() < static_cast<std::size_t>(config_.min_cluster_size)){
                continue;
            }

            pcl::search::KdTree<PointT>::Ptr search_tree(new pcl::search::KdTree<PointT>);

            search_tree->setInputCloud(bin_cloud);

            pcl::EuclideanClusterExtraction<PointT> extraction;
            extraction.setClusterTolerance(static_cast<double>(cluster_tolerance));
            extraction.setMinClusterSize(config_.min_cluster_size);
            extraction.setMaxClusterSize(config_.max_cluster_size);
            extraction.setSearchMethod(search_tree);
            extraction.setInputCloud(bin_cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            extraction.extract(cluster_indices);

            const bool last_bin = core_max >= config_.max_range;

            for (const auto & point_indices : cluster_indices){
                if (point_indices.indices.empty()){
                    continue;
                }

                Cloud::Ptr candidate_cloud(new Cloud);
                candidate_cloud->header = non_ground_cloud->header;
                candidate_cloud->sensor_origin_ = non_ground_cloud->sensor_origin_;
                candidate_cloud->sensor_orientation_ = non_ground_cloud->sensor_orientation_;

                candidate_cloud->points.reserve(point_indices.indices.size());

                for (const int point_index : point_indices.indices) {
                    if (point_index < 0){
                        continue;
                    }

                    const auto index = static_cast<std::size_t>(point_index);

                    if (index >= bin_cloud->points.size()){
                        continue;
                    }

                    candidate_cloud->points.push_back(bin_cloud->points[index]);
                }

                if (candidate_cloud->empty()){
                    continue;
                }

                candidate_cloud->width = static_cast<std::uint32_t>(candidate_cloud->points.size());
                candidate_cloud->height = 1U;
                candidate_cloud->is_dense = true;

                Eigen::Vector4f centroid;
                const unsigned int valid_point_count = pcl::compute3DCentroid(*candidate_cloud, centroid);

                if (valid_point_count == 0U)
                {
                    continue;
                }

                const float center_distance = std::hypot(centroid.x(), centroid.y());

                const bool center_in_core = center_distance >= core_min 
                && (center_distance < core_max || (last_bin && center_distance <= core_max));

                if (!center_in_core){
                    continue;
                }

                ClusterCandidate candidate;
                candidate.cloud = std::move(candidate_cloud);
                candidate.center_x = centroid.x();
                candidate.center_y = centroid.y();
                candidate.center_z = centroid.z();
                candidate.distance_xy = center_distance;
                candidate.point_count = candidate.cloud->size();
                candidate.cluster_tolerance = cluster_tolerance;

                candidates.push_back(std::move(candidate));
            
            }
        }

        std::sort(candidates.begin(), candidates.end(), 
                    [](const ClusterCandidate & left,
                        const ClusterCandidate & right)
                        {
                        if (left.distance_xy != right.distance_xy){
                            return left.distance_xy < right.distance_xy;
                        }

                        if (left.center_x != right.center_x){
                            return left.center_x < right.center_x;
                        }

                        return left.center_y < right.center_y;
                        });
        
        return candidates;

     }

} // namespace cone_detection
