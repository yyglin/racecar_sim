#ifndef CONE_DETECTION__ADAPTIVE_CLUSTERER_HPP_
#define CONE_DETECTION__ADAPTIVE_CLUSTERER_HPP_

#include <cstddef>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cone_detection
{

    struct AdaptiveClusterConfig
    {
        float min_range{0.5F};
        float max_range{30.0F};

        float range_bin_size{5.0};

        float base_tolerance{0.08F};
        float tolerance_gain{0.008F};
        float max_tolerance{0.30F};

        int min_cluster_size{3};
        int max_cluster_size{300};
    };

    struct ClusterCandidate
    {
        using PointT = pcl::PointXYZI;
        using Cloud = pcl::PointCloud<PointT>;

        Cloud::Ptr cloud;

        float center_x{0.0F};
        float center_y{0.0F};
        float center_z{0.0F};

        float distance_xy{0.0F};

        std::size_t point_count{0U};

        float cluster_tolerance{0.0F};

    };

    class AdaptiveClusterer
    {
        public:
           using PointT = pcl::PointXYZI;
           using Cloud = pcl::PointCloud<PointT>;
           using CloudConsPtr = Cloud::ConstPtr;

           explicit AdaptiveClusterer(const AdaptiveClusterConfig & config = AdaptiveClusterConfig{});

           void setConfig(const AdaptiveClusterConfig & config);

           const AdaptiveClusterConfig & getConfig() const noexcept;

           std::vector<ClusterCandidate> cluster(const CloudConsPtr & non_ground_cloud) const;

        private:
           static void validateConfig(const AdaptiveClusterConfig & config);

           float toleranceForDistance(float distance) const noexcept;

           AdaptiveClusterConfig config_;
    
           
    };

    
}  // namespace cone_detection 

#endif  // CONE_DETECTION__ADAPTIVE_CLUSTERER_HPP_