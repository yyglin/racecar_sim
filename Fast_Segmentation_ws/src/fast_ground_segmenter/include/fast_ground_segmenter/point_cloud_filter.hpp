#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fast_ground_segmenter{
    struct FilterConfig{
        float min_range{1.0F};
        float max_range{80.0F};
        float min_z{-3.0f};
        float max_z{2.0f};
    };

    class PointCloudFilter{
        public:
          explicit PointCloudFilter(const FilterConfig & config);

          pcl::PointCloud<pcl::PointXYZI> filter(
            const pcl::PointCloud<pcl::PointXYZI> & input 
          ) const;

        private:
          FilterConfig config_;
    };
} // namesoace fast_ground_segmenter