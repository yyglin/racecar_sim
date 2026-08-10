#pragma once

#include <cstddef>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "fast_ground_segmenter/types.hpp"

namespace fast_ground_segmenter
{
    struct PolarGridConfig
    {
        float min_range{1.0F};
        float max_range{80.0F};
        int num_segments{360};
        int num_bins{120};
    };


    class PolarGrid
    {
        public:
          explicit PolarGrid(const PolarGridConfig & config);

          void build(const pcl::PointCloud<pcl::PointXYZI> & cloud);

          pcl::PointCloud<pcl::PointXYZI> getBinMinimumPoints(
            const pcl::PointCloud<pcl::PointXYZI> & cloud
          ) const;

          const PolarGridData & bins() const;

        private:
          void reset();

          PolarGridConfig config_;
          PolarGridData grid_;
    };
}// namespace fast_ground_segmenter
