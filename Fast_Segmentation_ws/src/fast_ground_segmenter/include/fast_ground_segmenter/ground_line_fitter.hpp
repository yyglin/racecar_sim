#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "fast_ground_segmenter/types.hpp"

namespace fast_ground_segmenter
{

    struct GroundLineFitterConfig
    {
        float max_slope_deg{10.0F};
        float max_fit_error{0.15F};
        float max_start_height_error{0.30F};
        float long_threshold{2.0F};
        float max_long_height_error{0.10F};
        float max_point_to_line_distance{0.20F};
    };

    class GroundLineFitter
    {
    public:
        explicit GroundLineFitter(const GroundLineFitterConfig & config);

        GroundLinesBySegment fit(const PolarGridData & grid) const;

        PointClassification classify(
            const pcl::PointCloud<pcl::PointXYZI> & cloud,
            const PolarGridData & grid,
            const GroundLinesBySegment & ground_lines 
        ) const;
    
    private:
       struct Candidate
       {
        float range{0.0F};
        float z{0.0F};
       };

       bool fitLine(
        const std::vector<Candidate> & candidates, GroundLine & line
       ) const;

       bool isLineValid(
        const std::vector<Candidate> & candidate, const GroundLine & line) const;

       std::vector<GroundLine> fitSegment (
        const std::vector<Bin> & bins
        ) const;

        GroundLineFitterConfig config_;
    };

} // namespace fast_ground_segmenter
