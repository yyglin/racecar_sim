#pragma once

#include <cstddef>
#include <vector>

namespace fast_ground_segmenter
{
    struct Bin
    {
        bool occupied{false};
        float min_z{0.0F};
        float min_range{0.0F};
        std::size_t min_point_index{0};
        std::vector<std::size_t> point_indices;
    };

    using PolarGridData = std::vector<std::vector<Bin>>;

    struct GroundLine
    {
        float start_range{0.0F};
        float end_range{0.0F};
        float slope{0.0F};
        float intercept{0.0F};
        std::size_t num_points{0};

        float predict(float range) const
        {
            return slope * range + intercept;
        }

        bool contains(float range) const
        {
            return range >= start_range && range <= end_range;
        }

    };

    using GroundLinesBySegment = std::vector<std::vector<GroundLine>>;

    struct PointClassification
    {
        std::vector<std::size_t> ground_indices;
        std::vector<std::size_t> nonground_indices;
    };


} //namespace fast_ground_segmenter
