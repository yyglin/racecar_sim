#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "fast_ground_segmenter/ground_line_fitter.hpp"

namespace fast_ground_segmenter
{
    namespace{
        constexpr double kPi = 3.14159265358979323846;
    }

    GroundLineFitter::GroundLineFitter(const GroundLineFitterConfig & config) : config_(config)
    {
        if (config_.max_slope_deg <= 0.0F || config_.max_slope_deg >= 90.0F)
        {
            throw std::invalid_argument("max_slope_deg must be between 0 and 90");
        } 

        if (config_.max_fit_error < 0.0F || 
            config_.max_start_height_error < 0.0F || 
            config_.long_threshold < 0.0F || 
            config_.max_long_height_error < 0.0F || 
            config_.max_point_to_line_distance < 0.0F)
        {
            throw std::invalid_argument("ground line thresholds must not be negative");
        }
    }

    bool GroundLineFitter::fitLine(const std::vector<Candidate> & candidates, GroundLine & line) const
    {

        if (candidates.size() < 2U) {
            return false;
        }

        double sum_r = 0.0;
        double sum_z = 0.0;
        double sum_rr = 0.0;
        double sum_rz = 0.0;

        for (const auto & candidate : candidates){
            const double r = candidate.range;
            const double z = candidate.z;

            sum_r += r;
            sum_z += z;
            sum_rr += r * r;
            sum_rz += r * z;
        }

        const double count = static_cast<double>(candidates.size());

        const double denominator = count * sum_rr - sum_r * sum_r;

        if (std::abs(denominator) < 1.0e-9){
            return false;
        }

        const double slope = (count * sum_rz - sum_r * sum_z) / denominator;

        const double intercept = (sum_z - slope * sum_r) / count;

        line.start_range = candidates.front().range;
        line.end_range = candidates.back().range;
        line.slope = static_cast<float>(slope);
        line.intercept = static_cast<float>(intercept);
        line.num_points = candidates.size();

        return true;
    }

    bool GroundLineFitter::isLineValid(
        const std::vector<Candidate> & candidates,
        const GroundLine & line
    ) const
    {
        const double slope_deg = std::atan(std::abs(static_cast<double>(line.slope))) * 180.0 / kPi;

        if (slope_deg > config_.max_slope_deg){
            return false;
        }

        for (const auto & candidate : candidates) {
            const float predicted_z = line.predict(candidate.range);

            const float error = std::abs(candidate.z - predicted_z);

            if (error > config_.max_fit_error){
                return false;
            }
        }

        return true;

    }

    std::vector<GroundLine> GroundLineFitter::fitSegment(
        const std::vector<Bin> & bins 
    ) const
    {
        std::vector<Candidate> candidates;
        candidates.reserve(bins.size());

        for (const auto & bin : bins) {
            if (!bin.occupied){
                continue;
            }

            candidates.push_back(Candidate{bin.min_range, bin.min_z});
        }

        std::vector<GroundLine> lines;
        std::vector<Candidate> current_candidates;
        std::optional<GroundLine> previous_line;

        for (const auto & candidate : candidates){
            if (current_candidates.empty()) {
                current_candidates.push_back(candidate);
                continue;
            }

            std::vector<Candidate> trial_candidates = current_candidates;

            trial_candidates.push_back(candidate);

            GroundLine trial_line;
            bool accepted = fitLine(trial_candidates, trial_line) && isLineValid(trial_candidates, trial_line);

            const float range_gap = candidate.range - current_candidates.back().range;

            if (accepted && range_gap > config_.long_threshold){
                float predicted_z = current_candidates.back().z;
                
                if (current_candidates.size() >= 2U) {
                    GroundLine current_line;

                    if (fitLine(current_candidates, current_line)){
                        predicted_z = current_line.predict(candidate.range);
                    }
                }

                const float long_height_error = std::abs(candidate.z - predicted_z);

                if (long_height_error > config_.max_long_height_error){
                    accepted = false;
                }

            }

            if (accepted && current_candidates.size()== 1U && previous_line.has_value() ){
                const auto & start_candidate = current_candidates.front();

                const float start_error = std::abs(start_candidate.z - previous_line->predict(start_candidate.range));

                if (start_error > config_.max_start_height_error){
                    accepted = false;
                }
            }

            if (accepted) {
                current_candidates.push_back(candidate);
                continue;
            }

            if (current_candidates.size() >= 2U) {
                GroundLine completed_line;

                if (fitLine(current_candidates, completed_line) && isLineValid(current_candidates, completed_line)){
                    lines.push_back(completed_line);
                    previous_line = completed_line;
                }
            }

            current_candidates.clear();
            current_candidates.push_back(candidate);
        }

        if (current_candidates.size() >= 2U) {
            GroundLine completed_line;

            if (fitLine(current_candidates, completed_line) && isLineValid(current_candidates, completed_line))
            {
                lines.push_back(completed_line);
            }
        }

        return lines;

    }

    GroundLinesBySegment GroundLineFitter::fit(const PolarGridData & grid) const
    {
        GroundLinesBySegment result;
        result.reserve(grid.size());

        for (const auto & segment : grid) {
            result.push_back(fitSegment(segment));
        }

        return result;
    }

    PointClassification GroundLineFitter::classify(
        const pcl::PointCloud<pcl::PointXYZI> & cloud,
        const PolarGridData & grid,
        const GroundLinesBySegment & ground_lines
    ) const
    {
        if (grid.size() != ground_lines.size()) {
            throw std::invalid_argument("grid and ground_lines must have the same segment count");
        }

        std::vector<bool> is_ground(cloud.points.size(), false);

        for (std::size_t segment_index = 0; segment_index < grid.size(); ++segment_index){
            const auto & bins = grid[segment_index];
            const auto & lines = ground_lines[segment_index];

            for (const auto & bin : bins){
                if (!bin.occupied){
                    continue;
                }

                const GroundLine * matched_line = nullptr;

                for (const auto & line : lines){
                    if (line.contains(bin.min_range)) {
                        matched_line = &line;
                        break;
                    }
                }

                if (matched_line == nullptr){
                    continue;
                }

                for (const std::size_t point_index : bin.point_indices){
                    if (point_index >= cloud.points.size()){
                        continue;
                    }

                    const auto & point = cloud.points[point_index];

                    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)){
                        continue;
                    }

                    const float range = std::hypot(point.x, point.y);

                    const float predicted_ground_z = matched_line->predict(range);

                    const float height_error = std::abs(point.z - predicted_ground_z);

                    if (height_error <= config_.max_point_to_line_distance){
                        is_ground[point_index] = true;
                    }
                }
            }
        }

        PointClassification result;
        result.ground_indices.reserve(cloud.points.size());
        result.nonground_indices.reserve(cloud.points.size());

        for (std::size_t point_index = 0; point_index < cloud.points.size(); ++point_index){
            if (is_ground[point_index]){
                result.ground_indices.push_back(point_index);
            }else{
                result.nonground_indices.push_back(point_index);
            }
        }

        return result;
    }
} // namespace fast_ground_segmenter
