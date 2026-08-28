#pragma once

#include <algorithm>
#include <cmath>

namespace groundgrid {

struct TrajectoryControlParams {
    double angular_feedforward_weight = 0.5;
    double max_linear_speed = 1.39;
    double max_angular_speed = 0.8;
};

// Blend the planner's angular feed-forward with geometric feedback while keeping the
// planner's linear speed authoritative. Returns false for an unsafe/non-finite input.
inline bool blendTrajectoryCommand(double planned_v, double planned_w, double feedback_w,
                                   const TrajectoryControlParams& params,
                                   double& desired_v, double& desired_w) {
    if(!std::isfinite(planned_v) || !std::isfinite(planned_w) ||
       !std::isfinite(feedback_w) || !std::isfinite(params.angular_feedforward_weight) ||
       !std::isfinite(params.max_linear_speed) || !std::isfinite(params.max_angular_speed) ||
       params.max_linear_speed <= 0.0 || params.max_angular_speed <= 0.0) {
        return false;
    }
    const double weight = std::clamp(params.angular_feedforward_weight, 0.0, 1.0);
    desired_v = std::clamp(planned_v, -params.max_linear_speed, params.max_linear_speed);
    desired_w = std::clamp(weight*planned_w + (1.0-weight)*feedback_w,
                           -params.max_angular_speed, params.max_angular_speed);
    return true;
}

} // namespace groundgrid
