#pragma once

#include <algorithm>
#include <cmath>

namespace groundgrid {

struct TrajectoryControlParams {
    double angular_feedforward_weight = 0.5;
    double max_linear_speed = 1.39;
    double max_angular_speed = 0.8;
};

// A lookahead may advance onto this pose, but it must not skip beyond it while a
// co-located heading change is unfinished. Invalid inputs conservatively stop traversal.
inline bool requiresInPlaceRotationTracking(double segment_distance,
                                            double next_yaw_error,
                                            double yaw_tolerance) {
    if(!std::isfinite(segment_distance) || !std::isfinite(next_yaw_error) ||
       !std::isfinite(yaw_tolerance) || segment_distance < 0.0 || yaw_tolerance < 0.0) {
        return true;
    }
    return segment_distance < 1e-3 && std::abs(next_yaw_error) >= yaw_tolerance;
}

// Pure-pursuit yaw feedback after the bearing error has been expressed in the direction
// of travel (body yaw for forward motion, body yaw + pi for reverse motion). At that point
// the yaw-rate sign is independent of the sign of v: a reverse arc with positive
// direction-frame error still needs positive yaw rate. Multiplying by signed v here would
// make reverse feedback oppose the planner feed-forward and cancel it at a 50/50 blend.
inline bool geometricAngularFeedback(double planned_v, double direction_frame_error,
                                     double target_distance, double max_angular_speed,
                                     double& feedback_w) {
    if(!std::isfinite(planned_v) || !std::isfinite(direction_frame_error) ||
       !std::isfinite(target_distance) || !std::isfinite(max_angular_speed) ||
       target_distance < 0.0 || max_angular_speed <= 0.0) {
        return false;
    }
    const double curvature = 2.0*std::sin(direction_frame_error) /
                             std::max(target_distance, 0.1);
    feedback_w = std::clamp(std::abs(planned_v)*curvature,
                            -max_angular_speed, max_angular_speed);
    return true;
}

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
