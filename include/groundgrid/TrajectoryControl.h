#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>

namespace groundgrid {

struct TrajectoryControlParams {
    double angular_feedforward_weight = 0.5;
    double max_linear_speed = 1.39;
    double max_angular_speed = 0.8;
};

// A failed fresh search does not invalidate a previously published route. It may remain the
// safest command available, but only in nominal mode and only after the entire untraversed
// remainder has been revalidated on the newest map. Kept pure so policy regressions can be
// checked without ROS.
inline bool retainedTrajectoryFallbackAllowed(bool nominal_mode,
                                              bool retained_path_valid) {
    return nominal_mode && retained_path_valid;
}

// Replanning remains mandatory outside the terminal region, in recovery, or whenever the
// retained path fails validation on the newest map. Invalid distances conservatively deny
// reuse. Kept pure so the safety gate can be exercised without ROS.
inline bool terminalTrajectoryReuseAllowed(double goal_distance,
                                           double terminal_replan_distance,
                                           bool nominal_mode,
                                           bool retained_path_valid) {
    return std::isfinite(goal_distance) && std::isfinite(terminal_replan_distance) &&
           goal_distance >= 0.0 && terminal_replan_distance > 0.0 &&
           retainedTrajectoryFallbackAllowed(nominal_mode, retained_path_valid) &&
           goal_distance <= terminal_replan_distance;
}

// A zero linear speed at a pose is a boundary condition, not a command to apply while
// that pose is still spatially ahead of the rover. This matters at internal
// translation/rotation junctions as well as at the final goal: lookahead can legitimately
// select the zero-speed pose before the body has arrived there. In that case retain the
// closest preceding translating sample from the same trajectory. Returns false when the
// trajectory asks the rover to reach a distant pose without any translating command.
template<typename LinearSpeedAt>
inline bool selectTrajectoryCommandIndex(std::size_t target_index,
                                         std::size_t trajectory_size,
                                         double target_distance,
                                         double arrival_distance,
                                         LinearSpeedAt linear_speed_at,
                                         std::size_t& command_index) {
    if(trajectory_size == 0 || target_index >= trajectory_size ||
       !std::isfinite(target_distance) || !std::isfinite(arrival_distance) ||
       target_distance < 0.0 || arrival_distance < 0.0) {
        return false;
    }
    command_index = target_index;
    double speed = linear_speed_at(command_index);
    if(!std::isfinite(speed)) return false;
    if(target_distance < arrival_distance || std::abs(speed) > 1e-3) return true;

    while(command_index > 0) {
        --command_index;
        speed = linear_speed_at(command_index);
        if(!std::isfinite(speed)) return false;
        if(std::abs(speed) > 1e-3) return true;
    }
    return false;
}

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

// Do not let ordinary lookahead replace the penultimate waypoint with the fixed goal while
// the rover is still far from that waypoint. With zero terminal feed-forward, doing so turns
// half-weight pure pursuit into a limit cycle whose radius is approximately the lookahead.
inline bool requiresTerminalWaypointTracking(bool next_is_goal,
                                             double current_target_distance,
                                             double position_tolerance) {
    if(!next_is_goal) return false;
    if(!std::isfinite(current_target_distance) || !std::isfinite(position_tolerance) ||
       current_target_distance < 0.0 || position_tolerance < 0.0) {
        return true;
    }
    return current_target_distance >= position_tolerance;
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
