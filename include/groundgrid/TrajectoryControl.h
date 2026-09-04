#pragma once

#include "groundgrid/SkidSteerModel.h"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>

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

// Keep one accepted route long enough for its ordered forward/reverse/rotation phases to
// finish. Replanning the same clear route every rolling-map frame can alternate its first
// direction and reset the follower forever. Every fresh map still validates the complete
// unexecuted sweep, and genuine no-progress still hands control to recovery.
inline bool stableTrajectoryReuseAllowed(bool active_goal_route,
                                         bool no_progress,
                                         bool nominal_mode,
                                         bool retained_path_valid) {
    return active_goal_route && !no_progress &&
           retainedTrajectoryFallbackAllowed(nominal_mode, retained_path_valid);
}

// Remaining route motion is the progress metric, rather than Euclidean goal distance.
// A valid detour is allowed to move away from the goal, and an in-place rotation still
// consumes corner motion. This is deliberately independent of ROS and map types.
template<typename PoseAt>
inline bool remainingTrajectoryMotion(const Pose2D& rover,
                                      std::size_t pose_count,
                                      double yaw_radius,
                                      PoseAt pose_at,
                                      double& remaining,
                                      std::size_t& nearest) {
    remaining = std::numeric_limits<double>::infinity();
    nearest = 0;
    if(pose_count == 0 || !std::isfinite(rover.x) || !std::isfinite(rover.y) ||
       !std::isfinite(rover.yaw) || !std::isfinite(yaw_radius) || yaw_radius < 0.0) {
        return false;
    }
    const auto motion = [yaw_radius](const Pose2D& a,const Pose2D& b) {
        return std::hypot(b.x-a.x,b.y-a.y) +
               yaw_radius*std::abs(SkidSteerModel::wrap(b.yaw-a.yaw));
    };
    double nearest_motion = std::numeric_limits<double>::infinity();
    for(std::size_t i=0; i<pose_count; ++i) {
        const Pose2D p=pose_at(i);
        if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw)) return false;
        const double d=motion(rover,p);
        if(d<nearest_motion) { nearest_motion=d; nearest=i; }
    }
    double value=nearest_motion;
    Pose2D previous=pose_at(nearest);
    for(std::size_t i=nearest+1; i<pose_count; ++i) {
        const Pose2D current=pose_at(i);
        value+=motion(previous,current);
        previous=current;
    }
    if(!std::isfinite(value)) return false;
    remaining=value;
    return true;
}

// Once the follower has acquired the actual endpoint of a retained trajectory, the planner
// must retire the operator goal as well. Otherwise the next rolling-map update can replace an
// already completed snapped route and start the rover moving again. The caller supplies a
// wrapped yaw error; this pure gate mirrors the follower's strict terminal tolerances.
inline bool trajectoryEndpointReached(double position_error,
                                      double yaw_error,
                                      double position_tolerance,
                                      double yaw_tolerance) {
    if(!std::isfinite(position_error) || !std::isfinite(yaw_error) ||
       !std::isfinite(position_tolerance) || !std::isfinite(yaw_tolerance) ||
       position_error < 0.0 || position_tolerance < 0.0 || yaw_tolerance < 0.0) {
        return false;
    }
    return position_error < position_tolerance &&
           std::abs(yaw_error) < yaw_tolerance;
}

// A follower endpoint is a mission completion only when the active trajectory was a route
// to the task goal, rather than a recovery rotate/back-out. Ordinary routes must also leave
// the rover inside the requested-goal tolerance; a deliberately snapped route is the narrow
// exception because its original goal is known to be unsafe.
inline bool missionGoalReached(bool endpoint_reached,
                               bool trajectory_reaches_goal,
                               bool trajectory_was_snapped,
                               double requested_goal_error,
                               double requested_goal_tolerance) {
    if(!endpoint_reached || !trajectory_reaches_goal ||
       !std::isfinite(requested_goal_error) ||
       !std::isfinite(requested_goal_tolerance) ||
       requested_goal_error < 0.0 || requested_goal_tolerance < 0.0) {
        return false;
    }
    return trajectory_was_snapped || requested_goal_error < requested_goal_tolerance;
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

// Once an in-place rotation pose is aligned, subsequent pure-pursuit nearest-point searches
// on the same trajectory must not fall back across that boundary. Otherwise a small heading
// change on the following translation can make the completed rotation look unfinished again
// and command the rover back around a terminal loop.
inline bool inPlaceRotationCompleted(double segment_distance,
                                     double next_yaw_error,
                                     double yaw_tolerance) {
    if(!std::isfinite(segment_distance) || !std::isfinite(next_yaw_error) ||
       !std::isfinite(yaw_tolerance) || segment_distance < 0.0 || yaw_tolerance < 0.0) {
        return false;
    }
    return segment_distance < 1e-3 && std::abs(next_yaw_error) < yaw_tolerance;
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
// Return the unsaturated feedback demand. The angular envelope is validated here but
// applied only AFTER feed-forward blending: clipping this term to w_max first would
// silently cap correction at half w_max whenever planned_w is zero.
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
    feedback_w = std::abs(planned_v)*curvature;
    return std::isfinite(feedback_w);
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
