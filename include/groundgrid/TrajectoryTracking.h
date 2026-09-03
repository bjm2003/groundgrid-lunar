#pragma once

#include "groundgrid/SkidSteerModel.h"
#include "groundgrid/TrajectoryControl.h"

#include <limits>
#include <utility>
#include <vector>

namespace groundgrid {

struct TrackingSample {
    Pose2D pose;
    double v = 0.0;
    double w = 0.0;
};

enum class MotionPhase { Hold, Forward, Reverse, RotateLeft, RotateRight };

inline const char* motionPhaseName(MotionPhase kind) {
    switch(kind) {
        case MotionPhase::Forward: return "forward";
        case MotionPhase::Reverse: return "reverse";
        case MotionPhase::RotateLeft: return "rotate_left";
        case MotionPhase::RotateRight: return "rotate_right";
        default: return "hold";
    }
}

struct TrackingParams {
    double lookahead = 0.8;
    double waypoint_arrival_distance = 0.20;
    double goal_position_tolerance = 0.25;
    double goal_yaw_tolerance = 0.174532925;
    TrajectoryControlParams control;
};

enum class TrackingStatus { Invalid, Tracking, GoalReached };

enum class TrackingFailure { None, InvalidInput, RotationAnchor, MissingCommand, InvalidFeedback, InvalidBlend };

inline const char* trackingFailureName(TrackingFailure reason) {
    switch(reason) {
        case TrackingFailure::None: return "none";
        case TrackingFailure::RotationAnchor: return "rotation_anchor_not_acquired";
        case TrackingFailure::MissingCommand: return "missing_phase_command";
        case TrackingFailure::InvalidFeedback: return "invalid_feedback";
        case TrackingFailure::InvalidBlend: return "invalid_blend";
        default: return "invalid_input";
    }
}

struct TrackingStep {
    TrackingStatus status = TrackingStatus::Invalid;
    TrackingFailure failure = TrackingFailure::InvalidInput;
    std::size_t phase_begin = 0, phase_end = 0;
    std::size_t nearest = 0, target = 0, command = 0;
    MotionPhase phase = MotionPhase::Hold;
    Pose2D phase_endpoint;
    double endpoint_distance = 0.0, arrival_distance = 0.0;
    double nearest_distance = 0.0, target_distance = 0.0;
    bool pose_capture = false;
    double planned_v = 0.0, planned_w = 0.0, feedback_w = 0.0;
    double desired_v = 0.0, desired_w = 0.0;
};

// Pure, stateful targeting + command core used by BOTH the ROS follower and the offline
// multi-step tests. A nearest-point search must not bypass a rotation or a direction cusp.
// Each phase therefore owns a bounded pose/command interval; only physically acquiring its
// endpoint and heading releases the next interval. Twists use the incoming-edge convention.
class TrajectoryTracking {
public:
    void clear() {
        samples_.clear();
        phases_.clear();
        phase_index_ = 0;
    }

    bool setTrajectory(std::vector<TrackingSample> samples, bool& changed) {
        changed = false;
        if(samples.empty()) { clear(); return false; }
        for(const auto& s : samples) {
            if(!finitePose(s.pose) || !std::isfinite(s.v) || !std::isfinite(s.w)) {
                clear();
                return false;
            }
        }
        bool same = samples.size() == samples_.size();
        if(same) {
            for(std::size_t i = 0; i < samples.size(); ++i) {
                if(distance(samples[i].pose, samples_[i].pose) > 1e-6 ||
                   std::abs(SkidSteerModel::wrap(samples[i].pose.yaw-samples_[i].pose.yaw)) > 1e-6 ||
                   (i > 0 && edgeKind(samples, i) != edgeKind(samples_, i))) {
                    same = false;
                    break;
                }
            }
        }
        samples_ = std::move(samples);
        if(!same) {
            changed = true;
            phase_index_ = 0;
            phases_.clear();
            std::size_t begin = 0;
            MotionPhase kind = MotionPhase::Hold;
            for(std::size_t i = 1; i < samples_.size(); ++i) {
                const MotionPhase next = edgeKind(samples_, i);
                if(next == MotionPhase::Hold) continue;  // duplicate boundary sample
                // Translation can share a lookahead interval. Rotation samples stay
                // separate: collapsing e.g. a planned 270-degree sweep to its final yaw
                // would let wrapped-angle feedback choose the opposite, unchecked turn.
                const bool rotating = kind == MotionPhase::RotateLeft ||
                                      kind == MotionPhase::RotateRight;
                if(kind != MotionPhase::Hold && (next != kind || rotating)) {
                    phases_.push_back({begin, i-1, kind});
                    begin = i-1;
                }
                kind = next;
            }
            phases_.push_back({begin, samples_.size()-1, kind});
        }
        return true;
    }

    const std::vector<TrackingSample>& samples() const { return samples_; }

    TrackingStep step(const Pose2D& rover, const TrackingParams& p) {
        TrackingStep result;
        if(samples_.empty() || phases_.empty() || !finitePose(rover) || !validParams(p))
            return result;
        result.failure = TrackingFailure::None;

        const auto& goal = samples_.back().pose;
        if(trajectoryEndpointReached(distance(rover, goal),
                                     SkidSteerModel::wrap(goal.yaw-rover.yaw),
                                     p.goal_position_tolerance, p.goal_yaw_tolerance)) {
            result.status = TrackingStatus::GoalReached;
            return result;
        }

        // This is the only progress update. Being geometrically closer to a future phase,
        // or merely looking ahead past it, is NOT evidence that this boundary was executed.
        while(phase_index_+1 < phases_.size()) {
            const auto& endpoint = samples_[phases_[phase_index_].end].pose;
            if(!trajectoryEndpointReached(distance(rover, endpoint),
                                          SkidSteerModel::wrap(endpoint.yaw-rover.yaw),
                                          p.waypoint_arrival_distance, p.goal_yaw_tolerance))
                break;
            ++phase_index_;
        }
        const auto& phase = phases_[phase_index_];
        result.phase_begin = phase.begin;
        result.phase_end = phase.end;
        result.phase = phase.kind;
        const bool translating = phase.kind == MotionPhase::Forward ||
                                 phase.kind == MotionPhase::Reverse;
        const double arrival = phase.end+1 == samples_.size() ? p.goal_position_tolerance
                                                             : p.waypoint_arrival_distance;
        result.nearest = phase.begin;
        result.nearest_distance = std::numeric_limits<double>::infinity();
        double nearest_yaw_error = std::numeric_limits<double>::infinity();
        for(std::size_t i = phase.begin; i <= phase.end; ++i) {
            const double d = distance(rover, samples_[i].pose);
            const double yaw_error = std::abs(SkidSteerModel::wrap(samples_[i].pose.yaw-rover.yaw));
            if(d < result.nearest_distance-1e-6 ||
               (std::abs(d-result.nearest_distance) <= 1e-6 && yaw_error < nearest_yaw_error)) {
                result.nearest = i;
                result.nearest_distance = d;
                nearest_yaw_error = yaw_error;
            }
        }

        const double endpoint_distance = distance(rover, samples_[phase.end].pose);
        result.phase_endpoint = samples_[phase.end].pose;
        result.endpoint_distance = endpoint_distance;
        result.arrival_distance = arrival;
        result.pose_capture = endpoint_distance < arrival;
        if(!translating && !result.pose_capture) {
            // No translation exists in a rotation-only phase. Do not borrow a command from
            // an already completed or future phase to make an unvalidated reconnection.
            result.failure = TrackingFailure::RotationAnchor;
            return result;
        }

        result.target = phase.end;
        if(translating && !result.pose_capture) {
            result.target = std::max(phase.begin+1, result.nearest);
            while(result.target < phase.end) {
                const double d = distance(rover, samples_[result.target].pose);
                if(d >= p.lookahead ||
                   requiresTerminalWaypointTracking(result.target+1 == phase.end,
                                                     d, p.waypoint_arrival_distance))
                    break;
                ++result.target;
            }
        }
        const auto& target = samples_[result.target].pose;
        result.target_distance = distance(rover, target);
        result.command = result.target;

        if(result.pose_capture) {
            const double yaw_error = SkidSteerModel::wrap(target.yaw-rover.yaw);
            // As with translating feedback, saturate only the final blended command.
            result.feedback_w = 1.5*yaw_error;
            if(!translating && phase.kind != MotionPhase::Hold) {
                // A rotation endpoint may carry w=0 after the planner's deceleration pass.
                // Only an incoming rotation command in THIS phase can provide feed-forward.
                if(!commandInPhase(phase, result.target, false, result.command)) {
                    result.failure = TrackingFailure::MissingCommand;
                    return result;
                }
                result.planned_w = samples_[result.command].w;
                // Once this angular sample has been passed, its old feed-forward must not
                // oppose closed-loop correction and create a non-zero steady-state error.
                if(result.planned_w*yaw_error <= 0.0) result.planned_w = 0.0;
            }
            // Translation is finished here. Its curved-motion feed-forward no longer
            // applies while holding position for yaw capture; the blend weight stays 0.5.
        } else {
            if(!commandInPhase(phase, result.target, true, result.command)) {
                result.failure = TrackingFailure::MissingCommand;
                return result;
            }
            result.planned_v = samples_[result.command].v;
            result.planned_w = samples_[result.command].w;
            const double bearing = std::atan2(target.y-rover.y, target.x-rover.x);
            const double reference_yaw = rover.yaw + (result.planned_v < 0.0 ? std::acos(-1.0) : 0.0);
            if(!geometricAngularFeedback(result.planned_v,
                                         SkidSteerModel::wrap(bearing-reference_yaw),
                                         result.target_distance, p.control.max_angular_speed,
                                         result.feedback_w)) {
                result.failure = TrackingFailure::InvalidFeedback;
                return result;
            }
        }
        if(!blendTrajectoryCommand(result.planned_v, result.planned_w, result.feedback_w,
                                   p.control, result.desired_v, result.desired_w)) {
            result.failure = TrackingFailure::InvalidBlend;
            return result;
        }
        result.status = TrackingStatus::Tracking;
        return result;
    }

private:
    struct Phase { std::size_t begin, end; MotionPhase kind; };
    std::vector<TrackingSample> samples_;
    std::vector<Phase> phases_;
    std::size_t phase_index_ = 0;

    static bool finitePose(const Pose2D& p) {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
    }
    static double distance(const Pose2D& a, const Pose2D& b) {
        return std::hypot(a.x-b.x, a.y-b.y);
    }
    static bool validParams(const TrackingParams& p) {
        return std::isfinite(p.lookahead) && p.lookahead > 0.0 &&
               std::isfinite(p.waypoint_arrival_distance) && p.waypoint_arrival_distance > 0.0 &&
               std::isfinite(p.goal_position_tolerance) && p.goal_position_tolerance > 0.0 &&
               std::isfinite(p.goal_yaw_tolerance) && p.goal_yaw_tolerance > 0.0 &&
               std::isfinite(p.control.max_linear_speed) && p.control.max_linear_speed > 0.0 &&
               std::isfinite(p.control.max_angular_speed) && p.control.max_angular_speed > 0.0 &&
               std::isfinite(p.control.angular_feedforward_weight);
    }
    static MotionPhase edgeKind(const std::vector<TrackingSample>& path, std::size_t i) {
        const auto& from = path[i-1].pose;
        const auto& to = path[i].pose;
        if(distance(from, to) < 1e-3) {
            const double dyaw = SkidSteerModel::wrap(to.yaw-from.yaw);
            if(std::abs(dyaw) < 1e-3) return MotionPhase::Hold;
            return dyaw > 0.0 ? MotionPhase::RotateLeft : MotionPhase::RotateRight;
        }
        // v=0 at rotation/reversal/end boundaries does not erase the direction of the
        // incoming edge. Infer it from geometry ONLY at those zero-speed boundaries.
        const double direction = std::abs(path[i].v) > 1e-3 ? path[i].v :
            (to.x-from.x)*std::cos(from.yaw) + (to.y-from.y)*std::sin(from.yaw);
        return direction >= 0.0 ? MotionPhase::Forward : MotionPhase::Reverse;
    }
    bool commandInPhase(const Phase& phase, std::size_t target, bool linear,
                        std::size_t& command) const {
        const auto usable = [&](std::size_t i) {
            if(i <= phase.begin || i > phase.end) return false;
            const double value = linear ? samples_[i].v : samples_[i].w;
            if(std::abs(value) <= 1e-3) return false;
            return !linear || ((value > 0.0) == (phase.kind == MotionPhase::Forward));
        };
        if(usable(target)) { command = target; return true; }
        for(std::size_t i = target; i > phase.begin+1;) {
            --i;
            if(usable(i)) { command = i; return true; }
        }
        for(std::size_t i = std::max(target+1, phase.begin+1); i <= phase.end; ++i) {
            if(usable(i)) { command = i; return true; }
        }
        // A pure rotation with a zero angular profile can still close yaw error geometrically.
        return !linear;
    }
};

} // namespace groundgrid
