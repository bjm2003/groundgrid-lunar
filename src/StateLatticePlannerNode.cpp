#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "groundgrid/MotionPrimitiveLibrary.h"

namespace groundgrid {

class StateLatticePlannerNode {
    struct State { int x, y, t; };
    struct QueueNode {
        float f; int key;
        bool operator<(const QueueNode& other) const { return f > other.f; }
    };
    enum class PlannerMode { Nominal, Recovery, Aborted };
    enum class RecoveryAction { None, Relax, Rotate, BackOut, Abort };

public:
    StateLatticePlannerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
        pnh_.param("heading_bins", bins_, 16);
        pnh_.param("primitive_length", primitive_length_, 0.45);
        pnh_.param("heuristic_weight", heuristic_weight_, 1.2);
        pnh_.param("reverse_cost", reverse_cost_, 1.3);
        pnh_.param("rotation_cost", rotation_cost_, 1.5);
        pnh_.param("max_planning_time", max_planning_time_, 1.0);
        pnh_.param("max_map_age", max_map_age_, 3.0);
        pnh_.param("goal_position_tolerance", goal_tolerance_, 0.30);
        pnh_.param("footprint_length", footprint_length_, 1.8);
        pnh_.param("footprint_width", footprint_width_, 1.5);
        pnh_.param("max_longitudinal_slope_deg", max_long_slope_, 20.0);
        pnh_.param("max_lateral_slope_deg", max_lat_slope_, 15.0);
        pnh_.param<std::string>("map_frame", map_frame_, "map");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");
        pnh_.param("use_dynamics_primitives", use_dynamics_primitives_, false);
        pnh_.param<std::string>("motion_primitive_file", motion_primitive_file_, "");
        pnh_.param("terrain_speed_gain", terrain_speed_gain_, 0.6);
        pnh_.param("min_speed_scale", min_speed_scale_, 0.25);
        pnh_.param("reverse_speed_frac", reverse_speed_frac_, 0.5);
        pnh_.param("max_snap_distance", max_snap_distance_, 1.5);
        pnh_.param("goal_snap_heading_span", goal_snap_heading_span_, 2);
        pnh_.param("goal_snap_heading_weight", goal_snap_heading_weight_, 0.25);
        pnh_.param("goal_snap_cost_weight", goal_snap_cost_weight_, 0.5);
        pnh_.param("recovery_fail_threshold", recovery_fail_threshold_, 3);
        pnh_.param("no_progress_timeout", no_progress_timeout_, 6.0);
        pnh_.param("progress_epsilon", progress_epsilon_, 0.10);
        pnh_.param("recovery_step_timeout", recovery_step_timeout_, 2.0);
        pnh_.param("recovery_confirm_count", recovery_confirm_count_, 2);
        pnh_.param("min_recovery_interval", min_recovery_interval_, 3.0);
        pnh_.param("recovery_rotate_step", recovery_rotate_step_, 0.6);
        pnh_.param("recovery_backout_distance", recovery_backout_distance_, 1.0);

        // Escalation ladder, tried in order: cheapest and safest first. Relax and Abort
        // command no motion at all; Rotate turns in place; BackOut is the only rung that
        // drives the vehicle, and only over ground it has already crossed. Abort is the
        // terminal rung -- retrying a hopeless goal forever burns CPU and makes the
        // success-rate metric meaningless.
        ladder_ = { RecoveryAction::Relax, RecoveryAction::Rotate,
                    RecoveryAction::BackOut, RecoveryAction::Abort };

        // Shared dynamics envelope: the arc-mode velocity profile has to honour the same
        // limits the offline primitive library was generated under, otherwise the A/B
        // comparison would run the two modes against different vehicles.
        nh_.param("skid_steer_model/v_max", sp_.v_max, sp_.v_max);
        nh_.param("skid_steer_model/w_max", sp_.w_max, sp_.w_max);
        nh_.param("skid_steer_model/a_max", sp_.a_max, sp_.a_max);
        nh_.param("skid_steer_model/alpha_max", sp_.alpha_max, sp_.alpha_max);

        if(use_dynamics_primitives_) {
            if(motion_primitive_file_.empty() || !primitive_lib_.load(motion_primitive_file_)) {
                ROS_ERROR("Failed to load motion primitive library '%s'; falling back to arc primitives.",
                          motion_primitive_file_.c_str());
                use_dynamics_primitives_ = false;
            } else if(primitive_lib_.headingBins() != bins_) {
                ROS_ERROR("Primitive library has %d heading bins but planner expects %d; falling back to arc primitives.",
                          primitive_lib_.headingBins(), bins_);
                use_dynamics_primitives_ = false;
            } else {
                ROS_INFO("Loaded dynamics motion primitive library '%s' (%d bins).",
                         motion_primitive_file_.c_str(), primitive_lib_.headingBins());
            }
        }

        map_sub_ = nh_.subscribe("/terrain/grid_map", 1, &StateLatticePlannerNode::mapCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &StateLatticePlannerNode::goalCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/lunar_planner/path", 1, true);
        vel_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/lunar_planner/velocity_profile", 1, true);
        status_pub_ = nh_.advertise<std_msgs::String>("/lunar_planner/status", 1, true);
        snapped_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/lunar_planner/snapped_goal", 1, true);
        diag_pub_ = nh_.advertise<std_msgs::String>("/lunar_planner/diagnostics", 1, true);
        service_ = nh_.advertiseService("/lunar_planner/make_plan", &StateLatticePlannerNode::serviceCallback, this);
        timer_ = nh_.createTimer(ros::Duration(0.5), &StateLatticePlannerNode::timerCallback, this);
    }

private:
    static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
    int key(const State& s, int cols) const { return (s.x * cols + s.y) * bins_ + s.t; }
    State stateFromKey(int k, int cols) const {
        State s; s.t = k % bins_; k /= bins_; s.y = k % cols; s.x = k / cols; return s;
    }
    double yawForBin(int t) const { return t * 2.0 * M_PI / bins_; }
    int binForYaw(double yaw) const {
        int b = static_cast<int>(std::lround(wrap(yaw) * bins_ / (2.0 * M_PI)));
        b %= bins_; if(b < 0) b += bins_; return b;
    }

    void publishStatus(const std::string& text) {
        std_msgs::String msg; msg.data = text; status_pub_.publish(msg);
    }

    static const char* modeName(PlannerMode m) {
        switch(m) { case PlannerMode::Recovery: return "recovery";
                    case PlannerMode::Aborted:  return "aborted";
                    default: return "nominal"; }
    }
    static const char* actionName(RecoveryAction a) {
        switch(a) { case RecoveryAction::Relax: return "relax";
                    case RecoveryAction::Rotate: return "rotate";
                    case RecoveryAction::BackOut: return "backout";
                    case RecoveryAction::Abort: return "abort";
                    default: return "none"; }
    }

    // Structured counterpart to /lunar_planner/status, which has to stay short single-word
    // tokens for `rostopic echo` diagnosis. recovery_events/successes are monotone counters
    // rather than events, so the test harness can diff them without caring about drops --
    // that difference is the 避障恢复率 metric, and plan_ms is 规划耗时.
    void publishDiagnostics() {
        const ros::Time now = ros::Time::now();
        const bool chatty = (mode_ == PlannerMode::Recovery);
        if(!chatty && !last_diag_time_.isZero() && (now - last_diag_time_).toSec() < 1.0) return;
        last_diag_time_ = now;
        char buf[320];
        std::snprintf(buf, sizeof(buf),
                      "mode=%s action=%s escalation=%d fails=%d recovery_events=%d "
                      "recovery_successes=%d last_fail=%s plan_ms=%.1f snap_m=%.2f",
                      modeName(mode_), actionName(action_), recovery_escalation_,
                      consecutive_failures_, recovery_events_, recovery_successes_,
                      last_fail_reason_.empty() ? "none" : last_fail_reason_.c_str(),
                      last_plan_ms_, last_snap_dist_);
        std_msgs::String msg; msg.data = buf; diag_pub_.publish(msg);
    }

    void resetRecoveryState() {
        mode_ = PlannerMode::Nominal; action_ = RecoveryAction::None;
        recovery_escalation_ = 0; consecutive_failures_ = 0; confirm_count_ = 0;
        best_goal_dist_ = std::numeric_limits<double>::infinity();
        last_progress_time_ = ros::Time::now();
        last_valid_path_.poses.clear();
    }

    void mapCallback(const grid_map_msgs::GridMapConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        grid_map::GridMapRosConverter::fromMessage(*msg, map_);
        map_stamp_ = msg->info.header.stamp;
        have_map_ = map_.exists("terrain_cost") && map_.exists("slope_x") && map_.exists("slope_y");
        if(have_map_) { buildTraversabilityCache(); replan_requested_ = true; }
    }

    // Flatten the per-cell traversability inputs into contiguous arrays so the footprint
    // check reads plain floats instead of doing a string-keyed grid_map layer lookup on
    // every one of its ~143 samples. slopemag is the yaw-independent slope magnitude used
    // by the gentle-slope fast path in footprintValid; gx/gy are kept for the rare steep
    // cells that still need the direction-dependent longitudinal/lateral check. Indexing
    // matches grid_map: lin = idx(0)*cols + idx(1), where idx comes from map_.getIndex().
    void buildTraversabilityCache() {
        const int rows = map_.getSize()(0), cols = map_.getSize()(1);
        cell_cols_ = cols;
        const size_t n = static_cast<size_t>(rows) * cols;
        const float nan = std::numeric_limits<float>::quiet_NaN();
        cell_cost_.assign(n, nan);
        cell_gx_.assign(n, nan);
        cell_gy_.assign(n, nan);
        cell_slopemag_.assign(n, nan);
        const auto& cost = map_["terrain_cost"];
        const auto& sx = map_["slope_x"];
        const auto& sy = map_["slope_y"];
        for(int i = 0; i < rows; ++i) {
            for(int j = 0; j < cols; ++j) {
                const size_t lin = static_cast<size_t>(i) * cols + j;
                cell_cost_[lin] = cost(i, j);
                const float gx = sx(i, j), gy = sy(i, j);
                cell_gx_[lin] = gx; cell_gy_[lin] = gy;
                if(std::isfinite(gx) && std::isfinite(gy))
                    cell_slopemag_[lin] = static_cast<float>(std::atan(std::hypot(gx, gy)) * 180.0 / M_PI);
            }
        }
    }

    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_ = *msg; have_goal_ = true; replan_requested_ = true;
        // A new goal must not inherit the previous goal's stuck state, or it would be
        // declared unreachable before it has been attempted even once.
        resetRecoveryState();
    }

    bool robotPose(geometry_msgs::PoseStamped& pose) {
        try {
            const auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0.05));
            pose.header = tf.header;
            pose.pose.position.x = tf.transform.translation.x;
            pose.pose.position.y = tf.transform.translation.y;
            pose.pose.position.z = tf.transform.translation.z;
            pose.pose.orientation = tf.transform.rotation;
            return true;
        } catch(const tf2::TransformException& e) {
            ROS_WARN_THROTTLE(1.0, "Planner cannot get robot pose: %s", e.what());
            return false;
        }
    }

    bool serviceCallback(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!have_map_) return false;
        std_msgs::Float32MultiArray vel;
        plan(req.start, req.goal, res.plan, vel);
        return true;
    }

    // Turn in place towards the goal bearing. Answers the case where the current heading
    // simply has no valid outgoing primitive but a slightly different one does, which is
    // what "stuck close to an obstacle" usually looks like on a skid-steer.
    bool recoveryRotate(const geometry_msgs::PoseStamped& start, nav_msgs::Path& path,
                        std_msgs::Float32MultiArray& vel) {
        const double x = start.pose.position.x, y = start.pose.position.y;
        const double yaw = tf2::getYaw(start.pose.orientation);
        double delta = wrap(std::atan2(goal_.pose.position.y - y, goal_.pose.position.x - x) - yaw);
        // Already pointing at the goal: probe by turning anyway, since the blockage is
        // evidently not a heading error in the obvious direction.
        if(std::abs(delta) < 1e-2) delta = recovery_rotate_step_;
        delta = std::clamp(delta, -recovery_rotate_step_, recovery_rotate_step_);
        const double target_yaw = wrap(yaw + delta);
        float cost;
        // allow_unknown for the same reason the start check uses it: the body occludes the
        // ground beneath itself, so those cells are never observed. Lethal cells and
        // over-slope cells are still rejected.
        if(!footprintValid(x, y, target_yaw, cost, true)) return false;

        path.header.frame_id = map_frame_; path.header.stamp = ros::Time::now();
        path.poses.clear();
        for(int i = 0; i < 2; ++i) {
            geometry_msgs::PoseStamped pose; pose.header = path.header;
            pose.pose.position.x = x; pose.pose.position.y = y;
            tf2::Quaternion q; q.setRPY(0, 0, i == 0 ? yaw : target_yaw);
            pose.pose.orientation = tf2::toMsg(q);
            path.poses.push_back(pose);
        }
        buildVelocityProfile(path, vel);
        return true;
    }

    // Retreat along the last path that actually planned, which by construction is ground the
    // vehicle has already traversed. Poses keep their forward orientation so the follower's
    // own reverse detection makes the rover back up rather than turn around in place.
    bool recoveryBackOut(const geometry_msgs::PoseStamped& start, nav_msgs::Path& path,
                         std_msgs::Float32MultiArray& vel) {
        if(last_valid_path_.poses.size() < 2) return false;
        const double x = start.pose.position.x, y = start.pose.position.y;
        size_t nearest = 0; double nearest_d = std::numeric_limits<double>::infinity();
        for(size_t i = 0; i < last_valid_path_.poses.size(); ++i) {
            const auto& p = last_valid_path_.poses[i].pose.position;
            const double d = std::hypot(p.x - x, p.y - y);
            if(d < nearest_d) { nearest_d = d; nearest = i; }
        }
        path.header.frame_id = map_frame_; path.header.stamp = ros::Time::now();
        path.poses.clear();
        geometry_msgs::PoseStamped first = start; first.header = path.header;
        path.poses.push_back(first);
        double travelled = 0.0;
        for(size_t i = nearest; i-- > 0;) {
            const auto& src = last_valid_path_.poses[i];
            // The map has moved on since this path was planned, and this is the only recovery
            // that puts the vehicle into space it cannot currently see -- so re-check every
            // pose strictly (no allow_unknown) and abandon the rung on the first rejection.
            float cost;
            if(!footprintValid(src.pose.position.x, src.pose.position.y,
                               tf2::getYaw(src.pose.orientation), cost)) return false;
            const auto& prev = path.poses.back().pose.position;
            travelled += std::hypot(src.pose.position.x - prev.x, src.pose.position.y - prev.y);
            geometry_msgs::PoseStamped pose = src; pose.header = path.header;
            path.poses.push_back(pose);
            if(travelled >= recovery_backout_distance_) break;
        }
        if(path.poses.size() < 2) return false;
        buildVelocityProfile(path, vel);
        return true;
    }

    // One rung of the escalation ladder. Returns whether it produced a usable path, and
    // sets `manoeuvre` when that path only repositions the vehicle instead of routing it to
    // the goal -- the caller must not count those two as the same kind of success.
    bool runRecovery(const geometry_msgs::PoseStamped& start, nav_msgs::Path& path,
                     std_msgs::Float32MultiArray& vel, bool& manoeuvre) {
        action_ = ladder_[std::min<size_t>(recovery_escalation_, ladder_.size()-1)];
        manoeuvre = false;
        switch(action_) {
            case RecoveryAction::Relax: {
                // Most failures are "the goal is barely out of reach": widen the acceptance
                // ball and the snap radius for this attempt only. The collision test itself
                // is untouched, so this trades goal precision, never clearance.
                const double saved_tol = goal_tolerance_, saved_snap = max_snap_distance_;
                goal_tolerance_ *= 2.0; max_snap_distance_ *= 2.0;
                const bool ok = plan(start, goal_, path, vel);
                goal_tolerance_ = saved_tol; max_snap_distance_ = saved_snap;
                return ok;
            }
            case RecoveryAction::Rotate:
                manoeuvre = true;
                return recoveryRotate(start, path, vel);
            case RecoveryAction::BackOut:
                manoeuvre = true;
                return recoveryBackOut(start, path, vel);
            case RecoveryAction::Abort:
                mode_ = PlannerMode::Aborted; have_goal_ = false;
                ROS_WARN("plan: giving up on this goal after %d failed attempts; send a new goal",
                         consecutive_failures_);
                return false;
            default:
                return plan(start, goal_, path, vel);
        }
    }

    void timerCallback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!have_map_) return;
        // Checked before have_goal_, which abort clears: the latched status has to keep
        // saying "aborted" so `rostopic echo /lunar_planner/status` explains the silence.
        if(mode_ == PlannerMode::Aborted) { publishStatus("aborted"); publishDiagnostics(); return; }
        if(!have_goal_) return;
        if((ros::Time::now() - map_stamp_).toSec() > max_map_age_) {
            publishStatus("stale_map"); return;
        }
        // Replanning is driven by map arrival (see mapCallback), not by this timer: at a
        // 0.5s tick against a ~1.1s perception cycle the same map was replanned twice, which
        // is what lets consecutive plans disagree and the rover oscillate.
        if(!replan_requested_) return;
        replan_requested_ = false;
        geometry_msgs::PoseStamped start;
        if(!robotPose(start)) { publishStatus("tf_unavailable"); return; }

        const double goal_dist = std::hypot(goal_.pose.position.x - start.pose.position.x,
                                            goal_.pose.position.y - start.pose.position.y);
        if(goal_dist < best_goal_dist_ - progress_epsilon_) {
            best_goal_dist_ = goal_dist; last_progress_time_ = ros::Time::now();
        }

        // Two independent triggers, because the task book names both failure modes: repeated
        // planning failure near obstacles, and making no headway while still producing paths
        // (an oscillating planner replans happily forever but never improves best_goal_dist_).
        const bool no_progress = goal_dist > goal_tolerance_ &&
            (ros::Time::now() - last_progress_time_).toSec() > no_progress_timeout_;
        if(mode_ == PlannerMode::Nominal) {
            const bool cooled_down = last_recovery_end_.isZero() ||
                (ros::Time::now() - last_recovery_end_).toSec() > min_recovery_interval_;
            if((consecutive_failures_ >= recovery_fail_threshold_ || no_progress) && cooled_down) {
                mode_ = PlannerMode::Recovery; recovery_escalation_ = 0; confirm_count_ = 0;
                recovery_step_start_ = ros::Time::now(); ++recovery_events_;
            }
        } else if((ros::Time::now() - recovery_step_start_).toSec() > recovery_step_timeout_) {
            ++recovery_escalation_; confirm_count_ = 0; recovery_step_start_ = ros::Time::now();
        }

        nav_msgs::Path path;
        std_msgs::Float32MultiArray vel;
        const bool in_recovery = (mode_ == PlannerMode::Recovery);
        bool manoeuvre = false;
        const bool ok = in_recovery ? runRecovery(start, path, vel, manoeuvre)
                                    : plan(start, goal_, path, vel);
        if(ok && manoeuvre) {
            // Rotate and back-out produce a path the follower can execute, but it does not
            // reach the goal. Counting it as a planning success would clear the failure
            // streak and satisfy the confirm counter, so the ladder would reset here and
            // never escalate to Abort -- a hopeless goal would be retried forever, and the
            // 避障恢复率 metric would score a manoeuvre as a recovery.
            last_path_ = path;
            path_pub_.publish(path); vel_pub_.publish(vel);
            publishStatus(recoveryStatus());
        } else if(ok) {
            consecutive_failures_ = 0;
            last_path_ = path;
            // Only genuine plans seed the back-out buffer: retreating along a previous
            // recovery manoeuvre would just replay the manoeuvre that already failed.
            if(!in_recovery) last_valid_path_ = path;
            path_pub_.publish(path); vel_pub_.publish(vel);
            if(in_recovery) {
                // Require repeated success before declaring recovery over, otherwise a
                // marginal situation chatters between recovery and nominal every cycle.
                if(++confirm_count_ >= recovery_confirm_count_) {
                    mode_ = PlannerMode::Nominal; action_ = RecoveryAction::None;
                    recovery_escalation_ = 0; confirm_count_ = 0;
                    last_recovery_end_ = ros::Time::now();
                    best_goal_dist_ = goal_dist; last_progress_time_ = ros::Time::now();
                    ++recovery_successes_;
                    publishStatus(snapped_goal_used_ ? "success_snapped" : "success");
                } else {
                    publishStatus(recoveryStatus());
                }
            } else {
                publishStatus(snapped_goal_used_ ? "success_snapped" : "success");
            }
        } else {
            ++consecutive_failures_; confirm_count_ = 0;
            last_path_.poses.clear(); path_pub_.publish(last_path_);
            vel_pub_.publish(std_msgs::Float32MultiArray());
            publishStatus(mode_ == PlannerMode::Aborted ? "aborted"
                          : (in_recovery ? recoveryStatus() : "no_path"));
        }
        publishDiagnostics();
    }

    std::string recoveryStatus() const { return std::string("recovery_") + actionName(action_); }

    bool poseToState(const geometry_msgs::PoseStamped& pose, State& state) const {
        if(pose.header.frame_id != map_frame_ && !pose.header.frame_id.empty()) return false;
        grid_map::Index idx;
        if(!map_.getIndex(grid_map::Position(pose.pose.position.x, pose.pose.position.y), idx)) return false;
        state.x = idx(0); state.y = idx(1);
        state.t = binForYaw(tf2::getYaw(pose.pose.orientation));
        return true;
    }

    // When allow_unknown is true, unobserved cells (NaN terrain_cost / slope, or off-map)
    // are tolerated instead of failing the check. Used only for the start footprint: the
    // vehicle is physically sitting on its current pose and its body occludes the ground
    // directly beneath it, so those cells are never observed. Genuinely lethal cells
    // (terrain_cost >= 100) and cells whose measured slope exceeds the limit are still rejected.
    // Reads the flattened traversability cache; behaviour is identical to a direct grid_map
    // scan, but cells whose slope magnitude is already within the (stricter) lateral limit
    // skip the direction-dependent slope trig entirely.
    bool footprintValid(double x, double y, double yaw, float& cost, bool allow_unknown = false) const {
        cost = 0.0f; int samples = 0;
        const double r = map_.getResolution();
        const double cyaw = std::cos(yaw), syaw = std::sin(yaw);
        for(double lx = -footprint_length_/2; lx <= footprint_length_/2 + 1e-6; lx += r) {
            for(double ly = -footprint_width_/2; ly <= footprint_width_/2 + 1e-6; ly += r) {
                const double wx = x + cyaw*lx - syaw*ly;
                const double wy = y + syaw*lx + cyaw*ly;
                grid_map::Index idx;
                if(!map_.getIndex(grid_map::Position(wx, wy), idx)) {
                    if(allow_unknown) continue;
                    return false;
                }
                const size_t lin = static_cast<size_t>(idx(0)) * cell_cols_ + idx(1);
                const float c = cell_cost_[lin];
                if(!std::isfinite(c)) {
                    if(allow_unknown) continue;
                    return false;
                }
                if(c >= 100.0f) return false;
                const float sm = cell_slopemag_[lin];
                if(!std::isfinite(sm)) {
                    if(allow_unknown) continue;
                    return false;
                }
                if(sm > max_lat_slope_) {  // steep cell: fall back to the directional check
                    const float gx = cell_gx_[lin], gy = cell_gy_[lin];
                    const double longitudinal = std::atan(std::abs(gx*cyaw + gy*syaw)) * 180.0/M_PI;
                    const double lateral = std::atan(std::abs(-gx*syaw + gy*cyaw)) * 180.0/M_PI;
                    if(longitudinal > max_long_slope_ || lateral > max_lat_slope_) return false;
                }
                cost += c; ++samples;
            }
        }
        if(samples) cost /= samples;
        return true;
    }

    // Transform primitive samples from the start-body frame into the world, validate the
    // swept footprint along the primitive, accumulate terrain cost, and return the endpoint.
    bool primitiveValid(double px, double py, double pyaw, const MotionPrimitive& prim,
                        float& avg_cost, double& ex, double& ey, double& eyaw) const {
        const int n = static_cast<int>(prim.samples.size());
        if(n == 0) return false;
        const double c = std::cos(pyaw), s = std::sin(pyaw);
        const double r = map_.getResolution();
        const double step_len = prim.length / std::max(1, n);
        const int stride = std::max(1, static_cast<int>(std::lround(r / std::max(step_len, 1e-3))));
        float cost_sum = 0.0f; int checks = 0;
        for(int i = 0; i < n; i += stride) {
            const auto& smp = prim.samples[i];
            const double wx = px + c*smp.x - s*smp.y;
            const double wy = py + s*smp.x + c*smp.y;
            const double wyaw = wrap(pyaw + smp.yaw);
            float terrain;
            if(!footprintValid(wx, wy, wyaw, terrain)) return false;
            cost_sum += terrain; ++checks;
        }
        const auto& last = prim.samples[n-1];
        ex = px + c*last.x - s*last.y;
        ey = py + s*last.x + c*last.y;
        eyaw = wrap(pyaw + last.yaw);
        float terrain_end;
        if(!footprintValid(ex, ey, eyaw, terrain_end)) return false;
        if((n-1) % stride != 0) { cost_sum += terrain_end; ++checks; }
        avg_cost = checks ? cost_sum / checks : terrain_end;
        return true;
    }

    bool transition(const State& from, int direction, int turn, State& to, float& edge_cost) const {
        grid_map::Position p;
        if(!map_.getPosition(grid_map::Index(from.x, from.y), p)) return false;
        const double yaw0 = yawForBin(from.t);
        if(direction == 0) {
            to = from; to.t = (from.t + turn + bins_) % bins_;
            float terrain;
            if(!footprintValid(p.x(), p.y(), yawForBin(to.t), terrain)) return false;
            edge_cost = static_cast<float>(rotation_cost_ * primitive_length_ + terrain * 0.002);
            return true;
        }

        const double dyaw = turn * 2.0 * M_PI / bins_;
        const double yaw1 = yaw0 + dyaw;
        // Collision-check the swept footprint at a coarse spacing. The footprint
        // (footprint_length_) is several times longer than one primitive step, so
        // consecutive footprints overlap heavily and the already-validated parent
        // footprint covers the rear; a spacing of half a body length is safe. Add
        // samples only if the heading sweeps more than ~11 deg across the step.
        const int n = std::max({1,
                                static_cast<int>(std::ceil(primitive_length_/(footprint_length_*0.5))),
                                static_cast<int>(std::ceil(std::abs(dyaw)/0.2))});
        float terrain_sum = 0.0f;
        for(int i=1; i<=n; ++i) {
            const double q = static_cast<double>(i)/n;
            const double yaw = yaw0 + q*dyaw;
            const double d = direction * primitive_length_ * q;
            const double x = p.x() + d*std::cos(yaw0 + q*dyaw*0.5);
            const double y = p.y() + d*std::sin(yaw0 + q*dyaw*0.5);
            float terrain;
            if(!footprintValid(x, y, yaw, terrain)) return false;
            terrain_sum += terrain;
        }
        const double x1 = p.x() + direction*primitive_length_*std::cos(yaw0 + dyaw*0.5);
        const double y1 = p.y() + direction*primitive_length_*std::sin(yaw0 + dyaw*0.5);
        grid_map::Index idx;
        if(!map_.getIndex(grid_map::Position(x1,y1), idx)) return false;
        to = {idx(0), idx(1), binForYaw(yaw1)};
        const double motion_factor = direction < 0 ? reverse_cost_ : 1.0;
        edge_cost = static_cast<float>(motion_factor * primitive_length_ *
                    (1.0 + 0.01 * terrain_sum/n));
        return !(to.x == from.x && to.y == from.y && to.t == from.t);
    }

    float heuristic(const State& a, const State& b) const {
        grid_map::Position pa, pb;
        map_.getPosition(grid_map::Index(a.x,a.y), pa);
        map_.getPosition(grid_map::Index(b.x,b.y), pb);
        const float distance = static_cast<float>((pa-pb).norm());
        int dt = std::abs(a.t-b.t); dt = std::min(dt, bins_-dt);
        return distance + static_cast<float>(dt * primitive_length_ * 0.25);
    }

    // Arc mode has no offline (v, w) library, so the planner profiles its own path here:
    // the desired linear/angular velocity is a required planner output, and without it the
    // follower falls back to a curvature guess and the skid_steer_model acceleration limits
    // are never enforced. Classic forward-backward trapezoidal pass; the emitted layout is
    // identical to the dynamics branch so LunarPathFollowerNode::plannedSpeedAt accepts it.
    void buildVelocityProfile(const nav_msgs::Path& path,
                              std_msgs::Float32MultiArray& vel_profile) const {
        const size_t n = path.poses.size();
        vel_profile.data.assign(2*n, 0.0f);
        vel_profile.layout.dim.resize(2);
        vel_profile.layout.dim[0].label = "pairs";
        vel_profile.layout.dim[0].size = n;
        vel_profile.layout.dim[0].stride = 2*n;
        vel_profile.layout.dim[1].label = "vw";
        vel_profile.layout.dim[1].size = 2;
        vel_profile.layout.dim[1].stride = 2;
        if(n < 2) return;

        const size_t segs = n - 1;
        prof_ds_.resize(segs); prof_dyaw_.resize(segs); prof_kappa_.resize(segs);
        prof_dir_.resize(segs); prof_v_.resize(n); prof_w_.resize(n);
        prof_yaw_.resize(n); prof_wmag_.resize(n);

        for(size_t i=0;i<n;++i) prof_yaw_[i] = tf2::getYaw(path.poses[i].pose.orientation);
        for(size_t i=0;i<segs;++i) {
            const double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
            const double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
            prof_ds_[i] = static_cast<float>(std::hypot(dx,dy));
            prof_dyaw_[i] = static_cast<float>(wrap(prof_yaw_[i+1]-prof_yaw_[i]));
            if(prof_ds_[i] < 1e-3f) {          // in-place rotation step
                prof_dir_[i] = 0;
                prof_kappa_[i] = 0.0f;
            } else {
                prof_dir_[i] = std::cos(wrap(std::atan2(dy,dx)-prof_yaw_[i])) >= 0.0 ? 1 : -1;
                prof_kappa_[i] = prof_dyaw_[i]/prof_ds_[i];
            }
        }

        for(size_t i=0;i<n;++i) {
            const size_t s = std::min(i, segs-1);
            double lim = sp_.v_max;
            const double k = std::abs(prof_kappa_[s]);
            if(k > 1e-3) lim = std::min(lim, sp_.w_max/k);
            grid_map::Index idx;
            if(map_.getIndex(grid_map::Position(path.poses[i].pose.position.x,
                                               path.poses[i].pose.position.y), idx)) {
                const size_t lin = static_cast<size_t>(idx(0))*cell_cols_ + idx(1);
                const float c = cell_cost_[lin];
                // Non-finite is legitimate here: the start footprint sits on the cells the
                // vehicle body occludes. Skip the factor rather than poisoning the profile.
                if(std::isfinite(c))
                    lim *= std::clamp(1.0 - terrain_speed_gain_*(c/99.0), min_speed_scale_, 1.0);
                const float sm = cell_slopemag_[lin];
                if(std::isfinite(sm) && max_long_slope_ > 1e-3)
                    lim *= std::clamp(1.0 - sm/max_long_slope_, min_speed_scale_, 1.0);
            }
            if(prof_dir_[s] < 0) lim = std::min(lim, reverse_speed_frac_*sp_.v_max);
            const bool rotating = (i > 0 && prof_dir_[i-1] == 0) || (i < segs && prof_dir_[i] == 0);
            const bool reversal = (i > 0 && i < segs && prof_dir_[i-1]*prof_dir_[i] < 0);
            if(i == 0 || i == n-1 || rotating || reversal) lim = 0.0;
            prof_v_[i] = static_cast<float>(std::max(lim, 0.0));
        }

        for(size_t i=1;i<n;++i)
            prof_v_[i] = std::min(prof_v_[i], static_cast<float>(
                std::sqrt(prof_v_[i-1]*prof_v_[i-1] + 2.0*sp_.a_max*prof_ds_[i-1])));
        for(size_t i=n-1;i-->0;)
            prof_v_[i] = std::min(prof_v_[i], static_cast<float>(
                std::sqrt(prof_v_[i+1]*prof_v_[i+1] + 2.0*sp_.a_max*prof_ds_[i])));

        // Angular: curvature-implied rate where the vehicle translates, and a dedicated
        // in-place rate where it does not (otherwise the profile is a dead zero exactly
        // where the rover is supposed to be turning on the spot).
        for(size_t i=0;i<n;++i) {
            const size_t s = std::min(i, segs-1);
            double w = prof_v_[i]*prof_kappa_[s];
            if(prof_dir_[s] == 0 && std::abs(prof_dyaw_[s]) > 1e-3) {
                const double mag = std::min(sp_.w_max,
                                            std::sqrt(2.0*sp_.alpha_max*std::abs(prof_dyaw_[s])));
                w = std::copysign(mag, prof_dyaw_[s]);
            }
            prof_w_[i] = static_cast<float>(std::clamp(w, -sp_.w_max, sp_.w_max));
        }
        // Same trapezoidal pass on |w| over angular arc length, so alpha_max is honoured.
        for(size_t i=0;i<n;++i) prof_wmag_[i] = std::abs(prof_w_[i]);
        for(size_t i=1;i<n;++i)
            prof_wmag_[i] = std::min(prof_wmag_[i], static_cast<float>(
                std::sqrt(prof_wmag_[i-1]*prof_wmag_[i-1] + 2.0*sp_.alpha_max*std::abs(prof_dyaw_[i-1]))));
        for(size_t i=n-1;i-->0;)
            prof_wmag_[i] = std::min(prof_wmag_[i], static_cast<float>(
                std::sqrt(prof_wmag_[i+1]*prof_wmag_[i+1] + 2.0*sp_.alpha_max*std::abs(prof_dyaw_[i]))));

        float v_peak = 0.0f;
        for(size_t i=0;i<n;++i) {
            const size_t s = std::min(i, segs-1);
            const float signed_v = prof_dir_[s] < 0 ? -prof_v_[i] : prof_v_[i];
            vel_profile.data[2*i]   = signed_v;
            vel_profile.data[2*i+1] = std::copysign(prof_wmag_[i], prof_w_[i]);
            v_peak = std::max(v_peak, prof_v_[i]);
        }
        ROS_INFO_THROTTLE(2.0, "vprofile: n=%zu v_peak=%.2f", n, v_peak);
    }

    // Nudge an unreachable goal onto the nearest pose whose footprint actually validates.
    // 要点13 is about planning close to obstacles: the strict footprint test rejects a goal
    // whose 1.8x1.5m box clips a single unobserved or lethal cell, which happens whenever the
    // operator clicks within about one body half-diagonal (hypot(0.9,0.75)=1.17m) of the edge
    // of the observed region -- even though a pose a few decimetres away is perfectly drivable.
    // This moves the goal; it does NOT relax the collision test. Rings are ordered by distance,
    // so the first ring containing any valid candidate is the best one and the search stops there.
    bool snapGoal(const State& requested, double max_distance, double budget_s,
                  const std::chrono::steady_clock::time_point& begin,
                  State& snapped, double& snap_dist) const {
        grid_map::Position rp;
        if(!map_.getPosition(grid_map::Index(requested.x, requested.y), rp)) return false;
        const double res = map_.getResolution();
        const int max_ring = std::max(1, static_cast<int>(std::ceil(max_distance/res)));
        const double heading_step = 2.0*M_PI/bins_;

        for(int r = 1; r <= max_ring; ++r) {
            if(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count() > budget_s)
                return false;
            double best_score = std::numeric_limits<double>::infinity();
            bool found = false;
            for(int dx = -r; dx <= r; ++dx) {
                for(int dy = -r; dy <= r; ++dy) {
                    if(std::max(std::abs(dx), std::abs(dy)) != r) continue;   // perimeter only
                    // Stepped in world space, not index space: grid_map is a circular buffer,
                    // so index arithmetic wraps to the wrong cell at the buffer seam.
                    grid_map::Index idx;
                    if(!map_.getIndex(grid_map::Position(rp.x()+dx*res, rp.y()+dy*res), idx)) continue;
                    grid_map::Position cp;
                    if(!map_.getPosition(idx, cp)) continue;
                    const double dist = (cp - rp).norm();
                    if(dist > max_distance) continue;
                    for(int db = -goal_snap_heading_span_; db <= goal_snap_heading_span_; ++db) {
                        const int t = ((requested.t + db) % bins_ + bins_) % bins_;
                        float cost;
                        if(!footprintValid(cp.x(), cp.y(), yawForBin(t), cost)) continue;
                        const double score = dist
                            + goal_snap_heading_weight_*std::abs(db)*heading_step*primitive_length_
                            + goal_snap_cost_weight_*(cost/99.0);
                        if(score < best_score) {
                            best_score = score; found = true;
                            snapped = {idx(0), idx(1), t}; snap_dist = dist;
                        }
                    }
                }
            }
            if(found) return true;
        }
        return false;
    }

    // Timed wrapper: goal snapping has to come out of max_planning_time rather than be
    // charged on top of it, and plan_ms is the 规划耗时 metric the harness reads back.
    bool plan(const geometry_msgs::PoseStamped& start_pose,
              const geometry_msgs::PoseStamped& goal_pose, nav_msgs::Path& path,
              std_msgs::Float32MultiArray& vel_profile) {
        const auto begin = std::chrono::steady_clock::now();
        last_fail_reason_.clear();
        const bool ok = planImpl(start_pose, goal_pose, path, vel_profile, begin);
        last_plan_ms_ = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
        return ok;
    }

    bool planImpl(const geometry_msgs::PoseStamped& start_pose,
                  const geometry_msgs::PoseStamped& goal_pose, nav_msgs::Path& path,
                  std_msgs::Float32MultiArray& vel_profile,
                  const std::chrono::steady_clock::time_point& begin) {
        path.poses.clear(); vel_profile.data.clear();
        snapped_goal_used_ = false; last_snap_dist_ = 0.0;
        State start, goal;
        if(!poseToState(start_pose,start)) {
            ROS_WARN_THROTTLE(1.0, "plan: start pose not in map (frame='%s', x=%.2f y=%.2f)",
                              start_pose.header.frame_id.c_str(),
                              start_pose.pose.position.x, start_pose.pose.position.y);
            last_fail_reason_ = "start_off_map";
            return false;
        }
        if(!poseToState(goal_pose,goal)) {
            ROS_WARN_THROTTLE(1.0, "plan: goal pose not in map (frame='%s', x=%.2f y=%.2f)",
                              goal_pose.header.frame_id.c_str(),
                              goal_pose.pose.position.x, goal_pose.pose.position.y);
            last_fail_reason_ = "goal_off_map";
            return false;
        }
        float dummy;
        grid_map::Position sp, gp;
        map_.getPosition(grid_map::Index(start.x,start.y),sp);
        map_.getPosition(grid_map::Index(goal.x,goal.y),gp);
        if(!footprintValid(sp.x(),sp.y(),yawForBin(start.t),dummy,/*allow_unknown=*/true)) {
            ROS_WARN_THROTTLE(1.0, "plan: START footprint invalid at (%.2f,%.2f) yaw=%.2f "
                              "(a LETHAL cell or over-limit slope lies under the vehicle; "
                              "unobserved cells are tolerated at the start)",
                              sp.x(), sp.y(), yawForBin(start.t));
            last_fail_reason_ = "start_footprint";
            return false;
        }
        if(!footprintValid(gp.x(),gp.y(),yawForBin(goal.t),dummy)) {
            State snapped; double snap_dist;
            if(!snapGoal(goal, max_snap_distance_, max_planning_time_*0.2, begin, snapped, snap_dist)) {
                ROS_WARN_THROTTLE(1.0, "plan: GOAL footprint invalid at (%.2f,%.2f) yaw=%.2f and no "
                                  "valid pose within %.2fm (pick a goal on clear, observed terrain)",
                                  gp.x(), gp.y(), yawForBin(goal.t), max_snap_distance_);
                last_fail_reason_ = "goal_invalid";
                return false;
            }
            goal = snapped;
            map_.getPosition(grid_map::Index(goal.x,goal.y),gp);
            snapped_goal_used_ = true; last_snap_dist_ = snap_dist;
            geometry_msgs::PoseStamped snapped_msg;
            snapped_msg.header.frame_id = map_frame_;
            snapped_msg.header.stamp = ros::Time::now();
            snapped_msg.pose.position.x = gp.x(); snapped_msg.pose.position.y = gp.y();
            tf2::Quaternion sq; sq.setRPY(0,0,yawForBin(goal.t));
            snapped_msg.pose.orientation = tf2::toMsg(sq);
            snapped_goal_pub_.publish(snapped_msg);
            ROS_WARN_THROTTLE(1.0, "plan: goal snapped %.2fm to (%.2f,%.2f) yaw=%.2f",
                              snap_dist, gp.x(), gp.y(), yawForBin(goal.t));
        }

        const bool use_dynamics = use_dynamics_primitives_ && !primitive_lib_.empty();
        const int rows=map_.getSize()(0), cols=map_.getSize()(1), count=rows*cols*bins_;
        std::vector<float> g(count,std::numeric_limits<float>::infinity());
        std::vector<int> parent(count,-1);
        std::vector<int> parent_prim(count,-1);
        std::vector<uint8_t> closed(count,0);
        std::priority_queue<QueueNode> open;
        const int sk=key(start,cols);
        g[sk]=0.0f; open.push({static_cast<float>(heuristic_weight_)*heuristic(start,goal),sk});
        int reached=-1;
        int expanded=0;
        while(!open.empty()) {
            if(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>max_planning_time_) break;
            const int ck=open.top().key; open.pop();
            if(closed[ck]) continue;
            closed[ck]=1; ++expanded;
            const State cur=stateFromKey(ck,cols);
            if(cur.t==goal.t && heuristic(cur,goal)<=goal_tolerance_) { reached=ck; break; }
            if(use_dynamics) {
                grid_map::Position cp;
                if(!map_.getPosition(grid_map::Index(cur.x,cur.y),cp)) continue;
                const double cyaw=yawForBin(cur.t);
                const auto& prims=primitive_lib_.primitivesFor(cur.t);
                for(size_t pi=0; pi<prims.size(); ++pi) {
                    const MotionPrimitive& prim=prims[pi];
                    float terrain; double ex,ey,eyaw;
                    if(!primitiveValid(cp.x(),cp.y(),cyaw,prim,terrain,ex,ey,eyaw)) continue;
                    grid_map::Index idx;
                    if(!map_.getIndex(grid_map::Position(ex,ey),idx)) continue;
                    State next{idx(0),idx(1),prim.end_bin};
                    if(next.x==cur.x && next.y==cur.y && next.t==cur.t) continue;
                    const int nk=key(next,cols); if(closed[nk]) continue;
                    const float ec=static_cast<float>(prim.base_cost*(1.0+0.01*terrain));
                    const float ng=g[ck]+ec;
                    if(ng<g[nk]) { g[nk]=ng; parent[nk]=ck; parent_prim[nk]=static_cast<int>(pi);
                        open.push({ng+static_cast<float>(heuristic_weight_)*heuristic(next,goal),nk}); }
                }
            } else {
                for(int direction : {-1,1}) for(int turn=-1;turn<=1;++turn) {
                    State next; float ec;
                    if(!transition(cur,direction,turn,next,ec)) continue;
                    const int nk=key(next,cols); if(closed[nk]) continue;
                    const float ng=g[ck]+ec;
                    if(ng<g[nk]) { g[nk]=ng; parent[nk]=ck; open.push({ng+static_cast<float>(heuristic_weight_)*heuristic(next,goal),nk}); }
                }
                for(int turn : {-1,1}) {
                    State next; float ec;
                    if(!transition(cur,0,turn,next,ec)) continue;
                    const int nk=key(next,cols); if(g[ck]+ec<g[nk]) { g[nk]=g[ck]+ec; parent[nk]=ck; open.push({g[nk]+static_cast<float>(heuristic_weight_)*heuristic(next,goal),nk}); }
                }
            }
        }
        if(reached<0) {
            const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
            ROS_WARN_THROTTLE(1.0, "plan: search exhausted without reaching goal "
                              "(mode=%s, expanded=%d nodes, %.3fs, goal_bin=%d, goal_tol=%.2f). "
                              "Start footprints ok, so this is search/goal-tolerance, not the start.",
                              use_dynamics?"dynamics":"arcs", expanded, elapsed, goal.t, goal_tolerance_);
            last_fail_reason_ = "search_exhausted";
            return false;
        }
        path.header.frame_id=map_frame_; path.header.stamp=ros::Time::now();

        if(use_dynamics) {
            std::vector<int> node_keys;
            for(int k=reached;k>=0;k=parent[k]) { node_keys.push_back(k); if(k==sk) break; }
            std::reverse(node_keys.begin(),node_keys.end());
            const State s0=stateFromKey(sk,cols);
            grid_map::Position p0; map_.getPosition(grid_map::Index(s0.x,s0.y),p0);
            geometry_msgs::PoseStamped pose0; pose0.header=path.header;
            pose0.pose.position.x=p0.x(); pose0.pose.position.y=p0.y();
            tf2::Quaternion q0; q0.setRPY(0,0,yawForBin(s0.t)); pose0.pose.orientation=tf2::toMsg(q0);
            path.poses.push_back(pose0);
            vel_profile.data.push_back(0.0f); vel_profile.data.push_back(0.0f);
            for(size_t e=1;e<node_keys.size();++e) {
                const int childK=node_keys[e], parentK=node_keys[e-1];
                const State ps=stateFromKey(parentK,cols);
                grid_map::Position pp; map_.getPosition(grid_map::Index(ps.x,ps.y),pp);
                const double pyaw=yawForBin(ps.t);
                const auto& prims=primitive_lib_.primitivesFor(ps.t);
                const int pi=parent_prim[childK];
                if(pi<0 || pi>=static_cast<int>(prims.size())) continue;
                const MotionPrimitive& prim=prims[pi];
                const double c=std::cos(pyaw), s=std::sin(pyaw);
                for(size_t i=0;i<prim.samples.size();++i) {
                    const auto& smp=prim.samples[i];
                    const double wx=pp.x()+c*smp.x - s*smp.y;
                    const double wy=pp.y()+s*smp.x + c*smp.y;
                    const double wyaw=wrap(pyaw+smp.yaw);
                    geometry_msgs::PoseStamped pose; pose.header=path.header;
                    pose.pose.position.x=wx; pose.pose.position.y=wy;
                    tf2::Quaternion q; q.setRPY(0,0,wyaw); pose.pose.orientation=tf2::toMsg(q);
                    path.poses.push_back(pose);
                    vel_profile.data.push_back(static_cast<float>(prim.v_profile[i]));
                    vel_profile.data.push_back(static_cast<float>(prim.w_profile[i]));
                }
            }
            vel_profile.layout.dim.resize(2);
            vel_profile.layout.dim[0].label="pairs"; vel_profile.layout.dim[0].size=path.poses.size(); vel_profile.layout.dim[0].stride=path.poses.size()*2;
            vel_profile.layout.dim[1].label="vw"; vel_profile.layout.dim[1].size=2; vel_profile.layout.dim[1].stride=2;
            return !path.poses.empty();
        }

        std::vector<State> states;
        for(int k=reached;k>=0;k=parent[k]) { states.push_back(stateFromKey(k,cols)); if(k==sk) break; }
        std::reverse(states.begin(),states.end());
        for(const auto& s:states) {
            grid_map::Position p; map_.getPosition(grid_map::Index(s.x,s.y),p);
            geometry_msgs::PoseStamped pose; pose.header=path.header;
            pose.pose.position.x=p.x(); pose.pose.position.y=p.y();
            tf2::Quaternion q; q.setRPY(0,0,yawForBin(s.t)); pose.pose.orientation=tf2::toMsg(q);
            path.poses.push_back(pose);
        }
        if(path.poses.empty()) return false;
        buildVelocityProfile(path, vel_profile);
        return true;
    }

    ros::NodeHandle nh_,pnh_; ros::Subscriber map_sub_,goal_sub_; ros::Publisher path_pub_,vel_pub_,status_pub_,snapped_goal_pub_,diag_pub_;
    ros::ServiceServer service_; ros::Timer timer_; tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;
    std::mutex mutex_; grid_map::GridMap map_; ros::Time map_stamp_; geometry_msgs::PoseStamped goal_; nav_msgs::Path last_path_;
    bool have_map_=false,have_goal_=false,replan_requested_=false;
    int bins_; double primitive_length_,heuristic_weight_,reverse_cost_,rotation_cost_,max_planning_time_,max_map_age_,goal_tolerance_;
    double footprint_length_,footprint_width_,max_long_slope_,max_lat_slope_;
    std::string map_frame_,base_frame_;
    bool use_dynamics_primitives_=false; std::string motion_primitive_file_;
    MotionPrimitiveLibrary primitive_lib_;
    std::vector<float> cell_cost_, cell_gx_, cell_gy_, cell_slopemag_; int cell_cols_=0;
    SkidSteerParams sp_;
    double terrain_speed_gain_, min_speed_scale_, reverse_speed_frac_;
    double max_snap_distance_, goal_snap_heading_weight_, goal_snap_cost_weight_;
    int goal_snap_heading_span_;
    bool snapped_goal_used_=false; double last_snap_dist_=0.0;

    PlannerMode mode_ = PlannerMode::Nominal;
    RecoveryAction action_ = RecoveryAction::None;
    std::vector<RecoveryAction> ladder_;
    int recovery_fail_threshold_, recovery_confirm_count_;
    double no_progress_timeout_, progress_epsilon_, recovery_step_timeout_, min_recovery_interval_;
    double recovery_rotate_step_, recovery_backout_distance_;
    int consecutive_failures_=0, recovery_escalation_=0, confirm_count_=0;
    int recovery_events_=0, recovery_successes_=0;
    double best_goal_dist_ = std::numeric_limits<double>::infinity();
    double last_plan_ms_ = 0.0;
    ros::Time last_progress_time_, recovery_step_start_, last_recovery_end_, last_diag_time_;
    nav_msgs::Path last_valid_path_;
    std::string last_fail_reason_;
    mutable std::vector<float> prof_ds_, prof_dyaw_, prof_kappa_, prof_v_, prof_w_, prof_wmag_;
    mutable std::vector<double> prof_yaw_;
    mutable std::vector<int> prof_dir_;
};

} // namespace groundgrid

int main(int argc,char** argv){ ros::init(argc,argv,"state_lattice_planner"); groundgrid::StateLatticePlannerNode n; ros::spin(); return 0; }
