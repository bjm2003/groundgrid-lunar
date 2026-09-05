#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
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
#include "groundgrid/LunarTrajectory.h"
#include "groundgrid/TrajectoryControl.h"
#include "groundgrid/RetainedTrajectoryCache.h"
#include "groundgrid/SweptFootprint.h"
#include "groundgrid/ReplanStopBarrier.h"
#include "groundgrid/FootprintRaster.h"
#include "groundgrid/BackoutRecovery.h"

namespace groundgrid {

class StateLatticePlannerNode {
    struct State { int x, y, t; };
    struct FootprintRejection {
        const char* reason="none";
        double sample_x=0.0, sample_y=0.0;
        int row=-1, col=-1;
    };
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
        pnh_.param<std::string>("odometry_topic", odometry_topic_, "/localization/odometry/filtered_map");
        pnh_.param("use_dynamics_primitives", use_dynamics_primitives_, false);
        pnh_.param("debug_start_rejections", debug_start_rejections_, false);
        pnh_.param<std::string>("motion_primitive_file", motion_primitive_file_, "");
        pnh_.param("terrain_speed_gain", terrain_speed_gain_, 0.6);
        pnh_.param("min_speed_scale", min_speed_scale_, 0.25);
        pnh_.param("reverse_speed_frac", reverse_speed_frac_, 0.5);
        pnh_.param("max_snap_distance", max_snap_distance_, 1.5);
        pnh_.param("goal_snap_heading_span", goal_snap_heading_span_, 2);
        pnh_.param("goal_snap_heading_weight", goal_snap_heading_weight_, 0.25);
        pnh_.param("goal_snap_cost_weight", goal_snap_cost_weight_, 0.5);
        // All trajectory poses reserve the normal tracking/quantisation margin. A snapped
        // endpoint reserves an additional band because it is selected exactly where a
        // rolling, occlusion-limited map is least stable; the boundary grew by ~0.25 m
        // between far and near observations in the mixed hard-goal diagnostic.
        pnh_.param("trajectory_clearance", trajectory_clearance_, 0.25);
        pnh_.param("goal_snap_clearance", goal_snap_clearance_, 0.50);
        trajectory_clearance_ = std::max(0.0, trajectory_clearance_);
        goal_snap_clearance_ = std::max(trajectory_clearance_, goal_snap_clearance_);
        pnh_.param("recovery_fail_threshold", recovery_fail_threshold_, 3);
        pnh_.param("no_progress_timeout", no_progress_timeout_, 6.0);
        pnh_.param("progress_epsilon", progress_epsilon_, 0.10);
        pnh_.param("recovery_step_timeout", recovery_step_timeout_, 2.0);
        pnh_.param("recovery_confirm_count", recovery_confirm_count_, 2);
        pnh_.param("min_recovery_interval", min_recovery_interval_, 3.0);
        pnh_.param("recovery_rotate_step", recovery_rotate_step_, 0.6);
        pnh_.param("recovery_backout_distance", recovery_backout_distance_, 1.0);
        pnh_.param("terminal_replan_distance", terminal_replan_distance_, 0.8);
        pnh_.param("execution_goal_position_tolerance", execution_goal_pos_tolerance_, 0.25);
        pnh_.param("execution_goal_yaw_tolerance", execution_goal_yaw_tolerance_,
                   10.0*M_PI/180.0);
        pnh_.param("requested_goal_position_tolerance",
                   requested_goal_pos_tolerance_, 0.50);
        execution_goal_pos_tolerance_ = std::max(0.0, execution_goal_pos_tolerance_);
        execution_goal_yaw_tolerance_ = std::max(0.0, execution_goal_yaw_tolerance_);
        requested_goal_pos_tolerance_ = std::max(0.0, requested_goal_pos_tolerance_);

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
        nh_.param("skid_steer_model/x_icr", sp_.x_icr, sp_.x_icr);
        nh_.param("skid_steer_model/alpha_v", sp_.alpha_v, sp_.alpha_v);
        nh_.param("skid_steer_model/alpha_w", sp_.alpha_w, sp_.alpha_w);
        nh_.param("skid_steer_model/slope_slip_gain", sp_.slope_slip_gain, sp_.slope_slip_gain);
        nh_.param("skid_steer_model/slope_grade_gain", sp_.slope_grade_gain, sp_.slope_grade_gain);

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
        // Keep observations queued through a blocking search; do not substitute the
        // planned route for actual motion. The history itself has independent bounds.
        odom_sub_ = nh_.subscribe(odometry_topic_, 200, &StateLatticePlannerNode::odometryCallback, this);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &StateLatticePlannerNode::goalCallback, this);
        follower_diag_sub_ = nh_.subscribe("/lunar_path_follower/diagnostics", 10,
                                           &StateLatticePlannerNode::followerDiagnosticCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/lunar_planner/path", 1, true);
        vel_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/lunar_planner/velocity_profile", 1, true);
        trajectory_pub_ = nh_.advertise<groundgrid::LunarTrajectory>("/lunar_planner/trajectory", 1, true);
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
        const bool changed = text != last_status_;
        last_status_ = text;
        std_msgs::String msg; msg.data = text; status_pub_.publish(msg);
        // Status, goal identity and counters travel in ONE snapshot. In particular Abort
        // must not race an older throttled back-out diagnostic in an external test.
        publishDiagnostics(changed);
    }

    // Publish one atomic trajectory for control, plus the two legacy topics retained for
    // RViz and existing metric tooling. Invalid planner output becomes an empty trajectory,
    // which makes the follower fail safe instead of pairing unrelated latched messages.
    void publishPlanOutputs(const nav_msgs::Path& path,
                            const std_msgs::Float32MultiArray& profile,
                            bool reaches_goal,
                            bool was_snapped,
                            bool retain_route = false,
                            bool preserve_backout = false) {
        path_pub_.publish(path);
        vel_pub_.publish(profile);

        groundgrid::LunarTrajectory trajectory;
        trajectory.path = path;
        // Only this nested Header carries goal identity, not the legacy Path transport's
        // sequence number. Its stamp remains the publication/freshness timestamp.
        trajectory.path.header.seq = goal_id_;
        const bool matched = profile.data.size() == path.poses.size()*2;
        bool finite = matched;
        for(const auto& stamped_pose : path.poses) {
            const auto& p = stamped_pose.pose.position;
            const auto& q = stamped_pose.pose.orientation;
            if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
               !std::isfinite(q.x) || !std::isfinite(q.y) ||
               !std::isfinite(q.z) || !std::isfinite(q.w)) {
                finite = false;
                break;
            }
        }
        if(matched) {
            trajectory.twists.resize(path.poses.size());
            for(size_t i=0; i<path.poses.size(); ++i) {
                const float v = profile.data[2*i];
                const float w = profile.data[2*i+1];
                if(!std::isfinite(v) || !std::isfinite(w) ||
                   std::abs(v) > sp_.v_max+1e-6 || std::abs(w) > sp_.w_max+1e-6) {
                    finite = false;
                    break;
                }
                trajectory.twists[i].linear.x = v;
                trajectory.twists[i].angular.z = w;
            }
        }
        if(!finite) {
            ROS_ERROR_THROTTLE(1.0, "planner produced an invalid trajectory: %zu poses, %zu velocity values",
                               path.poses.size(), profile.data.size());
            trajectory.path.poses.clear();
            trajectory.twists.clear();
        }
        active_trajectory_reaches_goal_ = finite && !path.poses.empty() && reaches_goal;
        active_trajectory_was_snapped_ = active_trajectory_reaches_goal_ && was_snapped;
        active_trajectory_map_stamp_ = finite && !path.poses.empty() ? map_stamp_ : ros::Time();
        if(!preserve_backout || !finite || path.poses.empty()) active_backout_.clear();
        // All publications pass through this handoff. Even a transient empty trajectory
        // clears follower phase progress, so an interrupted cache cannot later be replayed
        // from an arbitrary nearest point. Back-out uses separate measured odometry.
        retained_route_.published(trajectory.path, profile, was_snapped,
                                  active_trajectory_reaches_goal_ && retain_route);
        trajectory_pub_.publish(trajectory);
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

    // CPU share of one core since the previous call, and resident set size. Sampled here
    // rather than from an external monitor so the figure follows the code onto the Atlas
    // board, where the CPU budget it exists to check (<40%) actually applies. Returns
    // false if /proc is unavailable, and the harness then drops the sample rather than
    // recording a zero -- a fabricated zero would silently pass the budget assertion.
    bool sampleResources(double& cpu_pct, double& rss_mb) {
        std::FILE* f = std::fopen("/proc/self/stat", "r");
        if(!f) return false;
        char line[2048];
        const bool read_ok = std::fgets(line, sizeof(line), f) != nullptr;
        std::fclose(f);
        // The comm field is parenthesised and may itself contain spaces, so fields are
        // only unambiguous after the last ')'. utime/stime are fields 14/15 counting
        // from 1, i.e. the 12th and 13th tokens after that point.
        const char* rest = read_ok ? std::strrchr(line, ')') : nullptr;
        if(!rest) return false;        long unsigned utime = 0, stime = 0;
        if(std::sscanf(rest + 1, " %*c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %lu %lu",
                       &utime, &stime) != 2) return false;
        const double ticks = static_cast<double>(utime + stime) / sysconf(_SC_CLK_TCK);
        const ros::Time now = ros::Time::now();
        const double dt = last_cpu_time_.isZero() ? 0.0 : (now - last_cpu_time_).toSec();
        cpu_pct = (last_cpu_ticks_ < 0.0 || dt <= 0.0)
                      ? -1.0 : 100.0 * (ticks - last_cpu_ticks_) / dt;
        last_cpu_ticks_ = ticks;
        last_cpu_time_ = now;

        rss_mb = -1.0;
        if(std::FILE* m = std::fopen("/proc/self/statm", "r")) {
            long unsigned total = 0, resident = 0;
            if(std::fscanf(m, "%lu %lu", &total, &resident) == 2)
                rss_mb = static_cast<double>(resident) * sysconf(_SC_PAGESIZE) / (1024.0*1024.0);
            std::fclose(m);
        }
        return true;
    }

    // Structured counterpart to /lunar_planner/status, which has to stay short single-word
    // tokens for `rostopic echo` diagnosis. recovery_events/successes/aborts are monotone
    // counters rather than events. Per-goal deltas are captured here, at the source, not
    // by subtracting independently received snapshots in a test. Aborted does not prove
    // unreachable; consumers must keep recovery counters and actual completion distinct.
    void publishDiagnostics(bool force = false) {
        const ros::Time now = ros::Time::now();
        const bool chatty = (mode_ == PlannerMode::Recovery);
        if(!force && !last_diag_time_.isZero() &&
           (now - last_diag_time_).toSec() < (chatty ? 0.5 : 1.0)) return;
        last_diag_time_ = now;
        double cpu_pct = -1.0, rss_mb = -1.0;
        // Urgent terminal telemetry must not turn a millisecond CPU tick difference into
        // a spurious resource-overrun sample. Keep resource intervals independent of it.
        if(last_cpu_time_.isZero() || (now-last_cpu_time_).toSec() >= 0.5)
            sampleResources(cpu_pct, rss_mb);
        char buf[448];
        std::snprintf(buf, sizeof(buf),
                      "mode=%s action=%s escalation=%d fails=%d recovery_events=%d "
                      "recovery_successes=%d recovery_aborts=%d last_fail=%s plan_ms=%.1f "
                      "snap_m=%.2f cpu_pct=%.1f rss_mb=%.1f",
                      modeName(mode_), actionName(action_), recovery_escalation_,
                      consecutive_failures_, recovery_events_, recovery_successes_,
                      recovery_aborts_,
                      last_fail_reason_.empty() ? "none" : last_fail_reason_.c_str(),
                      last_plan_ms_, last_snap_dist_, cpu_pct, rss_mb);
        std::ostringstream snapshot;
        snapshot << buf << " goal_id=" << goal_id_
                 << " goal_stamp_ns=" << goal_.header.stamp.toNSec()
                 << " snapshot_seq=" << ++diagnostic_seq_
                 << " status=" << last_status_
                 << " goal_recovery_events=" << recovery_events_-goal_events_start_
                 << " goal_recovery_successes=" << recovery_successes_-goal_successes_start_
                 << " goal_recovery_aborts=" << recovery_aborts_-goal_aborts_start_;
        std_msgs::String msg; msg.data = snapshot.str(); diag_pub_.publish(msg);
    }

    void resetRecoveryState() {
        // A new mission clears goal-owned routes/retreat execution, not measured motion
        // history or an outstanding stop scoped to the revoked trajectory's old goal.
        mode_ = PlannerMode::Nominal; action_ = RecoveryAction::None;
        recovery_escalation_ = 0; consecutive_failures_ = 0; confirm_count_ = 0;
        best_progress_distance_ = std::numeric_limits<double>::infinity();
        last_progress_time_ = ros::Time::now();
        retained_route_.clear();
        active_backout_.clear(); pending_backout_.clear();
        post_backout_replan_.clear();
        active_trajectory_reaches_goal_ = false;
        active_trajectory_was_snapped_ = false;
        last_path_.poses.clear();
    }

    bool retainedTrajectoryStillValid(const geometry_msgs::PoseStamped& start) const {
        if(!retained_route_.reusable()) return false;
        const auto& path = retained_route_.path();
        if(path.poses.empty() ||
           retained_route_.profile().data.size() != path.poses.size()*2) {
            return false;
        }
        return executionTrajectoryStillValid(start,path);
    }

    bool routeRemainingMotion(const geometry_msgs::PoseStamped& start,
                              const nav_msgs::Path& path,
                              double& remaining) const {
        std::size_t nearest=0;
        const Pose2D rover{start.pose.position.x,start.pose.position.y,
                           tf2::getYaw(start.pose.orientation)};
        return remainingTrajectoryMotion(rover,path.poses.size(),cornerRadius(),
            [&path](std::size_t i) {
                const auto& pose=path.poses[i].pose;
                return Pose2D{pose.position.x,pose.position.y,tf2::getYaw(pose.orientation)};
            },remaining,nearest);
    }

    void resetRouteProgress(const geometry_msgs::PoseStamped& start,
                            const nav_msgs::Path& path) {
        double remaining=0.0;
        best_progress_distance_=routeRemainingMotion(start,path,remaining)
            ? remaining : std::numeric_limits<double>::infinity();
        last_progress_time_=ros::Time::now();
    }

    // Check the path currently at the follower, even if it is an unconfirmed recovery
    // route/manoeuvre and therefore ineligible for retained-cache reuse.
    bool executionTrajectoryStillValid(const geometry_msgs::PoseStamped& start,
                                       const nav_msgs::Path& path) const {
        if(path.poses.empty()) return false;
        size_t nearest = 0;
        double nearest_distance = std::numeric_limits<double>::infinity();
        for(size_t i=0; i<path.poses.size(); ++i) {
            const auto& p = path.poses[i].pose.position;
            const double distance = std::hypot(p.x-start.pose.position.x,
                                               p.y-start.pose.position.y);
            if(distance < nearest_distance) {
                nearest_distance = distance;
                nearest = i;
            }
        }
        // A stale path that the rover has already departed cannot be made safe by choosing
        // an arbitrary nearest point and cutting back to it. Normal tracking error is much
        // smaller than the follower lookahead; this bound catches a genuinely lost route.
        if(nearest_distance > terminal_replan_distance_) {
            ROS_WARN_THROTTLE(1.0,
                              "retained trajectory invalid: rover is %.3fm from nearest pose",
                              nearest_distance);
            return false;
        }

        auto segmentValid = [this](const geometry_msgs::Pose& from,
                                   const geometry_msgs::Pose& to,
                                   bool allow_start_unknown,
                                   double clearance0,
                                   double clearance1) {
            float cost;
            return sweptSegmentValid(
                {from.position.x, from.position.y, tf2::getYaw(from.orientation)},
                {to.position.x, to.position.y, tf2::getYaw(to.orientation)},
                clearance0, clearance1, allow_start_unknown, cost,"active_sweep");
        };

        // The current body may occlude its own cells; every point after it is strict. Check
        // the connector to the nearest retained pose. Reuse is limited to an uninterrupted
        // route above; the follower therefore retains its completed rotation/cusp progress.
        geometry_msgs::Pose from = start.pose;
        if(!segmentValid(from, path.poses[nearest].pose,
                         /*allow_start_unknown=*/true,
                         /*clearance0=*/0.0, /*clearance1=*/0.0)) {
            float actual_cost;
            const bool actual_valid = footprintValid(from.position.x, from.position.y,
                tf2::getYaw(from.orientation), actual_cost, /*allow_unknown=*/true);
            const auto& nearest_pose = path.poses[nearest].pose;
            ROS_WARN_THROTTLE(1.0, "retained trajectory invalid on current-pose connector: "
                              "goal_id=%u actual=(%.3f,%.3f,%.3f) actual_body_valid=%s "
                              "nearest=%zu nearest_pose=(%.3f,%.3f,%.3f) map_stamp=%.6f",
                              goal_id_, from.position.x, from.position.y,
                              tf2::getYaw(from.orientation), actual_valid ? "true" : "false",
                              nearest, nearest_pose.position.x, nearest_pose.position.y,
                              tf2::getYaw(nearest_pose.orientation), map_stamp_.toSec());
            if(!actual_valid) logFootprintRejection(
                {from.position.x,from.position.y,tf2::getYaw(from.orientation)},"current_pose");
            return false;
        }
        for(size_t i=nearest+1; i<path.poses.size(); ++i) {
            // The current/nearest pose may already sit inside the desired buffer because
            // of tracking error or a newly refined obstacle boundary. It is not legal to
            // ignore a lethal cell under the physical body, but it must be legal to follow
            // a collision-free route *out* of the buffer. Restore the full margin over the
            // first outgoing segment; all later segments remain fully inflated.
            const double clearance0 = (i == nearest+1) ? 0.0 : trajectory_clearance_;
            if(!segmentValid(path.poses[i-1].pose,
                             path.poses[i].pose,
                             /*allow_start_unknown=*/false,
                             clearance0, trajectory_clearance_)) {
                ROS_WARN_THROTTLE(1.0,
                                  "retained trajectory invalid on swept segment %zu -> %zu "
                                  "goal_id=%u validated_map=%.6f current_map=%.6f",
                                  i-1,i,goal_id_,active_trajectory_map_stamp_.toSec(),map_stamp_.toSec());
                return false;
            }
        }
        return true;
    }

    void republishRetainedTrajectory(bool after_failed_replan = false) {
        nav_msgs::Path path = retained_route_.path();
        path.header.stamp = ros::Time::now();
        for(auto& pose : path.poses) pose.header = path.header;
        last_path_ = path;
        publishPlanOutputs(path, retained_route_.profile(),
                           /*reaches_goal=*/true, retained_route_.wasSnapped(),
                           /*retain_route=*/true);
        if(after_failed_replan) {
            if(last_fail_reason_.empty()) last_fail_reason_ = "replan_failed_reused";
            else last_fail_reason_ += "_reused";
        } else {
            last_plan_ms_ = 0.0;
            last_fail_reason_.clear();
        }
        publishStatus(retained_route_.wasSnapped() ? "success_snapped" : "success");
    }

    void mapCallback(const grid_map_msgs::GridMapConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        grid_map::GridMapRosConverter::fromMessage(*msg, map_);
        map_stamp_ = msg->info.header.stamp;
        have_map_ = msg->info.header.frame_id==map_frame_ && map_.exists("terrain_cost") &&
                    map_.exists("slope_x") && map_.exists("slope_y");
        if(have_map_) {
            if(history_resolution_!=map_.getResolution()) {
                if(active_backout_.active()) history_discontinuous_=true;
                history_resolution_=map_.getResolution();
                motion_history_.configure(cornerRadius(),history_resolution_,sp_.v_max,sp_.w_max,
                                          4.0*recovery_backout_distance_);
            }
            buildTraversabilityCache(); replan_requested_ = true;
        }
    }

    void odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!have_map_) return;
        const auto& p=msg->pose.pose.position;
        const auto& q=msg->pose.pose.orientation;
        const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        if(msg->header.frame_id!=map_frame_ || msg->child_frame_id!=base_frame_ ||
           !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
           !std::isfinite(norm) || std::abs(norm-1.0)>1e-3 ||
           msg->header.stamp>ros::Time::now()+ros::Duration(0.5)) {
            motion_history_.clear();
            history_discontinuous_=true;
            ROS_WARN_THROTTLE(1.0,"backout_history rejected odometry frame/pose/stamp; "
                              "expected %s -> %s",map_frame_.c_str(),base_frame_.c_str());
            return;
        }
        if(!motion_history_.observe({p.x,p.y,tf2::getYaw(q)},msg->header.stamp.toNSec())) {
            history_discontinuous_=true;
            ROS_WARN_THROTTLE(1.0,"backout_history discontinuity; previous measured route discarded");
        }
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
        // Revoke an in-flight retreat using its OLD goal id before accepting another
        // mission. resetRecoveryState must not clear this exact stop acknowledgement.
        if(active_backout_.active()) withdrawInvalidTrajectory("backout_goal_changed");
        goal_ = *msg; have_goal_ = true; replan_requested_ = true;
        ++goal_id_;
        if(goal_id_ == 0) ++goal_id_;  // reserve zero for unscoped/startup telemetry
        goal_events_start_ = recovery_events_;
        goal_successes_start_ = recovery_successes_;
        goal_aborts_start_ = recovery_aborts_;
        // A new goal must not inherit the previous goal's stuck state, or it would be
        // declared unreachable before it has been attempted even once.
        resetRecoveryState();
        last_fail_reason_.clear(); last_plan_ms_ = 0.0; last_snap_dist_ = 0.0;
        last_status_.clear();
        publishStatus("goal_received");
    }

    void followerDiagnosticCallback(const std_msgs::StringConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(stop_barrier_.observe(msg->data,ros::Time::now().toNSec())) {
            replan_requested_=true;
            ROS_INFO("plan: stop acknowledged goal_id=%u trajectory_stamp_ns=%llu; "
                     "next search requires a fresh pose",stop_barrier_.goal(),
                     static_cast<unsigned long long>(stop_barrier_.stamp()));
        }
    }

    void withdrawInvalidTrajectory(const char* reason="active_trajectory_invalid") {
        nav_msgs::Path empty;
        empty.header.frame_id=map_frame_;
        empty.header.stamp=ros::Time::now();
        // Set the barrier BEFORE publishing, so even a prompt acknowledgement is scoped.
        stop_barrier_.request(goal_id_,empty.header.stamp.toNSec());
        last_path_=empty;
        publishPlanOutputs(empty,std_msgs::Float32MultiArray(),false,false);
        last_fail_reason_=reason;
        replan_requested_=true;
        publishStatus("no_path");
        ROS_WARN("plan: withdrawing invalid execution trajectory before search: "
                 "goal_id=%u trajectory_stamp_ns=%llu reason=%s",goal_id_,
                 static_cast<unsigned long long>(stop_barrier_.stamp()),reason);
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
        // over-slope cells are still rejected, and the whole sweep is checked, not just
        // the end yaw -- this probe is issued when the rover is already boxed in, which is
        // exactly where a corner has something to hit.
        SweptFootprintRejection rejected;
        if(!rotationValid(x, y, yaw, target_yaw, cost, /*departure=*/true,
                          debug_start_rejections_ ? &rejected : nullptr)) {
            if(debug_start_rejections_) logSweptRejection("recovery_rotate",rejected);
            return false;
        }

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

    // Retrace stamped localisation, NOT the last planned path (which may never have been
    // driven). Keep the observed orientations; the normal profiler determines reverse,
    // rotation and direction changes. No goal/profile data crosses the history boundary.
    bool recoveryBackOut(const geometry_msgs::PoseStamped& start, nav_msgs::Path& path,
                         std_msgs::Float32MultiArray& vel) {
        pending_backout_.clear();
        std::vector<Pose2D> poses;
        const char* reason="none";
        const Pose2D current{start.pose.position.x,start.pose.position.y,tf2::getYaw(start.pose.orientation)};
        if(!motion_history_.backtrack(current,start.header.stamp.toNSec(),
                                      recovery_backout_distance_,poses,reason)) {
            ROS_WARN("recovery_reject goal_id=%u action=backout reason=%s observations=%zu",
                     goal_id_,reason,motion_history_.size());
            return false;
        }
        auto check=[this](const Pose2D& pose,double margin,bool unknown,float& cost) {
            return footprintWithClearanceValid(pose.x,pose.y,pose.yaw,cost,unknown,margin);
        };
        SweptFootprintRejection rejected;
        // BackOut is a repositioning manoeuvre, not a route to the snapped endpoint. Restore
        // the ordinary trajectory reserve here. If the next route still needs goal snapping,
        // normal search must independently grow and hold goal_snap_clearance_. Using the last
        // failed snap attempt here incorrectly raised the escape requirement from 0.25 to 0.50m.
        if(!pending_backout_.prepare(poses,cornerRadius(),map_.getResolution(),
                                     recovery_backout_distance_,trajectory_clearance_,check,&rejected)) {
            logSweptRejection("recovery_backout",rejected);
            return false;
        }
        // A path whose endpoint already satisfies the follower's pose gate would be
        // reported reached without any movement. It cannot promise to regain clearance.
        if(trajectoryEndpointReached(std::hypot(poses.back().x-current.x,poses.back().y-current.y),
                wrap(poses.back().yaw-current.yaw),execution_goal_pos_tolerance_,execution_goal_yaw_tolerance_)) {
            pending_backout_.clear();
            ROS_WARN("recovery_reject goal_id=%u action=backout reason=endpoint_already_acquired",goal_id_);
            return false;
        }
        path.header.frame_id=map_frame_; path.header.stamp=ros::Time::now();
        path.poses.clear();
        for(const auto& observed : poses) {
            geometry_msgs::PoseStamped pose; pose.header=path.header;
            pose.pose.position.x=observed.x; pose.pose.position.y=observed.y;
            tf2::Quaternion q; q.setRPY(0,0,observed.yaw); pose.pose.orientation=tf2::toMsg(q);
            path.poses.push_back(pose);
        }
        buildVelocityProfile(path, vel);
        // The profiler inserts midpoint samples. Freeze/recheck the exact exported
        // geometry; cumulative-motion margins are invariant under that densification.
        poses.clear();
        for(const auto& pose : path.poses)
            poses.push_back({pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation)});
        if(!pending_backout_.prepare(poses,cornerRadius(),map_.getResolution(),
                                     recovery_backout_distance_,trajectory_clearance_,check,&rejected)) {
            logSweptRejection("recovery_backout_export",rejected);
            return false;
        }
        return true;
    }

    double backoutDuration(const nav_msgs::Path& path, const std_msgs::Float32MultiArray& vel) const {
        if(path.poses.size()<2 || vel.data.size()!=2*path.poses.size()) return 0.0;
        double duration=0.0;
        for(size_t i=1;i<path.poses.size();++i) {
            const auto& a=path.poses[i-1].pose; const auto& b=path.poses[i].pose;
            const double ds=std::hypot(b.position.x-a.position.x,b.position.y-a.position.y);
            const double yaw=std::abs(wrap(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation)));
            const double v=.5*(std::abs(vel.data[2*i])+std::abs(vel.data[2*(i-1)]));
            const double w=.5*(std::abs(vel.data[2*i+1])+std::abs(vel.data[2*(i-1)+1]));
            if(!std::isfinite(v) || !std::isfinite(w) || v>sp_.v_max+1e-6 || w>sp_.w_max+1e-6) return 0.0;
            if(ds>1e-3) {
                if(v<1e-6) return 0.0;
                duration+=std::max(ds/v,yaw/(w>1e-6 ? w : sp_.w_max));
            } else if(yaw>1e-3) {
                if(w<1e-6) return 0.0;
                duration+=yaw/w;
            }
        }
        return std::isfinite(duration) ? duration : 0.0;
    }

    void finishBackout(bool completed, const char* reason) {
        ROS_WARN("backout_execution goal_id=%u result=%s reason=%s progress=%.3f length=%.3f actual_motion=%.3f",
                 goal_id_,completed ? "clearance_restored" : "stopped",reason,
                 active_backout_.progress(),active_backout_.length(),active_backout_.actualMotion());
        // Completion permits one bounded Relax retry, never a new Rotate/BackOut cycle.
        // The retry clock starts only after the stop acknowledgement and a fresh pose.
        // Failed retreat advances to Abort after the same safe stop.
        if(completed) post_backout_replan_.request();
        recovery_escalation_=completed ? 0 : 3;
        action_=completed ? RecoveryAction::Relax : RecoveryAction::Abort;
        confirm_count_=0;
        recovery_step_start_=ros::Time::now();
        withdrawInvalidTrajectory(reason);
    }

    void executeBackout(const geometry_msgs::PoseStamped& start) {
        if(history_discontinuous_) {
            finishBackout(false,"backout_localisation_discontinuity");
            return;
        }
        const Pose2D current{start.pose.position.x,start.pose.position.y,tf2::getYaw(start.pose.orientation)};
        auto check=[this](const Pose2D& pose,double margin,bool unknown,float& cost) {
            return footprintWithClearanceValid(pose.x,pose.y,pose.yaw,cost,unknown,margin);
        };
        SweptFootprintRejection rejected;
        if(!active_backout_.validate(current,terminal_replan_distance_,check,&rejected)) {
            logSweptRejection("active_backout",rejected);
            finishBackout(false,active_backout_.failure());
            return;
        }
        const auto& endpoint=active_backout_.poses().back();
        const bool acquired=trajectoryEndpointReached(std::hypot(endpoint.x-current.x,endpoint.y-current.y),
                wrap(endpoint.yaw-current.yaw),execution_goal_pos_tolerance_,execution_goal_yaw_tolerance_);
        const bool restored=acquired && active_backout_.clearanceRestored(current,check);
        if(!backout_lease_.update(ros::Time::now().toSec(),active_backout_.progress())) {
            finishBackout(false,acquired && !restored ? "backout_actual_clearance_missing" : "backout_execution_timeout");
            return;
        }
        if(restored) {
            finishBackout(true,"backout_clearance_restored");
            return;
        }
        // No replanning/restarting of the escape ramp while it executes. Refresh only
        // timestamps, leaving follower phase progress and the fixed lease unchanged.
        last_path_.header.stamp=ros::Time::now();
        for(auto& pose : last_path_.poses) pose.header=last_path_.header;
        publishPlanOutputs(last_path_,backout_profile_,false,false,false,/*preserve_backout=*/true);
        replan_requested_=false;
        publishStatus("recovery_backout");
        ROS_INFO_THROTTLE(1.0,"backout_execution goal_id=%u progress=%.3f length=%.3f full_margin=%.3f",
                          goal_id_,active_backout_.progress(),active_backout_.length(),active_backout_.fullClearance());
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
                mode_ = PlannerMode::Aborted; have_goal_ = false; ++recovery_aborts_;
                ROS_WARN("plan: giving up on this goal after %d failed attempts; send a new goal",
                         consecutive_failures_);
                return false;
            default:
                return plan(start, goal_, path, vel);
        }
    }

    void timerCallback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!have_map_) {
            if(active_backout_.active()) finishBackout(false,"backout_map_unavailable");
            return;
        }
        // Checked before have_goal_, which abort clears: the latched status has to keep
        // saying "aborted" so `rostopic echo /lunar_planner/status` explains the silence.
        if(mode_ == PlannerMode::Aborted) { publishStatus("aborted"); return; }
        if(!have_goal_) return;
        if(stop_barrier_.waiting()) {
            ROS_WARN_THROTTLE(1.0,"plan: waiting for trajectory stop acknowledgement: goal_id=%u "
                              "trajectory_stamp_ns=%llu; search held",stop_barrier_.goal(),
                              static_cast<unsigned long long>(stop_barrier_.stamp()));
            return;
        }
        geometry_msgs::PoseStamped start;
        if(!robotPose(start)) {
            if(active_backout_.active()) finishBackout(false,"backout_tf_unavailable");
            else publishStatus("tf_unavailable");
            return;
        }
        if(!stop_barrier_.canSearch(start.header.stamp.toNSec())) {
            ROS_WARN_THROTTLE(1.0,"plan: stop acknowledged but waiting for fresh start TF: goal_id=%u",
                              goal_id_);
            return;
        }
        stop_barrier_.clear();

        // The user-authorised bounded retreat has its own frozen clearance schedule.
        // Neither ordinary revalidation nor the two-second escalation rung may replace
        // it halfway through. It still fails closed on stale terrain/TF.
        if(active_backout_.active()) {
            if((ros::Time::now()-map_stamp_).toSec()>max_map_age_)
                finishBackout(false,"backout_stale_map");
            else if((ros::Time::now()-start.header.stamp).toSec()>max_map_age_)
                finishBackout(false,"backout_stale_tf");
            else executeBackout(start);
            return;
        }

        // The follower stops against the *active* trajectory endpoint. That can differ from
        // retained_route_ while recovery owns a rotate/back-out or a relaxed goal plan, so
        // completion must be classified with the exact trajectory currently at the follower.
        // A recovery manoeuvre ending is not mission success. An ordinary tolerance endpoint
        // outside the requested-goal radius is an intermediate waypoint and triggers a fresh
        // refinement; a snapped route is the deliberate unsafe-goal exception.
        if(active_trajectory_reaches_goal_ && !last_path_.poses.empty()) {
            const auto& endpoint = last_path_.poses.back().pose;
            const double endpoint_dist = std::hypot(
                endpoint.position.x-start.pose.position.x,
                endpoint.position.y-start.pose.position.y);
            const double endpoint_yaw_error = wrap(
                tf2::getYaw(endpoint.orientation)-tf2::getYaw(start.pose.orientation));
            const bool endpoint_reached = trajectoryEndpointReached(
                endpoint_dist, endpoint_yaw_error,
                execution_goal_pos_tolerance_, execution_goal_yaw_tolerance_);
            const double requested_goal_dist = std::hypot(
                goal_.pose.position.x-start.pose.position.x,
                goal_.pose.position.y-start.pose.position.y);
            if(missionGoalReached(endpoint_reached,
                                  active_trajectory_reaches_goal_,
                                  active_trajectory_was_snapped_,
                                  requested_goal_dist,
                                  requested_goal_pos_tolerance_)) {
                ROS_INFO("plan: execution endpoint reached (position_error=%.3fm, "
                         "yaw_error=%.3frad, requested_error=%.3fm, snapped=%s); "
                         "retiring goal",
                         endpoint_dist, std::abs(endpoint_yaw_error), requested_goal_dist,
                         active_trajectory_was_snapped_ ? "true" : "false");
                publishStatus("goal_reached");
                have_goal_ = false;
                replan_requested_ = false;
                retained_route_.clear();
                active_trajectory_reaches_goal_ = false;
                active_trajectory_was_snapped_ = false;
                return;
            }
            if(endpoint_reached && !active_trajectory_was_snapped_) {
                ROS_INFO("plan: intermediate tolerance endpoint reached %.3fm from requested "
                         "goal; replanning for final accuracy", requested_goal_dist);
                mode_ = PlannerMode::Nominal;
                action_ = RecoveryAction::None;
                consecutive_failures_ = 0;
                recovery_escalation_ = 0;
                confirm_count_ = 0;
                post_backout_replan_.clear();
                best_progress_distance_ = requested_goal_dist;
                last_progress_time_ = ros::Time::now();
                retained_route_.clear();
                active_trajectory_reaches_goal_ = false;
                active_trajectory_was_snapped_ = false;
                last_path_.poses.clear();
                replan_requested_ = true;
                return;
            }
        }
        if((ros::Time::now() - map_stamp_).toSec() > max_map_age_) {
            publishStatus("stale_map"); return;
        }
        // Replanning is driven by map arrival (see mapCallback), not by this timer: at a
        // 0.5s tick against a ~1.1s perception cycle the same map was replanned twice, which
        // is what lets consecutive plans disagree and the rover oscillate.
        if(!replan_requested_) return;
        replan_requested_ = false;

        // Never spend a planning budget while the follower executes a path this map has
        // already invalidated. Publish an empty atomic trajectory and RETURN; waiting for
        // its exact acknowledgement also prevents a fast replan from overwriting the stop
        // in a queue of size one. The next timer obtains a post-stop start TF.
        bool active_path_valid=true;
        if(!last_path_.poses.empty()) {
            active_path_valid=executionTrajectoryStillValid(start,last_path_);
            if(!active_path_valid) {
                withdrawInvalidTrajectory();
                return;
            }
        }

        const double requested_goal_dist = std::hypot(
            goal_.pose.position.x - start.pose.position.x,
            goal_.pose.position.y - start.pose.position.y);
        // Completion measures the endpoint the rover is actually executing. Progress uses
        // remaining route motion below: Euclidean goal distance can increase on a valid
        // detour, and treating that as stuck caused recovery to interrupt the manoeuvre.
        double execution_goal_dist = requested_goal_dist;
        if(active_trajectory_reaches_goal_ && !last_path_.poses.empty()) {
            const auto& endpoint = last_path_.poses.back().pose.position;
            execution_goal_dist = std::hypot(endpoint.x-start.pose.position.x,
                                             endpoint.y-start.pose.position.y);
        }
        double progress_distance=execution_goal_dist;
        double route_remaining=0.0;
        if(active_trajectory_reaches_goal_ &&
           routeRemainingMotion(start,last_path_,route_remaining)) {
            progress_distance=route_remaining;
        }
        if(progress_distance < best_progress_distance_ - progress_epsilon_) {
            best_progress_distance_ = progress_distance; last_progress_time_ = ros::Time::now();
        }

        // Compute this before any path reuse. Otherwise a latched terminal route returns
        // early forever and silently disables the independent no-progress recovery gate.
        const bool no_progress = progress_distance > goal_tolerance_ &&
            (ros::Time::now() - last_progress_time_).toSec() > no_progress_timeout_;

        // Keep the complete accepted goal route while it remains valid and makes measured
        // progress. A rolling-map replan every ~0.5 s can alternate equally valid initial
        // forward/reverse actions before either finishes. The current map already revalidated
        // the active path above; no-progress and newly revealed hazards retain authority.
        const bool active_goal_route=retained_route_.reusable() &&
            active_trajectory_reaches_goal_ && !last_path_.poses.empty();
        const bool stable_route_candidate = stableTrajectoryReuseAllowed(
            active_goal_route, no_progress,
            mode_ == PlannerMode::Nominal,active_path_valid);
        if(stable_route_candidate) {
            if(stableTrajectoryReuseAllowed(active_goal_route,no_progress,
                                            mode_ == PlannerMode::Nominal,
                                            active_path_valid)) {
                ROS_INFO_THROTTLE(1.0,
                                  "plan: reusing stable %s trajectory (%zu poses, "
                                  "remaining_motion=%.3fm, endpoint_dist=%.3fm)",
                                  retained_route_.wasSnapped() ? "snapped" : "goal",
                                  retained_route_.path().poses.size(),progress_distance,
                                  execution_goal_dist);
                republishRetainedTrajectory();
                return;
            }
        }

        // Two independent triggers, because the task book names both failure modes: repeated
        // planning failure near obstacles, and making no headway while still producing paths
        // (an oscillating planner replans happily forever but never improves route progress).
        if(mode_ == PlannerMode::Nominal) {
            const bool cooled_down = last_recovery_end_.isZero() ||
                (ros::Time::now() - last_recovery_end_).toSec() > min_recovery_interval_;
            if((consecutive_failures_ >= recovery_fail_threshold_ || no_progress) && cooled_down) {
                mode_ = PlannerMode::Recovery; recovery_escalation_ = 0; confirm_count_ = 0;
                post_backout_replan_.clear();
                recovery_step_start_ = ros::Time::now(); ++recovery_events_;
            }
        } else if(post_backout_replan_.pending()) {
            const bool already_started=post_backout_replan_.started();
            if(post_backout_replan_.allow(ros::Time::now().toSec(),recovery_step_timeout_)) {
                recovery_escalation_=0;
                if(!already_started) ROS_INFO("post_backout_replan goal_id=%u result=started budget=%.3fs",
                                              goal_id_,recovery_step_timeout_);
            } else {
                if(recovery_escalation_!=3) ROS_WARN("post_backout_replan goal_id=%u result=exhausted; "
                                                    "no confirmed goal route, advancing to Abort",goal_id_);
                recovery_escalation_=3;
                confirm_count_=0;
            }
        } else if((ros::Time::now() - recovery_step_start_).toSec() > recovery_step_timeout_) {
            ++recovery_escalation_; confirm_count_ = 0; recovery_step_start_ = ros::Time::now();
        }

        // A still-valid rotate/relaxed path may be moving when its rung expires. Stop it
        // before taking the backtrack snapshot, not after computing a route from a pose
        // that has already changed. The exact acknowledgement gate above owns the wait.
        if(mode_==PlannerMode::Recovery &&
           ladder_[std::min<size_t>(recovery_escalation_,ladder_.size()-1)]==RecoveryAction::BackOut &&
           !last_path_.poses.empty()) {
            withdrawInvalidTrajectory("backout_prepare_stop");
            return;
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
            const bool backout=action_==RecoveryAction::BackOut;
            if(backout) {
                const double duration=backoutDuration(path,vel);
                if(!pending_backout_.active() || duration<=0.0) {
                    pending_backout_.clear();
                    ++consecutive_failures_;
                    last_path_.poses.clear();
                    publishPlanOutputs(last_path_,std_msgs::Float32MultiArray(),false,false);
                    ROS_WARN("recovery_reject goal_id=%u action=backout reason=invalid_velocity_profile",goal_id_);
                    publishStatus(recoveryStatus());
                    return;
                }
                active_backout_=std::move(pending_backout_);
                pending_backout_.clear();
                history_discontinuous_=false;
                backout_profile_=vel;
                backout_lease_.start(ros::Time::now().toSec(),duration,no_progress_timeout_,progress_epsilon_);
                ROS_INFO("backout_execution goal_id=%u result=started length=%.3f full_margin=%.3f "
                         "estimated_duration=%.3f observations=%zu",goal_id_,active_backout_.length(),
                         active_backout_.fullClearance(),duration,motion_history_.size());
            }
            last_path_ = path;
            publishPlanOutputs(path, vel,
                               /*reaches_goal=*/false, /*was_snapped=*/false, false,backout);
            publishStatus(recoveryStatus());
        } else if(ok) {
            consecutive_failures_ = 0;
            last_path_ = path;
            const bool recovery_confirmed = in_recovery &&
                ++confirm_count_ >= recovery_confirm_count_;
            // A confirmed relaxed plan is the NEW nominal route. Keeping the pre-recovery
            // cache here resurrected its completed first rotation at the next reuse tick.
            // Unconfirmed plans and manoeuvres must not become reusable history.
            publishPlanOutputs(path, vel,
                               /*reaches_goal=*/true, snapped_goal_used_,
                               /*retain_route=*/!in_recovery || recovery_confirmed);
            if(!in_recovery || recovery_confirmed) resetRouteProgress(start,path);
            if(in_recovery) {
                // Require repeated success before declaring recovery over, otherwise a
                // marginal situation chatters between recovery and nominal every cycle.
                if(recovery_confirmed) {
                    mode_ = PlannerMode::Nominal; action_ = RecoveryAction::None;
                    recovery_escalation_ = 0; confirm_count_ = 0;
                    post_backout_replan_.clear();
                    last_recovery_end_ = ros::Time::now();
                    ROS_INFO("plan: recovery confirmed; retained route replaced (%zu poses)",
                             path.poses.size());
                    ++recovery_successes_;
                    publishStatus(snapped_goal_used_ ? "success_snapped" : "success");
                } else {
                    publishStatus(recoveryStatus());
                }
            } else {
                publishStatus(snapped_goal_used_ ? "success_snapped" : "success");
            }
        } else {
            // Search can transiently time out as the rolling perception map changes. Do not
            // turn that into a stop/recovery if the route already being executed is still a
            // complete safe solution on this exact map. Its untraversed body+clearance and
            // every inter-pose sweep are revalidated above; a changed hazard therefore
            // still withdraws the route immediately. No-progress remains an independent
            // recovery trigger if the rover keeps a valid route but fails to advance.
            const bool retained_valid = !in_recovery && retainedTrajectoryStillValid(start);
            if(retainedTrajectoryFallbackAllowed(mode_ == PlannerMode::Nominal,
                                                 retained_valid)) {
                ROS_WARN_THROTTLE(1.0,
                                  "plan: fresh search failed (%s); continuing revalidated "
                                  "trajectory (%zu poses)",
                                  last_fail_reason_.empty() ? "unknown" : last_fail_reason_.c_str(),
                                  retained_route_.path().poses.size());
                consecutive_failures_ = 0;
                confirm_count_ = 0;
                republishRetainedTrajectory(/*after_failed_replan=*/true);
                return;
            }
            ++consecutive_failures_; confirm_count_ = 0;
            last_path_.poses.clear();
            publishPlanOutputs(last_path_, std_msgs::Float32MultiArray(),
                               /*reaches_goal=*/false, /*was_snapped=*/false);
            publishStatus(mode_ == PlannerMode::Aborted ? "aborted"
                          : (in_recovery ? recoveryStatus() : "no_path"));
        }
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
    // Rasterise ALL intersected grid cells rather than a rotated body-coordinate point
    // lattice. A larger clearance rectangle must include every smaller one's cells.
    // Gentle cells still skip the direction-dependent slope trigonometry.
    bool footprintValid(double x, double y, double yaw, float& cost, bool allow_unknown = false,
                        double margin = 0.0, FootprintRejection* rejection = nullptr) const {
        if(rejection) *rejection=FootprintRejection{};
        const auto reject = [rejection](const char* reason,double wx,double wy,int row,int col) {
            if(rejection) *rejection={reason,wx,wy,row,col};
            return false;
        };
        cost = 0.0f; int samples = 0;
        const double r = map_.getResolution();
        const double cyaw = std::cos(yaw), syaw = std::sin(yaw);
        const double origin_x=map_.getPosition().x()-0.5*map_.getLength().x();
        const double origin_y=map_.getPosition().y()-0.5*map_.getLength().y();
        const bool valid=visitFootprintCells({x,y,yaw},footprint_length_,footprint_width_,margin,
            r,origin_x,origin_y,[&](double wx,double wy) {
                grid_map::Index idx;
                if(!map_.getIndex(grid_map::Position(wx, wy), idx)) {
                    if(allow_unknown) return true;
                    return reject("off_map",wx,wy,-1,-1);
                }
                const size_t lin = static_cast<size_t>(idx(0)) * cell_cols_ + idx(1);
                const float c = cell_cost_[lin];
                if(!std::isfinite(c)) {
                    if(allow_unknown) return true;
                    return reject("unknown_cost",wx,wy,idx(0),idx(1));
                }
                if(c >= 100.0f) return reject("lethal_cost",wx,wy,idx(0),idx(1));
                const float sm = cell_slopemag_[lin];
                if(!std::isfinite(sm)) {
                    if(allow_unknown) return true;
                    return reject("unknown_slope",wx,wy,idx(0),idx(1));
                }
                if(sm > max_lat_slope_) {  // steep cell: fall back to the directional check
                    const float gx = cell_gx_[lin], gy = cell_gy_[lin];
                    const double longitudinal = std::atan(std::abs(gx*cyaw + gy*syaw)) * 180.0/M_PI;
                    const double lateral = std::atan(std::abs(-gx*syaw + gy*cyaw)) * 180.0/M_PI;
                    if(longitudinal > max_long_slope_ || lateral > max_lat_slope_)
                        return reject(longitudinal > max_long_slope_ ? "longitudinal_slope"
                                                                     : "lateral_slope",
                                      wx,wy,idx(0),idx(1));
                }
                cost += c; ++samples;
                return true;
            });
        if(samples) cost /= samples;
        return valid;
    }

    // Observability only: report the exact rejected cell and original terrain layers.
    // Ground-truth clearance does not justify overriding a perceived lethal cell. These
    // fields distinguish obstacle/step/slope rejection before considering a map change.
    void logFootprintRejection(const Pose2D& pose,const char* context,
                               double margin=0.0,bool allow_body_unknown=true,
                               bool force=false) const {
        FootprintRejection rejection;
        float cost;
        double rejected_margin=0.0;
        // Mirror the actual body/band policy, including their different unknown rules.
        if(footprintValid(pose.x,pose.y,pose.yaw,cost,allow_body_unknown,0.0,&rejection)) {
            if(margin<=0.0 || footprintValid(pose.x,pose.y,pose.yaw,
                                             cost,true,margin,&rejection)) return;
            rejected_margin=margin;
        }
        const bool indexed=rejection.row>=0 && rejection.col>=0;
        const grid_map::Index index(rejection.row,rejection.col);
        const double nan=std::numeric_limits<double>::quiet_NaN();
        grid_map::Position cell(nan,nan);
        if(indexed) map_.getPosition(index,cell);
        const auto layer=[&](const char* name) {
            return indexed && map_.exists(name) ? double(map_.at(name,index)) : nan;
        };
        char message[2048];
        std::snprintf(message,sizeof(message),
                          "footprint_reject goal_id=%u context=%s reason=%s margin=%.3f "
                          "pose=(%.3f,%.3f,%.3f) sample=(%.3f,%.3f) cell=(%.3f,%.3f) "
                          "terrain_cost=%.3f slope_x=%.4f slope_y=%.4f slope=%.3f "
                          "step_height=%.3f obstacle_height=%.3f obstacle_confidence=%.3f "
                          "roughness=%.3f ground=%.3f ground_corrected=%.3f "
                          "elevation_raw=%.3f observed=%.3f observation_age=%.3f "
                          "ground_confidence=%.3f points_raw=%.3f points_used=%.3f "
                          "map_stamp=%.6f",
                          goal_id_,context,rejection.reason,rejected_margin,
                          pose.x,pose.y,pose.yaw,
                          rejection.sample_x,rejection.sample_y,cell.x(),cell.y(),
                          layer("terrain_cost"),layer("slope_x"),layer("slope_y"),layer("slope"),
                          layer("step_height"),layer("obstacle_height"),layer("obstacle_confidence"),
                          layer("roughness"),layer("ground"),layer("ground_corrected"),
                          layer("elevation_raw"),layer("observed"),layer("observation_age"),
                          layer("groundpatch"),layer("pointsRaw"),layer("points"),
                          map_stamp_.toSec());
        if(force) { ROS_WARN("%s",message); }
        else { ROS_WARN_THROTTLE(1.0,"%s",message); }
    }

    void logSweptRejection(const char* context,const SweptFootprintRejection& rejected) const {
        ROS_WARN("sweep_reject goal_id=%u context=%s sample=%s "
                 "pose=(%.6f,%.6f,%.6f) margin=%.6f allow_unknown=%s map_stamp=%.6f",
                 goal_id_,context,rejected.has_sample ? "true" : "false",
                 rejected.pose.x,rejected.pose.y,rejected.pose.yaw,rejected.clearance,
                 rejected.allow_unknown ? "true" : "false",map_stamp_.toSec());
        if(rejected.has_sample)
            logFootprintRejection(rejected.pose,context,rejected.clearance,
                                  rejected.allow_unknown,/*force=*/true);
    }

    // Validate the physical body strictly, then reserve a configurable band around it from
    // every *known* hazard. Unknown cells are tolerated only in that extra band: requiring
    // observed terrain beyond the body would make goals in a LiDAR occlusion shadow
    // impossible, while allowing unknown cells under the body would weaken the normal
    // planner invariant. The returned terrain cost remains the body's cost, so adding a
    // safety band does not also apply terrain speed scaling a second time.
    bool footprintWithClearanceValid(double x, double y, double yaw, float& cost,
                                     bool allow_body_unknown,
                                     double clearance) const {
        if(!footprintValid(x, y, yaw, cost, allow_body_unknown)) return false;
        if(clearance <= 0.0) return true;
        float inflated_cost;
        return footprintValid(x, y, yaw, inflated_cost,
                              /*allow_unknown=*/true, clearance);
    }

    double trajectoryClearance() const {
        // A route to a snapped goal is planned with an uncertainty reserve along its whole
        // length, not only at the endpoint. Retained-path validation deliberately uses the
        // ordinary trajectory_clearance_ explicitly: the difference is hysteresis that the
        // far-to-near rolling-map refinement may consume without causing path churn.
        return snapped_goal_used_ ? goal_snap_clearance_ : trajectory_clearance_;
    }

    bool trajectoryFootprintValid(double x, double y, double yaw, float& cost,
                                  bool allow_body_unknown = false) const {
        return footprintWithClearanceValid(x, y, yaw, cost, allow_body_unknown,
                                           trajectoryClearance());
    }

    bool sweptSegmentValid(const Pose2D& from, const Pose2D& to,
                            double clearance0, double clearance1,
                            bool allow_start_unknown, float& cost,
                            const char* rejection_context=nullptr,
                            SweptFootprintRejection* rejection=nullptr) const {
        if(allow_start_unknown && clearance0==0.0 && clearance1>0.0 &&
           footprintWithClearanceValid(from.x,from.y,from.yaw,cost,true,clearance1))
            clearance0=clearance1;
        return sweptFootprintValid(from, to, cornerRadius(), map_.getResolution(),
            clearance0, clearance1, allow_start_unknown,
            [this,rejection_context](const Pose2D& pose, double clearance,
                                     bool allow_unknown, float& terrain) {
                const bool valid=footprintWithClearanceValid(pose.x,pose.y,pose.yaw,
                                                              terrain,allow_unknown,clearance);
                if(!valid && rejection_context)
                    logFootprintRejection(pose,rejection_context,clearance,allow_unknown);
                return valid;
            }, cost, rejection);
    }

    // Half the body diagonal: how far a corner stands from the centre, and therefore the
    // radius its arc has when the body turns on the spot.
    double cornerRadius() const {
        return std::hypot(footprint_length_, footprint_width_) / 2.0;
    }

    // Validate a turn on the spot. Checking only the end yaw is not enough: a corner
    // reaches 0.42 m further out than the broadside rectangle, so a pose that is clear at
    // every axis-aligned heading can still graze an obstacle mid-turn. Measured on the
    // mixed scenario -- the rover sat 0.12 m clear of the (0,-2) boulder, rotated during
    // recovery, and its corner passed 0.15 m inside it. Arc primitives have always sampled
    // their sweep; rotations, which sweep the most, did not. The step keeps corner travel
    // under one cell, which is the spacing footprintValid samples the body at anyway.
    bool rotationValid(double x, double y, double yaw0, double yaw1, float& cost,
                       bool departure = false, SweptFootprintRejection* rejection=nullptr) const {
        return sweptSegmentValid({x,y,yaw0}, {x,y,yaw1},
                                 departure ? 0.0 : trajectoryClearance(),
                                 trajectoryClearance(), departure, cost, nullptr, rejection);
    }

    // Transform primitive samples from the start-body frame into the world, validate the
    // swept footprint along the primitive, accumulate terrain cost, and return the endpoint.
    bool primitiveValid(double px, double py, double pyaw, const MotionPrimitive& prim,
                        float& avg_cost, double& ex, double& ey, double& eyaw,
                        bool departure = false) const {
        if(prim.samples.empty()) return false;
        const double c = std::cos(pyaw), s = std::sin(pyaw);
        auto worldPose = [&](size_t i) {
            const auto& smp = prim.samples[i];
            return Pose2D{px+c*smp.x-s*smp.y,py+s*smp.x+c*smp.y,wrap(pyaw+smp.yaw)};
        };
        const auto last = worldPose(prim.samples.size()-1);
        ex=last.x; ey=last.y; eyaw=last.yaw;
        return sampledFootprintValid({px,py,pyaw},prim.samples.size(),worldPose,
            cornerRadius(),map_.getResolution(),trajectoryClearance(),departure,
            [this](const Pose2D& pose,double clearance,bool allow_unknown,float& cost) {
                return footprintWithClearanceValid(pose.x,pose.y,pose.yaw,
                                                    cost,allow_unknown,clearance);
            },avg_cost);
    }

    bool transition(const State& from, int direction, int turn, State& to, float& edge_cost,
                    bool departure = false, SweptFootprintRejection* rejection=nullptr) const {
        if(rejection) *rejection=SweptFootprintRejection{};
        grid_map::Position p;
        if(!map_.getPosition(grid_map::Index(from.x, from.y), p)) return false;
        const double yaw0 = yawForBin(from.t);
        if(direction == 0) {
            to = from; to.t = (from.t + turn + bins_) % bins_;
            float terrain;
            if(!rotationValid(p.x(), p.y(), yaw0, yawForBin(to.t), terrain, departure,rejection)) return false;
            edge_cost = static_cast<float>(rotation_cost_ * primitive_length_ + terrain * 0.002);
            return true;
        }

        const double dyaw = turn * 2.0 * M_PI / bins_;
        const double yaw1 = yaw0 + dyaw;
        const double x1 = p.x() + direction*primitive_length_*std::cos(yaw0 + dyaw*0.5);
        const double y1 = p.y() + direction*primitive_length_*std::sin(yaw0 + dyaw*0.5);
        grid_map::Index idx;
        if(!map_.getIndex(grid_map::Position(x1,y1), idx)) return false;
        to = {idx(0), idx(1), binForYaw(yaw1)};
        grid_map::Position endpoint;
        float terrain;
        if(!map_.getPosition(idx, endpoint) ||
           !latticeArcFootprintValid({p.x(),p.y(),yaw0},
                {endpoint.x(),endpoint.y(),yawForBin(to.t)}, direction*primitive_length_,dyaw,
                cornerRadius(),map_.getResolution(),trajectoryClearance(),departure,
                [this](const Pose2D& pose, double clearance, bool allow_unknown, float& cost) {
                    return footprintWithClearanceValid(pose.x,pose.y,pose.yaw,
                                                        cost,allow_unknown,clearance);
                },terrain,rejection)) return false;
        const double motion_factor = direction < 0 ? reverse_cost_ : 1.0;
        edge_cost = static_cast<float>(motion_factor * primitive_length_ *
                    (1.0 + 0.01 * terrain));
        return !(to.x == from.x && to.y == from.y && to.t == from.t);
    }

    // Re-run only the eight root actions after an exhausted arc search. This uses the
    // same production checker on the same locked map, but cannot publish or select a
    // route. Batch throttling retains every action's reason instead of suppressing seven
    // of them with the ordinary shared per-cell log throttle.
    void diagnoseRootActions(const State& start) {
        if(!debug_start_rejections_) return;
        const ros::Time now=ros::Time::now();
        if(last_root_debug_goal_==goal_id_ && !last_root_debug_time_.isZero() &&
           (now-last_root_debug_time_).toSec()<3.0) return;
        last_root_debug_goal_=goal_id_;
        last_root_debug_time_=now;
        grid_map::Position origin;
        if(!map_.getPosition(grid_map::Index(start.x,start.y),origin)) return;
        ROS_WARN("root_actions goal_id=%u mode=arcs start=(%.6f,%.6f,%.6f) "
                 "clearance=%.3f map_stamp=%.6f",goal_id_,origin.x(),origin.y(),
                 yawForBin(start.t),trajectoryClearance(),map_stamp_.toSec());
        for(int direction : {-1,0,1}) for(int turn : {-1,0,1}) {
            if(direction==0 && turn==0) continue;
            State next{};
            float cost=0.0f;
            SweptFootprintRejection rejected;
            const bool valid=transition(start,direction,turn,next,cost,true,&rejected);
            char context[64];
            std::snprintf(context,sizeof(context),"root_d%d_t%d",direction,turn);
            ROS_WARN("root_action goal_id=%u context=%s valid=%s",goal_id_,context,
                     valid ? "true" : "false");
            if(!valid) logSweptRejection(context,rejected);
        }
    }

    float heuristic(const State& a, const State& b) const {
        grid_map::Position pa, pb;
        map_.getPosition(grid_map::Index(a.x,a.y), pa);
        map_.getPosition(grid_map::Index(b.x,b.y), pb);
        const float distance = static_cast<float>((pa-pb).norm());
        int dt = std::abs(a.t-b.t); dt = std::min(dt, bins_-dt);
        return distance + static_cast<float>(dt * primitive_length_ * 0.25);
    }

    double terrainSpeedScaleAt(double x, double y) const {
        grid_map::Index idx;
        if(!map_.getIndex(grid_map::Position(x,y), idx)) return min_speed_scale_;
        const size_t lin = static_cast<size_t>(idx(0))*cell_cols_ + idx(1);
        double scale = 1.0;
        const float c = cell_cost_[lin];
        // Non-finite is legitimate at the vehicle-occluded start footprint. It must not
        // poison the whole profile, but known terrain is always allowed to slow it down.
        if(std::isfinite(c))
            scale *= std::clamp(1.0 - terrain_speed_gain_*(c/99.0), min_speed_scale_, 1.0);
        const float sm = cell_slopemag_[lin];
        if(std::isfinite(sm) && max_long_slope_ > 1e-3)
            scale *= std::clamp(1.0 - sm/max_long_slope_, min_speed_scale_, 1.0);
        return std::clamp(scale, min_speed_scale_, 1.0);
    }

    // A final mode-independent envelope. Dynamics primitives are individually feasible,
    // but concatenating them can otherwise introduce an instantaneous command jump at an
    // edge. Arc profiles already satisfy these passes; applying them again is idempotent.
    void enforceVelocityEnvelope(const nav_msgs::Path& path,
                                 std_msgs::Float32MultiArray& profile) const {
        const size_t n = path.poses.size();
        if(n < 2 || profile.data.size() != 2*n) return;
        std::vector<double> ds(n-1), dyaw(n-1), raw_v(n), vmag(n), wmag(n);
        std::vector<int> vsign(n,1), wsign(n,1);
        for(size_t i=0; i+1<n; ++i) {
            const auto& a = path.poses[i].pose;
            const auto& b = path.poses[i+1].pose;
            ds[i] = std::hypot(b.position.x-a.position.x, b.position.y-a.position.y);
            dyaw[i] = std::abs(wrap(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation)));
        }
        for(size_t i=0; i<n; ++i) {
            raw_v[i] = profile.data[2*i];
            vsign[i] = std::signbit(raw_v[i]) ? -1 : 1;
            wsign[i] = std::signbit(profile.data[2*i+1]) ? -1 : 1;
            vmag[i] = std::min(std::abs(raw_v[i]), sp_.v_max);
            wmag[i] = std::min(std::abs(static_cast<double>(profile.data[2*i+1])), sp_.w_max);
        }
        vmag.front() = 0.0;
        vmag.back() = 0.0;
        for(size_t i=1; i<n; ++i) {
            if(std::abs(raw_v[i-1]) > 1e-6 && std::abs(raw_v[i]) > 1e-6 &&
               vsign[i-1] != vsign[i]) {
                vmag[i] = 0.0;
            }
        }
        for(size_t i=1; i<n; ++i)
            vmag[i] = std::min(vmag[i], std::sqrt(vmag[i-1]*vmag[i-1] + 2.0*sp_.a_max*ds[i-1]));
        for(size_t i=n-1; i-->0;)
            vmag[i] = std::min(vmag[i], std::sqrt(vmag[i+1]*vmag[i+1] + 2.0*sp_.a_max*ds[i]));

        // When linear acceleration reduces a translating sample, scale yaw rate with it so
        // the primitive curvature is retained before the angular envelope is applied.
        for(size_t i=0; i<n; ++i) {
            if(std::abs(raw_v[i]) > 1e-6)
                wmag[i] *= vmag[i]/std::abs(raw_v[i]);
        }
        for(size_t i=1; i<n; ++i) {
            const double previous_w = profile.data[2*(i-1)+1];
            const double current_w = profile.data[2*i+1];
            if(std::abs(previous_w) > 1e-6 && std::abs(current_w) > 1e-6 &&
               wsign[i-1] != wsign[i]) {
                wmag[i] = 0.0;
            }
        }
        for(size_t i=1; i<n; ++i)
            wmag[i] = std::min(wmag[i], std::sqrt(wmag[i-1]*wmag[i-1] + 2.0*sp_.alpha_max*dyaw[i-1]));
        for(size_t i=n-1; i-->0;)
            wmag[i] = std::min(wmag[i], std::sqrt(wmag[i+1]*wmag[i+1] + 2.0*sp_.alpha_max*dyaw[i]));

        for(size_t i=0; i<n; ++i) {
            profile.data[2*i] = static_cast<float>(vsign[i]*vmag[i]);
            profile.data[2*i+1] = static_cast<float>(wsign[i]*wmag[i]);
        }
    }

    // Arc mode has no offline (v, w) library, so the planner profiles its own path here.
    // The atomic trajectory requires one desired effective body twist per pose. A classic
    // forward/backward trapezoidal pass keeps those values within the shared dynamics
    // envelope before the follower applies inverse slip compensation once.
    void buildVelocityProfile(nav_msgs::Path& path,
                              std_msgs::Float32MultiArray& vel_profile) const {
        // Ideal lattice edges only contain their endpoints. Densify every already
        // collision-checked edge once so the incoming-command convention has a usable
        // non-zero sample between zero-speed boundaries. This is especially important for
        // in-place rotations followed by translation: without an interior yaw sample the
        // angular deceleration pass correctly reduces the shared endpoint to w=0, leaving
        // no feed-forward sample with which to execute the rotation.
        if(path.poses.size() >= 2) {
            std::vector<geometry_msgs::PoseStamped> dense;
            dense.reserve(path.poses.size()*2-1);
            for(size_t i=0; i+1<path.poses.size(); ++i) {
                const auto& first = path.poses[i];
                const auto& last = path.poses[i+1];
                dense.push_back(first);
                const double dx = last.pose.position.x-first.pose.position.x;
                const double dy = last.pose.position.y-first.pose.position.y;
                const double first_yaw = tf2::getYaw(first.pose.orientation);
                const double dyaw = wrap(tf2::getYaw(last.pose.orientation)-first_yaw);
                if(std::hypot(dx,dy) > 1e-3 || std::abs(dyaw) > 1e-3) {
                    geometry_msgs::PoseStamped middle = first;
                    middle.pose.position.x = 0.5*(first.pose.position.x+last.pose.position.x);
                    middle.pose.position.y = 0.5*(first.pose.position.y+last.pose.position.y);
                    middle.pose.position.z = 0.5*(first.pose.position.z+last.pose.position.z);
                    tf2::Quaternion q;
                    q.setRPY(0.0,0.0,wrap(first_yaw+0.5*dyaw));
                    middle.pose.orientation = tf2::toMsg(q);
                    dense.push_back(middle);
                }
            }
            dense.push_back(path.poses.back());
            path.poses.swap(dense);
        }
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
            // Match dynamics-primitives semantics: sample i carries the command that
            // arrives at pose i, so every pose after the first uses its incoming segment.
            const size_t s = i == 0 ? 0 : i-1;
            double lim = sp_.v_max;
            const double k = std::abs(prof_kappa_[s]);
            if(k > 1e-3) lim = std::min(lim, sp_.w_max/k);
            lim *= terrainSpeedScaleAt(path.poses[i].pose.position.x,
                                       path.poses[i].pose.position.y);
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
            const size_t s = i == 0 ? 0 : i-1;
            double w = prof_v_[i]*prof_kappa_[s];
            if(i == 0) {
                w = 0.0;  // same zero-speed boundary as the dynamics branch
            } else if(prof_dir_[s] == 0 && std::abs(prof_dyaw_[s]) > 1e-3) {
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
            const size_t s = i == 0 ? 0 : i-1;
            const float signed_v = prof_dir_[s] < 0 ? -prof_v_[i] : prof_v_[i];
            vel_profile.data[2*i]   = signed_v;
            vel_profile.data[2*i+1] = std::copysign(prof_wmag_[i], prof_w_[i]);
            v_peak = std::max(v_peak, prof_v_[i]);
        }
        enforceVelocityEnvelope(path, vel_profile);
        ROS_INFO_THROTTLE(2.0, "vprofile: n=%zu v_peak=%.2f", n, v_peak);
    }

    // Nudge an unreachable goal onto the nearest pose whose footprint actually validates.
    // 要点13 is about planning close to obstacles: the strict footprint test rejects a goal
    // whose 1.8x1.5m box clips a single unobserved or lethal cell, which happens whenever the
    // operator clicks within about one body half-diagonal (hypot(0.9,0.75)=1.17m) of the edge
    // of the observed region -- even though a pose a few decimetres away is perfectly drivable.
    // This moves the goal; it does NOT relax the collision test. Rings are ordered by distance,
    // so the first ring containing any valid candidate is the best one and the search stops there.
    //
    // Candidates are checked with the footprint inflated by goal_snap_clearance_, because
    // "nearest pose that validates" is by construction a pose on the lethal boundary, and
    // parking there leaves nothing to absorb the two errors that are always present: the
    // hazard map is quantised to one cell (a corner can sit 0.106 m past the last cell centre
    // the check looked at), and the follower stops within goal_yaw_tolerance of the commanded
    // heading (10 deg of yaw slews a corner 0.12 m further out than the yaw that was checked).
    // Measured without the margin: the rover parked 0.13 m inside the (0,-2) boulder having
    // passed its own footprint test. The ordinary 0.25 m trajectory band covers those two
    // terms; the snapped endpoint adds the measured far-to-near map-boundary change, for a
    // 0.50 m default. The margin band tolerates unobserved cells -- see the call site.
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
                        // The body itself must stand on known drivable ground, strictly;
                        // only the clearance band may contain unknown cells.
                        if(!footprintWithClearanceValid(cp.x(), cp.y(), yawForBin(t), cost,
                                                       /*allow_body_unknown=*/false,
                                                       goal_snap_clearance_)) continue;
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
        // The current body cannot be required to retroactively satisfy a clearance margin
        // that a refined map has just moved across it. Known lethal terrain under the
        // physical rectangle is still rejected. On the root's outgoing edge only, grow
        // the extra band to full clearance at its endpoint; requiring the full band at
        // the very first swept sample could leave a safe departure with zero successors.
        if(!footprintValid(sp.x(),sp.y(),yawForBin(start.t),dummy,
                           /*allow_unknown=*/true)) {
            float actual_cost;
            const auto& actual = start_pose.pose;
            const bool actual_valid = footprintValid(actual.position.x, actual.position.y,
                tf2::getYaw(actual.orientation), actual_cost, /*allow_unknown=*/true);
            ROS_WARN_THROTTLE(1.0, "plan: START footprint invalid at (%.2f,%.2f) yaw=%.2f "
                              "(a LETHAL cell or over-limit slope lies under the vehicle; "
                              "unobserved cells are tolerated at the start); goal_id=%u "
                              "actual=(%.3f,%.3f,%.3f) actual_body_valid=%s map_stamp=%.6f",
                              sp.x(), sp.y(), yawForBin(start.t), goal_id_,
                              actual.position.x, actual.position.y, tf2::getYaw(actual.orientation),
                              actual_valid ? "true" : "false", map_stamp_.toSec());
            logFootprintRejection({actual.position.x,actual.position.y,
                                   tf2::getYaw(actual.orientation)},"start_body");
            last_fail_reason_ = "start_footprint";
            return false;
        }
        const auto& actual_start = start_pose.pose;
        if(!sweptSegmentValid(
                {actual_start.position.x, actual_start.position.y,
                 tf2::getYaw(actual_start.orientation)},
                {sp.x(), sp.y(), yawForBin(start.t)},
                0.0, 0.0, /*allow_start_unknown=*/true, dummy)) {
            ROS_WARN_THROTTLE(1.0, "plan: START lattice connector invalid: goal_id=%u "
                              "actual=(%.3f,%.3f,%.3f) lattice=(%.3f,%.3f,%.3f)",
                              goal_id_, actual_start.position.x, actual_start.position.y,
                              tf2::getYaw(actual_start.orientation),
                              sp.x(), sp.y(), yawForBin(start.t));
            last_fail_reason_ = "start_connector";
            return false;
        }
        if(!trajectoryFootprintValid(gp.x(),gp.y(),yawForBin(goal.t),dummy)) {
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
        float reached_error=std::numeric_limits<float>::infinity();
        int expanded=0, root_successors=0;
        while(!open.empty()) {
            if(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>max_planning_time_) break;
            const int ck=open.top().key; open.pop();
            if(closed[ck]) continue;
            closed[ck]=1; ++expanded;
            const State cur=stateFromKey(ck,cols);
            if(cur.t==goal.t) {
                const float error = heuristic(cur,goal);
                // A tolerance hit is a valid fallback, but publishing the first such
                // state makes the endpoint jump by several cells between replans. Near
                // the goal that changed the first segment from forward to reverse on
                // alternate maps and produced a persistent limit cycle. Prefer the exact
                // lattice goal whenever it is reachable; if the time budget expires,
                // retain the closest safe tolerance candidate found so recovery/goal
                // snapping still have their intended escape hatch.
                if(error <= goal_tolerance_ && error < reached_error) {
                    bool endpoint_safe = true;
                    if(snapped_goal_used_) {
                        grid_map::Position candidate;
                        float endpoint_cost;
                        endpoint_safe = map_.getPosition(grid_map::Index(cur.x, cur.y), candidate) &&
                            footprintWithClearanceValid(candidate.x(), candidate.y(),
                                                        yawForBin(cur.t), endpoint_cost,
                                                        /*allow_body_unknown=*/false,
                                                        goal_snap_clearance_);
                    }
                    if(endpoint_safe) {
                        reached=ck;
                        reached_error=error;
                    }
                }
                if(cur.x==goal.x && cur.y==goal.y) {
                    reached=ck;
                    reached_error=0.0f;
                    break;
                }
            }
            if(use_dynamics) {
                grid_map::Position cp;
                if(!map_.getPosition(grid_map::Index(cur.x,cur.y),cp)) continue;
                const double cyaw=yawForBin(cur.t);
                const auto& prims=primitive_lib_.primitivesFor(cur.t);
                for(size_t pi=0; pi<prims.size(); ++pi) {
                    const MotionPrimitive& prim=prims[pi];
                    float terrain; double ex,ey,eyaw;
                    if(!primitiveValid(cp.x(),cp.y(),cyaw,prim,terrain,ex,ey,eyaw,
                                       /*departure=*/ck==sk)) continue;
                    grid_map::Index idx;
                    if(!map_.getIndex(grid_map::Position(ex,ey),idx)) continue;
                    State next{idx(0),idx(1),prim.end_bin};
                    if(next.x==cur.x && next.y==cur.y && next.t==cur.t) continue;
                    // The next primitive starts at the lattice centre/bin, not at the
                    // previous primitive's unquantised last sample. Validate that join.
                    grid_map::Position next_position;
                    float join_cost;
                    if(!map_.getPosition(idx,next_position) ||
                       !sweptSegmentValid({ex,ey,eyaw},
                            {next_position.x(),next_position.y(),yawForBin(next.t)},
                            trajectoryClearance(), trajectoryClearance(), false, join_cost)) continue;
                    if(ck==sk) ++root_successors;
                    const int nk=key(next,cols); if(closed[nk]) continue;
                    const float ec=static_cast<float>(prim.base_cost*(1.0+0.01*terrain));
                    const float ng=g[ck]+ec;
                    if(ng<g[nk]) { g[nk]=ng; parent[nk]=ck; parent_prim[nk]=static_cast<int>(pi);
                        open.push({ng+static_cast<float>(heuristic_weight_)*heuristic(next,goal),nk}); }
                }
            } else {
                for(int direction : {-1,1}) for(int turn=-1;turn<=1;++turn) {
                    State next; float ec;
                    if(!transition(cur,direction,turn,next,ec,/*departure=*/ck==sk)) continue;
                    if(ck==sk) ++root_successors;
                    const int nk=key(next,cols); if(closed[nk]) continue;
                    const float ng=g[ck]+ec;
                    if(ng<g[nk]) { g[nk]=ng; parent[nk]=ck; open.push({ng+static_cast<float>(heuristic_weight_)*heuristic(next,goal),nk}); }
                }
                for(int turn : {-1,1}) {
                    State next; float ec;
                    if(!transition(cur,0,turn,next,ec,/*departure=*/ck==sk)) continue;
                    if(ck==sk) ++root_successors;
                    const int nk=key(next,cols); if(g[ck]+ec<g[nk]) { g[nk]=g[ck]+ec; parent[nk]=ck; open.push({g[nk]+static_cast<float>(heuristic_weight_)*heuristic(next,goal),nk}); }
                }
            }
        }
        if(reached<0) {
            const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
            const bool start_has_band = footprintWithClearanceValid(
                sp.x(),sp.y(),yawForBin(start.t),dummy,true,trajectoryClearance());
            ROS_WARN_THROTTLE(1.0, "plan: search exhausted without reaching goal "
                              "(goal_id=%u mode=%s expanded=%d nodes root_successors=%d "
                              "elapsed=%.3fs goal_bin=%d goal_tol=%.2f "
                              "start_clearance_valid=%s clearance=%.2fm budget_exhausted=%s).",
                              goal_id_, use_dynamics?"dynamics":"arcs", expanded, root_successors,
                              elapsed, goal.t, goal_tolerance_, start_has_band ? "true" : "false",
                              trajectoryClearance(), elapsed >= max_planning_time_ ? "true" : "false");
            last_fail_reason_ = expanded==1 && root_successors==0 ? "start_no_successor"
                                 : (elapsed >= max_planning_time_ ? "search_timeout" : "search_exhausted");
            if(expanded==1 && root_successors==0) {
                logFootprintRejection({sp.x(),sp.y(),yawForBin(start.t)},
                                       "start_clearance",trajectoryClearance());
                if(!use_dynamics) diagnoseRootActions(start);
            }
            return false;
        }
        if(reached_error > 1e-4f) {
            ROS_WARN_THROTTLE(1.0,
                              "plan: exact goal state unavailable within budget; "
                              "using closest tolerance endpoint %.3fm away",
                              reached_error);
        }
        path.header.frame_id=map_frame_; path.header.stamp=ros::Time::now();

        if(use_dynamics) {
            const SkidSteerModel nominal_model(sp_);
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
                    // Primitive files store wheel-command inputs because those commands are
                    // what generated the samples. The public trajectory instead carries the
                    // desired effective body twist, so the follower can apply inverse slip
                    // exactly once. Scale both components together on translating samples to
                    // preserve curvature while applying the same terrain policy as arc mode.
                    BodyTwist desired = nominal_model.effectiveTwist(
                        prim.v_profile[i], prim.w_profile[i]);
                    if(prim.direction != 0) {
                        const double terrain_scale = terrainSpeedScaleAt(wx, wy);
                        desired.vx *= terrain_scale;
                        desired.omega *= terrain_scale;
                    }
                    vel_profile.data.push_back(static_cast<float>(desired.vx));
                    vel_profile.data.push_back(static_cast<float>(desired.omega));
                }
            }
            vel_profile.layout.dim.resize(2);
            vel_profile.layout.dim[0].label="pairs"; vel_profile.layout.dim[0].size=path.poses.size(); vel_profile.layout.dim[0].stride=path.poses.size()*2;
            vel_profile.layout.dim[1].label="vw"; vel_profile.layout.dim[1].size=2; vel_profile.layout.dim[1].stride=2;
            enforceVelocityEnvelope(path, vel_profile);
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

    ros::NodeHandle nh_,pnh_; ros::Subscriber map_sub_,goal_sub_,follower_diag_sub_,odom_sub_;
    ros::Publisher path_pub_,vel_pub_,trajectory_pub_,status_pub_,snapped_goal_pub_,diag_pub_;
    ros::ServiceServer service_; ros::Timer timer_; tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;
    std::mutex mutex_; grid_map::GridMap map_; ros::Time map_stamp_; geometry_msgs::PoseStamped goal_; nav_msgs::Path last_path_;
    bool have_map_=false,have_goal_=false,replan_requested_=false;
    int bins_; double primitive_length_,heuristic_weight_,reverse_cost_,rotation_cost_,max_planning_time_,max_map_age_,goal_tolerance_;
    double footprint_length_,footprint_width_,max_long_slope_,max_lat_slope_;
    std::string map_frame_,base_frame_,odometry_topic_;
    bool use_dynamics_primitives_=false; std::string motion_primitive_file_;
    bool debug_start_rejections_=false;
    uint32_t last_root_debug_goal_=0;
    ros::Time last_root_debug_time_;
    MotionPrimitiveLibrary primitive_lib_;
    std::vector<float> cell_cost_, cell_gx_, cell_gy_, cell_slopemag_; int cell_cols_=0;
    SkidSteerParams sp_;
    double terrain_speed_gain_, min_speed_scale_, reverse_speed_frac_;
    double max_snap_distance_, goal_snap_heading_weight_, goal_snap_cost_weight_;
    double trajectory_clearance_, goal_snap_clearance_;
    int goal_snap_heading_span_;
    bool snapped_goal_used_=false; double last_snap_dist_=0.0;

    PlannerMode mode_ = PlannerMode::Nominal;
    RecoveryAction action_ = RecoveryAction::None;
    std::vector<RecoveryAction> ladder_;
    int recovery_fail_threshold_, recovery_confirm_count_;
    double no_progress_timeout_, progress_epsilon_, recovery_step_timeout_, min_recovery_interval_;
    double recovery_rotate_step_, recovery_backout_distance_, terminal_replan_distance_;
    double execution_goal_pos_tolerance_, execution_goal_yaw_tolerance_;
    double requested_goal_pos_tolerance_;
    bool active_trajectory_reaches_goal_=false, active_trajectory_was_snapped_=false;
    int consecutive_failures_=0, recovery_escalation_=0, confirm_count_=0;
    int recovery_events_=0, recovery_successes_=0, recovery_aborts_=0;
    int goal_events_start_=0, goal_successes_start_=0, goal_aborts_start_=0;
    uint32_t goal_id_=0;
    uint64_t diagnostic_seq_=0;
    std::string last_status_;
    double best_progress_distance_ = std::numeric_limits<double>::infinity();
    double last_plan_ms_ = 0.0;
    double last_cpu_ticks_ = -1.0;
    ros::Time last_cpu_time_;
    ros::Time last_progress_time_, recovery_step_start_, last_recovery_end_, last_diag_time_;
    RetainedTrajectoryCache<nav_msgs::Path, std_msgs::Float32MultiArray> retained_route_;
    ReplanStopBarrier stop_barrier_;
    RecentMotionHistory motion_history_;
    double history_resolution_=0.0;
    bool history_discontinuous_=false;
    BoundedBackout active_backout_, pending_backout_;
    BackoutExecutionLease backout_lease_;
    PostBackoutReplanWindow post_backout_replan_;
    std_msgs::Float32MultiArray backout_profile_;
    ros::Time active_trajectory_map_stamp_;
    std::string last_fail_reason_;
    mutable std::vector<float> prof_ds_, prof_dyaw_, prof_kappa_, prof_v_, prof_w_, prof_wmag_;
    mutable std::vector<double> prof_yaw_;
    mutable std::vector<int> prof_dir_;
};

} // namespace groundgrid

int main(int argc,char** argv){ ros::init(argc,argv,"state_lattice_planner"); groundgrid::StateLatticePlannerNode n; ros::spin(); return 0; }
