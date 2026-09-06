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
#include <stdexcept>
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
#include "groundgrid/LatticePlannerCore.h"
#include "groundgrid/PlanningSnapshot.h"

namespace groundgrid {

class StateLatticePlannerNode : public LatticePlannerCore {
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
        std::string snap_strategy;
        pnh_.param<std::string>("snap_strategy",snap_strategy,"legacy_nearest");
        if(snap_strategy!="legacy_nearest" && snap_strategy!="reachable_cost")
            throw std::runtime_error("snap_strategy must be legacy_nearest or reachable_cost");
        reachable_snap_=snap_strategy=="reachable_cost";
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
        attempt_pub_ = nh_.advertise<std_msgs::String>("/lunar_planner/planning_attempt", 100, false);
        std::string snapshot_directory;
        pnh_.param<std::string>("planning_snapshot_directory",snapshot_directory,"");
        snapshot_writer_.start(snapshot_directory);
        service_ = nh_.advertiseService("/lunar_planner/make_plan", &StateLatticePlannerNode::serviceCallback, this);
        timer_ = nh_.createTimer(ros::Duration(0.5), &StateLatticePlannerNode::timerCallback, this);
    }

private:
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
                 << " attempt_id=" << last_mission_attempt_id_
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
        core_map_.rows=rows; core_map_.cols=cols;
        core_map_.start_row=map_.getStartIndex()(0); core_map_.start_col=map_.getStartIndex()(1);
        core_map_.resolution=map_.getResolution();
        core_map_.length_x=map_.getLength().x(); core_map_.length_y=map_.getLength().y();
        core_map_.center_x=map_.getPosition().x(); core_map_.center_y=map_.getPosition().y();
        const size_t n = static_cast<size_t>(rows) * cols;
        const float nan = std::numeric_limits<float>::quiet_NaN();
        core_map_.cost.assign(n, nan);
        core_map_.gx.assign(n, nan);
        core_map_.gy.assign(n, nan);
        core_map_.slope.assign(n, nan);
        const auto& cost = map_["terrain_cost"];
        const auto& sx = map_["slope_x"];
        const auto& sy = map_["slope_y"];
        for(int i = 0; i < rows; ++i) {
            for(int j = 0; j < cols; ++j) {
                const size_t lin = static_cast<size_t>(i) * cols + j;
                core_map_.cost[lin] = cost(i, j);
                const float gx = sx(i, j), gy = sy(i, j);
                core_map_.gx[lin] = gx; core_map_.gy[lin] = gy;
                if(std::isfinite(gx) && std::isfinite(gy))
                    core_map_.slope[lin] = static_cast<float>(std::atan(std::hypot(gx, gy)) * 180.0 / M_PI);
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
        last_mission_attempt_id_=0;
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
        plan(req.start, req.goal, res.plan, vel, /*query_only=*/true);
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

    // Observability only: report the exact rejected cell and original terrain layers.
    // Ground-truth clearance does not justify overriding a perceived lethal cell. These
    // fields distinguish obstacle/step/slope rejection before considering a map change.
    void logFootprintRejection(const Pose2D& pose,const char* context,
                               double margin=0.0,bool allow_body_unknown=true,
                               bool force=false) const override {
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

    static Pose2D corePose(const geometry_msgs::PoseStamped& pose) {
        const auto& q=pose.pose.orientation;
        const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        if(!std::isfinite(norm) || std::abs(norm-1.0)>1e-3 || !std::isfinite(pose.pose.position.z))
            return {pose.pose.position.x,pose.pose.position.y,std::numeric_limits<double>::quiet_NaN()};
        return {pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation)};
    }
    static void rosProfile(const PlannerProfile& profile,std_msgs::Float32MultiArray& output) {
        output.data=profile.data;
        output.layout.dim.resize(2);
        output.layout.dim[0].label="pairs";
        output.layout.dim[0].size=profile.data.size()/2;
        output.layout.dim[0].stride=profile.data.size();
        output.layout.dim[1].label="vw";
        output.layout.dim[1].size=2; output.layout.dim[1].stride=2;
    }
    static void rosPath(const PlannerPath& path,nav_msgs::Path& output) {
        output.poses.clear();
        for(const auto& p:path.poses) {
            geometry_msgs::PoseStamped pose; pose.header=output.header;
            pose.pose.position.x=p.x; pose.pose.position.y=p.y;
            tf2::Quaternion q; q.setRPY(0,0,p.yaw); pose.pose.orientation=tf2::toMsg(q);
            output.poses.push_back(pose);
        }
    }
    void buildVelocityProfile(nav_msgs::Path& path,std_msgs::Float32MultiArray& profile) const {
        PlannerPath plain; PlannerProfile velocities;
        for(const auto& p:path.poses) plain.poses.push_back(corePose(p));
        LatticePlannerCore::buildVelocityProfile(plain,velocities);
        rosPath(plain,path); rosProfile(velocities,profile);
    }
    bool plan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,
              nav_msgs::Path& path,std_msgs::Float32MultiArray& profile,bool query_only=false) {
        const auto attempt_begin=std::chrono::steady_clock::now();
        path.poses.clear(); profile.data.clear();
        if((!start.header.frame_id.empty() && start.header.frame_id!=map_frame_) ||
           (!goal.header.frame_id.empty() && goal.header.frame_id!=map_frame_)) {
            if(!query_only) last_fail_reason_="invalid_frame";
            return false;
        }
        // Called under the ROS adapter's existing mutex: capture exactly the consumed
        // map/cache and effective Relax parameters, not the latest asynchronous topics.
        PlanningInput input;
        if(snapshot_writer_.enabled() || query_only) input=captureInput(corePose(start),corePose(goal));
        else { input.start=corePose(start); input.goal=corePose(goal); input.config=static_cast<const PlannerConfig&>(*this); }
        input.attempt_id=++attempt_id_;
        input.goal_id=query_only ? 0 : goal_id_;
        input.goal_stamp_ns=goal.header.stamp.toNSec();
        input.start_stamp_ns=start.header.stamp.toNSec(); input.map_stamp_ns=map_stamp_.toNSec();
        input.frame=map_frame_; input.source=query_only ? "service" : "mission";
        PlanningResult result;
        if(query_only) {
            // A service calculation must not change active-goal snapping, failure or
            // recovery state. The same production algorithm runs in an isolated context.
            LatticePlannerCore query(input);
            result=query.planCore(corePose(start),corePose(goal));
        } else {
            result=planCore(corePose(start),corePose(goal));
            if(!result.ok && result.reason=="start_no_successor") {
                State root;
                if(LatticePlannerCore::poseToState(corePose(start),root)) diagnoseRootActions(root);
            }
            if(result.snapped) {
                geometry_msgs::PoseStamped snap; snap.header.frame_id=map_frame_;
                snap.header.stamp=ros::Time::now(); snap.header.seq=goal_id_;
                snap.pose.position.x=result.selected_goal.x; snap.pose.position.y=result.selected_goal.y;
                tf2::Quaternion q; q.setRPY(0,0,result.selected_goal.yaw); snap.pose.orientation=tf2::toMsg(q);
                snapped_goal_pub_.publish(snap);
                ROS_WARN_THROTTLE(1.0,"plan: goal snapped %.2fm to (%.2f,%.2f) yaw=%.2f",
                    result.snap_distance,result.selected_goal.x,result.selected_goal.y,result.selected_goal.yaw);
            }
        }
        path.header.frame_id=map_frame_; path.header.stamp=ros::Time::now();
        rosPath(result.path,path); rosProfile(result.profile,profile);
        result.total_ms=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-attempt_begin).count();
        if(!query_only) { last_plan_ms_=result.total_ms; last_mission_attempt_id_=input.attempt_id; }
        std_msgs::String attempt; attempt.data=planningResultJson(input,result); attempt_pub_.publish(attempt);
        ROS_INFO("planning_attempt %s",attempt.data.c_str());
        if(snapshot_writer_.enabled()) {
            if(!snapshot_writer_.submit(std::move(input),std::move(result)))
                ROS_ERROR("planning snapshot dropped: writer queue full");
            const auto stats=snapshot_writer_.stats();
            if(stats.failed || stats.dropped)
                ROS_ERROR_THROTTLE(1.0,"planning snapshot incomplete: failed=%llu dropped=%llu",
                    static_cast<unsigned long long>(stats.failed),static_cast<unsigned long long>(stats.dropped));
        }
        return !path.poses.empty();
    }

    ros::NodeHandle nh_,pnh_; ros::Subscriber map_sub_,goal_sub_,follower_diag_sub_,odom_sub_;
    ros::Publisher path_pub_,vel_pub_,trajectory_pub_,status_pub_,snapped_goal_pub_,diag_pub_,attempt_pub_;
    PlanningSnapshotWriter snapshot_writer_;
    std::uint64_t attempt_id_=0,last_mission_attempt_id_=0;
    ros::ServiceServer service_; ros::Timer timer_; tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;
    std::mutex mutex_; grid_map::GridMap map_; ros::Time map_stamp_; geometry_msgs::PoseStamped goal_; nav_msgs::Path last_path_;
    bool have_map_=false,have_goal_=false,replan_requested_=false;
    double max_map_age_;
    std::string map_frame_,base_frame_,odometry_topic_;
    std::string motion_primitive_file_;
    bool debug_start_rejections_=false;
    uint32_t last_root_debug_goal_=0;
    ros::Time last_root_debug_time_;

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
};

} // namespace groundgrid

int main(int argc,char** argv){ ros::init(argc,argv,"state_lattice_planner"); groundgrid::StateLatticePlannerNode n; ros::spin(); return 0; }
