#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <grid_map_msgs/GridMap.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include "groundgrid/LunarTrajectory.h"
#include "groundgrid/SkidSteerModel.h"
#include "groundgrid/TrajectoryControl.h"
#include "groundgrid/TrajectoryTracking.h"

namespace groundgrid {

class LunarPathFollowerNode {
public:
    LunarPathFollowerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
        pnh_.param("lookahead", lookahead_, 0.8);
        pnh_.param("max_linear_speed", max_v_, 0.5);
        pnh_.param("max_angular_speed", max_w_, 0.6);
        pnh_.param("goal_position_tolerance", goal_pos_tol_, 0.25);
        pnh_.param("goal_yaw_tolerance", goal_yaw_tol_, 10.0*M_PI/180.0);
        pnh_.param("waypoint_arrival_distance", waypoint_arrival_distance_, 0.20);
        pnh_.param("path_timeout", path_timeout_, 3.0);
        pnh_.param("terrain_timeout", terrain_timeout_, 3.0);
        pnh_.param("angular_feedforward_weight", angular_feedforward_weight_, 0.5);
        pnh_.param("debug_control", debug_control_, false);
        pnh_.param<std::string>("map_frame", map_frame_, "map");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");
        waypoint_arrival_distance_ = std::clamp(
            waypoint_arrival_distance_, 0.01, std::max(0.01, goal_pos_tol_));

        // Slip parameters (loaded globally from the skid_steer_model block) drive the
        // one and only inverse-command conversion in the local-navigation pipeline.
        SkidSteerParams sp;
        nh_.param("skid_steer_model/x_icr", sp.x_icr, sp.x_icr);
        nh_.param("skid_steer_model/alpha_v", sp.alpha_v, sp.alpha_v);
        nh_.param("skid_steer_model/alpha_w", sp.alpha_w, sp.alpha_w);
        nh_.param("skid_steer_model/slope_slip_gain", sp.slope_slip_gain, sp.slope_slip_gain);
        nh_.param("skid_steer_model/slope_grade_gain", sp.slope_grade_gain, sp.slope_grade_gain);
        nh_.param("skid_steer_model/v_max", sp.v_max, std::max(sp.v_max, max_v_));
        nh_.param("skid_steer_model/w_max", sp.w_max, std::max(sp.w_max, max_w_));
        nh_.param("skid_steer_model/a_max", sp.a_max, sp.a_max);
        nh_.param("skid_steer_model/alpha_max", sp.alpha_max, sp.alpha_max);
        model_.setParams(sp);

        trajectory_sub_ = nh_.subscribe("/lunar_planner/trajectory", 1,
                                        &LunarPathFollowerNode::trajectoryCallback, this);
        terrain_sub_ = nh_.subscribe("/terrain/grid_map", 1,
                                     &LunarPathFollowerNode::terrainCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        status_pub_ = nh_.advertise<std_msgs::String>("/lunar_path_follower/status", 1, true);
        timer_ = nh_.createTimer(ros::Duration(0.05), &LunarPathFollowerNode::control, this);
    }

    ~LunarPathFollowerNode() { cmd_pub_.publish(geometry_msgs::Twist()); }

private:
    void publishStatus(const std::string& status) {
        if(status == last_status_) return;
        last_status_ = status;
        std_msgs::String msg;
        msg.data = status;
        status_pub_.publish(msg);
    }

    void stop(const std::string& status) {
        cmd_pub_.publish(geometry_msgs::Twist());
        publishStatus(status);
    }

    void trajectoryCallback(const groundgrid::LunarTrajectoryConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        received_ = msg->path.header.stamp.isZero() ? ros::Time::now()
                                                    : msg->path.header.stamp;
        if(msg->path.poses.empty()) {
            path_ = msg->path;
            tracker_.clear();
            stop("empty_trajectory");
            return;
        }
        bool valid = msg->twists.size() == msg->path.poses.size() &&
                     (msg->path.header.frame_id.empty() ||
                      msg->path.header.frame_id == map_frame_);
        if(valid) {
            for(size_t i=0; i<msg->twists.size(); ++i) {
                const auto& pose = msg->path.poses[i].pose;
                const auto& twist = msg->twists[i];
                if(!std::isfinite(pose.position.x) ||
                   !std::isfinite(pose.position.y) ||
                   !std::isfinite(pose.position.z) ||
                   !std::isfinite(pose.orientation.x) ||
                   !std::isfinite(pose.orientation.y) ||
                   !std::isfinite(pose.orientation.z) ||
                   !std::isfinite(pose.orientation.w) ||
                   !std::isfinite(twist.linear.x) ||
                   !std::isfinite(twist.linear.y) ||
                   !std::isfinite(twist.linear.z) ||
                   !std::isfinite(twist.angular.x) ||
                   !std::isfinite(twist.angular.y) ||
                   !std::isfinite(twist.angular.z)) {
                    valid = false;
                    break;
                }
            }
        }
        if(!valid) {
            path_.poses.clear();
            tracker_.clear();
            stop("invalid_trajectory");
            return;
        }
        std::vector<TrackingSample> samples;
        samples.reserve(msg->path.poses.size());
        for(size_t i=0; i<msg->path.poses.size(); ++i) {
            const auto& pose = msg->path.poses[i].pose;
            samples.push_back({{pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)},
                               msg->twists[i].linear.x, msg->twists[i].angular.z});
        }
        bool changed = false;
        if(!tracker_.setTrajectory(std::move(samples), changed)) {
            path_.poses.clear();
            stop("invalid_trajectory");
            return;
        }
        path_ = msg->path;
        if(changed) {
            ++trajectory_id_;
            if(debug_control_) {
                for(size_t i=0; i<tracker_.samples().size(); ++i) {
                    const auto& s = tracker_.samples()[i];
                    ROS_INFO("trajectory_debug id=%zu point=%zu x=%.6f y=%.6f yaw=%.6f v=%.6f w=%.6f",
                             trajectory_id_, i, s.pose.x, s.pose.y, s.pose.yaw, s.v, s.w);
                }
            }
        }
        // Force one fresh tracking status for every accepted atomic trajectory so external
        // tests can scope subsequent stale-stop events to the plan that caused them.
        last_status_.clear();
    }

    void terrainCallback(const grid_map_msgs::GridMapConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        terrain_received_ = msg->info.header.stamp.isZero() ? ros::Time::now()
                                                            : msg->info.header.stamp;
    }

    void control(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        const ros::Time now = ros::Time::now();
        if(path_.poses.empty()) {
            // Preserve the latched terminal reason (goal/invalid/empty) until a new
            // trajectory arrives. Re-labelling it as empty on the next 20 Hz tick would
            // make the more useful state practically unobservable.
            cmd_pub_.publish(geometry_msgs::Twist());
            if(last_status_.empty()) publishStatus("empty_trajectory");
            return;
        }
        if((now-received_).toSec() > path_timeout_) {
            stop("stale_trajectory");
            return;
        }
        if(terrain_received_.isZero() || (now-terrain_received_).toSec() > terrain_timeout_) {
            stop("stale_terrain");
            return;
        }

        geometry_msgs::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform(map_frame_,base_frame_,ros::Time(0),
                                            ros::Duration(0.02));
        } catch(const tf2::TransformException&) {
            stop("tf_unavailable");
            return;
        }
        const double x = tf.transform.translation.x;
        const double y = tf.transform.translation.y;
        const double yaw = tf2::getYaw(tf.transform.rotation);

        TrackingParams params;
        params.lookahead = lookahead_;
        params.waypoint_arrival_distance = waypoint_arrival_distance_;
        params.goal_position_tolerance = goal_pos_tol_;
        params.goal_yaw_tolerance = goal_yaw_tol_;
        params.control.angular_feedforward_weight = angular_feedforward_weight_;
        params.control.max_linear_speed = max_v_;
        params.control.max_angular_speed = max_w_;
        const TrackingStep step = tracker_.step({x,y,yaw}, params);
        if(step.status == TrackingStatus::GoalReached) {
            path_.poses.clear();
            tracker_.clear();
            stop("goal_reached");
            return;
        }
        if(step.status != TrackingStatus::Tracking) {
            ROS_WARN("follow_phase_invalid id=%zu phase=%s begin=%zu end=%zu "
                     "reason=%s x=%.3f y=%.3f yaw=%.3f "
                     "anchor_x=%.3f anchor_y=%.3f anchor_yaw=%.3f "
                     "anchor_d=%.3f arrival=%.3f nearest_d=%.3f",
                     trajectory_id_, motionPhaseName(step.phase), step.phase_begin, step.phase_end,
                     trackingFailureName(step.failure), x,y,yaw,
                     step.phase_endpoint.x, step.phase_endpoint.y, step.phase_endpoint.yaw,
                     step.endpoint_distance, step.arrival_distance, step.nearest_distance);
            path_.poses.clear();
            tracker_.clear();
            stop("invalid_trajectory");
            return;
        }

        double v_cmd = step.desired_v;
        double w_cmd = step.desired_w;
        model_.inverseCommand(step.desired_v,step.desired_w,v_cmd,w_cmd);
        if(!std::isfinite(v_cmd) || !std::isfinite(w_cmd)) {
            path_.poses.clear();
            tracker_.clear();
            stop("invalid_trajectory");
            return;
        }
        geometry_msgs::Twist cmd;
        cmd.linear.x = v_cmd;
        cmd.angular.z = w_cmd;
        if(debug_control_) {
            const auto& target = tracker_.samples()[step.target].pose;
            const auto& goal = tracker_.samples().back().pose;
            ROS_INFO_THROTTLE(
                0.5,
                "follow_debug id=%zu x=%.3f y=%.3f yaw=%.3f goal_x=%.3f goal_y=%.3f "
                "goal_d=%.3f path_n=%zu phase=%s phase_begin=%zu phase_end=%zu "
                "nearest=%zu nearest_d=%.3f target=%zu command=%zu target_d=%.3f "
                "target_x=%.3f target_y=%.3f target_yaw=%.3f capture=%s "
                "planned_v=%.3f planned_w=%.3f feedback_w=%.3f "
                "desired_v=%.3f desired_w=%.3f cmd_v=%.3f cmd_w=%.3f",
                trajectory_id_, x, y, yaw, goal.x, goal.y, std::hypot(goal.x-x,goal.y-y),
                path_.poses.size(), motionPhaseName(step.phase), step.phase_begin, step.phase_end,
                step.nearest, step.nearest_distance, step.target, step.command, step.target_distance,
                target.x, target.y, target.yaw, step.pose_capture ? "true" : "false",
                step.planned_v, step.planned_w, step.feedback_w,
                step.desired_v, step.desired_w, v_cmd, w_cmd);
        }
        cmd_pub_.publish(cmd);
        publishStatus("tracking");
    }

    ros::NodeHandle nh_,pnh_;
    ros::Subscriber trajectory_sub_,terrain_sub_;
    ros::Publisher cmd_pub_,status_pub_;
    ros::Timer timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::mutex mutex_;
    nav_msgs::Path path_;
    TrajectoryTracking tracker_;
    ros::Time received_,terrain_received_;
    std::string map_frame_,base_frame_,last_status_;
    SkidSteerModel model_;
    double lookahead_,max_v_,max_w_,goal_pos_tol_,goal_yaw_tol_;
    double waypoint_arrival_distance_;
    double path_timeout_,terrain_timeout_,angular_feedforward_weight_;
    bool debug_control_;
    size_t trajectory_id_=0;
};

} // namespace groundgrid

int main(int argc,char** argv) {
    ros::init(argc,argv,"lunar_path_follower");
    groundgrid::LunarPathFollowerNode node;
    ros::spin();
    return 0;
}
