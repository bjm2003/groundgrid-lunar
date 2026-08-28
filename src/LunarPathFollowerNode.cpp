#include <algorithm>
#include <cmath>
#include <limits>
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

namespace groundgrid {

class LunarPathFollowerNode {
public:
    LunarPathFollowerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
        pnh_.param("lookahead", lookahead_, 0.8);
        pnh_.param("max_linear_speed", max_v_, 0.5);
        pnh_.param("max_angular_speed", max_w_, 0.6);
        pnh_.param("goal_position_tolerance", goal_pos_tol_, 0.25);
        pnh_.param("goal_yaw_tolerance", goal_yaw_tol_, 10.0*M_PI/180.0);
        pnh_.param("path_timeout", path_timeout_, 3.0);
        pnh_.param("terrain_timeout", terrain_timeout_, 3.0);
        pnh_.param("angular_feedforward_weight", angular_feedforward_weight_, 0.5);
        pnh_.param<std::string>("map_frame", map_frame_, "map");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");

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
    static double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }

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
            twists_.clear();
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
            twists_.clear();
            stop("invalid_trajectory");
            return;
        }
        path_ = msg->path;
        twists_ = msg->twists;
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

        size_t nearest = 0;
        double nearest_d = std::numeric_limits<double>::infinity();
        for(size_t i=0; i<path_.poses.size(); ++i) {
            const double d = std::hypot(path_.poses[i].pose.position.x-x,
                                        path_.poses[i].pose.position.y-y);
            if(d < nearest_d) {
                nearest_d = d;
                nearest = i;
            }
        }

        const auto& goal = path_.poses.back().pose;
        const double goal_d = std::hypot(goal.position.x-x,goal.position.y-y);
        const double goal_yaw = tf2::getYaw(goal.orientation);
        if(goal_d < goal_pos_tol_ && std::abs(wrap(goal_yaw-yaw)) < goal_yaw_tol_) {
            path_.poses.clear();
            twists_.clear();
            stop("goal_reached");
            return;
        }

        size_t target = nearest;
        while(target+1 < path_.poses.size() &&
              std::hypot(path_.poses[target].pose.position.x-x,
                         path_.poses[target].pose.position.y-y) < lookahead_) {
            ++target;
        }
        const auto& tp = path_.poses[target].pose;
        const double dx = tp.position.x-x;
        const double dy = tp.position.y-y;
        const double dist = std::hypot(dx,dy);

        // The final profile sample is a required zero-speed boundary. While the vehicle is
        // still translating, use the preceding sample and apply the braking envelope below;
        // otherwise an index-based follower would stop one lookahead distance before goal.
        size_t command_index = target;
        if(dist >= 0.20 && command_index+1 == twists_.size() && command_index > 0)
            --command_index;
        double planned_v = twists_[command_index].linear.x;
        const double planned_w = twists_[command_index].angular.z;

        double feedback_w = 0.0;
        if(dist < 0.20) {
            feedback_w = std::clamp(1.5*wrap(tf2::getYaw(tp.orientation)-yaw),
                                    -max_w_, max_w_);
            planned_v = 0.0;
        } else {
            const double bearing = std::atan2(dy,dx);
            bool reverse;
            if(std::abs(planned_v) > 1e-3)
                reverse = planned_v < 0.0;
            else
                reverse = std::cos(wrap(bearing-yaw)) < 0.0;
            const double reference_yaw = reverse ? wrap(yaw+M_PI) : yaw;
            const double alpha = wrap(bearing-reference_yaw);
            const double curvature = 2.0*std::sin(alpha)/std::max(dist,0.1);
            feedback_w = std::clamp(std::abs(planned_v)*curvature*(reverse?-1.0:1.0),
                                    -max_w_, max_w_);
        }

        TrajectoryControlParams control_params;
        control_params.angular_feedforward_weight = angular_feedforward_weight_;
        control_params.max_linear_speed = max_v_;
        control_params.max_angular_speed = max_w_;
        double desired_v = 0.0;
        double desired_w = 0.0;
        if(!blendTrajectoryCommand(planned_v, planned_w, feedback_w, control_params,
                                   desired_v, desired_w)) {
            path_.poses.clear();
            twists_.clear();
            stop("invalid_trajectory");
            return;
        }

        double v_cmd = desired_v;
        double w_cmd = desired_w;
        model_.inverseCommand(desired_v,desired_w,v_cmd,w_cmd);
        if(!std::isfinite(v_cmd) || !std::isfinite(w_cmd)) {
            path_.poses.clear();
            twists_.clear();
            stop("invalid_trajectory");
            return;
        }
        geometry_msgs::Twist cmd;
        cmd.linear.x = v_cmd;
        cmd.angular.z = w_cmd;
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
    std::vector<geometry_msgs::Twist> twists_;
    ros::Time received_,terrain_received_;
    std::string map_frame_,base_frame_,last_status_;
    SkidSteerModel model_;
    double lookahead_,max_v_,max_w_,goal_pos_tol_,goal_yaw_tol_;
    double path_timeout_,terrain_timeout_,angular_feedforward_weight_;
};

} // namespace groundgrid

int main(int argc,char** argv) {
    ros::init(argc,argv,"lunar_path_follower");
    groundgrid::LunarPathFollowerNode node;
    ros::spin();
    return 0;
}
