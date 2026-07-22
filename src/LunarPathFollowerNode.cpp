#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <limits>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include "groundgrid/SkidSteerModel.h"

namespace groundgrid {

class LunarPathFollowerNode {
public:
    LunarPathFollowerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
        pnh_.param("lookahead", lookahead_, 0.8);
        pnh_.param("max_linear_speed", max_v_, 0.5);
        pnh_.param("max_angular_speed", max_w_, 0.6);
        pnh_.param("goal_position_tolerance", goal_pos_tol_, 0.25);
        pnh_.param("goal_yaw_tolerance", goal_yaw_tol_, 10.0*M_PI/180.0);
        pnh_.param("path_timeout", path_timeout_, 1.0);
        pnh_.param<std::string>("map_frame", map_frame_, "map");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");

        // Slip parameters (loaded globally from the skid_steer_model block) drive the
        // inverse-command feed-forward. Falls back to identity if the block is absent.
        SkidSteerParams sp;
        nh_.param("skid_steer_model/x_icr", sp.x_icr, sp.x_icr);
        nh_.param("skid_steer_model/alpha_v", sp.alpha_v, sp.alpha_v);
        nh_.param("skid_steer_model/alpha_w", sp.alpha_w, sp.alpha_w);
        nh_.param("skid_steer_model/v_max", sp.v_max, std::max(sp.v_max, max_v_));
        nh_.param("skid_steer_model/w_max", sp.w_max, std::max(sp.w_max, max_w_));
        model_.setParams(sp);

        path_sub_ = nh_.subscribe("/lunar_planner/path", 1, &LunarPathFollowerNode::pathCallback, this);
        vel_sub_ = nh_.subscribe("/lunar_planner/velocity_profile", 1, &LunarPathFollowerNode::velCallback, this);
        terrain_sub_ = nh_.subscribe("/terrain/grid_map", 1, &LunarPathFollowerNode::terrainCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        timer_ = nh_.createTimer(ros::Duration(0.05), &LunarPathFollowerNode::control, this);
    }

    ~LunarPathFollowerNode() { stop(); }

private:
    static double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
    void stop() { cmd_pub_.publish(geometry_msgs::Twist()); }

    void pathCallback(const nav_msgs::PathConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        path_=*msg; received_=ros::Time::now();
        if(path_.poses.empty()) stop();
    }

    void velCallback(const std_msgs::Float32MultiArrayConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        vel_profile_=msg->data;
    }

    // Planned linear speed for a path index, when a matching velocity profile is present.
    bool plannedSpeedAt(size_t idx, double& v) const {
        if(vel_profile_.size() != path_.poses.size()*2) return false;
        if(2*idx+1 >= vel_profile_.size()) return false;
        v = vel_profile_[2*idx];
        return true;
    }

    void terrainCallback(const grid_map_msgs::GridMapConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        grid_map::GridMapRosConverter::fromMessage(*msg, terrain_);
        terrain_received_=ros::Time::now();
    }

    void control(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(path_.poses.empty() || terrain_received_.isZero() ||
           (ros::Time::now()-received_).toSec()>path_timeout_ ||
           (ros::Time::now()-terrain_received_).toSec()>path_timeout_) { stop(); return; }
        geometry_msgs::TransformStamped tf;
        try { tf=tf_buffer_.lookupTransform(map_frame_,base_frame_,ros::Time(0),ros::Duration(0.02)); }
        catch(const tf2::TransformException&) { stop(); return; }
        const double x=tf.transform.translation.x, y=tf.transform.translation.y;
        const double yaw=tf2::getYaw(tf.transform.rotation);

        size_t nearest=0; double nearest_d=std::numeric_limits<double>::infinity();
        for(size_t i=0;i<path_.poses.size();++i){
            const double d=std::hypot(path_.poses[i].pose.position.x-x,path_.poses[i].pose.position.y-y);
            if(d<nearest_d){nearest_d=d;nearest=i;}
        }
        const auto& goal=path_.poses.back().pose;
        const double goal_d=std::hypot(goal.position.x-x,goal.position.y-y);
        const double goal_yaw=tf2::getYaw(goal.orientation);
        if(goal_d<goal_pos_tol_ && std::abs(wrap(goal_yaw-yaw))<goal_yaw_tol_){ path_.poses.clear(); stop(); return; }

        size_t target=nearest;
        while(target+1<path_.poses.size() &&
              std::hypot(path_.poses[target].pose.position.x-x,path_.poses[target].pose.position.y-y)<lookahead_) ++target;
        const auto& tp=path_.poses[target].pose;
        const double dx=tp.position.x-x,dy=tp.position.y-y,dist=std::hypot(dx,dy);
        geometry_msgs::Twist cmd;
        if(dist<0.20){
            const double err=wrap(tf2::getYaw(tp.orientation)-yaw);
            cmd.angular.z=std::clamp(1.5*err,-max_w_,max_w_);
        } else {
            const double bearing=std::atan2(dy,dx);
            const bool reverse=std::cos(wrap(bearing-yaw))<0.0;
            const double reference_yaw=reverse?wrap(yaw+M_PI):yaw;
            const double alpha=wrap(bearing-reference_yaw);
            const double curvature=2.0*std::sin(alpha)/std::max(dist,0.1);
            double terrain_scale=1.0;
            grid_map::Index idx;
            if(terrain_.getIndex(grid_map::Position(x,y),idx) && terrain_.exists("slope"))
                terrain_scale=std::clamp(1.0-std::abs(terrain_.at("slope",idx))/25.0,0.25,1.0);
            // Base speed: prefer the planner's dynamics-feasible v-profile as feed-forward;
            // otherwise fall back to the curvature-limited heuristic.
            double base_speed=max_v_*std::clamp(1.0-0.5*std::abs(curvature),0.25,1.0);
            double planned_v;
            if(plannedSpeedAt(target,planned_v) && std::abs(planned_v)>1e-3)
                base_speed=std::min(std::abs(planned_v),max_v_);
            const double v_des=(reverse?-1.0:1.0)*base_speed*terrain_scale;
            const double w_des=std::clamp(std::abs(v_des)*curvature*(reverse?-1.0:1.0),-max_w_,max_w_);
            // Slip compensation: solve for the command that nominally yields (v_des, w_des).
            double v_cmd=v_des, w_cmd=w_des;
            model_.inverseCommand(v_des,w_des,v_cmd,w_cmd);
            cmd.linear.x=v_cmd;
            cmd.angular.z=w_cmd;
        }
        cmd_pub_.publish(cmd);
    }

    ros::NodeHandle nh_,pnh_; ros::Subscriber path_sub_,vel_sub_,terrain_sub_; ros::Publisher cmd_pub_; ros::Timer timer_;
    tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_; std::mutex mutex_;
    nav_msgs::Path path_; std::vector<float> vel_profile_; grid_map::GridMap terrain_;
    ros::Time received_,terrain_received_; std::string map_frame_,base_frame_;
    SkidSteerModel model_;
    double lookahead_,max_v_,max_w_,goal_pos_tol_,goal_yaw_tol_,path_timeout_;
};

} // namespace groundgrid

int main(int argc,char** argv){ros::init(argc,argv,"lunar_path_follower");groundgrid::LunarPathFollowerNode n;ros::spin();return 0;}
