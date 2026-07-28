#include <algorithm>
#include <chrono>
#include <cmath>
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

public:
    StateLatticePlannerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
        pnh_.param("heading_bins", bins_, 16);
        pnh_.param("primitive_length", primitive_length_, 0.45);
        pnh_.param("heuristic_weight", heuristic_weight_, 1.2);
        pnh_.param("reverse_cost", reverse_cost_, 1.3);
        pnh_.param("rotation_cost", rotation_cost_, 1.5);
        pnh_.param("max_planning_time", max_planning_time_, 1.0);
        pnh_.param("goal_position_tolerance", goal_tolerance_, 0.30);
        pnh_.param("footprint_length", footprint_length_, 1.8);
        pnh_.param("footprint_width", footprint_width_, 1.5);
        pnh_.param("max_longitudinal_slope_deg", max_long_slope_, 20.0);
        pnh_.param("max_lateral_slope_deg", max_lat_slope_, 15.0);
        pnh_.param<std::string>("map_frame", map_frame_, "map");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");
        pnh_.param("use_dynamics_primitives", use_dynamics_primitives_, false);
        pnh_.param<std::string>("motion_primitive_file", motion_primitive_file_, "");

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

    void mapCallback(const grid_map_msgs::GridMapConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        grid_map::GridMapRosConverter::fromMessage(*msg, map_);
        map_stamp_ = msg->info.header.stamp;
        have_map_ = map_.exists("terrain_cost") && map_.exists("slope_x") && map_.exists("slope_y");
    }

    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_ = *msg; have_goal_ = true; replan_requested_ = true;
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

    void timerCallback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!have_map_ || !have_goal_) return;
        if((ros::Time::now() - map_stamp_).toSec() > 1.0) {
            publishStatus("stale_map"); return;
        }
        if(!replan_requested_ && !last_path_.poses.empty()) replan_requested_ = true;
        if(!replan_requested_) return;
        geometry_msgs::PoseStamped start;
        if(!robotPose(start)) { publishStatus("tf_unavailable"); return; }
        nav_msgs::Path path;
        std_msgs::Float32MultiArray vel;
        if(plan(start, goal_, path, vel)) {
            last_path_ = path; path_pub_.publish(path); vel_pub_.publish(vel); publishStatus("success");
        } else {
            last_path_.poses.clear(); path_pub_.publish(last_path_);
            vel_pub_.publish(std_msgs::Float32MultiArray()); publishStatus("no_path");
        }
        replan_requested_ = false;
    }

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
    bool footprintValid(double x, double y, double yaw, float& cost, bool allow_unknown = false) const {
        cost = 0.0f; int samples = 0;
        const double r = map_.getResolution();
        for(double lx = -footprint_length_/2; lx <= footprint_length_/2 + 1e-6; lx += r) {
            for(double ly = -footprint_width_/2; ly <= footprint_width_/2 + 1e-6; ly += r) {
                const double wx = x + std::cos(yaw)*lx - std::sin(yaw)*ly;
                const double wy = y + std::sin(yaw)*lx + std::cos(yaw)*ly;
                grid_map::Index idx;
                if(!map_.getIndex(grid_map::Position(wx, wy), idx)) {
                    if(allow_unknown) continue;
                    return false;
                }
                const float c = map_.at("terrain_cost", idx);
                if(!std::isfinite(c)) {
                    if(allow_unknown) continue;
                    return false;
                }
                if(c >= 100.0f) return false;
                const float gx = map_.at("slope_x", idx), gy = map_.at("slope_y", idx);
                if(!std::isfinite(gx) || !std::isfinite(gy)) {
                    if(allow_unknown) continue;
                    return false;
                }
                const double longitudinal = std::atan(std::abs(gx*std::cos(yaw) + gy*std::sin(yaw))) * 180.0/M_PI;
                const double lateral = std::atan(std::abs(-gx*std::sin(yaw) + gy*std::cos(yaw))) * 180.0/M_PI;
                if(longitudinal > max_long_slope_ || lateral > max_lat_slope_) return false;
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
        const int n = std::max(2, static_cast<int>(std::ceil(primitive_length_/(map_.getResolution()*0.5))));
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

    bool plan(const geometry_msgs::PoseStamped& start_pose,
              const geometry_msgs::PoseStamped& goal_pose, nav_msgs::Path& path,
              std_msgs::Float32MultiArray& vel_profile) {
        path.poses.clear(); vel_profile.data.clear();
        State start, goal;
        if(!poseToState(start_pose,start)) {
            ROS_WARN_THROTTLE(1.0, "plan: start pose not in map (frame='%s', x=%.2f y=%.2f)",
                              start_pose.header.frame_id.c_str(),
                              start_pose.pose.position.x, start_pose.pose.position.y);
            return false;
        }
        if(!poseToState(goal_pose,goal)) {
            ROS_WARN_THROTTLE(1.0, "plan: goal pose not in map (frame='%s', x=%.2f y=%.2f)",
                              goal_pose.header.frame_id.c_str(),
                              goal_pose.pose.position.x, goal_pose.pose.position.y);
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
            return false;
        }
        if(!footprintValid(gp.x(),gp.y(),yawForBin(goal.t),dummy)) {
            ROS_WARN_THROTTLE(1.0, "plan: GOAL footprint invalid at (%.2f,%.2f) yaw=%.2f "
                              "(pick a goal on clear, observed terrain)",
                              gp.x(), gp.y(), yawForBin(goal.t));
            return false;
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
        const auto begin=std::chrono::steady_clock::now();
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
        return !path.poses.empty();
    }

    ros::NodeHandle nh_,pnh_; ros::Subscriber map_sub_,goal_sub_; ros::Publisher path_pub_,vel_pub_,status_pub_;
    ros::ServiceServer service_; ros::Timer timer_; tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;
    std::mutex mutex_; grid_map::GridMap map_; ros::Time map_stamp_; geometry_msgs::PoseStamped goal_; nav_msgs::Path last_path_;
    bool have_map_=false,have_goal_=false,replan_requested_=false;
    int bins_; double primitive_length_,heuristic_weight_,reverse_cost_,rotation_cost_,max_planning_time_,goal_tolerance_;
    double footprint_length_,footprint_width_,max_long_slope_,max_lat_slope_;
    std::string map_frame_,base_frame_;
    bool use_dynamics_primitives_=false; std::string motion_primitive_file_;
    MotionPrimitiveLibrary primitive_lib_;
};

} // namespace groundgrid

int main(int argc,char** argv){ ros::init(argc,argv,"state_lattice_planner"); groundgrid::StateLatticePlannerNode n; ros::spin(); return 0; }
