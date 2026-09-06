#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <vector>
#include <unordered_map>
#include "groundgrid/PlanningGrid.h"
#include "groundgrid/MotionPrimitiveLibrary.h"
#include "groundgrid/SweptFootprint.h"
#include "groundgrid/FootprintRaster.h"

namespace groundgrid {

inline constexpr double kPlannerPi=3.141592653589793238462643383279502884;

// Names match the pre-extraction ROS parameters internally. No ROS types/clock/IO.
struct PlannerConfig {
    int bins_=16;
    double primitive_length_=0.45;
    double heuristic_weight_=1.2;
    double reverse_cost_=1.3;
    double rotation_cost_=1.5;
    double max_planning_time_=1.0;
    double goal_tolerance_=0.30;
    double footprint_length_=1.8;
    double footprint_width_=1.5;
    double max_long_slope_=20.0;
    double max_lat_slope_=15.0;
    bool use_dynamics_primitives_=false;
    double terrain_speed_gain_=0.6;
    double min_speed_scale_=0.25;
    double reverse_speed_frac_=0.5;
    double max_snap_distance_=1.5;
    int goal_snap_heading_span_=2;
    double goal_snap_heading_weight_=0.25;
    double goal_snap_cost_weight_=0.5;
    double trajectory_clearance_=0.25;
    double goal_snap_clearance_=0.50;
    // Rollout gate: legacy remains the ROS default until Ubuntu input parity/capture.
    bool reachable_snap_=false;
    SkidSteerParams sp_;
    template<class Visitor> void visit(Visitor&& v) {
        v("bins",bins_);
        v("primitive_length",primitive_length_);
        v("heuristic_weight",heuristic_weight_);
        v("reverse_cost",reverse_cost_);
        v("rotation_cost",rotation_cost_);
        v("max_planning_time",max_planning_time_);
        v("goal_tolerance",goal_tolerance_);
        v("footprint_length",footprint_length_);
        v("footprint_width",footprint_width_);
        v("max_long_slope",max_long_slope_);
        v("max_lat_slope",max_lat_slope_);
        v("use_dynamics_primitives",use_dynamics_primitives_);
        v("terrain_speed_gain",terrain_speed_gain_);
        v("min_speed_scale",min_speed_scale_);
        v("reverse_speed_frac",reverse_speed_frac_);
        v("max_snap_distance",max_snap_distance_);
        v("goal_snap_heading_span",goal_snap_heading_span_);
        v("goal_snap_heading_weight",goal_snap_heading_weight_);
        v("goal_snap_cost_weight",goal_snap_cost_weight_);
        v("trajectory_clearance",trajectory_clearance_);
        v("goal_snap_clearance",goal_snap_clearance_);
        v("x_icr",sp_.x_icr);
        v("alpha_v",sp_.alpha_v);
        v("alpha_w",sp_.alpha_w);
        v("slope_slip_gain",sp_.slope_slip_gain);
        v("slope_grade_gain",sp_.slope_grade_gain);
        v("v_max",sp_.v_max);
        v("w_max",sp_.w_max);
        v("a_max",sp_.a_max);
        v("alpha_max",sp_.alpha_max);
        v("kappa_max",sp_.kappa_max);
        v("reachable_snap",reachable_snap_);
    }
    template<class Visitor> void visit(Visitor&& v) const {
        v("bins",bins_);
        v("primitive_length",primitive_length_);
        v("heuristic_weight",heuristic_weight_);
        v("reverse_cost",reverse_cost_);
        v("rotation_cost",rotation_cost_);
        v("max_planning_time",max_planning_time_);
        v("goal_tolerance",goal_tolerance_);
        v("footprint_length",footprint_length_);
        v("footprint_width",footprint_width_);
        v("max_long_slope",max_long_slope_);
        v("max_lat_slope",max_lat_slope_);
        v("use_dynamics_primitives",use_dynamics_primitives_);
        v("terrain_speed_gain",terrain_speed_gain_);
        v("min_speed_scale",min_speed_scale_);
        v("reverse_speed_frac",reverse_speed_frac_);
        v("max_snap_distance",max_snap_distance_);
        v("goal_snap_heading_span",goal_snap_heading_span_);
        v("goal_snap_heading_weight",goal_snap_heading_weight_);
        v("goal_snap_cost_weight",goal_snap_cost_weight_);
        v("trajectory_clearance",trajectory_clearance_);
        v("goal_snap_clearance",goal_snap_clearance_);
        v("x_icr",sp_.x_icr);
        v("alpha_v",sp_.alpha_v);
        v("alpha_w",sp_.alpha_w);
        v("slope_slip_gain",sp_.slope_slip_gain);
        v("slope_grade_gain",sp_.slope_grade_gain);
        v("v_max",sp_.v_max);
        v("w_max",sp_.w_max);
        v("a_max",sp_.a_max);
        v("alpha_max",sp_.alpha_max);
        v("kappa_max",sp_.kappa_max);
        v("reachable_snap",reachable_snap_);
    }
};

struct PlannerPath { std::vector<Pose2D> poses; };
struct PlannerProfile { std::vector<float> data; };
struct PlanningInput {
    PlannerConfig config;
    PlanningGrid map;
    MotionPrimitiveLibrary primitives;
    Pose2D start, goal;
    std::uint64_t attempt_id=0, goal_id=0, goal_stamp_ns=0, start_stamp_ns=0, map_stamp_ns=0;
    std::string frame, source="mission";
};
struct PlanningResult {
    bool ok=false, snapped=false;
    bool selected_goal_valid=false;
    std::string reason;
    Pose2D selected_goal;
    double snap_distance=0.0, total_ms=0.0, snap_ms=0.0, search_ms=0.0, profile_ms=0.0;
    double path_length=0.0, reverse_length=0.0, route_cost=0.0;
    int expanded=0, root_successors=0;
    int candidates_checked=0;
    int candidate_cache_hits=0;
    std::size_t departure_end_index=0;
    bool budget_exhausted=false;
    PlannerPath path;
    PlannerProfile profile;
};

class LatticePlannerCore : public PlannerConfig {
public:
    LatticePlannerCore() = default;
    explicit LatticePlannerCore(const PlanningInput& input) {
        static_cast<PlannerConfig&>(*this)=input.config;
        core_map_=input.map;
        primitive_lib_=input.primitives;
    }
    virtual ~LatticePlannerCore() = default;
    PlanningInput captureInput(const Pose2D& start,const Pose2D& goal) const {
        PlanningInput input;
        input.config=static_cast<const PlannerConfig&>(*this);
        input.map=core_map_;
        input.primitives=primitive_lib_;
        input.start=start; input.goal=goal;
        return input;
    }
    PlanningResult planCore(const Pose2D& start,const Pose2D& goal,
                            std::uint64_t expansion_limit=0) {
        const auto begin=std::chrono::steady_clock::now();
        result_=PlanningResult{};
        result_.selected_goal=goal;
        last_fail_reason_.clear();
        expansion_limit_=expansion_limit;
        snapped_goal_used_=false; last_snap_dist_=0.0;
        if(!validInput(start,goal)) last_fail_reason_="invalid_input";
        else result_.ok=planImpl(start,goal,result_.path,result_.profile,begin);
        result_.total_ms=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-begin).count();
        result_.reason=last_fail_reason_;
        result_.snapped=snapped_goal_used_ && result_.selected_goal_valid;
        if(!result_.selected_goal_valid) snapped_goal_used_=false;
        result_.snap_distance=last_snap_dist_;
        if(!result_.ok) { result_.path.poses.clear(); result_.profile.data.clear(); }
        for(std::size_t i=1;i<result_.path.poses.size();++i) {
            const auto& a=result_.path.poses[i-1]; const auto& b=result_.path.poses[i];
            const double dx=b.x-a.x,dy=b.y-a.y,ds=std::hypot(dx,dy);
            result_.path_length+=ds;
            if(dx*std::cos(a.yaw)+dy*std::sin(a.yaw)<0.0) result_.reverse_length+=ds;
        }
        return result_;
    }
    bool validInput(const Pose2D& start,const Pose2D& goal) const {
        if(!core_map_.valid() || !std::isfinite(start.x) || !std::isfinite(start.y) ||
           !std::isfinite(start.yaw) || !std::isfinite(goal.x) || !std::isfinite(goal.y) ||
           !std::isfinite(goal.yaw)) return false;
        bool finite=true;
        visit([&](const char*,auto v){ finite=finite && std::isfinite(double(v)); });
        return finite && bins_>=4 && bins_<=128 &&
               std::uint64_t(core_map_.rows)*core_map_.cols*bins_<=32000000 &&
               primitive_length_>0.0 && max_planning_time_>0.0 &&
               heuristic_weight_>0.0 && reverse_cost_>0.0 && rotation_cost_>0.0 &&
               goal_tolerance_>=0.0 && footprint_length_>0.0 && footprint_width_>0.0 &&
               max_long_slope_>0.0 && max_lat_slope_>0.0 &&
               terrain_speed_gain_>=0.0 && min_speed_scale_>0.0 && min_speed_scale_<=1.0 &&
               reverse_speed_frac_>0.0 && reverse_speed_frac_<=1.0 &&
               max_snap_distance_>=0.0 && goal_snap_heading_span_>=0 && goal_snap_heading_span_<bins_ &&
               goal_snap_heading_weight_>=0.0 && goal_snap_cost_weight_>=0.0 &&
               trajectory_clearance_>=0.0 && goal_snap_clearance_>=trajectory_clearance_ &&
               sp_.v_max>0.0 && sp_.w_max>0.0 && sp_.a_max>0.0 && sp_.alpha_max>0.0 &&
               (!use_dynamics_primitives_ || (!primitive_lib_.empty() && primitive_lib_.headingBins()==bins_));
    }
    struct State { int x,y,t; };
    struct FootprintRejection {
        const char* reason="none";
        double sample_x=0.0,sample_y=0.0;
        int row=-1,col=-1;
    };
    struct QueueNode {
        float f; int key;
        bool operator<(const QueueNode& other) const { return f>other.f; }
    };
    static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

    int key(const State& s, int cols) const { return (s.x * cols + s.y) * bins_ + s.t; }

    State stateFromKey(int k, int cols) const {
        State s; s.t = k % bins_; k /= bins_; s.y = k % cols; s.x = k / cols; return s;
    }

    double yawForBin(int t) const { return t * 2.0 * kPlannerPi / bins_; }

    int binForYaw(double yaw) const {
        int b = static_cast<int>(std::lround(wrap(yaw) * bins_ / (2.0 * kPlannerPi)));
        b %= bins_; if(b < 0) b += bins_; return b;
    }

    bool poseToState(const Pose2D& pose, State& state) const {
        PlanningIndex idx;
        if(!core_map_.getIndex(PlanningPosition(pose.x, pose.y), idx)) return false;
        state.x = idx(0); state.y = idx(1);
        state.t = binForYaw(pose.yaw);
        return true;
    }

    bool footprintValid(double x, double y, double yaw, float& cost, bool allow_unknown = false,
                        double margin = 0.0, FootprintRejection* rejection = nullptr) const {
        if(rejection) *rejection=FootprintRejection{};
        const auto reject = [rejection](const char* reason,double wx,double wy,int row,int col) {
            if(rejection) *rejection={reason,wx,wy,row,col};
            return false;
        };
        cost = 0.0f; int samples = 0;
        const double r = core_map_.getResolution();
        const double cyaw = std::cos(yaw), syaw = std::sin(yaw);
        const double origin_x=core_map_.getPosition().x()-0.5*core_map_.getLength().x();
        const double origin_y=core_map_.getPosition().y()-0.5*core_map_.getLength().y();
        const bool valid=visitFootprintCells({x,y,yaw},footprint_length_,footprint_width_,margin,
            r,origin_x,origin_y,[&](double wx,double wy) {
                PlanningIndex idx;
                if(!core_map_.getIndex(PlanningPosition(wx, wy), idx)) {
                    if(allow_unknown) return true;
                    return reject("off_map",wx,wy,-1,-1);
                }
                const size_t lin = static_cast<size_t>(idx(0)) * core_map_.cols + idx(1);
                const float c = core_map_.cost[lin];
                if(!std::isfinite(c)) {
                    if(allow_unknown) return true;
                    return reject("unknown_cost",wx,wy,idx(0),idx(1));
                }
                if(c >= 100.0f) return reject("lethal_cost",wx,wy,idx(0),idx(1));
                const float sm = core_map_.slope[lin];
                if(!std::isfinite(sm)) {
                    if(allow_unknown) return true;
                    return reject("unknown_slope",wx,wy,idx(0),idx(1));
                }
                if(sm > max_lat_slope_) {  // steep cell: fall back to the directional check
                    const float gx = core_map_.gx[lin], gy = core_map_.gy[lin];
                    const double longitudinal = std::atan(std::abs(gx*cyaw + gy*syaw)) * 180.0/kPlannerPi;
                    const double lateral = std::atan(std::abs(-gx*syaw + gy*cyaw)) * 180.0/kPlannerPi;
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
        return sweptFootprintValid(from, to, cornerRadius(), core_map_.getResolution(),
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

    double cornerRadius() const {
        return std::hypot(footprint_length_, footprint_width_) / 2.0;
    }

    bool rotationValid(double x, double y, double yaw0, double yaw1, float& cost,
                       bool departure = false, SweptFootprintRejection* rejection=nullptr) const {
        return sweptSegmentValid({x,y,yaw0}, {x,y,yaw1},
                                 departure ? 0.0 : trajectoryClearance(),
                                 trajectoryClearance(), departure, cost, nullptr, rejection);
    }

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
            cornerRadius(),core_map_.getResolution(),trajectoryClearance(),departure,
            [this](const Pose2D& pose,double clearance,bool allow_unknown,float& cost) {
                return footprintWithClearanceValid(pose.x,pose.y,pose.yaw,
                                                    cost,allow_unknown,clearance);
            },avg_cost);
    }

    bool transition(const State& from, int direction, int turn, State& to, float& edge_cost,
                    bool departure = false, SweptFootprintRejection* rejection=nullptr) const {
        if(rejection) *rejection=SweptFootprintRejection{};
        PlanningPosition p;
        if(!core_map_.getPosition(PlanningIndex(from.x, from.y), p)) return false;
        const double yaw0 = yawForBin(from.t);
        if(direction == 0) {
            to = from; to.t = (from.t + turn + bins_) % bins_;
            float terrain;
            if(!rotationValid(p.x(), p.y(), yaw0, yawForBin(to.t), terrain, departure,rejection)) return false;
            edge_cost = static_cast<float>(rotation_cost_ * primitive_length_ + terrain * 0.002);
            return true;
        }

        const double dyaw = turn * 2.0 * kPlannerPi / bins_;
        const double yaw1 = yaw0 + dyaw;
        const double x1 = p.x() + direction*primitive_length_*std::cos(yaw0 + dyaw*0.5);
        const double y1 = p.y() + direction*primitive_length_*std::sin(yaw0 + dyaw*0.5);
        PlanningIndex idx;
        if(!core_map_.getIndex(PlanningPosition(x1,y1), idx)) return false;
        to = {idx(0), idx(1), binForYaw(yaw1)};
        PlanningPosition endpoint;
        float terrain;
        if(!core_map_.getPosition(idx, endpoint) ||
           !latticeArcFootprintValid({p.x(),p.y(),yaw0},
                {endpoint.x(),endpoint.y(),yawForBin(to.t)}, direction*primitive_length_,dyaw,
                cornerRadius(),core_map_.getResolution(),trajectoryClearance(),departure,
                [this](const Pose2D& pose, double clearance, bool allow_unknown, float& cost) {
                    return footprintWithClearanceValid(pose.x,pose.y,pose.yaw,
                                                        cost,allow_unknown,clearance);
                },terrain,rejection)) return false;
        const double motion_factor = direction < 0 ? reverse_cost_ : 1.0;
        edge_cost = static_cast<float>(motion_factor * primitive_length_ *
                    (1.0 + 0.01 * terrain));
        return !(to.x == from.x && to.y == from.y && to.t == from.t);
    }

    float heuristic(const State& a, const State& b) const {
        PlanningPosition pa, pb;
        core_map_.getPosition(PlanningIndex(a.x,a.y), pa);
        core_map_.getPosition(PlanningIndex(b.x,b.y), pb);
        const float distance = static_cast<float>((pa-pb).norm());
        int dt = std::abs(a.t-b.t); dt = std::min(dt, bins_-dt);
        return distance + static_cast<float>(dt * primitive_length_ * 0.25);
    }

    double terrainSpeedScaleAt(double x, double y) const {
        PlanningIndex idx;
        if(!core_map_.getIndex(PlanningPosition(x,y), idx)) return min_speed_scale_;
        const size_t lin = static_cast<size_t>(idx(0))*core_map_.cols + idx(1);
        double scale = 1.0;
        const float c = core_map_.cost[lin];
        // Non-finite is legitimate at the vehicle-occluded start footprint. It must not
        // poison the whole profile, but known terrain is always allowed to slow it down.
        if(std::isfinite(c))
            scale *= std::clamp(1.0 - terrain_speed_gain_*(c/99.0), min_speed_scale_, 1.0);
        const float sm = core_map_.slope[lin];
        if(std::isfinite(sm) && max_long_slope_ > 1e-3)
            scale *= std::clamp(1.0 - sm/max_long_slope_, min_speed_scale_, 1.0);
        return std::clamp(scale, min_speed_scale_, 1.0);
    }

    void enforceVelocityEnvelope(const PlannerPath& path,
                                 PlannerProfile& profile) const {
        const size_t n = path.poses.size();
        if(n < 2 || profile.data.size() != 2*n) return;
        std::vector<double> ds(n-1), dyaw(n-1), raw_v(n), vmag(n), wmag(n);
        std::vector<int> vsign(n,1), wsign(n,1);
        for(size_t i=0; i+1<n; ++i) {
            const auto& a = path.poses[i];
            const auto& b = path.poses[i+1];
            ds[i] = std::hypot(b.x-a.x, b.y-a.y);
            dyaw[i] = std::abs(wrap(b.yaw-a.yaw));
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

    void buildVelocityProfile(PlannerPath& path,
                              PlannerProfile& vel_profile) const {
        // Ideal lattice edges only contain their endpoints. Densify every already
        // collision-checked edge once so the incoming-command convention has a usable
        // non-zero sample between zero-speed boundaries. This is especially important for
        // in-place rotations followed by translation: without an interior yaw sample the
        // angular deceleration pass correctly reduces the shared endpoint to w=0, leaving
        // no feed-forward sample with which to execute the rotation.
        if(path.poses.size() >= 2) {
            std::vector<Pose2D> dense;
            dense.reserve(path.poses.size()*2-1);
            for(size_t i=0; i+1<path.poses.size(); ++i) {
                const auto& first = path.poses[i];
                const auto& last = path.poses[i+1];
                dense.push_back(first);
                const double dx = last.x-first.x;
                const double dy = last.y-first.y;
                const double first_yaw = first.yaw;
                const double dyaw = wrap(last.yaw-first_yaw);
                if(std::hypot(dx,dy) > 1e-3 || std::abs(dyaw) > 1e-3) {
                    Pose2D middle = first;
                    middle.x = 0.5*(first.x+last.x);
                    middle.y = 0.5*(first.y+last.y);
                    middle.yaw = wrap(first_yaw+0.5*dyaw);
                    dense.push_back(middle);
                }
            }
            dense.push_back(path.poses.back());
            path.poses.swap(dense);
        }
        const size_t n = path.poses.size();
        vel_profile.data.assign(2*n, 0.0f);
        if(n < 2) return;

        const size_t segs = n - 1;
        prof_ds_.resize(segs); prof_dyaw_.resize(segs); prof_kappa_.resize(segs);
        prof_dir_.resize(segs); prof_v_.resize(n); prof_w_.resize(n);
        prof_yaw_.resize(n); prof_wmag_.resize(n);

        for(size_t i=0;i<n;++i) prof_yaw_[i] = path.poses[i].yaw;
        for(size_t i=0;i<segs;++i) {
            const double dx = path.poses[i+1].x - path.poses[i].x;
            const double dy = path.poses[i+1].y - path.poses[i].y;
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
            lim *= terrainSpeedScaleAt(path.poses[i].x,
                                       path.poses[i].y);
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

        for(size_t i=0;i<n;++i) {
            const size_t s = i == 0 ? 0 : i-1;
            const float signed_v = prof_dir_[s] < 0 ? -prof_v_[i] : prof_v_[i];
            vel_profile.data[2*i]   = signed_v;
            vel_profile.data[2*i+1] = std::copysign(prof_wmag_[i], prof_w_[i]);
        }
        enforceVelocityEnvelope(path, vel_profile);
    }

    bool snapGoal(const State& requested, double max_distance, double budget_s,
                  const std::chrono::steady_clock::time_point& begin,
                  State& snapped, double& snap_dist) const {
        PlanningPosition rp;
        if(!core_map_.getPosition(PlanningIndex(requested.x, requested.y), rp)) return false;
        const double res = core_map_.getResolution();
        const int max_ring = std::max(1, static_cast<int>(std::ceil(max_distance/res)));
        const double heading_step = 2.0*kPlannerPi/bins_;

        for(int r = 1; r <= max_ring; ++r) {
            if(expansion_limit_==0 && std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count() > budget_s)
                return false;
            double best_score = std::numeric_limits<double>::infinity();
            bool found = false;
            for(int dx = -r; dx <= r; ++dx) {
                for(int dy = -r; dy <= r; ++dy) {
                    if(std::max(std::abs(dx), std::abs(dy)) != r) continue;   // perimeter only
                    // Stepped in world space, not index space: grid_map is a circular buffer,
                    // so index arithmetic wraps to the wrong cell at the buffer seam.
                    PlanningIndex idx;
                    if(!core_map_.getIndex(PlanningPosition(rp.x()+dx*res, rp.y()+dy*res), idx)) continue;
                    PlanningPosition cp;
                    if(!core_map_.getPosition(idx, cp)) continue;
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

    struct CandidateCheck { bool valid=false; float terrain=0.0f; };
    using CandidateCache=std::unordered_map<int,CandidateCheck>;

    // The cache belongs to ONE planImpl call, so no map/start/clearance change can
    // reuse an old certificate. Store only safety/terrain; request-dependent penalties
    // are always recomputed. A cheap geometric heuristic is never a safety certificate.
    bool snapCandidate(const State& candidate,const State& requested,
                       double& penalty,double& distance,CandidateCache* cache=nullptr) {
        int db=std::abs(candidate.t-requested.t); db=std::min(db,bins_-db);
        if(db>goal_snap_heading_span_) return false;
        PlanningPosition p,r;
        if(!core_map_.getPosition({candidate.x,candidate.y},p) ||
           !core_map_.getPosition({requested.x,requested.y},r)) return false;
        distance=(p-r).norm();
        if(distance>max_snap_distance_) return false;
        CandidateCheck evaluation;
        const int candidate_key=key(candidate,core_map_.cols);
        const auto cached=cache ? cache->find(candidate_key) : CandidateCache::iterator{};
        if(cache && cached!=cache->end()) {
            ++result_.candidate_cache_hits;
            evaluation=cached->second;
        } else {
            ++result_.candidates_checked;
            evaluation.valid=footprintWithClearanceValid(p.x(),p.y(),yawForBin(candidate.t),
                                                        evaluation.terrain,false,goal_snap_clearance_);
            if(cache) cache->emplace(candidate_key,evaluation);
        }
        if(!evaluation.valid) return false;
        penalty=distance+goal_snap_heading_weight_*db*(2.0*kPlannerPi/bins_)*primitive_length_
                        +goal_snap_cost_weight_*(evaluation.terrain/99.0);
        return std::isfinite(penalty);
    }

    bool hasSnapCandidate(const State& requested,CandidateCache& cache,
                          const std::chrono::steady_clock::time_point& begin) {
        const int rows=core_map_.rows,cols=core_map_.cols;
        const int ur=(requested.x-core_map_.start_row+rows)%rows;
        const int uc=(requested.y-core_map_.start_col+cols)%cols;
        const int max_ring=static_cast<int>(std::min(double(std::max(rows,cols)),
                              std::ceil(max_snap_distance_/core_map_.resolution)));
        // Existence witness only, not a chosen endpoint. Stop at the first safe pose;
        // the later shared search can still choose ANY safe candidate, reachable or not
        // from this witness's side. Exhausting an empty goal set avoids an entire futile
        // A* budget. Enumeration uses logical cells, not rounded world-to-buffer steps.
        for(int ring=0;ring<=max_ring;++ring) {
            for(int dr=-ring;dr<=ring;++dr) for(int dc=-ring;dc<=ring;++dc) {
                // Include enumeration overhead in the one shared clock, including cells
                // outside a rectangular map. No second per-candidate time allowance.
                if(expansion_limit_==0 &&
                   std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()
                       >max_planning_time_) {
                    result_.budget_exhausted=true;
                    return false;
                }
                if(std::max(std::abs(dr),std::abs(dc))!=ring ||
                   ur+dr<0 || ur+dr>=rows || uc+dc<0 || uc+dc>=cols) continue;
                for(int db=-goal_snap_heading_span_;db<=goal_snap_heading_span_;++db) {
                    if(expansion_limit_==0 &&
                       std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()
                           >max_planning_time_) {
                        result_.budget_exhausted=true;
                        return false;
                    }
                    State candidate{(ur+dr+core_map_.start_row)%rows,
                                    (uc+dc+core_map_.start_col)%cols,
                                    (requested.t+db+bins_)%bins_};
                    double penalty=0.0,distance=0.0;
                    if(snapCandidate(candidate,requested,penalty,distance,&cache)) return true;
                }
            }
        }
        return false;
    }

    float goalSetHeuristic(const State& state,const State& requested) const {
        PlanningPosition p,r;
        core_map_.getPosition({state.x,state.y},p); core_map_.getPosition({requested.x,requested.y},r);
        // Route distance PLUS endpoint displacement has a geometric lower estimate
        // given by distance to the request (triangle inequality). Subtracting the disc
        // radius would omit that endpoint cost and flood the frontier inside the disc.
        // This never chooses a single endpoint and never replaces the safety checker.
        return static_cast<float>((p-r).norm());
    }

    bool planImpl(const Pose2D& start_pose,
                  const Pose2D& goal_pose, PlannerPath& path,
                  PlannerProfile& vel_profile,
                  const std::chrono::steady_clock::time_point& begin) {
        path.poses.clear(); vel_profile.data.clear();
        snapped_goal_used_ = false; last_snap_dist_ = 0.0;
        State start, goal;
        if(!poseToState(start_pose,start)) {
            last_fail_reason_ = "start_off_map";
            return false;
        }
        if(!poseToState(goal_pose,goal)) {
            last_fail_reason_ = "goal_off_map";
            return false;
        }
        float dummy;
        PlanningPosition sp, gp;
        core_map_.getPosition(PlanningIndex(start.x,start.y),sp);
        core_map_.getPosition(PlanningIndex(goal.x,goal.y),gp);
        result_.selected_goal={gp.x(),gp.y(),yawForBin(goal.t)};
        result_.selected_goal_valid=true;
        // The current body cannot be required to retroactively satisfy a clearance margin
        // that a refined map has just moved across it. Known lethal terrain under the
        // physical rectangle is still rejected. On the root's outgoing edge only, grow
        // the extra band to full clearance at its endpoint; requiring the full band at
        // the very first swept sample could leave a safe departure with zero successors.
        if(!footprintValid(sp.x(),sp.y(),yawForBin(start.t),dummy,
                           /*allow_unknown=*/true)) {
            last_fail_reason_ = "start_footprint";
            return false;
        }
        const auto& actual_start = start_pose;
        if(!sweptSegmentValid(
                {actual_start.x, actual_start.y,
                 actual_start.yaw},
                {sp.x(), sp.y(), yawForBin(start.t)},
                0.0, 0.0, /*allow_start_unknown=*/true, dummy)) {
            last_fail_reason_ = "start_connector";
            return false;
        }
        const State requested=goal;
        const bool invalid_goal=!trajectoryFootprintValid(gp.x(),gp.y(),yawForBin(goal.t),dummy);
        const bool multi_goal=reachable_snap_ && invalid_goal;
        CandidateCache candidate_cache;
        if(invalid_goal) result_.selected_goal_valid=false;
        if(multi_goal) {
            // The entire snapped route holds the same larger margin as legacy snapping.
            snapped_goal_used_=true;
            if(!hasSnapCandidate(requested,candidate_cache,begin)) {
                result_.snap_ms=std::chrono::duration<double,std::milli>(
                    std::chrono::steady_clock::now()-begin).count();
                last_fail_reason_=result_.budget_exhausted ? "snap_timeout" : "goal_invalid";
                return false;
            }
        } else if(invalid_goal) {
            State snapped; double snap_dist;
            if(!snapGoal(goal, max_snap_distance_, max_planning_time_*0.2, begin, snapped, snap_dist)) {
                last_fail_reason_ = "goal_invalid";
                return false;
            }
            goal = snapped;
            core_map_.getPosition(PlanningIndex(goal.x,goal.y),gp);
            snapped_goal_used_ = true; last_snap_dist_ = snap_dist;
            result_.selected_goal={gp.x(),gp.y(),yawForBin(goal.t)};
            result_.selected_goal_valid=true;
        }

        result_.snap_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
        const auto search_begin=std::chrono::steady_clock::now();
        const bool use_dynamics = use_dynamics_primitives_ && !primitive_lib_.empty();
        const int rows=core_map_.getSize()(0), cols=core_map_.getSize()(1), count=rows*cols*bins_;
        std::vector<float> g(count,std::numeric_limits<float>::infinity());
        std::vector<int> parent(count,-1);
        std::vector<int> parent_prim(count,-1);
        std::vector<uint8_t> closed(count,0);
        std::priority_queue<QueueNode> open;
        const int sk=key(start,cols);
        const auto estimate=[&](const State& s) {
            return multi_goal ? goalSetHeuristic(s,requested) : heuristic(s,goal);
        };
        g[sk]=0.0f; open.push({static_cast<float>(heuristic_weight_)*estimate(start),sk});
        int reached=-1;
        float reached_error=std::numeric_limits<float>::infinity();
        double best_goal_score=std::numeric_limits<double>::infinity();
        int expanded=0, root_successors=0;
        while(!open.empty()) {
            if(expansion_limit_ ? std::uint64_t(expanded)>=expansion_limit_ :
               std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>max_planning_time_) {
                result_.budget_exhausted=true; break;
            }
            const int ck=open.top().key; open.pop();
            // Virtual sink edges carry the existing endpoint penalty. Every candidate
            // competes in ONE frontier/budget. This is bounded weighted A*, not a claim
            // of globally optimal execution time or of an unreachable-world proof.
            if(ck==-1) break;
            if(closed[ck]) continue;
            closed[ck]=1; ++expanded;
            const State cur=stateFromKey(ck,cols);
            if(multi_goal) {
                double penalty=0.0,distance=0.0;
                if(snapCandidate(cur,requested,penalty,distance,&candidate_cache) && g[ck]+penalty<best_goal_score) {
                    best_goal_score=g[ck]+penalty;
                    reached=ck; goal=cur; last_snap_dist_=distance;
                    PlanningPosition selected; core_map_.getPosition({cur.x,cur.y},selected);
                    result_.selected_goal={selected.x(),selected.y(),yawForBin(cur.t)};
                    result_.selected_goal_valid=true;
                    open.push({static_cast<float>(best_goal_score),-1});
                }
            } else if(cur.t==goal.t) {
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
                        PlanningPosition candidate;
                        float endpoint_cost;
                        endpoint_safe = core_map_.getPosition(PlanningIndex(cur.x, cur.y), candidate) &&
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
                PlanningPosition cp;
                if(!core_map_.getPosition(PlanningIndex(cur.x,cur.y),cp)) continue;
                const double cyaw=yawForBin(cur.t);
                const auto& prims=primitive_lib_.primitivesFor(cur.t);
                for(size_t pi=0; pi<prims.size(); ++pi) {
                    const MotionPrimitive& prim=prims[pi];
                    float terrain; double ex,ey,eyaw;
                    if(!primitiveValid(cp.x(),cp.y(),cyaw,prim,terrain,ex,ey,eyaw,
                                       /*departure=*/ck==sk)) continue;
                    PlanningIndex idx;
                    if(!core_map_.getIndex(PlanningPosition(ex,ey),idx)) continue;
                    State next{idx(0),idx(1),prim.end_bin};
                    if(next.x==cur.x && next.y==cur.y && next.t==cur.t) continue;
                    // The next primitive starts at the lattice centre/bin, not at the
                    // previous primitive's unquantised last sample. Validate that join.
                    PlanningPosition next_position;
                    float join_cost;
                    if(!core_map_.getPosition(idx,next_position) ||
                       !sweptSegmentValid({ex,ey,eyaw},
                            {next_position.x(),next_position.y(),yawForBin(next.t)},
                            trajectoryClearance(), trajectoryClearance(), false, join_cost)) continue;
                    if(ck==sk) ++root_successors;
                    const int nk=key(next,cols); if(closed[nk]) continue;
                    const float ec=static_cast<float>(prim.base_cost*(1.0+0.01*terrain));
                    const float ng=g[ck]+ec;
                    if(ng<g[nk]) { g[nk]=ng; parent[nk]=ck; parent_prim[nk]=static_cast<int>(pi);
                        open.push({ng+static_cast<float>(heuristic_weight_)*estimate(next),nk}); }
                }
            } else {
                for(int direction : {-1,1}) for(int turn=-1;turn<=1;++turn) {
                    State next; float ec;
                    if(!transition(cur,direction,turn,next,ec,/*departure=*/ck==sk)) continue;
                    if(ck==sk) ++root_successors;
                    const int nk=key(next,cols); if(closed[nk]) continue;
                    const float ng=g[ck]+ec;
                    if(ng<g[nk]) { g[nk]=ng; parent[nk]=ck; open.push({ng+static_cast<float>(heuristic_weight_)*estimate(next),nk}); }
                }
                for(int turn : {-1,1}) {
                    State next; float ec;
                    if(!transition(cur,0,turn,next,ec,/*departure=*/ck==sk)) continue;
                    if(ck==sk) ++root_successors;
                    const int nk=key(next,cols); if(g[ck]+ec<g[nk]) { g[nk]=g[ck]+ec; parent[nk]=ck; open.push({g[nk]+static_cast<float>(heuristic_weight_)*estimate(next),nk}); }
                }
            }
        }
        result_.expanded=expanded; result_.root_successors=root_successors;
        result_.search_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-search_begin).count();
        if(reached<0) {
            const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
            last_fail_reason_ = expanded==1 && root_successors==0 ? "start_no_successor"
                                 : ((expansion_limit_ ? std::uint64_t(expanded)>=expansion_limit_ : elapsed >= max_planning_time_) ? "search_timeout" : "search_exhausted");
            return false;
        }
        result_.route_cost=g[reached];
        const auto profile_begin=std::chrono::steady_clock::now();

        if(use_dynamics) {
            const SkidSteerModel nominal_model(sp_);
            std::vector<int> node_keys;
            for(int k=reached;k>=0;k=parent[k]) { node_keys.push_back(k); if(k==sk) break; }
            std::reverse(node_keys.begin(),node_keys.end());
            const State s0=stateFromKey(sk,cols);
            PlanningPosition p0; core_map_.getPosition(PlanningIndex(s0.x,s0.y),p0);
            Pose2D pose0;
            pose0.x=p0.x(); pose0.y=p0.y();
            pose0.yaw=yawForBin(s0.t);
            path.poses.push_back(pose0);
            vel_profile.data.push_back(0.0f); vel_profile.data.push_back(0.0f);
            for(size_t e=1;e<node_keys.size();++e) {
                const int childK=node_keys[e], parentK=node_keys[e-1];
                const State ps=stateFromKey(parentK,cols);
                PlanningPosition pp; core_map_.getPosition(PlanningIndex(ps.x,ps.y),pp);
                const double pyaw=yawForBin(ps.t);
                const auto& prims=primitive_lib_.primitivesFor(ps.t);
                const int pi=parent_prim[childK];
                if(pi<0 || pi>=static_cast<int>(prims.size())) continue;
                const MotionPrimitive& prim=prims[pi];
                if(e==1) result_.departure_end_index=prim.samples.size();
                const double c=std::cos(pyaw), s=std::sin(pyaw);
                for(size_t i=0;i<prim.samples.size();++i) {
                    const auto& smp=prim.samples[i];
                    const double wx=pp.x()+c*smp.x - s*smp.y;
                    const double wy=pp.y()+s*smp.x + c*smp.y;
                    const double wyaw=wrap(pyaw+smp.yaw);
                    Pose2D pose;
                    pose.x=wx; pose.y=wy;
                    pose.yaw=wyaw;
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
            enforceVelocityEnvelope(path, vel_profile);
            result_.profile_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-profile_begin).count();
            return !path.poses.empty();
        }

        std::vector<State> states;
        for(int k=reached;k>=0;k=parent[k]) { states.push_back(stateFromKey(k,cols)); if(k==sk) break; }
        std::reverse(states.begin(),states.end());
        for(const auto& s:states) {
            PlanningPosition p; core_map_.getPosition(PlanningIndex(s.x,s.y),p);
            Pose2D pose;
            pose.x=p.x(); pose.y=p.y();
            pose.yaw=yawForBin(s.t);
            path.poses.push_back(pose);
        }
        if(path.poses.empty()) return false;
        buildVelocityProfile(path, vel_profile);
        result_.departure_end_index=path.poses.size()>2 ? 2 : path.poses.size()-1;
        result_.profile_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-profile_begin).count();
        return true;
    }

protected:
    // Diagnostic hook only: never selects a route or changes acceptance.
    virtual void logFootprintRejection(const Pose2D&,const char*,double=0.0,
                                       bool=true,bool=false) const {}
    PlanningGrid core_map_;
    MotionPrimitiveLibrary primitive_lib_;
    bool snapped_goal_used_=false;
    double last_snap_dist_=0.0;
    std::string last_fail_reason_;
private:
    PlanningResult result_;
    std::uint64_t expansion_limit_=0;
    mutable std::vector<float> prof_ds_,prof_dyaw_,prof_kappa_,prof_v_,prof_w_,prof_wmag_;
    mutable std::vector<double> prof_yaw_;
    mutable std::vector<int> prof_dir_;
};
} // namespace groundgrid
