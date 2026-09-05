#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include "groundgrid/SweptFootprint.h"

namespace groundgrid {

inline bool recoveryPoseFinite(const Pose2D& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}

// Upper bound on corner motion. Counting rotation as well as translation bounds a
// manoeuvre even when the recent history contains an in-place turn or a direction cusp.
inline double recoveryMotion(const Pose2D& a, const Pose2D& b, double radius) {
    return std::hypot(b.x-a.x,b.y-a.y) +
           radius*std::abs(SkidSteerModel::wrap(b.yaw-a.yaw));
}

inline Pose2D recoveryInterpolate(const Pose2D& a, const Pose2D& b, double q) {
    return {a.x+q*(b.x-a.x), a.y+q*(b.y-a.y),
            SkidSteerModel::wrap(a.yaw+q*SkidSteerModel::wrap(b.yaw-a.yaw))};
}

struct RecoveryObservation { Pose2D pose; std::uint64_t stamp = 0; };

// Actual, stamped localisation samples ONLY. This object has no goal/path/profile input:
// a new goal cannot resurrect an old command, nor erase independently observed motion.
// Discontinuous localisation breaks the history instead of inventing a connecting path.
class RecentMotionHistory {
public:
    void configure(double radius, double resolution, double v_max, double w_max,
                   double span, double max_age = 60.0, double max_gap = 0.5) {
        clear();
        radius_=radius; resolution_=resolution; v_max_=v_max; w_max_=w_max;
        span_=span; max_age_=max_age; max_gap_=max_gap;
    }
    void clear() { samples_.clear(); span_used_=0.0; }
    std::size_t size() const { return samples_.size(); }

    bool observe(const Pose2D& pose, std::uint64_t stamp) {
        if(!configured() || !recoveryPoseFinite(pose) || stamp==0) { clear(); return false; }
        bool continuous=true;
        if(!samples_.empty()) {
            const auto& last=samples_.back();
            if(stamp==last.stamp && recoveryMotion(last.pose,pose,radius_)<1e-6) return true;
            if(stamp<=last.stamp || !connected(last,{pose,stamp})) {
                clear(); continuous=false;
            }
        }
        if(!samples_.empty()) span_used_+=recoveryMotion(samples_.back().pose,pose,radius_);
        samples_.push_back({pose,stamp});
        // Keep stamped stationary samples too: otherwise replacing the last timestamp
        // would discard the only pose usable by a TF that lags odometry by one message.
        prune(stamp);
        return continuous;
    }

    bool backtrack(const Pose2D& start, std::uint64_t stamp, double budget,
                   std::vector<Pose2D>& path, const char*& reason) const {
        path.clear(); reason="no_history";
        if(!configured() || !recoveryPoseFinite(start) || stamp==0 ||
           !std::isfinite(budget) || budget<=0.0) { reason="invalid_history_input"; return false; }
        // TF can trail the most recently received odometry; never use future samples.
        std::size_t end=samples_.size();
        while(end && samples_[end-1].stamp>stamp) --end;
        if(!end) return false;
        const auto& last=samples_[end-1];
        if(stamp<last.stamp || double(stamp-last.stamp)*1e-9>max_gap_ ||
           !connected(last,{start,stamp})) { reason="history_tf_discontinuity"; return false; }
        path.push_back(start);
        double distance=0.0;
        for(std::size_t i=end; i-->0;) {
            if(double(stamp-samples_[i].stamp)*1e-9>max_age_) break;
            const auto& next=samples_[i].pose;
            const double step=recoveryMotion(path.back(),next,radius_);
            if(step<1e-6) continue;
            const double remaining=budget-distance;
            if(step>=remaining) {
                path.push_back(recoveryInterpolate(path.back(),next,remaining/step));
                distance=budget;
                break;
            }
            path.push_back(next); distance+=step;
        }
        if(path.size()<2) { path.clear(); reason="no_earlier_motion"; return false; }
        reason="none";
        return true;
    }

private:
    bool configured() const {
        return std::isfinite(radius_) && radius_>0 && std::isfinite(resolution_) && resolution_>0 &&
            std::isfinite(v_max_) && v_max_>0 && std::isfinite(w_max_) && w_max_>0 &&
            std::isfinite(span_) && span_>0 && std::isfinite(max_age_) && max_age_>0 &&
            std::isfinite(max_gap_) && max_gap_>0;
    }
    bool connected(const RecoveryObservation& a, const RecoveryObservation& b) const {
        if(b.stamp<a.stamp) return false;
        const double dt=double(b.stamp-a.stamp)*1e-9;
        return dt<=max_gap_ && std::hypot(b.pose.x-a.pose.x,b.pose.y-a.pose.y)<=v_max_*dt+resolution_ &&
            std::abs(SkidSteerModel::wrap(b.pose.yaw-a.pose.yaw))<=w_max_*dt+resolution_/radius_;
    }
    void prune(std::uint64_t stamp) {
        while(samples_.size()>1 && (span_used_>span_ || samples_.size()>4096 ||
              double(stamp-samples_.front().stamp)*1e-9>max_age_)) {
            span_used_-=recoveryMotion(samples_[0].pose,samples_[1].pose,radius_);
            samples_.pop_front();
        }
    }
    std::deque<RecoveryObservation> samples_;
    double radius_=0, resolution_=0, v_max_=0, w_max_=0, span_=0, span_used_=0;
    double max_age_=60.0, max_gap_=0.5;
};

// A frozen exception ONLY for one bounded observed-history retreat. Normal search and
// rotate rules do not change. Clearance grows with cumulative corner motion, independent
// of sample spacing, and never restarts on republish, a new map, or observed regression.
// Preparation checks the complete manoeuvre. During execution, newly observed hazards
// invalidate actual motion since the previous check and the unexecuted suffix; a hazard
// discovered only behind the rover cannot retroactively make already-observed motion unsafe.
class BoundedBackout {
public:
    void clear() { poses_.clear(); progress_.clear(); observed_progress_=0.0; actual_motion_=0.0; }
    bool active() const { return poses_.size()>1; }
    const std::vector<Pose2D>& poses() const { return poses_; }
    double length() const { return progress_.empty() ? 0.0 : progress_.back(); }
    double progress() const { return observed_progress_; }
    double actualMotion() const { return actual_motion_; }
    const char* failure() const { return failure_; }
    double fullClearance() const { return full_; }
    double marginAt(double distance) const {
        return active() ? initial_+(full_-initial_)*std::clamp(distance/length(),0.0,1.0) : 0.0;
    }

    template<typename CheckFootprint>
    bool prepare(std::vector<Pose2D> poses, double radius, double resolution, double budget,
                 double clearance, CheckFootprint check, SweptFootprintRejection* rejection=nullptr) {
        clear(); if(rejection) *rejection={};
        if(poses.size()<2 || poses.size()>4096 || !std::isfinite(radius) || radius<=0 ||
           !std::isfinite(resolution) || resolution<=0 || !std::isfinite(budget) || budget<=0 ||
           !std::isfinite(clearance) || clearance<0) return false;
        std::vector<double> progress(poses.size(),0.0);
        for(std::size_t i=0;i<poses.size();++i) {
            if(!recoveryPoseFinite(poses[i])) return false;
            if(i) progress[i]=progress[i-1]+recoveryMotion(poses[i-1],poses[i],radius);
        }
        if(progress.back()<1e-6 || progress.back()>budget+1e-6) return false;
        poses_=std::move(poses); progress_=std::move(progress);
        radius_=radius; resolution_=resolution; full_=clearance; budget_=budget;
        previous_actual_=poses_.front();
        initial_=departureClearance(poses_.front(),clearance,true,check);
        if(!validateFrozen(check,true,rejection)) { clear(); return false; }
        return true;
    }

    template<typename CheckFootprint>
    bool validate(const Pose2D& current, double max_tracking_error, CheckFootprint check,
                   SweptFootprintRejection* rejection=nullptr) {
        if(rejection) *rejection={};
        failure_="backout_invalid_pose";
        if(!active() || !recoveryPoseFinite(current) || !std::isfinite(max_tracking_error) ||
           max_tracking_error<=0) return false;
        const Pose2D previous_actual=previous_actual_;
        const double previous_progress=observed_progress_;
        actual_motion_+=recoveryMotion(previous_actual,current,radius_);
        if(actual_motion_>budget_+1e-6) { failure_="backout_motion_budget"; return false; }
        // Project using position AND yaw so a stationary rotation is not mistaken for
        // completed translation. The requirement can advance but never decrease.
        double best=std::numeric_limits<double>::infinity(), distance=0.0;
        for(std::size_t i=1;i<poses_.size();++i) {
            const auto& a=poses_[i-1]; const auto& b=poses_[i];
            const double dx=b.x-a.x, dy=b.y-a.y;
            const double dz=radius_*SkidSteerModel::wrap(b.yaw-a.yaw);
            const double denominator=dx*dx+dy*dy+dz*dz;
            const double q=denominator>1e-12 ? std::clamp(((current.x-a.x)*dx+(current.y-a.y)*dy+
                radius_*SkidSteerModel::wrap(current.yaw-a.yaw)*dz)/denominator,0.0,1.0) : 0.0;
            const auto candidate=recoveryInterpolate(a,b,q);
            const double error=recoveryMotion(current,candidate,radius_);
            if(error<best) {
                best=error;
                distance=progress_[i-1]+q*(progress_[i]-progress_[i-1]);
            }
        }
        if(best>max_tracking_error) { failure_="backout_tracking_error"; return false; }
        observed_progress_=std::max(observed_progress_,distance);
        const Pose2D anchor=poseAt(observed_progress_);
        const double previous_margin=marginAt(previous_progress);
        const double margin=marginAt(observed_progress_);
        float cost;
        // Check what the rover really did since the previous timer tick. This prevents a
        // projection jump from skipping a cusp or a hazard between localisation samples.
        failure_="backout_actual_sweep";
        if(!sweptFootprintValid(previous_actual,current,radius_,resolution_,
                                previous_margin,margin,true,check,cost,rejection)) return false;
        failure_="backout_current_connector";
        if(!sweptFootprintValid(current,anchor,radius_,resolution_,margin,margin,true,
                                check,cost,rejection)) return false;
        // The schedule and geometry remain frozen, but only its unexecuted suffix can still
        // collide. Rechecking the prefix made a freshly perceived hazard behind the rover
        // abort an otherwise safe retreat after the body had already passed it.
        failure_="backout_remaining_sweep";
        if(!validateRemaining(anchor,observed_progress_,check,rejection)) return false;
        previous_actual_=current;
        failure_="none";
        return true;
    }

    template<typename CheckFootprint>
    bool clearanceRestored(const Pose2D& current, CheckFootprint check) const {
        float cost=0;
        return active() && recoveryPoseFinite(current) && check(current,full_,false,cost) &&
               std::isfinite(cost);
    }

private:
    Pose2D poseAt(double distance) const {
        if(distance<=0.0) return poses_.front();
        if(distance>=length()) return poses_.back();
        const auto upper=std::upper_bound(progress_.begin(),progress_.end(),distance);
        const std::size_t i=static_cast<std::size_t>(upper-progress_.begin());
        const double span=progress_[i]-progress_[i-1];
        const double q=span>1e-12 ? (distance-progress_[i-1])/span : 0.0;
        return recoveryInterpolate(poses_[i-1],poses_[i],q);
    }

    template<typename CheckFootprint>
    bool validateFrozen(CheckFootprint check, bool allow_initial_unknown,
                         SweptFootprintRejection* rejection) const {
        for(std::size_t i=1;i<poses_.size();++i) {
            float cost;
            if(!sweptFootprintValid(poses_[i-1],poses_[i],radius_,resolution_,
                marginAt(progress_[i-1]),marginAt(progress_[i]),allow_initial_unknown && i==1,
                check,cost,rejection)) return false;
        }
        return true;
    }

    template<typename CheckFootprint>
    bool validateRemaining(const Pose2D& anchor, double distance, CheckFootprint check,
                           SweptFootprintRejection* rejection) const {
        if(distance>=length()-1e-9) return true;
        const auto upper=std::upper_bound(progress_.begin(),progress_.end(),distance);
        std::size_t i=static_cast<std::size_t>(upper-progress_.begin());
        if(i==0 || i>=poses_.size()) return false;
        float cost;
        if(!sweptFootprintValid(anchor,poses_[i],radius_,resolution_,marginAt(distance),
                                marginAt(progress_[i]),false,check,cost,rejection)) return false;
        for(++i;i<poses_.size();++i) {
            if(!sweptFootprintValid(poses_[i-1],poses_[i],radius_,resolution_,
                                    marginAt(progress_[i-1]),marginAt(progress_[i]),
                                    false,check,cost,rejection)) return false;
        }
        return true;
    }
    std::vector<Pose2D> poses_;
    std::vector<double> progress_;
    double initial_=0, full_=0, radius_=0, resolution_=0, observed_progress_=0;
    double budget_=0, actual_motion_=0;
    Pose2D previous_actual_;
    const char* failure_="none";
};

// Execution is held on one immutable manoeuvre, not regenerated at each timer tick.
// Both the fixed deadline and the original no-progress timeout survive republication.
class BackoutExecutionLease {
public:
    void start(double now, double estimated_duration, double no_progress, double epsilon) {
        start_=last_progress_=last_check_=now; deadline_=now+estimated_duration+no_progress;
        timeout_=no_progress; epsilon_=epsilon; best_=0;
        valid_=std::isfinite(now) && std::isfinite(estimated_duration) && estimated_duration>0 &&
               std::isfinite(no_progress) && no_progress>0 && std::isfinite(epsilon) && epsilon>0 &&
               std::isfinite(deadline_);
    }
    bool update(double now, double progress) {
        if(!valid_ || !std::isfinite(now) || !std::isfinite(progress) || now<start_ ||
           now<last_check_ || now>deadline_ || now-last_progress_>timeout_) return false;
        last_check_=now;
        if(progress>best_+epsilon_) { best_=progress; last_progress_=now; }
        return true;
    }
private:
    bool valid_=false;
    double start_=0, last_progress_=0, last_check_=0, deadline_=0, timeout_=0, epsilon_=0, best_=0;
};

// Restoring clearance permits one bounded retry of the goal, not another full
// Rotate/BackOut cycle. The caller opens this window only after the retreat's exact stop
// acknowledgement and a fresh pose; waiting for that acknowledgement spends no search
// budget. A publication, failed search or duplicate request cannot extend the deadline.
// Clear only when the goal/recovery episode ends, never on a failed search or publication.
class PostBackoutReplanWindow {
public:
    void clear() { *this=PostBackoutReplanWindow{}; }
    void request() { pending_=true; }
    bool pending() const { return pending_; }
    bool started() const { return started_; }

    bool allow(double now,double duration) {
        if(!pending_ || exhausted_) return false;
        if(!std::isfinite(now) || !std::isfinite(duration) || duration<=0.0) {
            exhausted_=true;
            return false;
        }
        if(!started_) {
            deadline_=now+duration;
            last_check_=now;
            started_=true;
        }
        if(!std::isfinite(deadline_) || now<last_check_ || now>deadline_) {
            exhausted_=true;
            return false;
        }
        last_check_=now;
        return true;
    }

private:
    bool pending_=false, started_=false, exhausted_=false;
    double deadline_=0.0, last_check_=0.0;
};

} // namespace groundgrid
