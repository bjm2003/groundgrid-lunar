// Stateful, multi-step controller regression tests. Fixtures are synthetic unless marked
// as rounded log-derived samples. They verify index/phase, blending and stop-barrier
// semantics, not ROS transport, physical braking, perception, or collision clearance.
#include "groundgrid/TrajectoryTracking.h"
#include "groundgrid/RetainedTrajectoryCache.h"
#include "groundgrid/ReplanStopBarrier.h"

#include <cstdio>
#include <vector>

using namespace groundgrid;

namespace {
int failures = 0;
void check(bool ok, const char* label) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", label);
    if(!ok) ++failures;
}
TrackingSample sample(double x, double y, double yaw, double v = 0.0, double w = 0.0) {
    return {{x,y,yaw},v,w};
}

// Reproduce the former unbounded nearest/lookahead selection to establish the regression
// fixture: lookahead crossed a reversing cusp and selected the later forward command.
std::size_t legacyTarget(const std::vector<TrackingSample>& path, const Pose2D& pose) {
    std::size_t target = 0;
    double nearest = std::numeric_limits<double>::infinity();
    for(std::size_t i = 0; i < path.size(); ++i) {
        const double d = std::hypot(path[i].pose.x-pose.x, path[i].pose.y-pose.y);
        if(d < nearest) { nearest = d; target = i; }
    }
    while(target+1 < path.size()) {
        const auto& p = path[target].pose;
        const auto& next = path[target+1].pose;
        const double d = std::hypot(p.x-pose.x, p.y-pose.y);
        if(d >= 0.8 || requiresTerminalWaypointTracking(target+2 == path.size(),d,0.20)) break;
        ++target;
        if(requiresInPlaceRotationTracking(std::hypot(next.x-p.x,next.y-p.y),
                                          SkidSteerModel::wrap(next.yaw-pose.yaw),0.174532925)) break;
    }
    return target;
}

bool replay(const std::vector<TrackingSample>& path, Pose2D rover, bool republish = false) {
    TrajectoryTracking tracker;
    TrackingParams params;
    bool changed = false;
    if(!tracker.setTrajectory(path, changed)) return false;
    SkidSteerModel model;
    std::size_t last_begin = 0;
    for(int tick = 0; tick < 1200; ++tick) {
        if(republish && tick%10 == 0) {
            if(!tracker.setTrajectory(path, changed) || changed) return false;
        }
        const auto out = tracker.step(rover, params);
        if(out.status == TrackingStatus::GoalReached) return true;
        if(out.status != TrackingStatus::Tracking || out.phase_begin < last_begin ||
           out.target < out.phase_begin || out.target > out.phase_end ||
           out.command < out.phase_begin || out.command > out.phase_end ||
           !std::isfinite(out.desired_v) || !std::isfinite(out.desired_w) ||
           std::abs(out.desired_v) > params.control.max_linear_speed+1e-12 ||
           std::abs(out.desired_w) > params.control.max_angular_speed+1e-12) return false;
        last_begin = out.phase_begin;
        if((out.phase == MotionPhase::Reverse && out.desired_v > 0.0) ||
           (out.phase == MotionPhase::Forward && out.desired_v < 0.0)) return false;
        rover = model.integrate(rover, out.desired_v, out.desired_w, 0.05);
    }
    std::printf("  replay timed out at (%.3f, %.3f, %.3f)\n",rover.x,rover.y,rover.yaw);
    return false;
}
}

int main() {
    const double pi = std::acos(-1.0);
    TrackingParams p;
    bool changed = false;
    const std::vector<TrackingSample> cusp{
        sample(1,0,0), sample(.75,0,0,-.5), sample(.5,0,0),
        sample(.725,-.075,-pi/16,.5,-.4), sample(.95,-.15,-pi/8)};
    {
        const auto old_target = legacyTarget(cusp,{1,0,0});
        check(old_target == 3 && cusp[old_target].v > 0,
              "old selection reproduces future forward command across reverse cusp");
        TrajectoryTracking tracker;
        check(tracker.setTrajectory(cusp,changed) && changed, "accept finite atomic samples");
        const auto out = tracker.step({1,0,0},p);
        check(out.status == TrackingStatus::Tracking && out.phase == MotionPhase::Reverse &&
              out.phase_end == 2 && out.command == 1 && out.desired_v < 0,
              "lookahead and velocity lookup remain in active reversing phase");
        check(replay(cusp,{1,0,0},true), "multi-step reverse cusp then forward arc reaches endpoint");
    }
    const std::vector<TrackingSample> turn{
        sample(0,0,0), sample(.3,0,0,.5), sample(.6,0,0),
        sample(.6,0,pi/4,0,.6), sample(.6,0,pi/2),
        sample(.6,.3,pi/2,.5), sample(.6,.6,pi/2)};
    {
        TrajectoryTracking tracker;
        tracker.setTrajectory(turn,changed);
        auto out = tracker.step({.6,0,0},p);
        check(out.phase == MotionPhase::RotateLeft && out.phase_begin == 2 && out.phase_end == 3 &&
              out.desired_v == 0 && out.desired_w > 0,
              "translation must acquire boundary before in-place rotation starts");
        out = tracker.step({.6,.18,.2},p);
        check(out.phase == MotionPhase::RotateLeft && out.target == 3 && out.nearest <= 3 &&
              out.desired_v == 0,
              "nearer future translation cannot bypass unfinished rotation");
        tracker.setTrajectory(turn,changed);
        tracker.step({.6,0,pi/4},p);
        out = tracker.step({.6,0,pi/2},p);
        check(!changed && out.phase == MotionPhase::Forward && out.phase_begin == 4,
              "same-geometry republish preserves acquired phase progress");
        out = tracker.step({.6,0,0},p);
        check(out.phase_begin == 4, "later yaw change cannot reactivate completed rotation");
        auto replacement = turn;
        replacement.back().pose.y += .15;
        tracker.setTrajectory(replacement,changed);
        out = tracker.step({0,0,0},p);
        check(changed && out.phase_begin == 0, "real geometric replan resets phase progress");
        check(replay(turn,{0,0,0},true), "multi-step translation rotation translation reaches endpoint");
    }
    {
        TrajectoryTracking tracker;
        std::vector<TrackingSample> bad_phase{
            sample(0,0,0),sample(.3,0,0,.5),sample(.6,0,0),
            sample(.3,0,0),sample(0,0,pi/4)};
        tracker.setTrajectory(bad_phase,changed);
        const auto out = tracker.step({.6,0,0},p);
        check(out.status == TrackingStatus::Invalid && out.desired_v == 0,
              "missing reverse speed cannot borrow earlier forward command");
    }
    {
        std::vector<TrackingSample> long_rotation;
        for(int i = 0; i <= 12; ++i) {
            long_rotation.push_back(sample(0,0,SkidSteerModel::wrap(i*pi/8),0,
                                           i == 0 || i == 12 ? 0.0 : .6));
        }
        TrajectoryTracking tracker;
        tracker.setTrajectory(long_rotation,changed);
        Pose2D rover{0,0,0};
        SkidSteerModel model;
        double swept = 0.0;
        bool correct_direction = true, reached = false;
        for(int tick = 0; tick < 1000; ++tick) {
            const auto out = tracker.step(rover,p);
            if(out.status == TrackingStatus::GoalReached) { reached = true; break; }
            if(out.status != TrackingStatus::Tracking || out.desired_v != 0 || out.desired_w < 0) {
                correct_direction = false;
                break;
            }
            swept += out.desired_w*.05;
            rover = model.integrate(rover,out.desired_v,out.desired_w,.05);
        }
        check(correct_direction && reached && swept > pi,
              "long rotation follows ordered swept headings across angle wrap");
    }
    {
        TrajectoryTracking tracker;
        std::vector<TrackingSample> rotation{
            sample(0,0,.8),sample(0,0,.6,0,-.686),sample(0,0,.4)};
        tracker.setTrajectory(rotation,changed);
        const auto out = tracker.step({0,0,.1},p);
        check(out.status == TrackingStatus::Tracking && out.planned_w == 0 && out.desired_w > 0,
              "passed rotation feedforward cannot oppose yaw correction");
        check(replay(rotation,{0,0,.1}), "overshot rotation still converges without biased equilibrium");
        const auto disconnected = tracker.step({1,0,.1},p);
        check(disconnected.status == TrackingStatus::Invalid,
              "rotation cannot invent translation to a distant anchor");
    }
    {
        TrajectoryTracking tracker;
        const std::vector<TrackingSample> path{sample(0,0,0),sample(.5,0,0,.6,.4),sample(1,0,0)};
        tracker.setTrajectory(path,changed);
        const auto out = tracker.step({.81,0,.5},p);
        check(out.pose_capture && out.desired_v == 0 && out.planned_w == 0 && out.desired_w < 0,
              "terminal yaw capture does not replay translating curvature");
        check(replay(path,{.81,0,.5}), "captured goal rotates to completion without leaving position tolerance");
    }
    for(int direction : {-1,1}) {
        std::vector<TrackingSample> straight, arc;
        for(int i = 0; i <= 8; ++i) {
            const double v = (i == 0 || i == 8) ? 0.0 : direction*.4;
            straight.push_back(sample(direction*.25*i,0,0,v));
            const double angle = (pi/2)*i/8;
            arc.push_back(sample(direction*std::sin(angle),direction*(1-std::cos(angle)),
                                 angle,v,(i == 0 || i == 8) ? 0.0 : .4));
        }
        check(replay(straight,{0,0,0}),direction < 0 ? "reverse straight replay" : "forward straight replay");
        check(replay(arc,{0,0,0}),direction < 0 ? "reverse arc replay" : "forward arc replay");
    }
    {
        // Rounded log-derived fixture from 0b314f1, trajectory 186. The truncated point-5
        // speed is reconstructed from command=5 in follow_debug. Test the first reverse
        // then forward boundary, NOT completion of the whole recovery manoeuvre: later
        // reverse tracking remains a known gap requiring ROS/map validation.
        const std::vector<TrackingSample> logged{
            sample(-2.275,-1.425,.392699),
            sample(-2.500,-1.500,.196350,-.533484,-.441662),
            sample(-2.725,-1.575,0),
            sample(-2.500,-1.650,-.196350,.533484,-.441662),
            sample(-2.275,-1.725,-.392699,.754460,-.624604),
            sample(-2.050,-1.800,-.196350,.533484,0),
            sample(-1.825,-1.875,0),
            sample(-2.050,-1.950,.196350,-.533484,.441662),
            sample(-2.275,-2.025,.392699)};
        TrajectoryTracking tracker;
        tracker.setTrajectory(logged,changed);
        Pose2D rover{-2.365,-1.422,.356};
        SkidSteerModel model;
        bool acquired=false, bounded=true, full_correction=false;
        for(int tick=0;tick<140;++tick) {
            const auto out=tracker.step(rover,p);
            if(out.status!=TrackingStatus::Tracking) break;
            if(out.phase_begin==6) { acquired=true; break; }
            bounded &= std::abs(out.desired_v)<=p.control.max_linear_speed &&
                       std::abs(out.desired_w)<=p.control.max_angular_speed;
            full_correction |= out.phase_begin==2 && out.planned_w==0 &&
                               std::abs(out.desired_w)>.79;
            rover=model.integrate(rover,out.desired_v,out.desired_w,.05);
        }
        check(acquired && bounded && full_correction,
              "logged first reversing cusp acquires forward endpoint without half-limit spiral");
        check(std::hypot(rover.x+1.825,rover.y+1.875)<p.waypoint_arrival_distance &&
              std::abs(rover.yaw)<p.goal_yaw_tolerance,
              "logged forward boundary meets unchanged position and yaw tolerances");
    }
    {
        TrajectoryTracking tracker;
        tracker.setTrajectory({sample(0,0,0),sample(0,0,2.0,0,.6)},changed);
        const auto out=tracker.step({0,0,0},p);
        check(out.status==TrackingStatus::Tracking && out.desired_v==0 &&
              out.feedback_w>p.control.max_angular_speed &&
              out.desired_w==p.control.max_angular_speed,
              "rotation feedback is mixed before the common angular limit");
    }
    {
        const std::vector<TrackingSample> compound{
            sample(0,0,0), sample(.45,0,0,.5), sample(.9,0,0),
            sample(.9,0,pi/4,0,.6), sample(.9,0,pi/2),
            sample(.9,-.45,pi/2,-.5), sample(.9,-.9,pi/2),
            sample(.9,-.45,pi/2,.5), sample(.9,0,pi/2,.5), sample(.9,.5,pi/2),
            sample(.9,.5,pi/4,0,-.6), sample(.9,.5,0)};
        check(replay(compound,{0,0,0},true), "combined rotation reverse forward terminal-rotation replay");
    }
    {
        // Synthetic planner/follower handoff: A's first rotation has been completed;
        // recovery publishes B farther along the route. The former planner kept A cached
        // on recovery confirmation and then sent A again, resetting tracking to its start.
        using Path = std::vector<TrackingSample>;
        using Profile = std::vector<double>;
        const Path old_route{sample(0,0,0), sample(0,0,-.4,0,-.6),
                             sample(.3,-.15,-.4,.5), sample(.95,-.45,-.4)};
        const Path recovered_route{sample(.3,-.15,-.4), sample(.6,-.3,-.4,.5),
                                   sample(1,-.45,-.4)};
        const Profile old_profile{0,0,0,-.6,.5,0,0,0};
        const Profile recovered_profile{0,0,.5,0,0,0};
        const Pose2D rover{.6,-.3,-.4};
        TrajectoryTracking tracker;
        RetainedTrajectoryCache<Path,Profile> cache;
        check(!cache.reusable(), "empty retained cache cannot be reused");
        cache.published(old_route,old_profile,false,true);
        tracker.setTrajectory(cache.path(),changed);
        check(cache.reusable() && tracker.step({0,0,-.4},p).phase == MotionPhase::Forward,
              "nominal cache and follower acquire old route rotation");

        cache.published(recovered_route,recovered_profile,true,false);
        tracker.setTrajectory(recovered_route,changed);
        check(!cache.reusable() && cache.path().size() == old_route.size() &&
              cache.profile() == old_profile && !cache.wasSnapped(),
              "unconfirmed recovery revokes reuse but preserves inactive route bookkeeping");
        tracker.setTrajectory(old_route,changed);  // former confirmed-recovery bug
        const auto rejected = tracker.step(rover,p);
        check(changed && rejected.status == TrackingStatus::Invalid &&
              rejected.failure == TrackingFailure::RotationAnchor &&
              rejected.phase_begin == 0 && rejected.phase_end == 1 &&
              rejected.endpoint_distance > rejected.arrival_distance &&
              rejected.desired_v == 0 && rejected.desired_w == 0,
              "old cache resurrection reproduces distant phase-zero rotation rejection");

        tracker.setTrajectory(recovered_route,changed);
        cache.published(recovered_route,recovered_profile,true,true);  // confirmed recovery
        tracker.setTrajectory(cache.path(),changed);
        const auto continued = tracker.step(rover,p);
        check(cache.reusable() && cache.profile() == recovered_profile && cache.wasSnapped() &&
              !changed && continued.status == TrackingStatus::Tracking && continued.desired_v > 0,
              "confirmed recovery promotes complete new route and preserves follower continuity");
        cache.published(cache.path(),cache.profile(),cache.wasSnapped(),true);
        check(cache.reusable() && cache.profile() == recovered_profile,
              "validated same-route republication safely preserves cache payload");

        cache.published(Path{},Profile{},false,false);  // stop/invalid atomic output
        tracker.clear();
        check(!cache.reusable() && cache.path().size() == recovered_route.size(),
              "empty publication revokes cache replay after follower progress is cleared");
        const Path manoeuvre{sample(.6,-.3,-.4),sample(.6,-.3,0,0,.6)};
        cache.published(manoeuvre,Profile{},false,false);
        check(!cache.reusable() && cache.profile() == recovered_profile && cache.wasSnapped(),
              "recovery manoeuvre neither reactivates nor overwrites inactive nominal route");
        cache.published(old_route,old_profile,false,true);
        check(cache.reusable() && !cache.wasSnapped() && cache.profile() == old_profile,
              "fresh validated nominal plan replaces interrupted cache and snap metadata");
        cache.clear();
        check(!cache.reusable() && cache.path().empty() && cache.profile().empty() &&
              !cache.wasSnapped(), "new goal clears all retained route state");
    }
    {
        TrajectoryTracking tracker;
        tracker.setTrajectory(cusp,changed);
        check(tracker.step({NAN,0,0},p).status == TrackingStatus::Invalid, "reject nonfinite rover pose");
        TrackingParams invalid = p; invalid.lookahead = NAN;
        check(tracker.step({1,0,0},invalid).status == TrackingStatus::Invalid, "reject nonfinite control parameter");
        auto bad = cusp; bad[1].v = NAN;
        check(!tracker.setTrajectory(bad,changed) && tracker.samples().empty(), "invalid atomic replacement clears old trajectory");
        check(!tracker.setTrajectory({},changed), "empty trajectory rejected");
        tracker.setTrajectory(cusp,changed);
        auto speed_change = cusp; speed_change[1].v = -.4;
        check(tracker.setTrajectory(speed_change,changed) && !changed, "speed-only update preserves phase identity");
    }
    {
        ReplanStopBarrier barrier;
        const std::uint64_t stamp=1788428079729038841ULL;
        const auto ack=[&](unsigned goal,std::uint64_t ts,const char* status) {
            return "goal_id="+std::to_string(goal)+" trajectory_stamp_ns="+std::to_string(ts)+
                   " snapshot_seq=123 status="+status;
        };
        const auto observe=[&](const std::string& message) { return barrier.observe(message,stamp); };
        check(barrier.canSearch(0) && !barrier.waiting(),
              "ordinary replanning has no stop barrier");
        barrier.request(15,stamp);
        check(barrier.waiting() && !barrier.canSearch(stamp+1000000000ULL),
              "revoked path blocks search even after a full planning-budget interval");
        check(!observe(ack(14,stamp,"empty_trajectory")) &&
              !observe(ack(15,stamp-1,"empty_trajectory")) &&
              !observe(ack(15,stamp,"goal_reached")) &&
              !observe(ack(15,stamp,"tracking")) && barrier.waiting(),
              "old goal, adjacent nanosecond stamp and non-stop status cannot release barrier");
        check(!observe("goal_id=15 status=empty_trajectory") &&
              !observe("goal_id=15 trajectory_stamp_ns=-1 status=empty_trajectory") &&
              !observe("goal_id=15 trajectory_stamp_ns=18446744073709551616 status=empty_trajectory") &&
              !observe(ack(15,stamp,"empty_trajectory")+" goal_id=15") &&
              !observe(ack(15,stamp,"empty_trajectory")+" trajectory_stamp_ns=4") &&
              !observe(ack(15,stamp,"empty_trajectory")+" status=empty_trajectory"),
              "missing, malformed, overflowing and duplicate acknowledgement fields fail closed");
        check(observe(ack(15,stamp,"empty_trajectory")) && !barrier.waiting() &&
              !barrier.canSearch(stamp-1) && !barrier.canSearch(stamp) && barrier.canSearch(stamp+1),
              "exact stop acknowledgement still requires a post-stop pose for search");
        check(!observe(ack(15,stamp,"empty_trajectory")),
              "duplicate acknowledgement cannot trigger another transition");
        barrier.request(15,stamp+100);
        check(!observe(ack(15,stamp,"empty_trajectory")) && barrier.waiting() &&
              observe(ack(15,stamp+100,"empty_trajectory")),
              "second stop of same goal requires its own trajectory acknowledgement");
        check(followerStatusChanged("empty_trajectory",stamp+100,"empty_trajectory",stamp) &&
              !followerStatusChanged("empty_trajectory",stamp,"empty_trajectory",stamp) &&
              followerStatusChanged("empty_trajectory",stamp,"tracking",stamp),
              "follower emits a new stop acknowledgement when only trajectory stamp changes");
        barrier.clear();
        check(!barrier.pending() && barrier.canSearch(0) &&
              !observe(ack(15,stamp+100,"empty_trajectory")),
              "completed barrier reset ignores old acknowledgement");
        barrier.request(15,stamp);
        check(!observe(ack(16,stamp,"empty_trajectory")) && barrier.waiting() &&
              observe(ack(15,stamp,"empty_trajectory")),
              "new mission cannot acknowledge a previous mission's outstanding stop");
        barrier.request(15,stamp);
        check(barrier.observe(ack(15,stamp,"empty_trajectory"),stamp+100) &&
              !barrier.canSearch(stamp+1) && !barrier.canSearch(stamp+100) &&
              barrier.canSearch(stamp+101),
              "TF newer than stop publication but older than acknowledgement cannot release search");
        barrier.request(0,0);
        check(barrier.waiting() && !observe(ack(0,0,"empty_trajectory")) &&
              !barrier.canSearch(stamp),"invalid stop identity cannot release safety barrier");

        TrajectoryTracking tracker;
        const std::vector<TrackingSample> path{sample(0,0,0),sample(1,0,0,.5),sample(2,0,0)};
        tracker.setTrajectory(path,changed);
        Pose2D old_rover{}, stopped_rover{};
        SkidSteerModel model;
        for(int tick=0;tick<20;++tick) {
            const auto out=tracker.step(old_rover,p);
            old_rover=model.integrate(old_rover,out.desired_v,out.desired_w,.05);
        }
        barrier.request(15,stamp);
        tracker.clear(); // empty atomic trajectory accepted by the follower before its ack
        bool held=true;
        for(int tick=0;tick<20;++tick) {
            const auto out=tracker.step(stopped_rover,p);
            held &= out.desired_v==0 && out.desired_w==0 && !barrier.canSearch(stamp+1);
            stopped_rover=model.integrate(stopped_rover,out.desired_v,out.desired_w,.05);
        }
        check(old_rover.x>.45 && held && stopped_rover.x==0,
              "synthetic one-second search no longer permits half-metre travel on revoked path");
    }
    {
        // A logged dynamics join used to introduce a zero-command reverse phase.
        // Runtime rejection must stop, but an identical re-publication must not reset
        // the already completed forward phase and command the rover back to phase zero.
        const std::vector<TrackingSample> broken{
            sample(0,0,0),sample(.5,0,0,.5),sample(1,0,0),
            sample(.94,.10,.04,0,.8),sample(.94,.10,.3,0,.8)};
        TrajectoryTracking tracker;tracker.setTrajectory(broken,changed);
        const Pose2D rover{.85,-.10,0};
        const auto first=tracker.step(rover,p);
        check(first.failure==TrackingFailure::MissingCommand && first.phase_begin==2,
              "phantom translation fixture reaches the same missing-command phase");
        check(tracker.setTrajectory(broken,changed) && !changed,
              "republished rejected geometry retains its identity after stopping");
        const auto repeated=tracker.step(rover,p);
        check(repeated.status==TrackingStatus::Invalid && repeated.phase_begin==2 &&
              repeated.desired_v==0 && repeated.desired_w==0,
              "republish after phase fault cannot restart completed motion");
        const std::vector<TrackingSample> replacement{
            sample(.85,-.10,0),sample(1.35,-.10,0,.5),sample(1.85,-.10,0)};
        check(tracker.setTrajectory(replacement,changed) && changed &&
              tracker.step(rover,p).status==TrackingStatus::Tracking,
              "genuinely new safe geometry can replace the stopped faulted route");
    }
    std::printf("trajectory_tracking_selfcheck: %d failure(s)\n",failures);
    return failures ? 1 : 0;
}
