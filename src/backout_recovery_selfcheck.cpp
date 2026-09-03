#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "groundgrid/BackoutRecovery.h"
#include "groundgrid/FootprintRaster.h"
#include "groundgrid/RetainedTrajectoryCache.h"
#include "groundgrid/TrajectoryTracking.h"
#include "groundgrid/ReplanStopBarrier.h"

using namespace groundgrid;

int main() {
    int checks=0, failures=0;
    const auto expect=[&](bool ok,const char* name) {
        ++checks; if(!ok) ++failures;
        std::printf("[%s] %s\n",ok ? "PASS" : "FAIL",name);
    };
    const double radius=std::hypot(.9,.75), nan=std::numeric_limits<double>::quiet_NaN();
    const auto stamp=[](double seconds) { return static_cast<std::uint64_t>(seconds*1e9); };
    const auto free=[](const Pose2D&,double,bool,float& cost) { cost=0; return true; };
    // A known lethal square just beyond the rover's front. Body rasterisation and all
    // swept checks use production code; this is a synthetic map, not the Ubuntu map.
    const auto obstacle=[](double hx,double hy) {
        return [=](const Pose2D& pose,double margin,bool,float& cost) {
            cost=0;
            return visitFootprintCells(pose,1.8,1.5,margin,.15,0,0,[=](double x,double y) {
                return std::hypot(x-hx,y-hy)>.01;
            });
        };
    };
    const auto front=obstacle(1.125,.075);
    auto route=[](int n) {
        std::vector<Pose2D> poses;
        for(int i=0;i<=n;++i) poses.push_back({-double(i)/n,0,0});
        return poses;
    };
    RecentMotionHistory history;
    history.configure(radius,.15,1.5,.8,4.0);
    for(int i=0;i<=20;++i) history.observe({-1.0+.05*i,0,0},stamp(1+.05*i));
    std::vector<Pose2D> back;
    const char* reason="none";
    expect(history.backtrack({0,0,0},stamp(2),1,back,reason) && back.front().x==0 &&
           std::abs(back.back().x+1)<1e-6,"backtrack uses actual observations in reverse time order");
    RetainedTrajectoryCache<int,int> goal_cache;
    goal_cache.published(123,456,true,true); goal_cache.clear();
    expect(history.backtrack({0,0,0},stamp(2),1,back,reason) && !goal_cache.reusable(),
           "new goal clears old command cache without destroying independent measured history");
    expect(history.backtrack({-.1,0,0},stamp(1.9),1,back,reason) && back.front().x==-.1,
           "TF older than newest odometry excludes future observations");
    expect(!history.backtrack({0,0,0},stamp(3),1,back,reason),"stale odometry cannot reconnect to current TF");
    expect(!history.backtrack({3,0,0},stamp(2),1,back,reason),"TF/odometry position disagreement rejects retreat");
    expect(!history.backtrack({0,0,1},stamp(2),1,back,reason),"TF/odometry yaw disagreement rejects retreat");
    expect(!history.observe({5,0,0},stamp(2.05)) && history.size()==1,
           "localisation jump discards history instead of inventing an escape segment");
    expect(!history.observe({5,0,0},stamp(1)) && history.size()==1,"clock rollback breaks history");
    expect(!history.observe({nan,0,0},stamp(1.1)) && history.size()==0,"nonfinite observation invalidates history");
    history.observe({0,0,0},stamp(1)); history.observe({.1,0,0},stamp(1.1));
    expect(!history.observe({.1,0,0},stamp(2)) && history.size()==1,"missing odometry interval breaks continuity");
    history.configure(radius,.15,1.5,.8,4.0,1.0);
    history.observe({-.1,0,0},stamp(1)); history.observe({0,0,0},stamp(1.1));
    for(int i=0;i<30;++i) history.observe({0,0,0},stamp(1.2+.1*i));
    expect(!history.backtrack({0,0,0},stamp(4.1),1,back,reason) && history.size()<=12,
           "stationary odometry stays fresh but expired motion is not retained indefinitely");
    history.configure(radius,.15,1.5,.8,4.0);
    for(int i=0;i<=20;++i) history.observe({-1.0+.05*i,0,0},stamp(1+.05*i));
    for(int i=1;i<=100;++i) history.observe({0,0,0},stamp(2+.05*i));
    expect(history.backtrack({0,0,0},stamp(6.95),1,back,reason) && back.back().x<-.9,
           "stationary history remains usable when latest odometry is one message ahead of TF");
    history.configure(radius,.15,1.5,.8,.5);
    for(int i=0;i<200;++i) history.observe({i*.05,0,0},stamp(1+i*.05));
    expect(history.backtrack({9.95,0,0},stamp(10.95),1,back,reason) &&
           std::abs(back.front().x-back.back().x)<=.5+1e-6,"history memory is distance bounded");
    history.configure(radius,.15,1.5,.8,4);
    for(int i=0;i<21;++i) history.observe({-.5+.025*i,0,.02*i},stamp(1+.05*i));
    expect(history.backtrack({0,0,.4},stamp(2),.5,back,reason) && back.size()>1,
           "history preserves curved motion and clips the last segment to the retreat budget");
    double travel=0;
    for(std::size_t i=1;i<back.size();++i) travel+=recoveryMotion(back[i-1],back[i],radius);
    expect(travel<=.5+1e-6,"turning corner motion is included in the distance bound");

    BoundedBackout plan;
    SweptFootprintRejection rejected;
    float cost;
    expect(!sweptFootprintValid({0,0,0},{-.1,0,0},radius,.15,0,.5,true,front,cost),
           "reproduce old first-history-sample full-band rejection");
    expect(plan.prepare(route(20),radius,.15,1,.5,front,&rejected),
           "bounded one-metre retreat restores the band across the whole manoeuvre");
    expect(plan.marginAt(.1)==.05 && plan.marginAt(1)==.5,
           "clearance follows cumulative motion rather than a count of short samples");
    BoundedBackout coarse;
    expect(coarse.prepare(route(4),radius,.15,1,.5,front) &&
           coarse.marginAt(.7)==plan.marginAt(.7),"densification does not change the clearance schedule");
    expect(plan.validate({-.5,0,0},.8,front,&rejected) && plan.progress()>.49,
           "execution revalidation uses the frozen ramp, not the ordinary first-edge rule");
    const double old_progress=plan.progress();
    expect(plan.validate({-.4,0,0},.8,front) && plan.progress()==old_progress,
           "moving backwards in plan progress cannot restart or reduce the clearance requirement");
    expect(plan.validate({-.6,0,0},.8,front) && plan.marginAt(plan.progress())>.29,
           "new map/republication advances the same immutable manoeuvre");
    expect(plan.clearanceRestored({-.8,0,0},front) && !plan.clearanceRestored({0,0,0},front),
           "completion requires full clearance at the actual rover, not just the planned endpoint");
    expect(!plan.validate({-.6,0,0},.8,obstacle(-.525,.075),&rejected),
           "new body hazard invalidates active retreat even after it started");
    expect(!plan.prepare(route(20),radius,.15,1,.5,obstacle(-.525,.075)),
           "known hazard anywhere along measured history is not assumed safe today");
    plan.prepare(route(20),radius,.15,1,.5,free);
    expect(plan.validate({-.4,0,0},.8,free),"retreat establishes checked execution progress");
    const auto past_hazard=[](const Pose2D& pose,double,bool,float& c) {
        c=0; return pose.x<=-.25;
    };
    expect(plan.validate({-.6,0,0},.8,past_hazard),
           "new hazard solely behind actual checked motion does not invalidate the remaining retreat");
    plan.prepare(route(20),radius,.15,1,.5,free);
    expect(plan.validate({-.4,0,0},.8,free),"actual-sweep fixture establishes prior pose");
    const auto actual_hazard=[](const Pose2D& pose,double,bool,float& c) {
        c=0; return pose.x>=-.48 || pose.x<=-.52;
    };
    expect(!plan.validate({-.6,0,0},.8,actual_hazard) &&
           std::string(plan.failure())=="backout_actual_sweep",
           "new hazard between localisation samples revokes the retreat");
    plan.prepare(route(20),radius,.15,1,.5,free);
    expect(plan.validate({-.6,0,0},.8,free),"future-sweep fixture establishes progress");
    const auto future_hazard=[](const Pose2D& pose,double,bool,float& c) {
        c=0; return pose.x>=-.8;
    };
    expect(!plan.validate({-.65,0,0},.8,future_hazard),
           "new hazard on the unexecuted suffix still revokes the retreat");
    const auto corner=obstacle(1.125,.075);
    expect(corner({0,0,0},0,false,cost) && corner({0,0,1.57079632679},0,false,cost),
           "rotation fixture has clear endpoint footprints");
    expect(!plan.prepare({{0,0,0},{0,0,1.57079632679}},radius,.15,2,.0,corner),
           "retreat rotation checks swept physical corners");
    expect(!plan.prepare({{0,0,0},{-.1,0,0}},radius,.15,1,.5,front),
           "short history without a full-clearance endpoint cannot produce a manoeuvre");
    expect(!plan.prepare({{0,0,0},{-1.01,0,0}},radius,.15,1,.5,free),
           "retreat cannot exceed authorised distance to find a convenient endpoint");
    expect(!plan.prepare({{0,0,0},{nan,0,0}},radius,.15,1,.5,free) && !plan.active(),
           "invalid replacement cannot leave an old retreat active");
    const auto unknown=[](const Pose2D&,double,bool allow,float& c) { c=0; return allow; };
    expect(!plan.prepare(route(20),radius,.15,1,.5,unknown),"unknown body terrain after start remains forbidden");
    const auto bad_cost=[=](const Pose2D&,double,bool,float& c) { c=float(nan); return true; };
    expect(!plan.prepare(route(20),radius,.15,1,.5,bad_cost),"nonfinite terrain cost fails closed");
    expect(plan.prepare(route(20),radius,.15,1,.5,free) && plan.marginAt(0)==.5,
           "already-clear start never receives the reduced-band exception");
    expect(!plan.validate({4,0,0},.8,free),"off-route rover cannot use retreat permission elsewhere");
    plan.prepare(route(20),radius,.15,1,.5,free);
    expect(plan.validate({-.6,0,0},.8,free) && !plan.validate({0,0,0},.8,free),
           "observed oscillation cannot spend more than the bounded retreat motion budget");
    plan.clear();
    expect(!plan.active() && !plan.validate({0,0,0},.8,free),"new goal/stop can revoke frozen retreat execution");

    BackoutExecutionLease lease;
    lease.start(10,3,6,.1);
    expect(lease.update(12.5,.3),"back-out may execute past the ordinary two-second ladder rung");
    expect(lease.update(15,.6),"actual retreat progress refreshes only the no-progress clock");
    expect(!lease.update(19.1,.9),"fixed execution deadline cannot be extended by repeated publication");
    lease.start(10,3,6,.1);
    expect(!lease.update(16.1,0),"stationary retreat times out rather than blocking recovery forever");
    lease.start(10,3,6,.1);
    expect(!lease.update(9,.2),"backwards execution clock fails closed");
    lease.start(10,3,6,.1);
    expect(lease.update(12,.2) && !lease.update(11,.2),"rollback within an active lease also fails closed");
    lease.start(10,nan,6,.1);
    expect(!lease.update(11,.2),"invalid time estimate cannot create an unbounded lease");
    {
        std::vector<TrackingSample> samples;
        const auto poses=route(20);
        for(std::size_t i=0;i<poses.size();++i) {
            const double travelled=-poses[i].x;
            const double v=-std::min({.5,std::sqrt(1.2*travelled),std::sqrt(1.2*(1-travelled))});
            samples.push_back({poses[i],v,0});
        }
        TrajectoryTracking tracker;
        TrackingParams parameters;
        SkidSteerModel model;
        bool changed=false, safe=tracker.setTrajectory(samples,changed);
        bool completed=false;
        Pose2D current{0,0,0};
        plan.prepare(poses,radius,.15,1,.5,front);
        lease.start(10,4,6,.1);
        for(int i=0;i<200 && safe;++i) {
            if(i%10==0) {
                safe=plan.validate(current,.8,front) && lease.update(10+.05*i,plan.progress()) &&
                     tracker.setTrajectory(samples,changed) && !changed;
                if(!safe) break;
            }
            const auto step=tracker.step(current,parameters);
            if(step.status==TrackingStatus::GoalReached) {
                completed=plan.clearanceRestored(current,front);
                break;
            }
            if(step.status!=TrackingStatus::Tracking || step.desired_v>0) { safe=false; break; }
            current=model.integrate(current,step.desired_v,step.desired_w,.05);
            float terrain;
            safe=front(current,0,false,terrain);
        }
        expect(safe && completed && current.x<-.75,
               "production follower completes checked reverse retreat through repeated map/publication cycles");
        expect(!missionGoalReached(true,false,false,0,.5),
               "retreat endpoint completion is not operator mission completion");
        ReplanStopBarrier barrier;
        barrier.request(15,1234);
        plan.clear(); goal_cache.clear();
        expect(!barrier.canSearch(5000) && !plan.active(),
               "new goal revokes retreat but does not cancel outstanding old-goal stop");
        expect(barrier.observe("goal_id=15 trajectory_stamp_ns=1234 status=empty_trajectory",6000) &&
               !barrier.canSearch(6000) && barrier.canSearch(6001),
               "next search waits for exact retreat stop acknowledgement and newer TF");
    }
    std::printf("backout_recovery_selfcheck: %d checks, %d failures\n",checks,failures);
    return failures ? 1 : 0;
}
