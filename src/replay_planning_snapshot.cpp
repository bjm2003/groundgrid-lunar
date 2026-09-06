#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include "groundgrid/PlanningSnapshot.h"

int main(int argc,char** argv) {
    try {
        if(argc<2) throw std::runtime_error("usage: replay_planning_snapshot FILE [--strategy legacy_nearest|reachable_cost] [--expansions N] [--repeat N] [--trajectory]");
        std::uint64_t expansions=0; int repeats=1; bool trajectory=false;
        std::string strategy;
        for(int i=2;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--trajectory") trajectory=true;
            else if(arg=="--strategy" && i+1<argc) {
                strategy=argv[++i];
                if(strategy!="legacy_nearest" && strategy!="reachable_cost") throw std::runtime_error("invalid strategy");
            }
            else if((arg=="--expansions" || arg=="--repeat") && i+1<argc) {
                const std::string text=argv[++i]; std::size_t used=0;
                if(text.empty() || text[0]=='-') throw std::runtime_error("positive count required");
                const auto n=std::stoull(text,&used);
                if(used!=text.size() || n==0) throw std::runtime_error("positive count required");
                if(arg=="--expansions") expansions=n;
                else { if(n>1000) throw std::runtime_error("too many repeats"); repeats=static_cast<int>(n); }
            } else throw std::runtime_error("unknown/incomplete replay option: "+arg);
        }
        auto input=groundgrid::loadPlanningSnapshot(argv[1]);
        if(!strategy.empty()) input.config.reachable_snap_=strategy=="reachable_cost";
        groundgrid::LatticePlannerCore core(input);
        for(int i=0;i<repeats;++i) {
            const auto result=core.planCore(input.start,input.goal,expansions);
            std::cout<<"{\"replay\":"<<i<<",\"budget_mode\":\""<<(expansions ? "expansions":"wall_time")
                     <<"\",\"result\":"<<groundgrid::planningResultJson(input,result)<<"}\n";
            if(trajectory) {
                std::cout<<std::setprecision(17)<<"{\"trajectory\":[";
                for(std::size_t j=0;j<result.path.poses.size();++j) {
                    const auto& p=result.path.poses[j];
                    if(j) std::cout<<',';
                    std::cout<<'['<<p.x<<','<<p.y<<','<<p.yaw<<','<<result.profile.data[2*j]
                             <<','<<result.profile.data[2*j+1]<<']';
                }
                std::cout<<"]}\n";
            }
        }
        // A faithfully replayed planning failure is data, not a replay-tool failure.
        return 0;
    } catch(const std::exception& e) { std::cerr<<"replay failed: "<<e.what()<<'\n'; return 1; }
}
