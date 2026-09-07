#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include "groundgrid/PlanningSnapshot.h"
#include "groundgrid/TerrainDiagnosticPatch.h"

using namespace groundgrid;
static void check(bool ok,const char* what) { if(!ok) throw std::runtime_error(what); }
int main() {
    try {
        const auto name="groundgrid-snapshot-check-"+std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count());
        const auto directory=std::filesystem::temp_directory_path()/name;
        check(std::filesystem::create_directory(directory),"fresh test directory");
        PlanningInput input;
        auto& m=input.map;
        m.rows=40;m.cols=40;m.start_row=7;m.start_col=19;
        m.resolution=0.15;m.length_x=m.length_y=6.0;
        m.cost.assign(1600,0.0f);m.gx=m.gy=m.slope=m.cost;
        std::uint32_t nan_bits=0x7fc00023U; std::memcpy(&m.cost[0],&nan_bits,4);
        m.gx[1]=-0.0f;
        input.goal_id=16;input.attempt_id=42;
        input.goal_stamp_ns=1788684359185188123ULL;
        input.start_stamp_ns=input.goal_stamp_ns+100;
        input.map_stamp_ns=input.goal_stamp_ns-100;
        input.start={-1.425,-1.425,0};input.goal={0.375,-1.425,0};input.frame="map";
        input.primitives.generate(SkidSteerModel{},PrimitiveGenConfig{});
        const auto snapshot=directory/"input.ggsnap";
        savePlanningSnapshot(snapshot.string(),input);
        const auto restored=loadPlanningSnapshot(snapshot.string());
        check(restored.goal_stamp_ns==input.goal_stamp_ns,"nanosecond identity");
        check(std::memcmp(m.cost.data(),restored.map.cost.data(),m.cost.size()*4)==0,"exact NaN bits");
        check(std::memcmp(m.gx.data(),restored.map.gx.data(),m.gx.size()*4)==0,"signed zero");
        check(restored.map.start_row==7 && restored.map.start_col==19,"circular indices");
        const auto again=directory/"again.ggsnap";savePlanningSnapshot(again.string(),restored);
        std::ifstream first(snapshot,std::ios::binary), second(again,std::ios::binary);
        check(std::string(std::istreambuf_iterator<char>(first),{})==
              std::string(std::istreambuf_iterator<char>(second),{}),"lossless whole-file round trip");
        const auto a=LatticePlannerCore(input).planCore(input.start,input.goal,10000);
        const auto b=LatticePlannerCore(restored).planCore(input.start,input.goal,10000);
        check(a.ok && b.ok && a.profile.data==b.profile.data && a.expanded==b.expanded,"replayed production result");
        bool refused=false;
        try { savePlanningSnapshot(snapshot.string(),input); } catch(const std::exception&) { refused=true; }
        check(refused,"never overwrite snapshots");
        auto damaged=directory/"damaged.ggsnap";
        std::filesystem::copy_file(snapshot,damaged);
        { std::fstream file(damaged,std::ios::binary|std::ios::in|std::ios::out);
          file.seekg(-1,std::ios::end); const int c=file.get();
          file.seekp(-1,std::ios::end);file.put(static_cast<char>(c^0x80)); }
        refused=false;
        try { (void)loadPlanningSnapshot(damaged.string()); } catch(const std::exception&) { refused=true; }
        check(refused,"corruption detected");
        PlanningSnapshotWriter writer; writer.start((directory/"async").string());
        check(writer.submit(input,a),"snapshot accepted");writer.finish();
        const auto stats=writer.stats();check(stats.written==1 && stats.failed==0 && stats.dropped==0,"async flush");
        check(std::filesystem::exists(directory/"async"/"writer-summary.json"),"writer summary");
        // Diagnostic patches use the production map geometry and must not confuse a
        // circular-buffer seam with the physical map edge or turn unknown into zero.
        int reads=0;
        const auto patch=terrainDiagnosticPatchJson(m,{0,0},input.goal_id,input.map_stamp_ns,
            [&](const char*,const PlanningIndex& idx) {
                ++reads; check(idx.a>=0 && idx.a<m.rows && idx.b>=0 && idx.b<m.cols,
                               "patch indices remain in buffer");
                return double(m.cost[idx.a*m.cols+idx.b]);
            });
        check(reads==25*13,"all 25 neighbouring cells across buffer seam are read");
        check(patch.find("\"index\":[39,39]")!=std::string::npos &&
              patch.find("\"index\":[1,1]")!=std::string::npos,"wrapped neighbours present");
        check(patch.find("\"values\":[null,null")!=std::string::npos,"unknown encoded as null");
        check(patch.find(std::to_string(input.map_stamp_ns))!=std::string::npos,
              "exact map timestamp retained");
        reads=0;
        const auto edge=terrainDiagnosticPatchJson(m,{m.start_row,m.start_col},16,100,
            [&](const char*,const PlanningIndex&) { ++reads; return 1.0; });
        check(reads==9*13 && edge.find("\"cells\":[null,null")!=std::string::npos,
              "physical edges are absent, not wrapped to opposite terrain");
        check(terrainDiagnosticPatchJson(m,{-1,0},16,100,
            [](const char*,const PlanningIndex&) { return 0.0; }).empty(),"invalid patch rejected");
        std::ofstream(directory/"terrain-patch.json")<<patch;
        std::ofstream(directory/"terrain-edge.json")<<edge;
        std::cout<<"planning_snapshot_selfcheck passed; artefacts: "<<directory.string()<<'\n';
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
