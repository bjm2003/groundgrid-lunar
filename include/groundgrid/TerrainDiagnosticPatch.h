#pragma once

#include <cstdint>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include "groundgrid/PlanningGrid.h"

namespace groundgrid {

// Observability only. Read every layer synchronously from the same immutable map
// used by the rejection check. No ground-truth data, interpolation or safety policy.
// A 5x5 patch contains the 3x3 derivative stencil and its one-pass smoothing inputs.
// Missing/non-finite values are JSON null, never fabricated zero terrain.
template<class LayerAt>
std::string terrainDiagnosticPatchJson(const PlanningGrid& map, PlanningIndex center,
                                      std::uint64_t goal_id, std::uint64_t stamp_ns,
                                      LayerAt layer_at) {
    if(!map.valid() || center.a<0 || center.b<0 ||
       center.a>=map.rows || center.b>=map.cols) return {};
    constexpr const char* layers[]={"ground","ground_corrected","elevation_raw",
        "groundpatch","observed","observation_age","pointsRaw","points",
        "slope_x","slope_y","terrain_cost","step_height","roughness"};
    std::ostringstream out;
    out.imbue(std::locale::classic()); out<<std::setprecision(17);
    const auto number=[&](double v) { if(std::isfinite(v)) out<<v; else out<<"null"; };
    out<<"{\"schema_version\":1,\"radius_cells\":2,\"goal_id\":"<<goal_id<<",\"map_stamp_ns\":"<<stamp_ns
       <<",\"center_index\":["<<center.a<<','<<center.b<<"],\"resolution\":";
    number(map.resolution);
    out<<",\"layers\":[";
    for(std::size_t i=0;i<sizeof(layers)/sizeof(layers[0]);++i) {
        if(i) out<<',';
        out<<'"'<<layers[i]<<'"';
    }
    out<<"],\"cells\":[";
    const int row=(center.a-map.start_row+map.rows)%map.rows;
    const int col=(center.b-map.start_col+map.cols)%map.cols;
    bool first=true;
    for(int dr=-2;dr<=2;++dr) for(int dc=-2;dc<=2;++dc) {
        if(!first) out<<',';
        first=false;
        // Buffer seams wrap, physical map edges do not.
        if(row+dr<0 || row+dr>=map.rows || col+dc<0 || col+dc>=map.cols) {
            out<<"null"; continue;
        }
        const PlanningIndex index((row+dr+map.start_row)%map.rows,
                                  (col+dc+map.start_col)%map.cols);
        PlanningPosition world; map.getPosition(index,world);
        out<<"{\"index\":["<<index.a<<','<<index.b<<"],\"xy\":[";
        number(world.x()); out<<','; number(world.y()); out<<"],\"values\":[";
        for(std::size_t i=0;i<sizeof(layers)/sizeof(layers[0]);++i) {
            if(i) out<<',';
            number(layer_at(layers[i],index));
        }
        out<<"]}";
    }
    out<<"]}";
    return out.str();
}

} // namespace groundgrid
