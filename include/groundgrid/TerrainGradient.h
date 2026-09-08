#pragma once

#include <cmath>
#include <limits>

namespace groundgrid {

struct TerrainGradient {
    float x = std::numeric_limits<float>::quiet_NaN();
    float y = std::numeric_limits<float>::quiet_NaN();
};

struct TerrainHeightSample { double x,y,z; };

// Least-squares plane at actual return coordinates. Centre the coordinates before
// solving so large world positions do not degrade conditioning. Missing/degenerate
// measurements are not evidence for a horizontal surface.
template<class SampleAt>
TerrainGradient estimateMeasuredTerrainGradient(SampleAt sample) {
    TerrainHeightSample points[9];
    double mx=0,my=0,mz=0; int n=0;
    for(int i=-1;i<=1;++i) for(int j=-1;j<=1;++j) {
        const auto p=sample(i,j);
        if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return {};
        points[n++]=p; mx+=p.x; my+=p.y; mz+=p.z;
    }
    mx/=9; my/=9; mz/=9;
    double xx=0,xy=0,yy=0,xz=0,yz=0;
    for(const auto& p:points) {
        const double x=p.x-mx,y=p.y-my,z=p.z-mz;
        xx+=x*x; xy+=x*y; yy+=y*y; xz+=x*z; yz+=y*z;
    }
    const double det=xx*yy-xy*xy;
    if(!std::isfinite(det) || xx<=0 || yy<=0 || det<=1e-9*xx*yy) return {};
    return {static_cast<float>((xz*yy-yz*xy)/det),
            static_cast<float>((yz*xx-xz*xy)/det)};
}

// Least-squares plane on a complete 3x3 or 5x5 neighbourhood. Coordinates here are
// matrix axes, matching the previous derivative convention. Do not modify height,
// step range, roughness or obstacle evidence: a discontinuity is not a safe ramp.
// Missing samples are not invented or replaced by a zero/safe gradient.
template<class HeightAt>
TerrainGradient estimateTerrainGradient(double resolution, HeightAt height, int radius=1) {
    if(!std::isfinite(resolution) || resolution<=0 || (radius!=1 && radius!=2)) return {};
    double sx=0, sy=0;
    double normalizer=0;
    for(int i=-radius;i<=radius;++i) for(int j=-radius;j<=radius;++j) {
        const double z=height(i,j);
        if(!std::isfinite(z)) return {};
        sx+=i*z; sy+=j*z;
        normalizer+=i*i;
    }
    // Symmetric coordinates remove the intercept; sums are 6 (3x3), 50 (5x5).
    return {static_cast<float>(sx/(normalizer*resolution)),
            static_cast<float>(sy/(normalizer*resolution))};
}

template<class HeightAt>
TerrainGradient estimateSupportedTerrainGradient(double resolution, HeightAt height,
                                                 bool outer_window_available) {
    auto g=estimateTerrainGradient(resolution,height);
    if(!outer_window_available || !std::isfinite(g.x) || !std::isfinite(g.y)) return g;
    const auto wider=estimateTerrainGradient(resolution,height,2);
    return std::isfinite(wider.x) && std::isfinite(wider.y) ? wider : g;
}

}  // namespace groundgrid
