#pragma once

#include <cmath>
#include <limits>

namespace groundgrid {

struct TerrainGradient {
    float x = std::numeric_limits<float>::quiet_NaN();
    float y = std::numeric_limits<float>::quiet_NaN();
};

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
