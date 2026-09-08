#pragma once

#include <cmath>
#include <limits>

namespace groundgrid {

struct TerrainGradient {
    float x = std::numeric_limits<float>::quiet_NaN();
    float y = std::numeric_limits<float>::quiet_NaN();
};

// Least-squares plane on the EXISTING 3x3 neighbourhood. Coordinates here are
// matrix axes, matching the previous derivative convention. Do not modify height,
// step range, roughness or obstacle evidence: a discontinuity is not a safe ramp.
// Missing samples are not invented or replaced by a zero/safe gradient.
template<class HeightAt>
TerrainGradient estimateTerrainGradient(double resolution, HeightAt height) {
    if(!std::isfinite(resolution) || resolution<=0) return {};
    double sx=0, sy=0;
    for(int i=-1;i<=1;++i) for(int j=-1;j<=1;++j) {
        const double z=height(i,j);
        if(!std::isfinite(z)) return {};
        sx+=i*z; sy+=j*z;
    }
    // Symmetric coordinates remove the intercept; sum(i*i)=sum(j*j)=6.
    return {static_cast<float>(sx/(6*resolution)),
            static_cast<float>(sy/(6*resolution))};
}

}  // namespace groundgrid
