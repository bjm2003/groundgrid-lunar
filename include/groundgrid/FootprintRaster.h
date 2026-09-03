#pragma once

#include <cmath>
#include <cstdint>
#include <initializer_list>

#include "groundgrid/SkidSteerModel.h"

namespace groundgrid {

// Visit each map cell whose CLOSED square touches the oriented body rectangle plus
// margin. origin is a grid CORNER, not a cell centre; it follows the rolling map.
//
// Sampling points in body coordinates is not a rasterisation: rotating that point
// lattice leaves holes, and changing margin shifts it. A 0.50 m sampled band could
// miss a lethal cell found by the smaller 0.25 m band, even on the SAME map.
// Rectangle-square intersection keeps coverage monotone as the margin grows. The
// square is checked on all four separating axes; this is not a circumscribed circle.
// Returning false from visit rejects immediately. Invalid/oversized geometry fails
// closed without calling visit. Unknown/lethal/slope policy belongs to the caller.
template<typename VisitCell>
bool visitFootprintCells(const Pose2D& pose,double length,double width,double margin,
                         double resolution,double origin_x,double origin_y,
                         VisitCell visit) {
    if(!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
       !std::isfinite(length) || length<=0 || !std::isfinite(width) || width<=0 ||
       !std::isfinite(margin) || margin<0 || !std::isfinite(resolution) || resolution<=0 ||
       !std::isfinite(origin_x) || !std::isfinite(origin_y)) return false;
    const double hl=length/2+margin, hw=width/2+margin, h=resolution/2;
    const double c=std::cos(pose.yaw), s=std::sin(pose.yaw);
    const double ac=std::abs(c), as=std::abs(s);
    const double ex=ac*hl+as*hw, ey=as*hl+ac*hw;
    const double eps=resolution*1e-9;
    // Include cells touching either boundary, also at negative coordinates. Cap index
    // magnitude before integer conversion so enormous finite inputs cannot overflow.
    const double ix0=std::ceil((pose.x-ex-origin_x-eps)/resolution)-1;
    const double ix1=std::floor((pose.x+ex-origin_x+eps)/resolution);
    const double iy0=std::ceil((pose.y-ey-origin_y-eps)/resolution)-1;
    const double iy1=std::floor((pose.y+ey-origin_y+eps)/resolution);
    constexpr double max_index=4503599627370496.0; // 2^52: unit indices remain exact
    for(double bound : {ix0,ix1,iy0,iy1})
        if(!std::isfinite(bound) || std::abs(bound)>=max_index) return false;
    const double nx=ix1-ix0+1, ny=iy1-iy0+1;
    if(nx<=0 || ny<=0 || nx*ny>1000000) return false;
    const auto x0=static_cast<std::int64_t>(ix0), x1=static_cast<std::int64_t>(ix1);
    const auto y0=static_cast<std::int64_t>(iy0), y1=static_cast<std::int64_t>(iy1);
    const double projected_cell=h*(ac+as);
    for(auto ix=x0; ix<=x1; ++ix) {
        const double wx=origin_x+(double(ix)+.5)*resolution, dx=wx-pose.x;
        if(std::abs(dx)>ex+h+eps) continue;
        for(auto iy=y0; iy<=y1; ++iy) {
            const double wy=origin_y+(double(iy)+.5)*resolution, dy=wy-pose.y;
            if(std::abs(dy)>ey+h+eps ||
               std::abs(c*dx+s*dy)>hl+projected_cell+eps ||
               std::abs(-s*dx+c*dy)>hw+projected_cell+eps) continue;
            if(!visit(wx,wy)) return false;
        }
    }
    return true;
}

} // namespace groundgrid
