#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace groundgrid {

struct PlanningIndex {
    int a=0, b=0;
    PlanningIndex() = default;
    PlanningIndex(int x,int y) : a(x), b(y) {}
    int& operator()(int axis) { return axis==0 ? a : b; }
    int operator()(int axis) const { return axis==0 ? a : b; }
};
struct PlanningPosition {
    double a=0.0, b=0.0;
    PlanningPosition() = default;
    PlanningPosition(double x,double y) : a(x), b(y) {}
    double x() const { return a; }
    double y() const { return b; }
    double norm() const { return std::sqrt(a*a+b*b); }
    PlanningPosition operator-(const PlanningPosition& p) const { return {a-p.a,b-p.b}; }
};

// Plain-data view of grid_map's circular buffer. Arrays use row*cols+col, NOT
// Eigen's native column-major storage. World axes run opposite to buffer axes.
// Keep the captured length (not a recomputed rows*resolution) and arithmetic
// order to match GridMapMath's boundary quantisation. ROS parity is tested too.
struct PlanningGrid {
    int rows=0, cols=0, start_row=0, start_col=0;
    double resolution=0.0, center_x=0.0, center_y=0.0, length_x=0.0, length_y=0.0;
    std::vector<float> cost, gx, gy, slope;

    bool valid() const {
        if(rows<=0 || cols<=0 || rows>10000 || cols>10000 ||
           start_row<0 || start_row>=rows || start_col<0 || start_col>=cols ||
           !std::isfinite(resolution) || resolution<=0.0 ||
           !std::isfinite(center_x) || !std::isfinite(center_y) ||
           !std::isfinite(length_x) || !std::isfinite(length_y) ||
           length_x<=0.0 || length_y<=0.0) return false;
        const auto n=static_cast<std::size_t>(rows)*cols;
        return n<=4000000 && cost.size()==n && gx.size()==n && gy.size()==n && slope.size()==n &&
               std::abs(length_x-rows*resolution)<1e-6 &&
               std::abs(length_y-cols*resolution)<1e-6;
    }
    double getResolution() const { return resolution; }
    PlanningIndex getSize() const { return {rows,cols}; }
    PlanningPosition getPosition() const { return {center_x,center_y}; }
    PlanningPosition getLength() const { return {length_x,length_y}; }
    bool getPosition(const PlanningIndex& i,PlanningPosition& p) const {
        if(i.a<0 || i.b<0 || i.a>=rows || i.b>=cols) return false;
        const int r=(i.a-start_row+rows)%rows, c=(i.b-start_col+cols)%cols;
        p={center_x+(0.5*length_x-0.5*resolution)-resolution*r,
           center_y+(0.5*length_y-0.5*resolution)-resolution*c};
        return true;
    }
    bool getIndex(const PlanningPosition& p,PlanningIndex& i) const {
        if(rows<=0 || cols<=0 || !std::isfinite(p.a) || !std::isfinite(p.b) ||
           !std::isfinite(resolution) || resolution<=0.0) return false;
        const double px=-(p.a-center_x-0.5*length_x);
        const double py=-(p.b-center_y-0.5*length_y);
        if(px<0.0 || py<0.0 || px>=length_x || py>=length_y) return false;
        const double rx=-(p.a-0.5*length_x-center_x)/resolution;
        const double cy=-(p.b-0.5*length_y-center_y)/resolution;
        if(rx<0.0 || cy<0.0 || rx>rows+1.0 || cy>cols+1.0) return false;
        i={(static_cast<int>(rx)+start_row)%rows,(static_cast<int>(cy)+start_col)%cols};
        return true;
    }
};

} // namespace groundgrid
