#pragma once

#include <cmath>

namespace groundgrid {

// Terrain directly under the rover cannot be re-observed by a roof-mounted LiDAR.  The
// rover pose nevertheless supplies a physical support plane there: the base xy plane in
// map coordinates.  Keeping this small calculation ROS-free makes the safety assumption
// explicit and independently testable.
class BlindZoneSupportPlane {
public:
    static BlindZoneSupportPlane fromPose(double x, double y, double z,
                                          double qx, double qy, double qz, double qw) {
        BlindZoneSupportPlane plane;
        if(!finite(x) || !finite(y) || !finite(z) ||
           !finite(qx) || !finite(qy) || !finite(qz) || !finite(qw)) return plane;
        const double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        if(!(norm > 1e-12) || !finite(norm)) return plane;
        qx /= norm; qy /= norm; qz /= norm; qw /= norm;

        // Third column of the quaternion rotation matrix: base-z expressed in map.
        plane.nx_ = 2.0*(qx*qz + qw*qy);
        plane.ny_ = 2.0*(qy*qz - qw*qx);
        plane.nz_ = 1.0 - 2.0*(qx*qx + qy*qy);
        if(std::abs(plane.nz_) < 1e-6 || !finite(plane.nx_) ||
           !finite(plane.ny_) || !finite(plane.nz_)) return BlindZoneSupportPlane{};
        plane.x_ = x; plane.y_ = y; plane.z_ = z; plane.valid_ = true;
        return plane;
    }

    bool heightForUnmeasuredCell(double x, double y,
                                 double mask_x, double mask_y, double exclusion_radius,
                                 double usable_points, double historical_height,
                                 double& height) const {
        if(!valid_ || !finite(x) || !finite(y) || !finite(mask_x) || !finite(mask_y) ||
           !finite(exclusion_radius) ||
           !(exclusion_radius > 0.0) || !finite(usable_points) || usable_points > 0.0)
            return false;
        const double mask_dx=x-mask_x, mask_dy=y-mask_y;
        if(mask_dx*mask_dx + mask_dy*mask_dy >
           exclusion_radius*exclusion_radius + 1e-12) return false;
        const double dx=x-x_, dy=y-y_;
        // A direct historical height is world-fixed evidence and wins.  The base plane
        // only fills cells that have never had such a return, so a previously measured
        // rock or step is not erased merely because it is under the rover now.
        height = finite(historical_height)
            ? historical_height
            : z_ - (nx_*dx + ny_*dy)/nz_;
        return finite(height);
    }

    bool valid() const { return valid_; }
    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }

private:
    static bool finite(double value) { return std::isfinite(value); }

    double x_=0.0, y_=0.0, z_=0.0;
    double nx_=0.0, ny_=0.0, nz_=1.0;
    bool valid_=false;
};

}  // namespace groundgrid
