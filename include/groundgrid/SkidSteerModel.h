#pragma once

#include <cmath>

namespace groundgrid {

// Planar pose in the world/navigation frame (ENU-consistent). yaw in radians.
struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// Body-frame twist. vx = longitudinal, vy = lateral (left positive), omega = yaw rate.
struct BodyTwist {
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
};

// Skid-steer slip parameters, identified offline. Pure data, no ROS dependency.
struct SkidSteerParams {
    // ICR longitudinal offset [m]: couples yaw rate to lateral body velocity (vy = -x_icr * omega).
    double x_icr = 0.0;
    // Longitudinal slip efficiency: effective vx = alpha_v * v_cmd.
    double alpha_v = 1.0;
    // Yaw slip efficiency: effective omega = alpha_w * w_cmd.
    double alpha_w = 1.0;
    // Lateral downslope drift per unit lateral gradient (side-slip on regolith).
    double slope_slip_gain = 0.0;
    // Longitudinal slowdown per unit uphill gradient (grade resistance / slip).
    double slope_grade_gain = 0.0;

    // Operational command envelope, shared by primitive generation and the follower.
    double v_max = 1.39;      // ~5 km/h
    double w_max = 0.8;       // rad/s
    double a_max = 0.6;       // m/s^2 linear acceleration limit
    double alpha_max = 1.2;   // rad/s^2 yaw acceleration limit
    double kappa_max = 1.0;   // 1/m maximum path curvature
};

// Kinematic skid-steer model with slip and slope coupling.
// Deliberately free of ROS/grid_map types so it can be extracted to the Atlas target.
class SkidSteerModel {
public:
    SkidSteerModel() = default;
    explicit SkidSteerModel(const SkidSteerParams& params) : params_(params) {}

    void setParams(const SkidSteerParams& p) { params_ = p; }
    const SkidSteerParams& params() const { return params_; }

    // Map commanded (v_cmd, w_cmd) to the actual body twist under slip and local slope.
    // grad_long: terrain gradient along heading (dz/ds forward, +uphill).
    // grad_lat:  terrain gradient along the left axis (+left uphill).
    BodyTwist effectiveTwist(double v_cmd, double w_cmd,
                             double grad_long = 0.0, double grad_lat = 0.0) const;

    // Forward-integrate one step of the commanded motion in the world frame.
    Pose2D integrate(const Pose2D& pose, double v_cmd, double w_cmd, double dt,
                     double grad_long = 0.0, double grad_lat = 0.0) const;

    // Recover the command that nominally yields a desired body-frame (v, w) under slip.
    // Slope terms are ignored (feed-forward baseline for the follower).
    void inverseCommand(double v_des, double w_des, double& v_cmd, double& w_cmd) const;

    static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

private:
    SkidSteerParams params_{};
};

} // namespace groundgrid
