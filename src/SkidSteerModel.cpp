#include "groundgrid/SkidSteerModel.h"

#include <algorithm>

namespace groundgrid {

BodyTwist SkidSteerModel::effectiveTwist(double v_cmd, double w_cmd,
                                         double grad_long, double grad_lat) const {
    BodyTwist t;
    // Uphill grade resistance / slip reduces forward progress; never reverses sign.
    const double uphill_scale = std::max(0.0, 1.0 - params_.slope_grade_gain * std::max(0.0, grad_long));
    t.vx = params_.alpha_v * v_cmd * uphill_scale;
    t.omega = params_.alpha_w * w_cmd;
    // Lateral velocity: kinematic ICR coupling to yaw rate + downslope side drift.
    t.vy = -params_.x_icr * t.omega - params_.slope_slip_gain * grad_lat;
    return t;
}

Pose2D SkidSteerModel::integrate(const Pose2D& pose, double v_cmd, double w_cmd, double dt,
                                 double grad_long, double grad_lat) const {
    const BodyTwist t = effectiveTwist(v_cmd, w_cmd, grad_long, grad_lat);
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    // Rotate the body-frame velocity into the world frame, then integrate.
    const double wx = c * t.vx - s * t.vy;
    const double wy = s * t.vx + c * t.vy;
    Pose2D next;
    next.x = pose.x + wx * dt;
    next.y = pose.y + wy * dt;
    next.yaw = wrap(pose.yaw + t.omega * dt);
    return next;
}

void SkidSteerModel::inverseCommand(double v_des, double w_des,
                                    double& v_cmd, double& w_cmd) const {
    v_cmd = (std::abs(params_.alpha_v) > 1e-6) ? v_des / params_.alpha_v : v_des;
    w_cmd = (std::abs(params_.alpha_w) > 1e-6) ? w_des / params_.alpha_w : w_des;
    v_cmd = std::clamp(v_cmd, -params_.v_max, params_.v_max);
    w_cmd = std::clamp(w_cmd, -params_.w_max, params_.w_max);
}

} // namespace groundgrid
