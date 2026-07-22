#pragma once

#include <vector>

#include "groundgrid/SkidSteerModel.h"

namespace groundgrid {

// A single dynamics-feasible motion primitive, generated offline by forward-
// simulating the SkidSteerModel from a discrete start heading bin.
//
// Poses are expressed in the primitive's start-body frame (start at origin, heading
// along +x). The planner rotates/translates them to the world frame at expansion time.
struct MotionPrimitive {
    int start_bin = 0;              // heading bin the primitive departs from
    int end_bin = 0;                // heading bin at the endpoint
    int direction = 1;              // +1 forward, -1 reverse, 0 in-place rotation
    double dx = 0.0;                // endpoint offset in start-body frame [m]
    double dy = 0.0;
    double dyaw = 0.0;              // net heading change [rad]
    double length = 0.0;            // travelled path length [m]
    double base_cost = 0.0;         // intrinsic cost (length, turn, reverse penalty)
    std::vector<Pose2D> samples;    // intermediate poses in start-body frame (incl. endpoint)
    std::vector<double> v_profile;  // commanded linear velocity per sample [m/s]
    std::vector<double> w_profile;  // commanded angular velocity per sample [rad/s]
};

} // namespace groundgrid
