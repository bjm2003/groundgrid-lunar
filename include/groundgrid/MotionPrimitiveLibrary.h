#pragma once

#include <string>
#include <vector>

#include "groundgrid/MotionPrimitive.h"
#include "groundgrid/SkidSteerModel.h"

namespace groundgrid {

// Settings controlling offline primitive generation.
struct PrimitiveGenConfig {
    int heading_bins = 16;         // must match the planner's heading_bins
    double horizon = 0.9;          // integration horizon per primitive [s]
    double dt = 0.05;              // integration step [s]
    int curvature_samples = 3;     // number of turn levels per side (excludes straight)
    bool enable_reverse = true;    // generate reverse variants
    bool enable_in_place = true;   // generate in-place rotations
    double reverse_penalty = 1.3;  // cost multiplier for reverse motion
    double rotation_penalty = 1.5; // cost multiplier for in-place rotation
};

// Library of dynamics-feasible motion primitives indexed by start heading bin.
// Pure C++/STL (no ROS), so it can be extracted from the ROS stack for the Atlas target.
class MotionPrimitiveLibrary {
public:
    // Generate the full library by forward-simulating the model from each heading bin.
    void generate(const SkidSteerModel& model, const PrimitiveGenConfig& cfg);

    // O(1) lookup of primitives departing from a heading bin.
    const std::vector<MotionPrimitive>& primitivesFor(int bin) const;

    // Persist / restore in a compact text format (zero external dependencies).
    bool save(const std::string& path) const;
    bool load(const std::string& path);

    // Lossless in-memory restore for planning snapshots (no text rounding or IO).
    // Validate before replacing the existing library; a failed restore leaves it intact.
    bool restore(int bins, const std::vector<MotionPrimitive>& primitives);

    int headingBins() const { return bins_; }
    bool empty() const { return by_bin_.empty(); }

private:
    int bins_ = 0;
    std::vector<std::vector<MotionPrimitive>> by_bin_;
    static const std::vector<MotionPrimitive> kEmpty;
};

} // namespace groundgrid
