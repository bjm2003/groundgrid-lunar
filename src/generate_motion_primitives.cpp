// Offline motion-primitive library generator (no ROS runtime).
// Reads the skid_steer_model block from the system yaml and writes a primitive file.
//
// Usage: generate_motion_primitives [config.yaml] [output.dat]
#include "groundgrid/MotionPrimitiveLibrary.h"
#include "groundgrid/SkidSteerModel.h"

#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

namespace {

// Minimal reader for a flat "key: value" block in a YAML file (single source of truth
// with the ROS params). Returns numeric values found under `block_name:`.
std::map<std::string, double> readYamlBlock(const std::string& path, const std::string& block_name) {
    std::map<std::string, double> out;
    std::ifstream in(path);
    if (!in) return out;

    std::string line;
    bool in_block = false;
    while (std::getline(in, line)) {
        // Strip inline comments.
        const auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;

        const bool indented = (line[0] == ' ' || line[0] == '\t');
        if (!indented) {
            std::string trimmed = line;
            const auto colon = trimmed.find(':');
            in_block = (colon != std::string::npos &&
                        trimmed.substr(0, colon) == block_name);
            continue;
        }
        if (!in_block) continue;

        const auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string key = line.substr(0, colon);
        std::string val = line.substr(colon + 1);
        // trim
        auto trim = [](std::string& s) {
            const auto a = s.find_first_not_of(" \t\r\n");
            const auto b = s.find_last_not_of(" \t\r\n");
            if (a == std::string::npos) { s.clear(); return; }
            s = s.substr(a, b - a + 1);
        };
        trim(key);
        trim(val);
        if (key.empty() || val.empty()) continue;
        if (val == "true") { out[key] = 1.0; continue; }
        if (val == "false") { out[key] = 0.0; continue; }
        try { out[key] = std::stod(val); } catch (...) { /* skip non-numeric */ }
    }
    return out;
}

double get(const std::map<std::string, double>& m, const std::string& k, double def) {
    auto it = m.find(k);
    return it == m.end() ? def : it->second;
}

} // namespace

int main(int argc, char** argv) {
    const std::string config = argc > 1 ? argv[1] : "config/lunar_system.yaml";
    const std::string output = argc > 2 ? argv[2] : "config/motion_primitives.dat";

    const auto ss = readYamlBlock(config, "skid_steer_model");
    const auto planner = readYamlBlock(config, "state_lattice_planner");

    groundgrid::SkidSteerParams params;
    params.x_icr = get(ss, "x_icr", params.x_icr);
    params.alpha_v = get(ss, "alpha_v", params.alpha_v);
    params.alpha_w = get(ss, "alpha_w", params.alpha_w);
    params.slope_slip_gain = get(ss, "slope_slip_gain", params.slope_slip_gain);
    params.slope_grade_gain = get(ss, "slope_grade_gain", params.slope_grade_gain);
    params.v_max = get(ss, "v_max", params.v_max);
    params.w_max = get(ss, "w_max", params.w_max);
    params.a_max = get(ss, "a_max", params.a_max);
    params.alpha_max = get(ss, "alpha_max", params.alpha_max);
    params.kappa_max = get(ss, "kappa_max", params.kappa_max);

    groundgrid::PrimitiveGenConfig cfg;
    cfg.heading_bins = static_cast<int>(get(planner, "heading_bins", cfg.heading_bins));
    cfg.horizon = get(ss, "horizon", cfg.horizon);
    cfg.dt = get(ss, "dt", cfg.dt);
    cfg.curvature_samples = static_cast<int>(get(ss, "curvature_samples", cfg.curvature_samples));
    cfg.enable_reverse = get(ss, "enable_reverse", 1.0) != 0.0;
    cfg.enable_in_place = get(ss, "enable_in_place", 1.0) != 0.0;

    groundgrid::SkidSteerModel model(params);
    groundgrid::MotionPrimitiveLibrary lib;
    lib.generate(model, cfg);

    if (!lib.save(output)) {
        std::fprintf(stderr, "Failed to write primitive library to %s\n", output.c_str());
        return 1;
    }

    std::size_t total = 0;
    for (int b = 0; b < cfg.heading_bins; ++b) total += lib.primitivesFor(b).size();
    std::printf("Generated %zu primitives (%d bins, v_max=%.2f m/s) -> %s\n",
                total, cfg.heading_bins, params.v_max, output.c_str());
    return 0;
}
