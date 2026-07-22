#include "groundgrid/MotionPrimitiveLibrary.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace groundgrid {

const std::vector<MotionPrimitive> MotionPrimitiveLibrary::kEmpty;

namespace {

constexpr double kTwoPi = 6.28318530717958647692;

int wrapBin(int b, int bins) {
    b %= bins;
    if (b < 0) b += bins;
    return b;
}

// Integrate one commanded (v, w) over the horizon in the start-body frame (no slope:
// primitive shape is nominal; terrain cost is applied online by the planner).
MotionPrimitive buildPrimitive(const SkidSteerModel& model, int start_bin, int bins,
                               double v_cmd, double w_cmd, int direction,
                               double horizon, double dt,
                               double reverse_penalty, double rotation_penalty) {
    MotionPrimitive prim;
    prim.start_bin = start_bin;
    prim.direction = direction;

    const int steps = std::max(1, static_cast<int>(std::lround(horizon / dt)));
    Pose2D p{0.0, 0.0, 0.0};
    prim.samples.reserve(steps);
    prim.v_profile.reserve(steps);
    prim.w_profile.reserve(steps);

    double length = 0.0;
    for (int i = 0; i < steps; ++i) {
        const Pose2D prev = p;
        p = model.integrate(p, v_cmd, w_cmd, dt);
        length += std::hypot(p.x - prev.x, p.y - prev.y);
        prim.samples.push_back(p);
        prim.v_profile.push_back(v_cmd);
        prim.w_profile.push_back(w_cmd);
    }

    prim.dx = p.x;
    prim.dy = p.y;
    prim.dyaw = SkidSteerModel::wrap(p.yaw);
    prim.length = length;

    const double bin_step = kTwoPi / bins;
    const int delta = static_cast<int>(std::lround(prim.dyaw / bin_step));
    prim.end_bin = wrapBin(start_bin + delta, bins);

    if (direction == 0) {
        prim.base_cost = rotation_penalty * std::abs(prim.dyaw) * 0.3;
    } else {
        const double dir_penalty = direction < 0 ? reverse_penalty : 1.0;
        prim.base_cost = length * dir_penalty + 0.15 * std::abs(prim.dyaw);
    }
    return prim;
}

} // namespace

void MotionPrimitiveLibrary::generate(const SkidSteerModel& model, const PrimitiveGenConfig& cfg) {
    bins_ = cfg.heading_bins;
    by_bin_.assign(bins_, {});

    const SkidSteerParams& sp = model.params();
    const double w_lim = std::min(sp.w_max, sp.v_max * sp.kappa_max);

    // Angular-velocity levels for arc primitives: straight + symmetric turns.
    std::vector<double> w_levels;
    w_levels.push_back(0.0);
    for (int k = 1; k <= cfg.curvature_samples; ++k) {
        const double frac = static_cast<double>(k) / cfg.curvature_samples;
        w_levels.push_back(frac * w_lim);
        w_levels.push_back(-frac * w_lim);
    }

    for (int b = 0; b < bins_; ++b) {
        auto& list = by_bin_[b];

        // Forward and (optionally) reverse arcs.
        std::vector<int> dirs = {1};
        if (cfg.enable_reverse) dirs.push_back(-1);
        for (int dir : dirs) {
            const double v_cmd = dir * sp.v_max;
            for (double w : w_levels) {
                list.push_back(buildPrimitive(model, b, bins_, v_cmd, w, dir,
                                              cfg.horizon, cfg.dt,
                                              cfg.reverse_penalty, cfg.rotation_penalty));
            }
        }

        // In-place rotations.
        if (cfg.enable_in_place) {
            for (double w : {sp.w_max, -sp.w_max}) {
                list.push_back(buildPrimitive(model, b, bins_, 0.0, w, 0,
                                              cfg.horizon, cfg.dt,
                                              cfg.reverse_penalty, cfg.rotation_penalty));
            }
        }
    }
}

const std::vector<MotionPrimitive>& MotionPrimitiveLibrary::primitivesFor(int bin) const {
    if (bin < 0 || bin >= bins_ || bin >= static_cast<int>(by_bin_.size())) return kEmpty;
    return by_bin_[bin];
}

bool MotionPrimitiveLibrary::save(const std::string& path) const {
    std::ofstream out(path);
    if (!out) return false;
    out.precision(9);

    std::size_t total = 0;
    for (const auto& list : by_bin_) total += list.size();
    out << "MPL " << bins_ << ' ' << total << '\n';

    for (const auto& list : by_bin_) {
        for (const auto& p : list) {
            out << "P " << p.start_bin << ' ' << p.end_bin << ' ' << p.direction << ' '
                << p.dx << ' ' << p.dy << ' ' << p.dyaw << ' ' << p.length << ' '
                << p.base_cost << ' ' << p.samples.size() << '\n';
            for (std::size_t i = 0; i < p.samples.size(); ++i) {
                out << p.samples[i].x << ' ' << p.samples[i].y << ' ' << p.samples[i].yaw
                    << ' ' << p.v_profile[i] << ' ' << p.w_profile[i] << '\n';
            }
        }
    }
    return static_cast<bool>(out);
}

bool MotionPrimitiveLibrary::load(const std::string& path) {
    std::ifstream in(path);
    if (!in) return false;

    std::string tag;
    std::size_t total = 0;
    in >> tag >> bins_ >> total;
    if (tag != "MPL" || bins_ <= 0) return false;

    by_bin_.assign(bins_, {});
    for (std::size_t n = 0; n < total; ++n) {
        std::string p_tag;
        in >> p_tag;
        if (p_tag != "P") return false;
        MotionPrimitive p;
        std::size_t nsamples = 0;
        in >> p.start_bin >> p.end_bin >> p.direction >> p.dx >> p.dy >> p.dyaw
           >> p.length >> p.base_cost >> nsamples;
        p.samples.resize(nsamples);
        p.v_profile.resize(nsamples);
        p.w_profile.resize(nsamples);
        for (std::size_t i = 0; i < nsamples; ++i) {
            in >> p.samples[i].x >> p.samples[i].y >> p.samples[i].yaw
               >> p.v_profile[i] >> p.w_profile[i];
        }
        if (!in) return false;
        if (p.start_bin >= 0 && p.start_bin < bins_) by_bin_[p.start_bin].push_back(std::move(p));
    }
    return true;
}

} // namespace groundgrid
