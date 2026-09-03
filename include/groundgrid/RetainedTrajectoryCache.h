#pragma once

namespace groundgrid {

// The last accepted route serves two different purposes: a continuously executing route
// may be republished, while an interrupted route is retained only as inactive bookkeeping.
// Replaying that history as a new route would reset the follower to its already completed
// first rotation/cusp. Back-out now uses separate actual localisation history, not this
// planned route. Keep publication continuity explicit instead of inferring it from
// the rover being near some point on the old route. Path/Profile are ROS-free test seams.
template<typename Path, typename Profile>
class RetainedTrajectoryCache {
public:
    void clear() {
        path_ = Path{};
        profile_ = Profile{};
        was_snapped_ = false;
        reusable_ = false;
    }

    // Called for EVERY control publication, after message validity checks. Retain only a
    // valid nominal goal route, a confirmed recovery goal route, or its validated repeat.
    // Stops, invalid output, unconfirmed recovery plans and manoeuvres revoke reuse while
    // preserving the historical route as inactive data, never an executable back-out route.
    void published(const Path& path, const Profile& profile, bool was_snapped, bool retain) {
        reusable_ = retain;
        if(retain) {
            path_ = path;
            profile_ = profile;
            was_snapped_ = was_snapped;
        }
    }

    bool reusable() const { return reusable_; }
    const Path& path() const { return path_; }
    const Profile& profile() const { return profile_; }
    bool wasSnapped() const { return was_snapped_; }

private:
    Path path_;
    Profile profile_;
    bool was_snapped_ = false;
    bool reusable_ = false;
};

} // namespace groundgrid
