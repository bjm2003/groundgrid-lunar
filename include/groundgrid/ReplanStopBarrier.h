#pragma once

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <sstream>
#include <string>
#include <system_error>

namespace groundgrid {

// A revoked route must not keep driving while a replacement search blocks the planner.
// The follower acknowledges the exact empty atomic trajectory AFTER clearing its tracker
// and publishing zero velocity. Only then may the planner obtain a fresh start pose.
// This confirms the software stop command, not a physical braking-distance guarantee.
class ReplanStopBarrier {
public:
    void clear() { pending_=false; acknowledged_=false; goal_=0; stamp_=0; pose_after_=0; }
    void request(std::uint32_t goal, std::uint64_t stamp) {
        goal_=goal; stamp_=stamp; pose_after_=stamp; pending_=true; acknowledged_=false;
    }
    bool waiting() const { return pending_ && !acknowledged_; }
    bool pending() const { return pending_; }
    std::uint32_t goal() const { return goal_; }
    std::uint64_t stamp() const { return stamp_; }
    bool canSearch(std::uint64_t pose_stamp) const {
        return !pending_ || (acknowledged_ && pose_stamp>pose_after_);
    }
    bool observe(const std::string& snapshot,std::uint64_t acknowledgement_stamp) {
        if(!waiting() || goal_==0 || stamp_==0) return false;
        std::uint64_t goal=0, stamp=0;
        bool have_goal=false, have_stamp=false, have_status=false;
        std::string status, token;
        std::istringstream input(snapshot);
        while(input>>token) {
            const auto equals=token.find('=');
            if(equals==std::string::npos) continue;
            const auto key=token.substr(0,equals), value=token.substr(equals+1);
            if(key=="goal_id") {
                if(have_goal || !unsignedValue(value,goal)) return false;
                have_goal=true;
            } else if(key=="trajectory_stamp_ns") {
                if(have_stamp || !unsignedValue(value,stamp)) return false;
                have_stamp=true;
            } else if(key=="status") {
                if(have_status) return false;
                have_status=true; status=value;
            }
        }
        if(!have_goal || !have_stamp || !have_status || goal!=goal_ || stamp!=stamp_ ||
           status!="empty_trajectory") return false;
        // A TF created after publication but before the follower received the stop is
        // still pre-stop. Require a TF newer than the receipt of its acknowledgement.
        pose_after_=std::max(stamp_,acknowledgement_stamp);
        acknowledged_=true;
        return true;
    }

private:
    static bool unsignedValue(const std::string& text,std::uint64_t& value) {
        if(text.empty()) return false;
        const auto result=std::from_chars(text.data(),text.data()+text.size(),value);
        return result.ec==std::errc{} && result.ptr==text.data()+text.size();
    }
    bool pending_=false, acknowledged_=false;
    std::uint32_t goal_=0;
    std::uint64_t stamp_=0, pose_after_=0;
};

// Status deduplication must include trajectory identity: two empty trajectories for the
// same goal but different stamps each need their own acknowledgement.
inline bool followerStatusChanged(const std::string& status,std::uint64_t stamp,
                                  const std::string& previous,std::uint64_t previous_stamp) {
    return status!=previous || stamp!=previous_stamp;
}

} // namespace groundgrid
