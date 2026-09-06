#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include "groundgrid/LatticePlannerCore.h"

namespace groundgrid {

// Version 2 (v1 readable as legacy snapping): little-endian scalars; exact IEEE float payloads including
// NaNs; named configuration; full in-memory primitives. A rolling checksum rejects
// damaged/truncated files. This is an integrity check, not an authenticity claim.
void savePlanningSnapshot(const std::string& path,const PlanningInput& input);
PlanningInput loadPlanningSnapshot(const std::string& path);
std::string planningResultJson(const PlanningInput& input,const PlanningResult& result);

class PlanningSnapshotWriter {
public:
    struct Stats { std::uint64_t submitted=0,written=0,dropped=0,failed=0; };
    PlanningSnapshotWriter() = default;
    ~PlanningSnapshotWriter();
    PlanningSnapshotWriter(const PlanningSnapshotWriter&)=delete;
    PlanningSnapshotWriter& operator=(const PlanningSnapshotWriter&)=delete;
    // Empty directory disables recording. Configure once, before callbacks start.
    void start(const std::string& directory);
    bool enabled() const { return !directory_.empty(); }
    bool submit(PlanningInput input,PlanningResult result);
    Stats stats() const;
    void finish();
private:
    struct Item { PlanningInput input; PlanningResult result; };
    void run();
    std::string directory_;
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<Item> queue_;
    std::thread worker_;
    Stats stats_;
    bool stopping_=false;
};
} // namespace groundgrid
