"""Goal-correlated ROS-free telemetry reduction for the pipeline test.

Legacy String statuses have no goal identity and cannot safely end a trial. Planner
snapshots carry the requested goal stamp, its execution id, state and per-goal counters
atomically. Follower snapshots and atomic trajectories refer to that execution id.
Callers provide locking; no callback arrival time is treated as message ownership.
"""

COUNTERS = ("recovery_events", "recovery_successes", "recovery_aborts")


def fields_of(text):
    return dict(token.split("=", 1) for token in text.split() if "=" in token)


def positive_int(value):
    try:
        parsed = int(value)
        return parsed if parsed > 0 else None
    except (TypeError, ValueError):
        return None


class TrialObservation:
    def __init__(self, goal_stamp_ns=None):
        self.goal_stamp_ns = positive_int(goal_stamp_ns)
        self.goal_id = None
        self.status = None
        self.statuses = set()
        self.diagnostics = {}
        self.counters = dict.fromkeys(COUNTERS, 0)
        self.planner_done = False
        self.follower_done = False
        self.follower_tracking = False
        self.follower_stale = False
        self._planner_seq = 0
        self._follower_seq = 0
        self._pending_follower = []

    def owns(self, goal_id):
        return self.goal_id is not None and positive_int(goal_id) == self.goal_id

    def planner(self, fields):
        if (self.goal_stamp_ns is None or
                positive_int(fields.get("goal_stamp_ns")) != self.goal_stamp_ns):
            return False
        goal_id = positive_int(fields.get("goal_id"))
        seq = positive_int(fields.get("snapshot_seq"))
        if (goal_id is None or seq is None or seq <= self._planner_seq or
                (self.goal_id is not None and goal_id != self.goal_id) or
                not fields.get("status")):
            return False
        try:
            counters = {key: int(fields["goal_" + key]) for key in COUNTERS}
        except (KeyError, TypeError, ValueError):
            return False
        if any(value < self.counters[key] for key, value in counters.items()):
            return False
        self.goal_id = goal_id
        self._planner_seq = seq
        self.status = fields["status"]
        self.statuses.add(self.status)
        self.planner_done = self.status == "goal_reached"
        self.diagnostics = dict(fields)
        self.counters = counters
        pending, self._pending_follower = self._pending_follower, []
        for follower in pending:
            self.follower(follower)
        return True

    def follower(self, fields):
        goal_id = positive_int(fields.get("goal_id"))
        seq = positive_int(fields.get("snapshot_seq"))
        if self.goal_stamp_ns is None or goal_id is None or seq is None:
            return False
        if self.goal_id is None:
            # Cross-topic delivery can put follower events before planner acknowledgement.
            # Bound storage, then replay only the execution id acknowledged for this goal.
            self._pending_follower.append(dict(fields))
            self._pending_follower = self._pending_follower[-64:]
            return False
        if not self.owns(goal_id) or seq <= self._follower_seq:
            return False
        self._follower_seq = seq
        status = fields.get("status")
        if status == "tracking":
            self.follower_tracking = True
        self.follower_done = status == "goal_reached" and self.follower_tracking
        if self.follower_tracking and status in ("stale_trajectory", "stale_terrain"):
            self.follower_stale = True
        return True
