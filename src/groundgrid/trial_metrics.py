"""ROS-free mission-completion accounting, distinct from distance-only reach metrics."""


def mission_completed(trial):
    """Require both completion signals; never promote a timeout/abort into completion.

    Older records do not contain end_reason, so their two explicit boolean signals are
    the available evidence. Missing signals are not inferred from a path, a success status,
    or ever having entered the requested goal's distance tolerance.
    """
    return (trial.get("planner_goal_reached") is True and
            trial.get("follower_goal_reached") is True and
            trial.get("end_reason") in (None, "completed"))


def completion_summary(every, tour):
    """Summarize all attempts and the nominal tour without changing either denominator."""
    completed = sum(mission_completed(t) for t in every)
    tour_completed = sum(mission_completed(t) for t in tour)
    return {
        "counts": {"missions_completed": completed,
                   "tour_missions_completed": tour_completed},
        "rates": {"completion_rate": completed / len(every) if every else float("nan"),
                  "completion_tour": tour_completed / len(tour) if tour else float("nan")},
    }


def require_tour_completion(tour):
    """Reject a vacuous navigation pass; this is NOT the 99% acceptance gate.

    Use goal-correlated completion signals, not paths, recovery confirmations or
    entry into the requested radius. Marginal scenarios must not pass with no work
    completed merely because their conditional recovery denominator is empty.
    """
    if not any(mission_completed(t) for t in tour):
        raise AssertionError("no tour mission completed; zero completion is not a navigation pass")
