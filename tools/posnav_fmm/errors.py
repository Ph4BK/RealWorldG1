"""Planning outcomes that navigation can retry after a dynamic map update."""


class PlanningRetry(ValueError):
    """No executable plan on this snapshot; retry after a map update."""
    pass


class GoalUnreachable(PlanningRetry):
    pass


class NoVisibleSTG(PlanningRetry):
    """Local candidate search failed; does not imply global unreachability."""
    pass
