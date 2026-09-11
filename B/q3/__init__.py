"""Question 3 search, cumulative localization, and guaranteed fallback."""

from .geometry import q3_coverage_certificate, seven_station_route, strip_clear_points
from .localize import build_cover_plan, choose_detection_from_region, update_outer_region
from .planner import (
    ChannelStatus,
    MeasurePurpose,
    PlannerPhase,
    Q3Action,
    Q3Config,
    Q3Planner,
    SessionStatus,
    q3_baseline_upper_bounds,
    q3_v2_upper_bounds,
)

__all__ = [
    "ChannelStatus",
    "MeasurePurpose",
    "PlannerPhase",
    "Q3Action",
    "Q3Config",
    "Q3Planner",
    "SessionStatus",
    "build_cover_plan",
    "choose_detection_from_region",
    "q3_coverage_certificate",
    "q3_baseline_upper_bounds",
    "q3_v2_upper_bounds",
    "seven_station_route",
    "strip_clear_points",
    "update_outer_region",
]
