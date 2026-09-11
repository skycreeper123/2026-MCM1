"""Question 2: safe second-station region and deterministic point selection."""

from .selection import (
    Q2Config,
    choose_second_detection,
    circle_outer_halfplanes,
    clip_convex_polygon,
    is_safe_candidate,
    response_radius_bound,
    safe_candidate_region,
)
from .regions import analyze_candidate_region, classify_reception, is_information_candidate

__all__ = ["Q2Config", "choose_second_detection", "circle_outer_halfplanes",
           "clip_convex_polygon", "is_safe_candidate", "response_radius_bound",
           "safe_candidate_region",
           "analyze_candidate_region", "classify_reception", "is_information_candidate"]
