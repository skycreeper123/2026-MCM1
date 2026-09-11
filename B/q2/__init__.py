"""Question 2: safe second-station region and deterministic point selection."""

from .selection import Q2Config, choose_second_detection, safe_candidate_region
from .regions import analyze_candidate_region, classify_reception, is_information_candidate

__all__ = ["Q2Config", "choose_second_detection", "safe_candidate_region",
           "analyze_candidate_region", "classify_reception", "is_information_candidate"]
