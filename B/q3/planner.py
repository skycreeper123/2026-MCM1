"""Event-driven global discovery and certified service planner for Question 3.

The planner is transport-independent. A client asks for the next action, sends
it serially, and applies only a validated accepted response. Repeated calls to
``propose_action`` return the same pending action, supporting idempotent HTTP
retries without advancing algorithm state.
"""

from dataclasses import asdict, dataclass, field
from enum import Enum
import math
import time

import numpy as np

from B.q2 import Q2Config

from .geometry import (
    distance,
    q3_coverage_certificate,
    seven_station_route,
    strip_clear_points,
    strip_coverage_bound_m,
)
from .localize import (
    ClearPlan,
    build_cover_plan,
    choose_detection_from_region,
    initialize_outer_region,
    update_outer_region,
    verify_cover_certificate,
    reprice_cover_plan,
    plan_route_distance,
)


class ChannelStatus(str, Enum):
    UNKNOWN = "UNKNOWN"
    DETECTED = "DETECTED"
    CLEARED = "CLEARED"
    ABSENT = "ABSENT"


class SessionStatus(str, Enum):
    ACTIVE = "ACTIVE"
    EXITED = "EXITED"
    MODEL_OR_PROTOCOL_INCONSISTENCY = "MODEL_OR_PROTOCOL_INCONSISTENCY"
    TIME_BUDGET_INCOMPLETE = "TIME_BUDGET_INCOMPLETE"


class PlannerPhase(str, Enum):
    SCAN = "SCAN"
    SERVICE = "SERVICE"
    DISCOVERY = "DISCOVERY"
    LOCALIZE_SERVICE = "LOCALIZE_SERVICE"
    ROUTE_EXECUTION = "ROUTE_EXECUTION"
    EMERGENCY_FALLBACK = "EMERGENCY_FALLBACK"


class MeasurePurpose(str, Enum):
    COVERAGE_SCAN = "COVERAGE_SCAN"
    LOCALIZE = "LOCALIZE"


class ServicePhase(str, Enum):
    NONE = "NONE"
    READY_CLEAR = "READY_CLEAR"
    LOCALIZING = "LOCALIZING"
    COVERING = "COVERING"
    FALLBACK = "FALLBACK"


SUPPORTED_STRATEGIES = ("b0_serial", "b0_batch_fifo", "v2_local", "v3_global")


@dataclass(frozen=True)
class Q3Config:
    strategy: str = "v3_global"
    channel_min: int = 1
    channel_max: int = 20
    source_count_upper_bound: int = 16
    target_radius_m: float = 1800.0
    minimum_receive_radius_m: float = 1000.0
    maximum_receive_radius_m: float = 1500.0
    clear_radius_m: float = 20.0
    safe_clear_radius_m: float = 19.9
    ring_radius_m: float = 1150.0
    speed_mps: float = 5.0
    measure_duration_s: float = 5.0
    switch_duration_s: float = 1.0
    failed_clear_duration_s: float = 3.0
    successful_clear_duration_s: float = 5.0
    angle_half_width_deg: float = 1.005
    circle_sides: int = 128
    strip_max_range_m: float = 1500.0
    strip_step_m: float = 20.0
    strip_offsets_m: tuple[float, float] = (-15.0, 15.0)
    local_grid_cell_m: float = 28.0
    max_extra_measurements_per_source: int = 3
    max_optimized_clear_attempts_per_source: int | None = None
    extra_route_budget_m_per_source: float = 4000.0
    q2_candidate_limit: int | None = None
    q2_response_intervals: int = 32
    q2_calculation_time_limit_s: float | None = None
    total_planning_time_limit_s: float = 60.0
    minimum_radius_improvement_fraction: float | None = None
    repeated_position_tolerance_m: float = 1e-6

    def __post_init__(self):
        # Preserve V2 comparison defaults while making V3 the default strategy.
        global_mode = self.strategy == "v3_global"
        for name, value in {
            "max_optimized_clear_attempts_per_source": 16 if global_mode else 8,
            "q2_candidate_limit": 16 if global_mode else 12,
            "q2_calculation_time_limit_s": 1.5 if global_mode else 1.0,
            "minimum_radius_improvement_fraction": 0.03 if global_mode else 0.05,
        }.items():
            if getattr(self, name) is None:
                object.__setattr__(self, name, value)


@dataclass(frozen=True)
class Q3Action:
    kind: str
    position: tuple[float, float] | None
    channel: int | None
    reason: str
    station_id: int | None = None
    strip_index: int | None = None
    purpose: str | None = None
    region_version: int | None = None
    decision_budget: dict | None = None

    def as_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class ServicePlan:
    channel: int
    region_version: int
    cover: ClearPlan

    def as_dict(self):
        return {"channel": self.channel, "region_version": self.region_version,
                "localization_points": (), "clear_points": self.cover.points,
                **self.cover.as_dict()}


@dataclass(frozen=True)
class Task:
    channel: int
    kind: str
    position: tuple[float, float]
    completion_upper_s: float
    continuation_target: tuple[float, float] | None
    service_plan: ServicePlan | None = None
    score: dict | None = None


@dataclass
class ChannelRecord:
    channel: int
    status: ChannelStatus = ChannelStatus.UNKNOWN
    service_phase: ServicePhase = ServicePhase.NONE
    no_signal_station_ids: set[int] = field(default_factory=set)
    anchor_position: tuple[float, float] | None = None
    anchor_bearing_deg: float | None = None
    clear_position: tuple[float, float] | None = None
    absence_reason: str | None = None
    observations: list[dict] = field(default_factory=list)
    measured_positions: list[tuple[float, float]] = field(default_factory=list)
    outer_vertices: np.ndarray | None = None
    geometry_valid: bool = True
    region_version: int = 0
    radius_history_m: list[float] = field(default_factory=list)
    extra_measure_count: int = 0
    optimized_clear_count: int = 0
    artificial_prefix_length_m: float = 0.0
    optimization_points: list[tuple[float, float]] = field(default_factory=list)
    fallback_used: bool = False
    geometry_failure_reason: str | None = None
    service_plan: ServicePlan | None = None
    near_position: tuple[float, float] | None = None
    detection_scores: list[dict] | None = None
    detection_score_version: int = -1


def _validate_config(config):
    if config.strategy not in SUPPORTED_STRATEGIES:
        raise ValueError(f"strategy must be one of {SUPPORTED_STRATEGIES}")
    integer_fields = (
        "channel_min", "channel_max", "source_count_upper_bound", "circle_sides",
        "max_extra_measurements_per_source", "max_optimized_clear_attempts_per_source",
        "q2_candidate_limit", "q2_response_intervals",
    )
    for name in integer_fields:
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{name} must be an integer")
    if config.channel_min < 1 or config.channel_max < config.channel_min:
        raise ValueError("invalid channel interval")
    channel_count = config.channel_max - config.channel_min + 1
    if not 1 <= config.source_count_upper_bound <= channel_count:
        raise ValueError("source_count_upper_bound must lie in the channel count")
    if config.circle_sides < 16:
        raise ValueError("circle_sides must be at least 16")
    for name in ("max_extra_measurements_per_source", "max_optimized_clear_attempts_per_source",
                 "q2_candidate_limit", "q2_response_intervals"):
        if getattr(config, name) <= 0:
            raise ValueError(f"{name} must be positive")
    positive_fields = (
        "target_radius_m", "minimum_receive_radius_m", "maximum_receive_radius_m",
        "clear_radius_m", "safe_clear_radius_m", "ring_radius_m", "speed_mps",
        "measure_duration_s", "switch_duration_s", "failed_clear_duration_s",
        "successful_clear_duration_s", "angle_half_width_deg", "strip_max_range_m",
        "strip_step_m", "local_grid_cell_m", "extra_route_budget_m_per_source",
        "q2_calculation_time_limit_s", "total_planning_time_limit_s",
    )
    for name in positive_fields:
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        if value <= 0:
            raise ValueError(f"{name} must be positive")
    if not 0 < config.angle_half_width_deg < 90:
        raise ValueError("angle_half_width_deg must be below 90 degrees")
    for name in ("minimum_radius_improvement_fraction", "repeated_position_tolerance_m"):
        value = getattr(config, name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be a nonnegative finite number")
    if not config.safe_clear_radius_m < config.clear_radius_m:
        raise ValueError("safe_clear_radius_m must be below clear_radius_m")
    if config.local_grid_cell_m > 28 or config.safe_clear_radius_m > 19.9:
        raise ValueError("certified local geometry requires cells <= 28m and safe radius <= 19.9m")
    if config.maximum_receive_radius_m < config.minimum_receive_radius_m:
        raise ValueError("maximum_receive_radius_m must be at least minimum_receive_radius_m")
    if config.strip_max_range_m < config.maximum_receive_radius_m:
        raise ValueError("strip_max_range_m must cover maximum_receive_radius_m")
    strip_clear_points(
        (0.0, 0.0), 0.0, config.strip_max_range_m,
        config.strip_step_m, config.strip_offsets_m
    )
    strip_bound = strip_coverage_bound_m(
        config.angle_half_width_deg, config.maximum_receive_radius_m,
        config.strip_step_m, config.strip_offsets_m
    )
    if strip_bound >= config.clear_radius_m:
        raise ValueError("configured strip does not guarantee the clear radius")
    certificate = q3_coverage_certificate(
        config.target_radius_m, config.minimum_receive_radius_m, config.ring_radius_m
    )
    if not certificate["covered"]:
        raise ValueError("configured seven-station geometry does not cover the target disk")


class Q3Planner:
    """Global adaptive strategy with explicit B0/V2 comparison modes."""

    def __init__(self, config=None):
        self.config = config or Q3Config()
        _validate_config(self.config)
        self.stations = seven_station_route(self.config.ring_radius_m)
        self.channels = {
            channel: ChannelRecord(channel)
            for channel in range(self.config.channel_min, self.config.channel_max + 1)
        }
        self.session_status = SessionStatus.ACTIVE
        self.phase = (PlannerPhase.DISCOVERY if self.config.strategy == "v3_global"
                      else PlannerPhase.SCAN)
        self.completion_reason = None
        self.current_position = (0.0, 0.0)
        self.current_channel = self.config.channel_min
        self.station_index = 0
        self.scan_cursor = 0
        self.batch_channels = []
        self.pending_sources: set[int] = set()
        self.service_plans: dict[int, ServicePlan] = {}
        self.active_task: Task | None = None
        self.task_history: list[dict] = []
        self.cover_plan_history: list[dict] = []
        self.active_service_channel = None
        self.active_plan_kind = None
        self.active_plan_points = ()
        self.active_plan_index = 0
        self.pending_action = None
        self.virtual_time_s = 0.0
        self.distance_m = 0.0
        self.measure_count = 0
        self.coverage_measure_count = 0
        self.localize_measure_count = 0
        self.switch_count = 0
        self.clear_attempt_count = 0
        self.failed_clear_count = 0
        self.successful_clear_count = 0
        self.cover_clear_count = 0
        self.fallback_clear_count = 0
        self.planning_time_s = 0.0

    def _station_order(self, station_index):
        channels = tuple(range(self.config.channel_min, self.config.channel_max + 1))
        return channels if station_index % 2 == 0 else tuple(reversed(channels))

    def _mark_upper_bound_absences(self):
        if self.config.strategy == "v3_global":
            known = sum(r.status in (ChannelStatus.DETECTED, ChannelStatus.CLEARED)
                        for r in self.channels.values())
            if known < self.config.source_count_upper_bound:
                return
        else:
            if self.successful_clear_count != self.config.source_count_upper_bound:
                return
            if any(record.status == ChannelStatus.DETECTED for record in self.channels.values()):
                return
        for record in self.channels.values():
            if record.status == ChannelStatus.UNKNOWN:
                record.status = ChannelStatus.ABSENT
                record.absence_reason = "COUNT_UPPER_BOUND"

    def _check_source_count_consistency(self):
        known = sum(record.status in (ChannelStatus.DETECTED, ChannelStatus.CLEARED)
                    for record in self.channels.values())
        if known > self.config.source_count_upper_bound:
            self.session_status = SessionStatus.MODEL_OR_PROTOCOL_INCONSISTENCY
            self.completion_reason = "SOURCE_COUNT_UPPER_BOUND_VIOLATED"

    def has_completion_certificate(self):
        if self.session_status in (SessionStatus.MODEL_OR_PROTOCOL_INCONSISTENCY,
                                   SessionStatus.TIME_BUDGET_INCOMPLETE):
            return False
        if any(record.status == ChannelStatus.DETECTED for record in self.channels.values()):
            return False
        return all(record.status in (ChannelStatus.CLEARED, ChannelStatus.ABSENT)
                   for record in self.channels.values())

    def mark_time_budget_incomplete(self):
        if not self.has_completion_certificate():
            self.session_status = SessionStatus.TIME_BUDGET_INCOMPLETE
            self.completion_reason = "TIME_BUDGET_INCOMPLETE"

    def _finish_action(self, reason):
        self.completion_reason = reason
        return Q3Action("EXIT", None, None, reason)

    def _next_station(self):
        index = self.station_index + 1
        return self.stations[index] if index < len(self.stations) else None

    def _set_plan(self, record, kind, points):
        self.active_plan_kind = kind
        self.active_plan_points = tuple(tuple(map(float, point)) for point in points)
        self.active_plan_index = 0
        if kind == "FIRST_BEARING_STRIP":
            record.service_phase = ServicePhase.FALLBACK
            record.fallback_used = True
        elif kind == "NEAR_SOURCE":
            record.service_phase = ServicePhase.READY_CLEAR
        else:
            record.service_phase = ServicePhase.COVERING

    def _activate_fallback(self, record):
        if record.anchor_position is None or record.anchor_bearing_deg is None:
            self.session_status = SessionStatus.MODEL_OR_PROTOCOL_INCONSISTENCY
            record.geometry_failure_reason = "MISSING_DIRECTION_ANCHOR"
            return
        if self.config.strategy == "v3_global":
            self.phase = PlannerPhase.EMERGENCY_FALLBACK
            if record.geometry_failure_reason is None:
                record.geometry_failure_reason = "NO_CERTIFIED_LOCAL_PLAN"
        self._set_plan(record, "FIRST_BEARING_STRIP", strip_clear_points(
            record.anchor_position, record.anchor_bearing_deg,
            self.config.strip_max_range_m, self.config.strip_step_m,
            self.config.strip_offsets_m,
        ))

    def _optimization_sequence_allowed(self, record, points):
        if record.anchor_position is None or not points:
            return False
        cursor = record.optimization_points[-1] if record.optimization_points else record.anchor_position
        prefix = record.artificial_prefix_length_m
        for point in points:
            prefix += distance(cursor, point)
            cursor = point
        return prefix + distance(cursor, record.anchor_position) \
            <= self.config.extra_route_budget_m_per_source + 1e-8

    def _record_optimization_point(self, record, point):
        cursor = record.optimization_points[-1] if record.optimization_points else record.anchor_position
        record.artificial_prefix_length_m += distance(cursor, point)
        record.optimization_points.append(tuple(point))

    def _weak_recent_updates(self, record):
        if len(record.radius_history_m) < 3:
            return False
        changes = [(old - new) / max(old, 1e-12)
                   for old, new in zip(record.radius_history_m[-3:-1], record.radius_history_m[-2:])]
        return all(value < self.config.minimum_radius_improvement_fraction for value in changes)

    def _q2_config(self):
        intervals = self.config.q2_response_intervals
        return Q2Config(
            target_radius_m=self.config.target_radius_m,
            maximum_reception_radius_m=self.config.maximum_receive_radius_m,
            minimum_reception_radius_m=self.config.minimum_receive_radius_m,
            movement_speed_mps=self.config.speed_mps,
            movement_weight_m_per_s=1.0,
            circle_sides=self.config.circle_sides,
            coarse_spacing_m=100.0,
            fine_spacing_m=20.0,
            max_coarse_candidates=self.config.q2_candidate_limit,
            max_fine_candidates=8,
            refinement_starts=1,
            max_response_intervals=intervals,
            shortlist_response_intervals=intervals,
            final_response_intervals=intervals,
            maximum_refinement_intervals=intervals,
            polish_starts=1,
            response_bound_tolerance_m=0.5,
            final_bound_tolerance_m=0.5,
            repeated_position_tolerance_m=self.config.repeated_position_tolerance_m,
        )

    def _select_localize_action(self, record, deadline_monotonic):
        if record.extra_measure_count >= self.config.max_extra_measurements_per_source:
            return None
        if record.optimized_clear_count >= self.config.max_optimized_clear_attempts_per_source:
            return None
        if self._weak_recent_updates(record) or record.outer_vertices is None:
            return None
        if self.planning_time_s >= self.config.total_planning_time_limit_s:
            return None
        started = time.monotonic()
        local_deadline = started + self.config.q2_calculation_time_limit_s
        if deadline_monotonic is not None:
            local_deadline = min(local_deadline, deadline_monotonic)
        result = choose_detection_from_region(
            record.outer_vertices, self.current_position, record.measured_positions,
            record.anchor_bearing_deg, self.config.angle_half_width_deg,
            self._next_station(), self._q2_config(), local_deadline,
            self.config.q2_candidate_limit,
        )
        self.planning_time_s += time.monotonic() - started
        if result["status"] != "OK":
            record.geometry_failure_reason = result.get("reason")
            return None
        selected = result["selected"]
        point = (selected["position"]["x"], selected["position"]["y"])
        if not self._optimization_sequence_allowed(record, (point,)):
            record.geometry_failure_reason = "EXTRA_ROUTE_BUDGET"
            return None
        if selected["worst_updated_cover_radius_m"] >= record.radius_history_m[-1] - 1e-6:
            record.geometry_failure_reason = "NO_PREDICTED_RADIUS_IMPROVEMENT"
            return None
        record.service_phase = ServicePhase.LOCALIZING
        return Q3Action(
            "MEASURE", point, record.channel, "CUMULATIVE_SAFE_LOCALIZATION",
            purpose=MeasurePurpose.LOCALIZE.value, region_version=record.region_version,
            decision_budget={
                "extra_measure_count": record.extra_measure_count,
                "extra_measure_limit": self.config.max_extra_measurements_per_source,
                "artificial_prefix_length_m": record.artificial_prefix_length_m,
                "extra_route_budget_m": self.config.extra_route_budget_m_per_source,
                "predicted_continuous_radius_upper_m": selected["worst_updated_cover_radius_m"],
                "selection_mode": result["selection_mode"],
                "timed_out": result["timed_out"],
            },
        )

    def _prepare_v2_service(self, record, deadline_monotonic):
        remaining = (self.config.max_optimized_clear_attempts_per_source
                     - record.optimized_clear_count)
        if record.geometry_valid and record.outer_vertices is not None and remaining > 0:
            started = time.monotonic()
            plan = build_cover_plan(
                record.outer_vertices, self.current_position, self._next_station(), remaining,
                (record.anchor_bearing_deg,), self.config.local_grid_cell_m,
                self.config.safe_clear_radius_m, 1e-7, self.config.speed_mps,
                self.config.failed_clear_duration_s, self.config.successful_clear_duration_s,
            )
            self.planning_time_s += time.monotonic() - started
            if plan is not None and self._optimization_sequence_allowed(record, plan.points):
                self._set_plan(record, plan.kind, plan.points)
                return None
            action = self._select_localize_action(record, deadline_monotonic)
            if action is not None:
                return action
        self._activate_fallback(record)
        return None

    def _ensure_active_service(self, deadline_monotonic):
        if self.active_service_channel is None:
            while self.batch_channels and self.channels[self.batch_channels[0]].status != ChannelStatus.DETECTED:
                self.batch_channels.pop(0)
            if not self.batch_channels:
                return None
            self.active_service_channel = self.batch_channels[0]
        record = self.channels[self.active_service_channel]
        if self.active_plan_points:
            return None
        if self.config.strategy == "v2_local" and record.geometry_valid:
            return self._prepare_v2_service(record, deadline_monotonic)
        self._activate_fallback(record)
        return None

    def _clear_action(self, record):
        point = self.active_plan_points[self.active_plan_index]
        return Q3Action(
            "CLEAR", point, record.channel, self.active_plan_kind,
            strip_index=self.active_plan_index if self.active_plan_kind == "FIRST_BEARING_STRIP" else None,
            purpose=self.active_plan_kind, region_version=record.region_version,
            decision_budget={
                "optimized_clear_count": record.optimized_clear_count,
                "optimized_clear_limit": self.config.max_optimized_clear_attempts_per_source,
                "artificial_prefix_length_m": record.artificial_prefix_length_m,
                "extra_route_budget_m": self.config.extra_route_budget_m_per_source,
            },
        )

    def _global_continuation(self, record):
        candidates = []
        for channel in sorted(self.pending_sources - {record.channel}):
            other = self.channels[channel]
            if other.service_plan is not None:
                candidates.append(other.service_plan.cover.entry_point)
            elif other.near_position is not None:
                candidates.append(other.near_position)
            elif other.outer_vertices is not None:
                candidates.append(tuple(np.mean(other.outer_vertices, axis=0)))
            elif other.anchor_position is not None:
                candidates.append(other.anchor_position)
        origin = (tuple(np.mean(record.outer_vertices, axis=0))
                  if record.outer_vertices is not None else record.near_position or record.anchor_position)
        return min(candidates, key=lambda point: distance(origin, point)) if candidates else None

    def _global_cover(self, record, continuation):
        if record.service_plan is not None and record.service_plan.region_version == record.region_version:
            cover = reprice_cover_plan(record.service_plan.cover, self.current_position,
                                       continuation, self.config)
        elif record.near_position is not None:
            point = record.near_position
            route = distance(self.current_position, point)
            if continuation is not None:
                route += distance(point, continuation)
            cover = ClearPlan("NEAR_SOURCE", (point,), True, 5.0, route,
                              route/self.config.speed_mps + self.config.successful_clear_duration_s,
                              cover_certificate={"method": "NEAR_RESPONSE", "radius_upper_m": 5.0})
        else:
            # The preferred count triggers extra localization. It is not a reason
            # to abandon a valid local certificate for the emergency strip.
            cover = build_cover_plan(
                record.outer_vertices, self.current_position, continuation, max_points=100000,
                preferred_orientations_deg=(record.anchor_bearing_deg,),
                cell_limit_m=self.config.local_grid_cell_m,
                clear_radius_m=self.config.safe_clear_radius_m, speed_mps=self.config.speed_mps,
                failed_clear_duration_s=self.config.failed_clear_duration_s,
                successful_clear_duration_s=self.config.successful_clear_duration_s)
        if cover is None or not cover.guaranteed or cover.cover_certificate is None:
            return None
        if record.near_position is None and not verify_cover_certificate(
                record.outer_vertices, cover, self.config.safe_clear_radius_m):
            return None
        plan = ServicePlan(record.channel, record.region_version, cover)
        record.service_plan = plan
        self.service_plans[record.channel] = plan
        return plan

    def _global_detection_tasks(self, record, continuation, deadline):
        if record.near_position is not None or record.outer_vertices is None \
                or record.extra_measure_count >= self.config.max_extra_measurements_per_source \
                or self._weak_recent_updates(record):
            return []
        if record.detection_score_version != record.region_version:
            record.detection_scores = []
            record.detection_score_version = record.region_version
            if self.planning_time_s >= self.config.total_planning_time_limit_s:
                return []
            started = time.monotonic()
            end = min(started + self.config.q2_calculation_time_limit_s,
                      started + max(0, self.config.total_planning_time_limit_s-self.planning_time_s))
            if deadline is not None:
                end = min(end, deadline)
            result = choose_detection_from_region(
                record.outer_vertices, self.current_position, record.measured_positions,
                record.anchor_bearing_deg, self.config.angle_half_width_deg, None,
                self._q2_config(), end, self.config.q2_candidate_limit,
                completion_config=self.config)
            if result["status"] == "OK":
                record.detection_scores = result["candidate_scores"]
        tasks = []
        for row in record.detection_scores or []:
            point = (row["position"]["x"], row["position"]["y"])
            if not self._optimization_sequence_allowed(record, (point,)):
                continue
            if row["worst_updated_cover_radius_m"] >= record.radius_history_m[-1] - 1e-6:
                continue
            # Cached geometry does not depend on robot position, channel, or the
            # next source. Reprice these costs at EVERY scheduling boundary.
            cost = row["completion_upper_s"] - row["movement_time_s"]
            cost += distance(self.current_position, point) / self.config.speed_mps
            cost += (self.current_channel != record.channel) * self.config.switch_duration_s
            if continuation is not None:
                cost += max(distance(p, continuation) for p in row["possible_exit_points"]) / self.config.speed_mps
            tasks.append(Task(record.channel, "LOCALIZE", point, cost, continuation, score=row))
        return tasks

    def _select_global_task(self, deadline):
        tasks = []
        # Materialize feasible entries before lookahead; never use a true source
        # coordinate or a future successful clear position as the next entry.
        for channel in sorted(self.pending_sources):
            record = self.channels[channel]
            if not record.geometry_valid:
                continue
            started = time.monotonic()
            try:
                self._global_cover(record, None)
            except (ValueError, ArithmeticError, np.linalg.LinAlgError) as exc:
                record.geometry_valid = False
                record.geometry_failure_reason = f"GLOBAL_GEOMETRY_ERROR: {exc}"
            self.planning_time_s += time.monotonic()-started
        for channel in sorted(self.pending_sources):
            record = self.channels[channel]
            continuation = self._global_continuation(record)
            started = time.monotonic()
            try:
                plan = self._global_cover(record, continuation) if record.geometry_valid else None
                detect_tasks = (self._global_detection_tasks(record, continuation, deadline)
                                if record.geometry_valid and (plan is None or plan.cover.point_count > 1)
                                else [])
            except (ValueError, ArithmeticError, np.linalg.LinAlgError) as exc:
                record.geometry_valid = False
                record.geometry_failure_reason = f"GLOBAL_GEOMETRY_ERROR: {exc}"
                plan, detect_tasks = None, []
            self.planning_time_s += time.monotonic() - started
            tasks.extend(detect_tasks)
            if plan is not None:
                # Large regions first receive a safe measurement if one remains.
                if plan.cover.point_count <= self.config.max_optimized_clear_attempts_per_source or not detect_tasks:
                    tasks.append(Task(channel, "CLEAR", plan.cover.entry_point,
                                      plan.cover.completion_upper_s, continuation, service_plan=plan))
            elif not detect_tasks:
                record.geometry_failure_reason = record.geometry_failure_reason or "NO_CERTIFIED_LOCAL_PLAN"
                anchor = record.anchor_position or self.current_position
                # Exceptional task only; the strip is not a competing normal plan.
                if record.anchor_position is not None and record.anchor_bearing_deg is not None:
                    strip = strip_clear_points(record.anchor_position, record.anchor_bearing_deg,
                                               self.config.strip_max_range_m, self.config.strip_step_m,
                                               self.config.strip_offsets_m)
                    cost = (plan_route_distance(strip, self.current_position, continuation)/self.config.speed_mps
                            + (len(strip)-1)*self.config.failed_clear_duration_s
                            + self.config.successful_clear_duration_s)
                else:
                    cost = 0.0  # immediately expose an unrecoverable missing anchor
                tasks.append(Task(channel, "FALLBACK", anchor, cost, continuation))
        if not tasks:
            return None
        task = min(tasks, key=lambda item: (item.completion_upper_s, item.channel, item.kind, item.position))
        self.task_history.append({"channel": task.channel, "kind": task.kind,
                                  "position": task.position,
                                  "completion_upper_s": task.completion_upper_s,
                                  "continuation_target": task.continuation_target,
                                  "candidate_count": len(tasks)})
        return task

    def _propose_global(self, deadline):
        if self.active_service_channel is not None and self.active_plan_points:
            return self._clear_action(self.channels[self.active_service_channel])
        if self.has_completion_certificate():
            return self._finish_action("ALL_CHANNELS_RESOLVED")
        while self.station_index < len(self.stations):
            self.phase = PlannerPhase.DISCOVERY
            order = self._station_order(self.station_index)
            while self.scan_cursor < len(order):
                channel = order[self.scan_cursor]
                if self.channels[channel].status != ChannelStatus.UNKNOWN:
                    self.scan_cursor += 1
                    continue
                return Q3Action("MEASURE", self.stations[self.station_index], channel,
                                "SEVEN_STATION_COVERAGE_SCAN", station_id=self.station_index,
                                purpose=MeasurePurpose.COVERAGE_SCAN.value)
            self.station_index += 1
            self.scan_cursor = 0
        self.phase = PlannerPhase.LOCALIZE_SERVICE
        task = self._select_global_task(deadline)
        if task is None:
            if self.has_completion_certificate():
                return self._finish_action("ALL_CHANNELS_RESOLVED")
            self.session_status = SessionStatus.MODEL_OR_PROTOCOL_INCONSISTENCY
            return self._finish_action("INCOMPLETE_COVERAGE_EVIDENCE")
        self.active_task = task
        self.active_service_channel = task.channel
        record = self.channels[task.channel]
        if task.kind == "LOCALIZE":
            record.service_phase = ServicePhase.LOCALIZING
            return Q3Action("MEASURE", task.position, task.channel, "GLOBAL_SAFE_LOCALIZATION",
                            purpose=MeasurePurpose.LOCALIZE.value, region_version=record.region_version,
                            decision_budget={"completion_upper_s": task.completion_upper_s,
                                             "predicted_continuous_radius_upper_m": task.score["worst_updated_cover_radius_m"],
                                             "worst_cover_point_count": task.score["worst_cover_point_count"],
                                             "selection_mode": "v3_global_completion_upper_bound"})
        if task.kind == "CLEAR":
            self.phase = PlannerPhase.ROUTE_EXECUTION
            self.cover_plan_history.append(task.service_plan.as_dict())
            self._set_plan(record, task.service_plan.cover.kind, task.service_plan.cover.points)
        else:
            self._activate_fallback(record)
        if self.session_status != SessionStatus.ACTIVE:
            return self._finish_action(self.session_status.value)
        return self._clear_action(record)

    def propose_action(self, deadline_monotonic=None):
        """Return one stable action; the deadline limits geometry planning only."""
        if self.session_status == SessionStatus.EXITED:
            return None
        if self.pending_action is not None:
            return self.pending_action
        if self.session_status != SessionStatus.ACTIVE:
            self.pending_action = self._finish_action(self.session_status.value)
            return self.pending_action
        self._mark_upper_bound_absences()
        if self.config.strategy == "v3_global":
            self.pending_action = self._propose_global(deadline_monotonic)
            return self.pending_action
        if self.active_service_channel is not None:
            record = self.channels[self.active_service_channel]
            if not self.active_plan_points:
                action = self._ensure_active_service(deadline_monotonic)
                if action is not None:
                    self.pending_action = action
                    return action
            if self.session_status != SessionStatus.ACTIVE:
                return self.propose_action(deadline_monotonic)
            self.pending_action = self._clear_action(record)
            return self.pending_action
        if self.has_completion_certificate():
            self.pending_action = self._finish_action("ALL_CHANNELS_RESOLVED")
            return self.pending_action

        while self.station_index < len(self.stations):
            if self.phase == PlannerPhase.SERVICE:
                action = self._ensure_active_service(deadline_monotonic)
                if action is not None:
                    self.pending_action = action
                    return action
                if self.active_service_channel is not None:
                    if self.session_status != SessionStatus.ACTIVE:
                        return self.propose_action(deadline_monotonic)
                    self.pending_action = self._clear_action(self.channels[self.active_service_channel])
                    return self.pending_action
                self.phase = PlannerPhase.SCAN
                self.batch_channels = []
                self.station_index += 1
                self.scan_cursor = 0
                continue

            order = self._station_order(self.station_index)
            while self.scan_cursor < len(order):
                channel = order[self.scan_cursor]
                if self.channels[channel].status != ChannelStatus.UNKNOWN:
                    self.scan_cursor += 1
                    continue
                self.pending_action = Q3Action(
                    "MEASURE", self.stations[self.station_index], channel,
                    "SEVEN_STATION_COVERAGE_SCAN", station_id=self.station_index,
                    purpose=MeasurePurpose.COVERAGE_SCAN.value,
                )
                return self.pending_action
            if self.batch_channels:
                self.phase = PlannerPhase.SERVICE
            else:
                self.station_index += 1
                self.scan_cursor = 0

        if self.has_completion_certificate():
            self.pending_action = self._finish_action("ALL_CHANNELS_RESOLVED")
        else:
            self.session_status = SessionStatus.MODEL_OR_PROTOCOL_INCONSISTENCY
            self.pending_action = self._finish_action("INCOMPLETE_COVERAGE_EVIDENCE")
        return self.pending_action

    def _require_pending(self, action, kind):
        if self.pending_action is None or action != self.pending_action:
            raise ValueError("action is not the current pending action")
        if action.kind != kind:
            raise ValueError(f"pending action is not {kind}")

    def _move_to(self, position):
        travelled = distance(self.current_position, position)
        self.distance_m += travelled
        self.virtual_time_s += travelled / self.config.speed_mps
        self.current_position = tuple(position)

    def _initialize_direction(self, record, action, bearing):
        observation = {"position": {"x": action.position[0], "y": action.position[1]},
                       "svd_deg": bearing}
        record.anchor_position = action.position
        record.anchor_bearing_deg = bearing
        record.observations.append(observation)
        record.measured_positions.append(action.position)
        region = initialize_outer_region(observation, self.config.angle_half_width_deg,
                                         self._q2_config())
        if region["status"] != "BOUNDED":
            record.geometry_valid = False
            record.geometry_failure_reason = f"INITIAL_REGION_{region['status']}"
            return
        record.outer_vertices = np.asarray(region["vertices"], dtype=float)
        record.region_version = 1
        record.radius_history_m.append(region["minimum_enclosing_circle"]["radius_m"])

    def _append_direction(self, record, action, bearing):
        observation = {"position": {"x": action.position[0], "y": action.position[1]},
                       "svd_deg": bearing}
        updated = update_outer_region(
            record.outer_vertices, observation, self.config.angle_half_width_deg,
            self.config.maximum_receive_radius_m, self.config.circle_sides,
        )
        record.observations.append(observation)
        record.measured_positions.append(action.position)
        if updated["status"] != "BOUNDED":
            record.geometry_valid = False
            record.geometry_failure_reason = f"UPDATE_REGION_{updated['status']}"
            return
        record.outer_vertices = np.asarray(updated["vertices"], dtype=float)
        record.region_version += 1
        record.radius_history_m.append(updated["minimum_enclosing_circle"]["radius_m"])

    def apply_measure_result(self, action, result, svd_deg=None):
        """Apply one accepted coverage or localization response exactly once."""
        self._require_pending(action, "MEASURE")
        if result not in ("no_signal", "near", "direction"):
            raise ValueError("invalid measure result")
        if result == "direction":
            if isinstance(svd_deg, bool) or not isinstance(svd_deg, (int, float)) \
                    or not math.isfinite(svd_deg):
                raise ValueError("direction requires a finite svd_deg")
            bearing = float(svd_deg) % 360.0
        elif svd_deg is not None:
            raise ValueError("svd_deg is only valid for direction")
        else:
            bearing = None
        purpose = action.purpose or MeasurePurpose.COVERAGE_SCAN.value
        if purpose not in (MeasurePurpose.COVERAGE_SCAN.value, MeasurePurpose.LOCALIZE.value):
            raise ValueError("invalid measure purpose")
        if purpose == MeasurePurpose.COVERAGE_SCAN.value and action.station_id is None:
            raise ValueError("coverage scan requires station_id")
        if purpose == MeasurePurpose.LOCALIZE.value and action.station_id is not None:
            raise ValueError("localization measurement cannot carry station_id")
        self._move_to(action.position)
        if self.current_channel != action.channel:
            self.current_channel = action.channel
            self.switch_count += 1
            self.virtual_time_s += self.config.switch_duration_s
        self.virtual_time_s += self.config.measure_duration_s
        self.measure_count += 1
        record = self.channels[action.channel]
        if purpose == MeasurePurpose.COVERAGE_SCAN.value:
            self.coverage_measure_count += 1
            self.scan_cursor += 1
            if result == "no_signal":
                record.no_signal_station_ids.add(action.station_id)
                if len(record.no_signal_station_ids) == len(self.stations):
                    record.status = ChannelStatus.ABSENT
                    record.absence_reason = "NO_SIGNAL_AT_ALL_COVERAGE_STATIONS"
            else:
                if record.status != ChannelStatus.UNKNOWN:
                    raise ValueError("coverage discovery requires an UNKNOWN channel")
                record.status = ChannelStatus.DETECTED
                self._check_source_count_consistency()
                if result == "direction":
                    if self.config.strategy in ("v2_local", "v3_global"):
                        try:
                            self._initialize_direction(record, action, bearing)
                        except (ValueError, ArithmeticError, np.linalg.LinAlgError) as exc:
                            record.geometry_valid = False
                            record.geometry_failure_reason = f"INITIAL_GEOMETRY_ERROR: {exc}"
                    else:
                        record.anchor_position = action.position
                        record.anchor_bearing_deg = bearing
                    if self.config.strategy == "b0_serial":
                        self.active_service_channel = record.channel
                        self._activate_fallback(record)
                    elif self.config.strategy == "v3_global":
                        self.pending_sources.add(record.channel)
                    else:
                        self.batch_channels.append(record.channel)
                else:
                    if self.config.strategy == "v3_global":
                        record.near_position = action.position
                        record.measured_positions.append(action.position)
                        self.pending_sources.add(record.channel)
                    else:
                        self.active_service_channel = record.channel
                        self._set_plan(record, "NEAR_SOURCE", (action.position,))
        else:
            self.localize_measure_count += 1
            record.extra_measure_count += 1
            self._record_optimization_point(record, action.position)
            if self.config.strategy == "v3_global":
                record.service_plan = None
                self.service_plans.pop(record.channel, None)
            if result == "no_signal":
                record.geometry_valid = False
                record.geometry_failure_reason = "SAFE_LOCALIZE_RETURNED_NO_SIGNAL"
                self._activate_fallback(record)
            elif result == "near":
                record.measured_positions.append(action.position)
                if self.config.strategy == "v3_global":
                    record.near_position = action.position
                    plan = self._global_cover(record, self._global_continuation(record))
                    self.cover_plan_history.append(plan.as_dict())
                    self.phase = PlannerPhase.ROUTE_EXECUTION
                self._set_plan(record, "NEAR_SOURCE", (action.position,))
            else:
                try:
                    self._append_direction(record, action, bearing)
                except (ValueError, ArithmeticError, np.linalg.LinAlgError) as exc:
                    record.geometry_valid = False
                    record.geometry_failure_reason = f"UPDATE_GEOMETRY_ERROR: {exc}"
                self.active_plan_kind = None
                self.active_plan_points = ()
                self.active_plan_index = 0
                if self.config.strategy == "v3_global":
                    self.active_service_channel = None
                    self.active_task = None
                    record.service_plan = None
                    self.service_plans.pop(record.channel, None)
                    self.phase = PlannerPhase.LOCALIZE_SERVICE
        self.pending_action = None

    def _finish_channel(self, record, position):
        record.status = ChannelStatus.CLEARED
        record.service_phase = ServicePhase.NONE
        record.clear_position = position
        self.successful_clear_count += 1
        self.pending_sources.discard(record.channel)
        self.service_plans.pop(record.channel, None)
        record.service_plan = None
        self.active_task = None
        if record.channel in self.batch_channels:
            self.batch_channels.remove(record.channel)
        self.active_service_channel = None
        self.active_plan_kind = None
        self.active_plan_points = ()
        self.active_plan_index = 0

    def apply_clear_result(self, action, result):
        """Apply one validated accepted clear response exactly once."""
        self._require_pending(action, "CLEAR")
        if result not in ("success", "no_target_in_range"):
            raise ValueError("invalid clear result")
        self._move_to(action.position)
        self.clear_attempt_count += 1
        self.virtual_time_s += (
            self.config.successful_clear_duration_s
            if result == "success" else self.config.failed_clear_duration_s
        )
        record = self.channels[action.channel]
        is_fallback = self.active_plan_kind == "FIRST_BEARING_STRIP"
        if is_fallback:
            self.fallback_clear_count += 1
        else:
            self.cover_clear_count += 1
            record.optimized_clear_count += 1
            # A coverage-scan near response has no direction anchor and needs
            # no artificial localization-route accounting.
            if record.anchor_position is not None:
                self._record_optimization_point(record, action.position)
        if result == "success":
            self._finish_channel(record, action.position)
        else:
            self.failed_clear_count += 1
            self.active_plan_index += 1
            if self.active_plan_index >= len(self.active_plan_points):
                if is_fallback:
                    self.session_status = SessionStatus.MODEL_OR_PROTOCOL_INCONSISTENCY
                    record.geometry_failure_reason = "FALLBACK_STRIP_EXHAUSTED"
                elif self.active_plan_kind == "NEAR_SOURCE":
                    self.session_status = SessionStatus.MODEL_OR_PROTOCOL_INCONSISTENCY
                    record.geometry_failure_reason = "NEAR_CLEAR_FAILED"
                else:
                    record.geometry_valid = False
                    record.geometry_failure_reason = "CERTIFIED_CLEAR_PLAN_FAILED"
                    self.active_plan_kind = None
                    self.active_plan_points = ()
                    self.active_plan_index = 0
                    self._activate_fallback(record)
        self.pending_action = None

    def apply_exit_result(self, action):
        self._require_pending(action, "EXIT")
        self.pending_action = None
        self.session_status = SessionStatus.EXITED

    def summary(self):
        counts = {
            status.value: sum(record.status == status for record in self.channels.values())
            for status in ChannelStatus
        }
        average = (self.virtual_time_s / self.successful_clear_count
                   if self.successful_clear_count else None)
        return {
            "strategy": self.config.strategy,
            "session_status": self.session_status.value,
            "planner_phase": self.phase.value,
            "completion_reason": self.completion_reason,
            "completion_certificate": self.has_completion_certificate(),
            "channel_counts": counts,
            "cleared_count": self.successful_clear_count,
            "virtual_time_s": self.virtual_time_s,
            "average_clear_time_s": average,
            "distance_m": self.distance_m,
            "measure_count": self.measure_count,
            "coverage_measure_count": self.coverage_measure_count,
            "localize_measure_count": self.localize_measure_count,
            "switch_count": self.switch_count,
            "clear_attempt_count": self.clear_attempt_count,
            "failed_clear_count": self.failed_clear_count,
            "cover_clear_count": self.cover_clear_count,
            "fallback_clear_count": self.fallback_clear_count,
            "planning_time_s": self.planning_time_s,
            "fallback_source_count": sum(r.fallback_used for r in self.channels.values()),
            "fallback_reasons": {str(r.channel): r.geometry_failure_reason
                                 for r in self.channels.values() if r.fallback_used},
            "local_cover_plan_point_counts": [p["point_count"] for p in self.cover_plan_history],
            "pending_sources": sorted(self.pending_sources),
            "task_selection_count": len(self.task_history),
            "batch_channels": tuple(self.batch_channels),
            "current_position": self.current_position,
            "current_channel": self.current_channel,
        }


def q3_baseline_upper_bounds(config=None):
    """Return the documented conservative B0 request and virtual-time bounds."""
    config = config or Q3Config()
    _validate_config(config)
    station_count = len(seven_station_route(config.ring_radius_m))
    channel_count = config.channel_max - config.channel_min + 1
    scan_count = station_count * channel_count
    strip_count = len(strip_clear_points(
        (0.0, 0.0), 0.0, config.strip_max_range_m,
        config.strip_step_m, config.strip_offsets_m
    ))
    station_route_m = q3_coverage_certificate(
        config.target_radius_m, config.minimum_receive_radius_m, config.ring_radius_m
    )["route_length_m"]
    strip_closed_route_m = (
        abs(config.strip_offsets_m[0]) + 2 * config.strip_max_range_m
        + abs(config.strip_offsets_m[1] - config.strip_offsets_m[0])
        + abs(config.strip_offsets_m[1])
    )
    per_source_clear_s = (
        strip_closed_route_m / config.speed_mps
        + (strip_count - 1) * config.failed_clear_duration_s
        + config.successful_clear_duration_s
    )
    virtual_time_s = (
        station_route_m / config.speed_mps
        + scan_count * config.measure_duration_s
        + scan_count * config.switch_duration_s
        + config.source_count_upper_bound * per_source_clear_s
    )
    return {
        "strategy_scope": "b0_serial_or_b0_batch_fifo_conservative_bound",
        "coverage_station_count": station_count,
        "scan_measurement_count": scan_count,
        "strip_point_count": strip_count,
        "clear_request_count": strip_count * config.source_count_upper_bound,
        "logical_request_count_including_enter_exit": (
            scan_count + strip_count * config.source_count_upper_bound + 2
        ),
        "station_route_m": station_route_m,
        "strip_closed_route_m_per_source": strip_closed_route_m,
        "clear_time_bound_s_per_source": per_source_clear_s,
        "virtual_time_bound_s": virtual_time_s,
    }


def q3_v2_upper_bounds(config=None):
    """Return the deliberately loose V2 bound with complete B0 fallback."""
    config = config or Q3Config(strategy="v2_local", max_optimized_clear_attempts_per_source=8)
    baseline = q3_baseline_upper_bounds(config)
    extra_per_source = (
        config.extra_route_budget_m_per_source / config.speed_mps
        + config.max_extra_measurements_per_source
        * (config.measure_duration_s + config.switch_duration_s)
        + config.max_optimized_clear_attempts_per_source
        * config.successful_clear_duration_s
    )
    result = dict(baseline)
    result.update({
        "strategy_scope": "v2_local_with_complete_152_point_fallback",
        "v2_extra_time_bound_s_per_source": extra_per_source,
        "virtual_time_bound_s": (baseline["virtual_time_bound_s"]
                                  + config.source_count_upper_bound * extra_per_source),
        "logical_request_count_including_enter_exit": (
            baseline["logical_request_count_including_enter_exit"]
            + config.source_count_upper_bound
            * (config.max_extra_measurements_per_source
               + config.max_optimized_clear_attempts_per_source)
        ),
    })
    return result


def q3_global_upper_bounds(config=None):
    """Loose finite bound; B0's return-to-scan argument does not apply to V3."""
    config = config or Q3Config()
    _validate_config(config)
    outer_radius = min(config.target_radius_m, config.maximum_receive_radius_m) / math.cos(math.pi/config.circle_sides)
    grid_count = min(100000, max(1, math.ceil((2*outer_radius + 2e-7)/config.local_grid_cell_m))**2)
    source_count = config.source_count_upper_bound
    scan_count = 7 * (config.channel_max-config.channel_min+1)
    strip_count = len(strip_clear_points((0., 0.), 0., config.strip_max_range_m,
                                        config.strip_step_m, config.strip_offsets_m))
    measures = scan_count + source_count*config.max_extra_measurements_per_source
    clears = source_count*(grid_count+strip_count)
    # Source envelopes are inside the circumscribed target disk. Safe points
    # are <=1000m from an envelope vertex; rectangle centres <=2*target_outer.
    target_outer = config.target_radius_m/math.cos(math.pi/config.circle_sides)
    position_norm = max(config.ring_radius_m + config.strip_max_range_m
                        + max(map(abs, config.strip_offsets_m)),
                        2*target_outer, target_outer+config.minimum_receive_radius_m)
    distance_bound = 2*position_norm*(measures+clears)
    return {"strategy_scope": "v3_global_finite_loose_bound_not_performance_prediction",
            "local_grid_point_bound_per_source": grid_count,
            "logical_request_count_including_enter_exit": 2+measures+clears,
            "distance_bound_m": distance_bound,
            "virtual_time_bound_s": distance_bound/config.speed_mps
            + measures*(config.measure_duration_s+config.switch_duration_s)
            + clears*max(config.failed_clear_duration_s, config.successful_clear_duration_s)}
