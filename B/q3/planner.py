"""Event-driven guaranteed B0 planner for Question 3.

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
    build_cover_plan,
    choose_detection_from_region,
    initialize_outer_region,
    update_outer_region,
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


class MeasurePurpose(str, Enum):
    COVERAGE_SCAN = "COVERAGE_SCAN"
    LOCALIZE = "LOCALIZE"


class ServicePhase(str, Enum):
    NONE = "NONE"
    READY_CLEAR = "READY_CLEAR"
    LOCALIZING = "LOCALIZING"
    COVERING = "COVERING"
    FALLBACK = "FALLBACK"


SUPPORTED_STRATEGIES = ("b0_serial", "b0_batch_fifo", "v2_local")


@dataclass(frozen=True)
class Q3Config:
    strategy: str = "v2_local"
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
    max_optimized_clear_attempts_per_source: int = 8
    extra_route_budget_m_per_source: float = 4000.0
    q2_candidate_limit: int = 12
    q2_response_intervals: int = 32
    q2_calculation_time_limit_s: float = 1.0
    total_planning_time_limit_s: float = 60.0
    minimum_radius_improvement_fraction: float = 0.05
    repeated_position_tolerance_m: float = 1e-6


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
    """Guaranteed seven-station scan plus first-bearing strip clear planner."""

    def __init__(self, config=None):
        self.config = config or Q3Config()
        _validate_config(self.config)
        self.stations = seven_station_route(self.config.ring_radius_m)
        self.channels = {
            channel: ChannelRecord(channel)
            for channel in range(self.config.channel_min, self.config.channel_max + 1)
        }
        self.session_status = SessionStatus.ACTIVE
        self.phase = PlannerPhase.SCAN
        self.completion_reason = None
        self.current_position = (0.0, 0.0)
        self.current_channel = self.config.channel_min
        self.station_index = 0
        self.scan_cursor = 0
        self.batch_channels = []
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

    def _all_resolved(self):
        return self.has_completion_certificate()

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
                    if self.config.strategy == "v2_local":
                        self._initialize_direction(record, action, bearing)
                    else:
                        record.anchor_position = action.position
                        record.anchor_bearing_deg = bearing
                    if self.config.strategy == "b0_serial":
                        self.active_service_channel = record.channel
                        self._activate_fallback(record)
                    else:
                        self.batch_channels.append(record.channel)
                else:
                    self.active_service_channel = record.channel
                    self._set_plan(record, "NEAR_SOURCE", (action.position,))
        else:
            self.localize_measure_count += 1
            record.extra_measure_count += 1
            self._record_optimization_point(record, action.position)
            if result == "no_signal":
                record.geometry_valid = False
                record.geometry_failure_reason = "SAFE_LOCALIZE_RETURNED_NO_SIGNAL"
                self._activate_fallback(record)
            elif result == "near":
                record.measured_positions.append(action.position)
                self._set_plan(record, "NEAR_SOURCE", (action.position,))
            else:
                self._append_direction(record, action, bearing)
                self.active_plan_kind = None
                self.active_plan_points = ()
                self.active_plan_index = 0
        self.pending_action = None

    def _finish_channel(self, record, position):
        record.status = ChannelStatus.CLEARED
        record.service_phase = ServicePhase.NONE
        record.clear_position = position
        self.successful_clear_count += 1
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
    config = config or Q3Config(strategy="v2_local")
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
