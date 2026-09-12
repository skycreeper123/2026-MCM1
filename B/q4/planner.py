"""Observation-only Q4 controller; hidden world data is never an input."""

from dataclasses import asdict, dataclass
import math
import time

from B.q3.localize import ClearPlan
from .geometry import (SourceRecord, build_clear_plan, classify_q4_reception,
                       load_and_verify_station_cover, update_positive_hull,
                       update_source_region, update_cover_after_failed_clear,
                       reprice, select_clear_route)
from .belief import update_belief_scenarios
from .policy import (MeasureTask, candidate_tasks, marginal_detour,
                     score_measure_policy)
from .routing import RouteTask, optimize_service_route, task_route_cost


@dataclass(frozen=True)
class Q4Config:
    strategy: str = "q4_adaptive_cooperative_v2"
    local_measurement_limit: int = 4
    local_distance_limit_m: float = 4000.0
    candidate_limit: int = 24
    response_intervals: int = 32
    refined_response_intervals: int = 128
    belief_scenarios: int = 512
    minimum_belief_scenarios: int = 24
    dynamic_candidate_limit: int = 4
    dynamic_refresh_m: float = 25.0
    direct_clear_points: int = 4
    large_region_points: int = 30
    uncertain_no_signal_limit: int = 2
    clear_batch_points: int = 8
    planning_call_s: float = 1.5
    planning_total_s: float = 60.0
    guaranteed_saving_s: float = 1.0
    uncertain_saving_s: float = 10.0
    uncertain_extra_s: float = 30.0
    services_per_station: int = 4
    route_depth: int = 3
    route_width: int = 12
    exit_reserve_s: float = 30.0

    def __post_init__(self):
        if self.strategy != "q4_adaptive_cooperative_v2":
            raise ValueError("unsupported Q4 strategy")
        for name in ("local_measurement_limit", "candidate_limit", "response_intervals",
                     "refined_response_intervals", "belief_scenarios", "minimum_belief_scenarios",
                     "dynamic_candidate_limit", "direct_clear_points", "large_region_points",
                     "uncertain_no_signal_limit", "clear_batch_points", "services_per_station",
                     "route_depth", "route_width"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        for name in ("local_distance_limit_m", "dynamic_refresh_m", "planning_call_s", "planning_total_s",
                     "guaranteed_saving_s", "uncertain_saving_s", "uncertain_extra_s", "exit_reserve_s"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be finite and nonnegative")


@dataclass(frozen=True)
class Q4Action:
    action_id: int
    kind: str
    position: tuple | None = None
    channel: int | None = None
    reason: str = ""
    purpose: str = ""
    station_id: int | None = None
    reception: str = "UNCERTAIN"

    def as_dict(self):
        return asdict(self)


def check_completion(channels, discovery_ledger, cover):
    cleared = [ch for ch, r in channels.items() if r.state == "CLEARED"]
    if len(cleared) == 16:
        return {"complete": True, "method": "16_DISTINCT_CHANNELS_CLEARED", "channels": cleared}
    absent = [ch for ch, r in channels.items() if r.state == "ABSENT" and
              r.absence_certificate is not None and
              r.absence_certificate.get("network_id") == cover.network_id]
    complete = cover.certificate.get("passed", False) and len(cleared)+len(absent) == 20
    return {"complete": complete, "method": "CLEARED_OR_FULL_NETWORK_ABSENT" if complete else "UNRESOLVED",
            "cleared": cleared, "absent": absent, "network_id": cover.network_id}


class Q4Planner:
    def __init__(self, config=None, cover=None):
        self.config = config or Q4Config()
        self.cover = cover or load_and_verify_station_cover()
        if not self.cover.certificate.get("passed"):
            raise ValueError("Uncertified station network")
        self.channels = {ch: SourceRecord(ch) for ch in range(1, 21)}
        self.discovery_ledger = {ch: set() for ch in self.channels}
        self.station_revisits = set()
        self.revisit_considered = set()
        self.position, self.receiver_channel = (0.0, 0.0), 1
        self.station_cursor, self.services_here = 0, 0
        self.pending = None
        self.active_clear = None
        self.clear_index = 0
        self.clear_batch_failures = 0
        self.pair_channel = None
        self.next_id, self.applied = 1, {}
        self.stop_reason, self.exited = None, False
        self.planning_elapsed_s = 0.0
        self.clear_cache = {}
        self.clear_route_cache = {}
        self.last_fallback_route = ()
        self.last_decision = None
        self.detour_reservation = None
        self.metrics = {"measurements": 0, "switches": 0, "clear_requests": 0,
                        "failed_clears": 0, "successful_clears": 0, "distance_m": 0.0,
                        "no_signal": 0, "no_signal_revisits": 0, "station_revisits": 0,
                        "pairs_started": 0, "pairs_received": 0, "local_measurements": 0}
        self.metrics.update({"belief_updates": 0, "belief_fallbacks": 0,
                             "dynamic_candidates_used": 0, "sparse_covers": 0,
                             "sparse_cells_removed": 0, "clear_replans": 0,
                             "early_absent_channels": 0, "fallback_count": 0,
                             "no_signal_by_purpose": {}})

    def _action(self, kind, position=None, channel=None, **kwargs):
        self.pending = Q4Action(self.next_id, kind, position, channel, **kwargs)
        self.next_id += 1
        return self.pending

    def _virtual_time(self):
        m = self.metrics
        return (m["distance_m"]/5 + 5*m["measurements"] + m["switches"]
                + 3*m["failed_clears"] + 5*m["successful_clears"])

    def _detected(self):
        return [r for r in self.channels.values() if r.state == "DETECTED"]

    def _known_count(self):
        return sum(r.state in {"DETECTED", "CLEARED"} for r in self.channels.values())

    def _scan_channels(self, station_id):
        if self._known_count() >= 16:
            return []
        order = sorted((ch for ch, r in self.channels.items() if r.state == "UNKNOWN" and
                        station_id not in self.discovery_ledger[ch]), reverse=bool(self.station_cursor % 2))
        if self.receiver_channel in order:
            order.remove(self.receiver_channel)
            order.insert(0, self.receiver_channel)
        return order

    def _backup(self, record, continuation=None, optimize_order=False):
        key = (record.channel, record.region_version, record.remaining_version)
        if key not in self.clear_cache:
            self.clear_cache[key] = build_clear_plan(record, self.position)
        if not optimize_order:
            return reprice(self.clear_cache[key], self.position, continuation)
        route_key = (key, id(record.belief), tuple(self.position),
                     tuple(continuation) if continuation is not None else None)
        if route_key not in self.clear_route_cache:
            self.clear_route_cache[route_key] = select_clear_route(
                record, self.clear_cache[key], self.position, continuation)
        return self.clear_route_cache[route_key]

    def _start_clear(self, record, plan):
        record.clear_certificate = plan.as_dict()
        self.active_clear, self.clear_index = (record.channel, plan), 0
        self.clear_batch_failures = 0
        if plan.kind == "SPARSE_REMAINING_GRID":
            self.metrics["sparse_covers"] += 1
            self.metrics["sparse_cells_removed"] += len(plan.cover_certificate["removed_cells"])
        if "STRIP" in plan.kind:
            self.metrics["fallback_count"] += 1
        return self._clear_action()

    def _absence_certified(self, channel):
        evidence = self.discovery_ledger[channel]
        if evidence == set(range(len(self.cover.stations))):
            return "FULL_NETWORK"
        if self.cover.provider_sets and all(providers <= evidence for providers in self.cover.provider_sets):
            return "LEAF_PROVIDER_SETS"
        return None

    def _update_belief(self, record):
        if self.planning_elapsed_s >= self.config.planning_total_s:
            record.belief = None
            self.metrics["belief_fallbacks"] += 1
            return
        belief = update_belief_scenarios(record, self.config.belief_scenarios,
                                         self.config.minimum_belief_scenarios)
        self.metrics["belief_updates"] += 1
        self.metrics["belief_fallbacks"] += int(not belief.active)

    def _dynamic_candidates(self, record, continuation):
        target = tuple(continuation) if continuation is not None else None
        refresh = (record.dynamic_cache_version != record.region_version or
                   record.dynamic_cache_origin is None or
                   math.dist(record.dynamic_cache_origin, self.position) >= self.config.dynamic_refresh_m or
                   record.dynamic_cache_target != target)
        if refresh:
            points = [self.position]
            if target is not None:
                points.extend(tuple(self.position[k]+fraction*(target[k]-self.position[k]) for k in (0, 1))
                              for fraction in (0.25, 0.5, 0.75))
            record.dynamic_candidates = tuple(dict.fromkeys(points))[:self.config.dynamic_candidate_limit]
            record.dynamic_cache_origin = self.position
            record.dynamic_cache_target = target
            record.dynamic_cache_version = record.region_version
        return record.dynamic_candidates

    def _clear_action(self):
        ch, plan = self.active_clear
        return self._action("CLEAR", plan.points[self.clear_index], ch,
                            purpose="CERTIFIED_CLEAR", reason=plan.kind)

    def _local_allowed(self, record, points, continuation=None):
        detour = marginal_detour(self.position, points, continuation)
        return (record.local_measurements+len(points) <= self.config.local_measurement_limit and
                record.local_travel_m+record.reserved_detour_m+detour <= self.config.local_distance_limit_m)

    def _service(self, record, continuation, deadline):
        backup = self._backup(record, continuation, optimize_order=True)
        # A single certified clear or an exhausted search budget needs no probe.
        if (backup.point_count <= self.config.direct_clear_points or time.monotonic() >= deadline
                or record.local_measurements >= self.config.local_measurement_limit):
            return self._start_clear(record, backup)
        future = [self.cover.stations[i] for i in self.cover.route[self.station_cursor+1:]]
        tasks = candidate_tasks(record, self.position, continuation, backup, future,
                                self.config.candidate_limit, deadline,
                                self._dynamic_candidates(record, continuation))
        selected = None
        for task in tasks:
            if time.monotonic() >= deadline:
                break
            guaranteed = task.reception in {"POSITIVE_HULL", "PAIR_AT_LEAST_ONE"}
            if (not guaranteed and record.consecutive_uncertain_no_signal >= self.config.uncertain_no_signal_limit):
                continue
            if not self._local_allowed(record, task.points, continuation):
                continue
            bound = score_measure_policy(record, task, self.position, self.receiver_channel,
                                         continuation, backup, self.config, deadline)
            if bound.eligible and (selected is None or bound.expected_s < selected[1].expected_s):
                selected = task, bound
        if selected is None:
            return self._start_clear(record, backup)
        task, bound = selected
        if time.monotonic() < deadline and self.config.refined_response_intervals > self.config.response_intervals:
            refined = score_measure_policy(record, task, self.position, self.receiver_channel,
                                           continuation, backup, self.config, deadline,
                                           response_intervals=self.config.refined_response_intervals)
            if refined.eligible:
                bound = refined
            else:
                return self._start_clear(record, backup)
        self.last_decision = asdict(bound)
        detour = marginal_detour(self.position, task.points, continuation)
        record.reserved_detour_m += detour
        self.detour_reservation = {"channel": record.channel, "origin": self.position,
                                   "points": task.points, "continuation": continuation,
                                   "reserved_m": detour}
        if task.pair is not None:
            record.pending_pair = task.pair
            self.pair_channel = record.channel
            self.metrics["pairs_started"] += 1
        if task.origin == "dynamic_route":
            self.metrics["dynamic_candidates_used"] += 1
        return self._action("MEASURE", task.points[0], record.channel,
                            purpose="LOCALIZE_PAIR_FIRST" if task.pair else "LOCALIZE_SINGLE",
                            reception=task.reception, reason="certified policy cost comparison")

    def _remaining_route(self, deadline):
        stations = []
        for sid in self.cover.route[self.station_cursor+1:]:
            channels = tuple(self._scan_channels(sid))
            if channels:
                stations.append(RouteTask(("STATION", sid), (self.cover.stations[sid],), "STATION", channels=channels))
        clears = [RouteTask(("CLEAR", r.channel), self._backup(r).points, "CLEAR", channel=r.channel)
                  for r in self._detected()]
        self.last_fallback_route = optimize_service_route(stations, clears, self.position,
                                                         self.receiver_channel, deadline,
                                                         self.config.route_depth, self.config.route_width)
        return self.last_fallback_route

    def propose_action(self, deadline_monotonic=math.inf):
        if self.pending is not None:
            return self.pending
        if self.exited:
            return None
        if self.stop_reason or self.has_completion_certificate():
            return self._action("EXIT", reason=self.stop_reason or "completion certificate")
        if self.active_clear is not None:
            return self._clear_action()
        if self.pair_channel is not None:
            r = self.channels[self.pair_channel]
            pair = r.pending_pair
            if pair is None or pair.region_version != r.region_version or pair.hull_version != r.hull_version:
                self.stop_reason = "INCONSISTENCY: stale pair certificate"
                return self._action("EXIT", reason=self.stop_reason)
            return self._action("MEASURE", pair.points[1], r.channel,
                                purpose="LOCALIZE_PAIR_SECOND", reception="PAIR_SECOND_GUARANTEED")
        start = time.monotonic()
        remaining = max(0.0, self.config.planning_total_s-self.planning_elapsed_s)
        deadline = min(deadline_monotonic, start+min(self.config.planning_call_s, remaining))
        try:
            while self.station_cursor < len(self.cover.route):
                sid = self.cover.route[self.station_cursor]
                station = self.cover.stations[sid]
                unknown = self._scan_channels(sid)
                if unknown:
                    return self._action("MEASURE", station, unknown[0], purpose="DISCOVERY_SCAN", station_id=sid)
                # Remeasures are 'on the way' only when actually at that station.
                if self.position == station:
                    for r in self._detected():
                        key = sid, r.channel
                        if (key in self.revisit_considered or station in r.used_positions or
                                r.local_measurements >= self.config.local_measurement_limit or
                                self.services_here >= self.config.services_per_station):
                            continue
                        self.revisit_considered.add(key)
                        if time.monotonic() >= deadline:
                            break
                        cert = classify_q4_reception(r, station)
                        if cert == "GUARANTEED_NO_SIGNAL":
                            continue
                        backup = self._backup(r)
                        task = MeasureTask((station,), cert)
                        bound = score_measure_policy(r, task, station, self.receiver_channel,
                                                     None, backup, self.config, deadline, station=True)
                        if bound.eligible:
                            self.last_decision = asdict(bound)
                            self.services_here += 1
                            return self._action("MEASURE", station, r.channel, purpose="STATION_REVISIT",
                                                station_id=sid, reception=cert)
                if self._detected() and self.services_here < self.config.services_per_station:
                    route = self._remaining_route(deadline)
                    if route and route[0].kind == "CLEAR":
                        self.services_here += 1
                        continuation = next((t.points[0] for t in route[1:] if t.kind == "STATION"), None)
                        return self._service(self.channels[route[0].channel], continuation, deadline)
                self.station_cursor += 1
                self.services_here = 0
            detected = self._detected()
            if detected:
                route = self._remaining_route(deadline)
                record = self.channels[route[0].channel]
                continuation = route[1].points[0] if len(route) > 1 else None
                return self._service(record, continuation, deadline)
            self.stop_reason = "INCOMPLETE: discovery ended without valid completion evidence"
            return self._action("EXIT", reason=self.stop_reason)
        except (ValueError, ArithmeticError) as exc:
            self.stop_reason = f"INCONSISTENCY: {exc}"
            return self._action("EXIT", reason=self.stop_reason)
        finally:
            self.planning_elapsed_s += time.monotonic()-start

    def _accept(self, action, result):
        if action.action_id in self.applied:
            if self.applied[action.action_id] != (action, result):
                raise ValueError("Conflicting duplicate response")
            return False
        if action != self.pending:
            raise ValueError("Response does not match pending action")
        self.applied[action.action_id] = action, result
        self.pending = None
        if action.position is not None:
            self.metrics["distance_m"] += math.dist(self.position, action.position)
            self.position = action.position
        return True

    def _settle_detour(self, action, result):
        reservation = self.detour_reservation
        if reservation is None or reservation["channel"] != action.channel:
            return
        record = self.channels[action.channel]
        is_pair_first = action.purpose == "LOCALIZE_PAIR_FIRST"
        if is_pair_first and result == "no_signal":
            return  # full two-point reservation remains mandatory
        points = reservation["points"]
        if is_pair_first:
            points = points[:1]
        actual = marginal_detour(reservation["origin"], points, reservation["continuation"])
        record.reserved_detour_m = max(0.0, record.reserved_detour_m-reservation["reserved_m"])
        record.local_travel_m += actual
        self.detour_reservation = None

    def apply_measure_result(self, action, result, svd_deg=None):
        if action.kind != "MEASURE" or result not in {"direction", "near", "no_signal"}:
            raise ValueError("Invalid measurement response")
        if result == "direction" and (isinstance(svd_deg, bool) or not isinstance(svd_deg, (int, float)) or not math.isfinite(svd_deg)):
            raise ValueError("Direction requires a finite bearing")
        if not self._accept(action, (result, svd_deg)):
            return
        self._settle_detour(action, result)
        self.metrics["measurements"] += 1
        self.metrics["switches"] += int(self.receiver_channel != action.channel)
        self.receiver_channel = action.channel
        r = self.channels[action.channel]
        obs = {"position": action.position, "result": result, "svd_deg": svd_deg,
               "purpose": action.purpose, "station_id": action.station_id}
        r.observations.append(obs)
        r.used_positions.add(action.position)
        if action.purpose.startswith("LOCALIZE") or action.purpose == "STATION_REVISIT":
            r.local_measurements += 1
            self.metrics["local_measurements"] += 1
            r.last_local_position = action.position
        if action.purpose == "STATION_REVISIT":
            self.station_revisits.add((action.station_id, action.channel))
            self.metrics["station_revisits"] += 1
        if result == "no_signal":
            self.metrics["no_signal"] += 1
            by_purpose = self.metrics["no_signal_by_purpose"]
            by_purpose[action.purpose] = by_purpose.get(action.purpose, 0)+1
            if action.purpose != "DISCOVERY_SCAN":
                self.metrics["no_signal_revisits"] += 1
            if action.reception in {"POSITIVE_HULL", "PAIR_SECOND_GUARANTEED"}:
                self.stop_reason = "INCONSISTENCY: guaranteed reception returned no_signal"
            elif action.purpose != "DISCOVERY_SCAN" and not action.purpose.startswith("LOCALIZE_PAIR"):
                r.consecutive_uncertain_no_signal += 1
            if action.purpose == "DISCOVERY_SCAN":
                self.discovery_ledger[action.channel].add(action.station_id)
                method = self._absence_certified(action.channel)
                if method is not None:
                    r.state = "ABSENT"
                    r.absence_certificate = {"method": method, "network_id": self.cover.network_id,
                                             "station_count": len(self.discovery_ledger[action.channel])}
                    self.metrics["early_absent_channels"] += int(method == "LEAF_PROVIDER_SETS")
            elif r.vertices is not None:
                update_start = time.monotonic()
                self._update_belief(r)
                self.planning_elapsed_s += time.monotonic()-update_start
            return
        was_unknown = r.state == "UNKNOWN"
        r.state = "DETECTED"
        if action.purpose != "DISCOVERY_SCAN":
            r.consecutive_uncertain_no_signal = 0
        if action.purpose.startswith("LOCALIZE_PAIR"):
            self.metrics["pairs_received"] += 1
            r.pending_pair, self.pair_channel = None, None
        update_positive_hull(r, obs)
        if result == "near":
            plan = ClearPlan("NEAR", (action.position,), True, 5.0, 0, 5,
                             cover_certificate={"method": "POSITIVE_NEAR", "observation": obs})
            r.clear_certificate = plan.as_dict()
            self.active_clear, self.clear_index = (r.channel, plan), 0
        else:
            try:
                update_start = time.monotonic()
                update_source_region(r, obs)
                self._update_belief(r)
                self.planning_elapsed_s += time.monotonic()-update_start
            except (ValueError, ArithmeticError) as exc:
                self.stop_reason = f"INCONSISTENCY: {exc}"
        if was_unknown and r.discovered_virtual_s is None:
            r.discovered_virtual_s = self._virtual_time()
        if self._known_count() > 16:
            self.stop_reason = "INCONSISTENCY: more than 16 distinct source channels"

    def apply_clear_result(self, action, result):
        if action.kind != "CLEAR" or result not in {"success", "no_target_in_range"}:
            raise ValueError("Invalid clear response")
        if not self._accept(action, result):
            return
        self.metrics["clear_requests"] += 1
        record = self.channels[action.channel]
        record.clear_requests += 1
        # Deliberately do NOT modify receiver_channel here.
        if result == "success":
            record.state = "CLEARED"
            self.metrics["successful_clears"] += 1
            record.cleared_virtual_s = self._virtual_time()
            self.active_clear = None
            self.clear_batch_failures = 0
        else:
            self.metrics["failed_clears"] += 1
            plan = self.active_clear[1]
            if plan.kind == "NEAR" or plan.point_count == 1:
                self.stop_reason = "INCONSISTENCY: certified clear plan exhausted"
                self.active_clear = None
            else:
                update_start = time.monotonic()
                update_cover_after_failed_clear(record, action.position)
                self._update_belief(record)
                self.planning_elapsed_s += time.monotonic()-update_start
                self.clear_index += 1
                self.clear_batch_failures += 1
                if self.clear_index >= plan.point_count:
                    self.stop_reason = "INCONSISTENCY: certified clear plan exhausted"
                    self.active_clear = None
                # A failed clearance invalidates the posterior ordering.  Rebuild
                # immediately while belief guidance is active; only the
                # geometry-only fallback may continue a bounded batch.
                elif ((record.belief is not None and record.belief.active)
                      or self.clear_batch_failures >= self.config.clear_batch_points):
                    self.metrics["clear_replans"] += 1
                    self.active_clear = None
                    self.clear_index = 0
                    self.clear_batch_failures = 0

    def has_completion_certificate(self):
        return not self.stop_reason and check_completion(self.channels, self.discovery_ledger, self.cover)["complete"]

    def mark_time_budget_incomplete(self):
        if not self.has_completion_certificate():
            self.stop_reason = "INCOMPLETE: real-time exit reserve reached"

    def apply_exit_result(self, action):
        if action.kind != "EXIT":
            raise ValueError("Expected EXIT action")
        if self._accept(action, "exit"):
            self.exited = True

    def summary(self):
        m = self.metrics
        virtual = self._virtual_time()
        clear_counts = sorted(r.clear_requests for r in self.channels.values() if r.state == "CLEARED")
        latencies = sorted(r.cleared_virtual_s-r.discovered_virtual_s for r in self.channels.values()
                           if r.cleared_virtual_s is not None and r.discovered_virtual_s is not None)
        percentile = lambda values, q: (values[min(len(values)-1, math.ceil(q*len(values))-1)] if values else None)
        return {"strategy": self.config.strategy, "cleared_count": m["successful_clears"],
                "known_count": self._known_count(), "channel_states": {ch: r.state for ch, r in self.channels.items()},
                "completion": check_completion(self.channels, self.discovery_ledger, self.cover),
                "stop_reason": self.stop_reason, "receiver_channel": self.receiver_channel,
                "position": self.position, "station_cursor": self.station_cursor,
                "station_count": len(self.cover.stations), "station_certificate": self.cover.certificate,
                "network_id": self.cover.network_id, "metrics": dict(m),
                "estimated_virtual_time_s": virtual,
                "average_clear_time_s": virtual/m["successful_clears"] if m["successful_clears"] else None,
                "clear_points_per_source": {"mean": sum(clear_counts)/len(clear_counts) if clear_counts else None,
                                            "p90": percentile(clear_counts, .9),
                                            "max": max(clear_counts) if clear_counts else None},
                "discovery_to_clear_s": {"p95": percentile(latencies, .95),
                                         "max": max(latencies) if latencies else None},
                "planning_elapsed_s": self.planning_elapsed_s, "last_decision": self.last_decision,
                "fallback_route_estimate_s": task_route_cost(self.last_fallback_route, self.position, self.receiver_channel),
                "fallback_route_scope": "last planning snapshot; reprice after every action, not a total-run bound",
                "source_budgets": {ch: {"measurements": r.local_measurements,
                                        "marginal_detour_used_m": r.local_travel_m,
                                        "reserved_detour_m": r.reserved_detour_m,
                                        "uncertain_no_signal_streak": r.consecutive_uncertain_no_signal,
                                        "failed_clear_disks": len(r.failed_clear_disks),
                                        "belief": r.belief.summary() if r.belief is not None else None}
                                   for ch, r in self.channels.items() if r.anchor is not None}}


def plan_next_action(planner, deadline_monotonic=math.inf):
    return planner.propose_action(deadline_monotonic)
