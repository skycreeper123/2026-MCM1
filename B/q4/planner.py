"""Observation-only Q4 controller; hidden world data is never an input."""

from dataclasses import asdict, dataclass
import math
import time

from B.q3.localize import ClearPlan
from .geometry import (SourceRecord, build_clear_plan, classify_q4_reception,
                       load_and_verify_station_cover, update_positive_hull,
                       update_source_region, reprice)
from .policy import MeasureTask, candidate_tasks, score_measure_policy
from .routing import RouteTask, optimize_service_route, task_route_cost


@dataclass(frozen=True)
class Q4Config:
    local_measurement_limit: int = 4
    local_distance_limit_m: float = 4000.0
    candidate_limit: int = 24
    response_intervals: int = 32
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
        for name in ("local_measurement_limit", "candidate_limit", "response_intervals",
                     "services_per_station", "route_depth", "route_width"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        for name in ("local_distance_limit_m", "planning_call_s", "planning_total_s",
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
              discovery_ledger[ch] == set(range(len(cover.stations)))]
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
        self.pair_channel = None
        self.next_id, self.applied = 1, {}
        self.stop_reason, self.exited = None, False
        self.planning_elapsed_s = 0.0
        self.clear_cache = {}
        self.last_fallback_route = ()
        self.last_decision = None
        self.metrics = {"measurements": 0, "switches": 0, "clear_requests": 0,
                        "failed_clears": 0, "successful_clears": 0, "distance_m": 0.0,
                        "no_signal": 0, "no_signal_revisits": 0, "station_revisits": 0,
                        "pairs_started": 0, "pairs_received": 0, "local_measurements": 0}

    def _action(self, kind, position=None, channel=None, **kwargs):
        self.pending = Q4Action(self.next_id, kind, position, channel, **kwargs)
        self.next_id += 1
        return self.pending

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

    def _backup(self, record, continuation=None):
        key = (record.channel, record.region_version)
        if key not in self.clear_cache:
            self.clear_cache[key] = build_clear_plan(record, self.position)
        return reprice(self.clear_cache[key], self.position, continuation)

    def _start_clear(self, record, plan):
        record.clear_certificate = plan.as_dict()
        self.active_clear, self.clear_index = (record.channel, plan), 0
        return self._clear_action()

    def _clear_action(self):
        ch, plan = self.active_clear
        return self._action("CLEAR", plan.points[self.clear_index], ch,
                            purpose="CERTIFIED_CLEAR", reason=plan.kind)

    def _closed_local_cost(self, record, points):
        a = record.anchor["position"]
        anchor = (a["x"], a["y"])
        path = (anchor, self.position) + points + (anchor,)
        return sum(math.dist(p, q) for p, q in zip(path, path[1:]))

    def _local_allowed(self, record, points):
        return (record.local_measurements+len(points) <= self.config.local_measurement_limit and
                record.local_travel_m+self._closed_local_cost(record, points) <= self.config.local_distance_limit_m)

    def _service(self, record, continuation, deadline):
        backup = self._backup(record, continuation)
        # A single certified clear or an exhausted search budget needs no probe.
        if backup.point_count == 1 or time.monotonic() >= deadline or record.local_measurements >= self.config.local_measurement_limit:
            return self._start_clear(record, backup)
        future = [self.cover.stations[i] for i in self.cover.route[self.station_cursor+1:]]
        tasks = candidate_tasks(record, self.position, continuation, backup, future,
                                self.config.candidate_limit, deadline)
        selected = None
        for task in tasks:
            if time.monotonic() >= deadline:
                break
            if not self._local_allowed(record, task.points):
                continue
            bound = score_measure_policy(record, task, self.position, self.receiver_channel,
                                         continuation, backup, self.config, deadline)
            if bound.eligible and (selected is None or bound.upper_s < selected[1].upper_s):
                selected = task, bound
        if selected is None:
            return self._start_clear(record, backup)
        task, bound = selected
        self.last_decision = asdict(bound)
        # Reserve the complete closed excursion and both pair measurements;
        # a first no_signal cannot lose its already-approved second action.
        record.local_travel_m += self._closed_local_cost(record, task.points)
        if task.pair is not None:
            record.pending_pair = task.pair
            self.pair_channel = record.channel
            self.metrics["pairs_started"] += 1
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
                        if key in self.revisit_considered or station in r.used_positions:
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

    def apply_measure_result(self, action, result, svd_deg=None):
        if action.kind != "MEASURE" or result not in {"direction", "near", "no_signal"}:
            raise ValueError("Invalid measurement response")
        if result == "direction" and (isinstance(svd_deg, bool) or not isinstance(svd_deg, (int, float)) or not math.isfinite(svd_deg)):
            raise ValueError("Direction requires a finite bearing")
        if not self._accept(action, (result, svd_deg)):
            return
        self.metrics["measurements"] += 1
        self.metrics["switches"] += int(self.receiver_channel != action.channel)
        self.receiver_channel = action.channel
        r = self.channels[action.channel]
        obs = {"position": action.position, "result": result, "svd_deg": svd_deg,
               "purpose": action.purpose, "station_id": action.station_id}
        r.observations.append(obs)
        r.used_positions.add(action.position)
        if action.purpose.startswith("LOCALIZE"):
            r.local_measurements += 1
            self.metrics["local_measurements"] += 1
            r.last_local_position = action.position
        if action.purpose == "STATION_REVISIT":
            self.station_revisits.add((action.station_id, action.channel))
            self.metrics["station_revisits"] += 1
        if result == "no_signal":
            self.metrics["no_signal"] += 1
            if action.purpose != "DISCOVERY_SCAN":
                self.metrics["no_signal_revisits"] += 1
            if action.reception in {"POSITIVE_HULL", "PAIR_SECOND_GUARANTEED"}:
                self.stop_reason = "INCONSISTENCY: guaranteed reception returned no_signal"
            if action.purpose == "DISCOVERY_SCAN":
                self.discovery_ledger[action.channel].add(action.station_id)
                if len(self.discovery_ledger[action.channel]) == len(self.cover.stations):
                    r.state = "ABSENT"
            return
        r.state = "DETECTED"
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
                update_source_region(r, obs)
            except (ValueError, ArithmeticError) as exc:
                self.stop_reason = f"INCONSISTENCY: {exc}"
        if self._known_count() > 16:
            self.stop_reason = "INCONSISTENCY: more than 16 distinct source channels"

    def apply_clear_result(self, action, result):
        if action.kind != "CLEAR" or result not in {"success", "no_target_in_range"}:
            raise ValueError("Invalid clear response")
        if not self._accept(action, result):
            return
        self.metrics["clear_requests"] += 1
        # Deliberately do NOT modify receiver_channel here.
        if result == "success":
            self.channels[action.channel].state = "CLEARED"
            self.metrics["successful_clears"] += 1
            self.active_clear = None
        else:
            self.metrics["failed_clears"] += 1
            self.clear_index += 1
            if self.clear_index >= self.active_clear[1].point_count:
                self.stop_reason = "INCONSISTENCY: certified clear plan exhausted"
                self.active_clear = None

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
        virtual = m["distance_m"]/5 + 5*m["measurements"]+m["switches"]+3*m["failed_clears"]+5*m["successful_clears"]
        return {"strategy": "q4_certified_cooperative", "cleared_count": m["successful_clears"],
                "known_count": self._known_count(), "channel_states": {ch: r.state for ch, r in self.channels.items()},
                "completion": check_completion(self.channels, self.discovery_ledger, self.cover),
                "stop_reason": self.stop_reason, "receiver_channel": self.receiver_channel,
                "position": self.position, "station_cursor": self.station_cursor,
                "station_count": len(self.cover.stations), "station_certificate": self.cover.certificate,
                "network_id": self.cover.network_id, "metrics": dict(m),
                "estimated_virtual_time_s": virtual,
                "average_clear_time_s": virtual/m["successful_clears"] if m["successful_clears"] else None,
                "planning_elapsed_s": self.planning_elapsed_s, "last_decision": self.last_decision,
                "fallback_route_estimate_s": task_route_cost(self.last_fallback_route, self.position, self.receiver_channel),
                "fallback_route_scope": "last planning snapshot; reprice after every action, not a total-run bound",
                "source_budgets": {ch: {"measurements": r.local_measurements, "closed_distance_m": r.local_travel_m}
                                   for ch, r in self.channels.items() if r.anchor is not None}}


def plan_next_action(planner, deadline_monotonic=math.inf):
    return planner.propose_action(deadline_monotonic)
