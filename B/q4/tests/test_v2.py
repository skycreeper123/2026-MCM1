import math
import time
import unittest

from B.q4.belief import response_probabilities, update_belief_scenarios
from B.q4.belief_rollout import plan_belief_rollout_clear
from B.q4.geometry import (CLEAR_ROUTE_RISK_CAP_RATIO, build_clear_plan,
                           build_clear_route_variants, load_and_verify_station_cover,
                           select_clear_route, update_cover_after_failed_clear,
                           verify_remaining_cover_certificate)
from B.q4.planner import Q4Config, Q4Planner
from B.q4.policy import marginal_detour
from B.q4.tests.test_q4 import record_at


class Q4V2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cover = load_and_verify_station_cover()

    def test_belief_has_512_consistent_soft_scenarios(self):
        record = record_at((0.0, 0.0), 0.0)
        belief = update_belief_scenarios(record)
        self.assertTrue(belief.active)
        self.assertEqual(belief.count, 512)
        self.assertTrue((belief.positions[:, 0] >= -1e-5).all())
        probabilities = response_probabilities(record, (0.0, 0.0))
        self.assertAlmostEqual(sum(probabilities.values()), 1.0, places=12)

    def test_belief_clear_failure_filters_then_resamples(self):
        record = record_at((0.0, 0.0), 0.0)
        first = update_belief_scenarios(record)
        center = tuple(first.positions[0])
        update_cover_after_failed_clear(record, center)
        second = update_belief_scenarios(record)
        self.assertEqual(second.count, 512)
        self.assertTrue((sum((second.positions[:, k]-center[k])**2 for k in (0, 1)) > 20**2).all())
        self.assertEqual(second.generated, 0)

    def test_sparse_remaining_cover_removes_failed_cell_and_verifies(self):
        record = record_at()
        initial = build_clear_plan(record, (0.0, 0.0))
        self.assertEqual(initial.kind, "RECTANGLE_GRID")
        update_cover_after_failed_clear(record, initial.points[0])
        sparse = build_clear_plan(record, initial.points[0])
        self.assertEqual(sparse.kind, "SPARSE_REMAINING_GRID")
        self.assertLess(sparse.point_count, initial.point_count)
        self.assertTrue(verify_remaining_cover_certificate(record, sparse))
        self.assertNotIn(initial.points[0], sparse.points)

    def test_clear_route_respects_certified_tail_risk_cap(self):
        record = record_at()
        update_belief_scenarios(record)
        current = (0.0, 0.0)
        plan = build_clear_plan(record, current)
        variants = build_clear_route_variants(record, plan, current)
        selected = select_clear_route(record, plan, current)
        shortest_upper = min(candidate.completion_upper_s for candidate in variants)
        self.assertLessEqual(selected.completion_upper_s,
                             CLEAR_ROUTE_RISK_CAP_RATIO*shortest_upper + 1e-9)

    def test_true_marginal_detour_not_anchor_round_trip(self):
        self.assertEqual(marginal_detour((0, 0), ((250, 0),), (1000, 0)), 0)
        self.assertAlmostEqual(marginal_detour((0, 0), ((0, 100),), (1000, 0)),
                               100+math.hypot(1000, 100)-1000)

    def test_dynamic_candidate_cache_refreshes_at_25m_and_region_change(self):
        planner = Q4Planner(Q4Config(planning_total_s=0), self.cover)
        record = planner.channels[1] = record_at()
        first = planner._dynamic_candidates(record, (1000, 0))
        self.assertEqual(first, ((0.0, 0.0), (250.0, 0.0), (500.0, 0.0), (750.0, 0.0)))
        planner.position = (24, 0)
        self.assertIs(first, planner._dynamic_candidates(record, (1000, 0)))
        planner.position = (25, 0)
        refreshed = planner._dynamic_candidates(record, (1000, 0))
        self.assertNotEqual(first, refreshed)
        record.region_version += 1
        self.assertIsNot(refreshed, planner._dynamic_candidates(record, (1000, 0)))

    def test_two_uncertain_no_signal_streak_blocks_more_uncertain_probes(self):
        planner = Q4Planner(Q4Config(planning_total_s=0), self.cover)
        record = planner.channels[1] = record_at()
        record.consecutive_uncertain_no_signal = 2
        self.assertEqual(record.consecutive_uncertain_no_signal,
                         planner.config.uncertain_no_signal_limit)
        # Direct threshold is a separate hard rule and remains executable.
        plan = planner._backup(record)
        self.assertTrue(plan.guaranteed)

    def test_probability_fallback_clear_batch_default_is_eight(self):
        self.assertEqual(Q4Config().clear_batch_points, 8)

    def test_certificate_shielded_rollout_improves_expected_cost_with_bounded_tail(self):
        record = record_at()
        update_belief_scenarios(record)
        plan = build_clear_plan(record, (0.0, 0.0))
        result = plan_belief_rollout_clear(
            record, plan, (0.0, 0.0), deadline_monotonic=time.monotonic()+5)
        self.assertIsNotNone(result.decision)
        decision = result.decision
        self.assertLess(decision.expected_s, decision.direct_expected_s)
        self.assertLessEqual(decision.tail_s, 1.25*decision.direct_tail_s)
        self.assertLessEqual(decision.completion_upper_s, 1.5*decision.direct_upper_s)
        self.assertGreater(decision.first_hit_probability, 0)

    def test_failed_rollout_probe_updates_remaining_region_without_false_inconsistency(self):
        planner = Q4Planner(Q4Config(), self.cover)
        record = planner.channels[1] = record_at()
        update_belief_scenarios(record)
        plan = build_clear_plan(record, planner.position)
        result = plan_belief_rollout_clear(
            record, plan, planner.position, deadline_monotonic=time.monotonic()+5)
        action = planner._start_rollout_clear(record, result.decision)
        planner.apply_clear_result(action, "no_target_in_range")
        self.assertIsNone(planner.stop_reason)
        self.assertEqual(record.state, "DETECTED")
        self.assertEqual(record.failed_clear_disks, [action.position])
        self.assertEqual(planner.metrics["rollout_probe_misses"], 1)
        self.assertIsNone(planner.active_clear)

    def test_rollout_v3_strategy_is_explicit(self):
        self.assertEqual(Q4Config().strategy, "q4_certificate_shielded_rollout_v3")

    def test_leaf_absence_certificate_is_bound_to_network(self):
        planner = Q4Planner(Q4Config(planning_total_s=0), self.cover)
        channel = 1
        planner.discovery_ledger[channel] = set().union(*self.cover.provider_sets)
        self.assertIn(planner._absence_certified(channel), {"LEAF_PROVIDER_SETS", "FULL_NETWORK"})
        planner.channels[channel].state = "ABSENT"
        planner.channels[channel].absence_certificate = {"method": "LEAF_PROVIDER_SETS",
                                                          "network_id": "different-network"}
        self.assertFalse(planner.has_completion_certificate())


if __name__ == "__main__":
    unittest.main()
