"""Tests for analysis/polish.py (board-plan polisher). Standard library only."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "analysis"))
import planner  # noqa: F401  (fleetkit discovery)
import polish
from fleetkit.baseline import greedy_plan
from fleetkit.generator import generate
from fleetkit.model import Scenario
from fleetkit.scorer import evaluate


class PolishTest(unittest.TestCase):
    def setUp(self):
        self.sc = Scenario.from_dict(generate("L2", 3))
        self.plan = greedy_plan(self.sc)
        self.assertTrue(evaluate(self.sc, self.plan).valid)

    def test_polish_keeps_plan_valid_and_never_worse(self):
        pol = polish.Polisher(self.sc, self.plan, log=lambda *_: None)
        before = pol.score
        after = pol.run(15.0)
        result = evaluate(self.sc, pol.plan())
        self.assertTrue(result.valid, [str(v) for v in result.violations])
        self.assertAlmostEqual(result.score, after, places=6)
        self.assertGreaterEqual(after, before - 1e-9)

    def test_retime_shifts_whole_strike_group(self):
        pol = polish.Polisher(self.sc, self.plan, log=lambda *_: None)
        routes = dict(pol.routes)
        scouts = pol.scouts(routes)
        if not scouts:
            self.skipTest("greedy plan has no scout")
        # Delay the scout by 5 s: strikes it confirms must move later together.
        sid = scouts[0]
        routes[sid] = [{**w, "t": w["t"] + 5.0} for w in routes[sid]]
        fixed = pol.retime(routes)
        if fixed is None:
            self.skipTest("windows too tight for this seed")
        groups = {}
        for aid, r in fixed.items():
            if r[-1].get("action") == "engage":
                groups.setdefault(r[-1]["target"], []).append(r[-1]["t"])
        for times in groups.values():
            self.assertLessEqual(max(times) - min(times), self.sc.rules.sync_window_s + 1e-6)


if __name__ == "__main__":
    unittest.main()
