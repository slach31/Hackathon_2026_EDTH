"""Run with python3 -m unittest discover -s tests -v (standard library only)."""
import copy
from dataclasses import replace
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
# Entry point also supplies the local fleetkit discovery used by these tests.
import planner
from candidate_matrix import CandidateMatrix
import search_planner
from search_planner import (DESTROY_OPERATORS, IncrementalValidator, Search, scout_tour,
                            unserved_reason)
from fleetkit.baseline import _Planner, greedy_plan
from fleetkit.generator import generate
from fleetkit.model import Scenario, save_json
from fleetkit.scorer import evaluate
import fleetkit


def scenario():
    """Serving A consumes both lights; skipping A allows the better B+C pair."""
    aircraft = []
    for kind, subtype, count in [('scout', None, 2), ('effector', 'light', 2)]:
        for i in range(count):
            aircraft.append(dict(id=f'{kind}{i}', type=kind, subtype=subtype,
                                 home=[0, len(aircraft)*60, 0], v_h_mps=30,
                                 v_up_mps=10, v_down_mps=15, capacity_wh=200,
                                 battery_pct=100, power=dict(p0_w=80, k_v=.1,
                                                            e_climb_wh_per_m=.01)))
    contacts = [dict(id=cid, **{'class': 'hostile'}, pos=pos, points=points,
                     engage_needs={'light': need}, window_s=[100, 800])
                for cid, pos, points, need in [('A', [1500, 0, 0], 90, 2),
                                               ('B', [1300, 500, 0], 70, 1),
                                               ('C', [1300, -500, 0], 70, 1)]]
    return Scenario.from_dict(dict(seed=1, level='test', horizon_s=900,
                                   aircraft=aircraft, contacts=contacts, zones=[]))


class IncrementalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sc = scenario()
        cls.plan = greedy_plan(cls.sc)
        assert evaluate(cls.sc, cls.plan).valid
        cls.routes = {r['aircraft']: r['waypoints'] for r in cls.plan['routes']}

    def checker(self, sc=None):
        search = Search(sc or self.sc, seconds=10)
        return search, search.checker

    def test_incremental_score_matches_official(self):
        search, checker = self.checker()
        state = checker.extend(search.root, self.routes)
        result = evaluate(self.sc, self.plan)
        self.assertIsNotNone(state)
        self.assertAlmostEqual(state.score, result.score)
        self.assertEqual(state.served, frozenset(result.served))

    def test_changed_routes_match_full_validator(self):
        search, checker = self.checker()
        parent = checker.extend(search.root, self.routes)
        rng = random.Random(42)
        for _ in range(120):
            aid = rng.choice(list(self.routes))
            route = copy.deepcopy(self.routes[aid])
            if rng.random() < .5:
                shift = rng.uniform(-60, 60)
                for wp in route:
                    wp['t'] += shift
            else:
                wp = rng.choice(route[1:])
                wp[rng.choice(['x', 'y', 'z'])] += rng.uniform(-150, 150)
            extra = {aid: route}
            full = parent.plan(self.sc.seed)
            for r in full['routes']:
                if r['aircraft'] == aid:
                    r['waypoints'] = route
            official = evaluate(self.sc, full)
            actual = checker.extend(parent, extra)
            self.assertEqual(actual is not None, official.valid)
            if actual is not None:
                self.assertAlmostEqual(actual.score, official.score)

    def test_cached_detour_matches_official(self):
        sc = Scenario.from_dict(generate('L3', 1))
        rng = random.Random(3)
        for _ in range(200):
            pts = [(rng.uniform(-500, 6000), rng.uniform(-500, 6000), rng.choice([40.0, 70.0, 130.0]))
                   for _ in range(2)]
            expected = search_planner._official_detour(list(pts), sc)
            self.assertEqual(search_planner._cached_detour(list(pts), sc), expected)
            self.assertEqual(search_planner._cached_detour(list(pts), sc), expected)

    def test_quick_reject_matches_full_validator(self):
        search, checker = self.checker()
        aid = next(iter(self.routes))
        for field, shift in (('t', -10000.0), ('t', 10000.0)):
            route = copy.deepcopy(self.routes[aid])
            for wp in route:
                wp[field] += shift
            rule = checker.quick_reject({aid: route})
            self.assertIsNotNone(rule)
            checker.last_rule = None
            self.assertIsNone(checker.extend(search.root, {aid: route}))
            self.assertEqual(checker.last_rule, rule)
        self.assertIsNone(checker.quick_reject(self.routes))

    def test_rebuilt_scout_must_keep_previous_confirmations(self):
        search, checker = self.checker()
        parent = checker.extend(search.root, self.routes)
        aid = next(a.id for a in self.sc.aircraft if a.type == 'scout' and a.id in self.routes)
        route = copy.deepcopy(self.routes[aid])
        for wp in route:
            wp.pop('action', None)
            wp.pop('target', None)
        self.assertIsNone(checker.extend(parent, {aid: route}))
        self.assertIn('R4', checker.rejections)

    def test_new_aircraft_conflicts_with_existing_route(self):
        sc = copy.deepcopy(self.sc)
        aid = next(a.id for a in sc.aircraft if a.type == 'scout' and a.id in self.routes)
        original = sc.aircraft_by_id()[aid]
        sc.aircraft.append(replace(original, id='duplicate-scout'))
        search, checker = self.checker(sc)
        parent = checker.extend(search.root, self.routes)
        self.assertIsNone(checker.extend(parent, {'duplicate-scout': self.routes[aid]}))
        self.assertIn('R8', checker.rejections)

    def test_signature_identifies_routes(self):
        search, checker = self.checker()
        a = checker.extend(search.root, self.routes)
        b = checker.extend(search.root, copy.deepcopy(self.routes))
        self.assertEqual(a.signature(), b.signature())
        aid = next(iter(self.routes))
        route = copy.deepcopy(self.routes[aid])
        route[-1]['t'] += 1.0
        c = checker.extend(a, {aid: route})
        if c is not None:
            self.assertNotEqual(a.signature(), c.signature())
        self.assertNotEqual(search.root.signature(), a.signature())

    def test_branches_do_not_mutate_parent(self):
        search, checker = self.checker()
        before = copy.deepcopy(search.root)
        child = checker.extend(search.root, self.routes)
        self.assertEqual(search.root, before)
        self.assertIsNotNone(child)
        self.assertEqual(search.root.routes, {})

    def test_matrix_respects_used_resources_and_exact_mix(self):
        matrix = CandidateMatrix(copy.deepcopy(self.sc))
        c = self.sc.contacts[0]
        group = matrix.groups(c, set())[0]
        self.assertEqual(len(group), 2)
        self.assertEqual(matrix.groups(c, {group[0].id}), [])
        # An optimistic cost estimate must not become a feasibility exclusion.
        for a in matrix.effectors:
            a.capacity_wh = .001
        self.assertTrue(matrix.groups(c, set()))


class SearchTests(unittest.TestCase):
    def test_skip_branch_finds_better_combination(self):
        sc = scenario()
        base = evaluate(sc, greedy_plan(sc))
        search = Search(sc, seconds=1.0, top_n=3, beam_width=6)
        verified = search.run()
        self.assertGreater(verified[0][1].score, base.score)
        self.assertEqual(set(verified[0][1].served), {'B', 'C'})
        self.assertEqual(len(verified), 3)
        self.assertEqual(len({s.signature() for s, _ in verified}), 3)
        scores = [r.score for _, r in verified]
        self.assertEqual(scores, sorted(scores, reverse=True))
        for state, _ in verified:
            self.assertTrue(evaluate(sc, state.plan(sc.seed)).valid)
        self.assertTrue(any(n['decision'].get('kind') == 'skip'
                            for n in search.trace if isinstance(n['decision'], dict)))
        for n in search.trace[1:]:
            self.assertLess(n['parent'], n['id'])

    def test_repair_adds_unserved_unknown(self):
        sc = scenario()
        plan = greedy_plan(sc)
        sc.contacts.append(replace(sc.contacts[1], id='U', cls='unknown', points=30,
                                   needs={}, pos=(900.0, 200.0, 0.0)))
        search = Search(sc, seconds=5)
        search.matrix = CandidateMatrix(sc)
        state = search.checker.extend(search.root, {r['aircraft']: r['waypoints'] for r in plan['routes']})
        self.assertNotIn('U', state.served)
        repaired = search.repair(state, 0)
        self.assertIn('U', repaired.served)
        self.assertGreater(repaired.score, state.score)
        result = evaluate(sc, repaired.plan(sc.seed))
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.score, repaired.score)

    def test_expired_budget_returns_valid_empty_plan(self):
        sc = scenario()
        search = Search(sc, seconds=0)
        verified = search.run()
        self.assertEqual(verified[0][1].score, 0)
        self.assertTrue(evaluate(sc, verified[0][0].plan(sc.seed)).valid)

    def test_no_serviceable_contacts(self):
        sc = scenario()
        sc.contacts = []
        result = Search(sc, seconds=.1).run()
        self.assertTrue(result[0][1].valid)
        self.assertEqual(result[0][1].score, 0)

    def test_standalone_handin_and_tree_artifacts(self):
        # Copy only hand-in modules, with fleetkit supplied by the runner.
        with tempfile.TemporaryDirectory() as tmp:
            work = Path(tmp)
            for name in ['planner.py', 'search_planner.py', 'candidate_matrix.py', 'tree_view.py']:
                shutil.copy2(ROOT / name, work / name)
            sc = generate('L1', 7)
            save_json(sc, work / 'scenario.json')
            env = dict(os.environ, PYTHONPATH=str(Path(fleetkit.__file__).resolve().parent.parent))
            start = time.monotonic()
            result = subprocess.run([sys.executable, 'planner.py', 'scenario.json', 'plan.json',
                                     '--seconds', '.5', '--top-n', '3', '--artifacts', 'artifacts'],
                                    cwd=work, env=env, timeout=5, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertLess(time.monotonic()-start, 5)
            parsed = Scenario.from_dict(sc)
            plan = json.loads((work / 'plan.json').read_text())
            self.assertTrue(evaluate(parsed, plan).valid)
            report = json.loads((work / 'artifacts/tree.json').read_text())
            self.assertTrue((work / 'artifacts/tree.html').exists())
            self.assertTrue(report['solutions'])
            for p in (work / 'artifacts').glob('plan-*.json'):
                self.assertTrue(evaluate(parsed, json.loads(p.read_text())).valid)



def diagnostic_scenario():
    """One contact per failure reason, next to the servable B and C."""
    sc = scenario()
    data = dict(seed=1, level='test', horizon_s=900, zones=[], aircraft=[
        dict(id=a.id, type=a.type, subtype=a.subtype, home=list(a.home), v_h_mps=a.v_h_mps,
             v_up_mps=a.v_up_mps, v_down_mps=a.v_down_mps, capacity_wh=a.capacity_wh,
             battery_pct=a.battery_pct, power=dict(p0_w=a.power.p0_w, k_v=a.power.k_v,
                                                  e_climb_wh_per_m=a.power.e_climb_wh_per_m))
        for a in sc.aircraft])
    # A spare light, so EARLY reaches the scout stage instead of running out of team.
    spare = dict(data['aircraft'][-1], id='effector2', home=[0, 240, 0])
    data['aircraft'].append(spare)
    hostile = lambda cid, pos, needs, window, points=60: dict(
        id=cid, **{'class': 'hostile'}, pos=pos, points=points, engage_needs=needs, window_s=window)
    data['contacts'] = [
        hostile('B', [1300, 500, 0], {'light': 1}, [100, 800], 70),
        hostile('C', [1300, -500, 0], {'light': 1}, [100, 800], 70),
        # A civilian inside the collateral radius: R7 forbids engaging it.
        hostile('BAN', [-1500, 0, 0], {'light': 1}, [100, 800]),
        dict(id='CIV', **{'class': 'civilian'}, pos=[-1450, 0, 0], window_s=None),
        # No heavy effector exists.
        hostile('HEAVY', [1300, 0, 0], {'heavy': 1}, [100, 800]),
        # Closes before any scout can get there.
        hostile('EARLY', [2500, 2500, 0], {'light': 1}, [0, 20]),
    ]
    return Scenario.from_dict(data)


class DiagnosticsTests(unittest.TestCase):
    def stats(self, outcomes, attempts=None, warm=0):
        return {'attempts': sum(outcomes.values()) if attempts is None else attempts,
                'warm_accepted': warm, 'outcomes': outcomes, 'rules': {}}

    def test_reason_classification(self):
        c = scenario().contacts[0]
        self.assertEqual(unserved_reason(c, {c.id}, None), 'collateral_ban')
        self.assertEqual(unserved_reason(replace(c, window=None), set(), None), 'no_window')
        self.assertEqual(unserved_reason(c, set(), None), 'never_tried')
        self.assertEqual(unserved_reason(c, set(), self.stats({}, 0)), 'never_tried')
        self.assertEqual(unserved_reason(c, set(), self.stats({'accepted': 1, 'time_cap': 9})),
                         'outranked')
        self.assertEqual(unserved_reason(c, set(), self.stats({}, 0, warm=1)), 'outranked')
        self.assertEqual(unserved_reason(c, set(), self.stats({'time_cap': 3, 'strike_rule:R8': 1})),
                         'time_cap')
        # Ties go to the attempt that got furthest.
        self.assertEqual(unserved_reason(c, set(), self.stats({'scout_route': 2, 'strike_rule:R8': 2})),
                         'strike_rule:R8')

    def test_every_unserved_contact_has_its_reason(self):
        sc = diagnostic_scenario()
        search = Search(sc, seconds=3.0)
        verified = search.run()
        result = verified[0][1]
        self.assertTrue(result.valid)
        report = search.report(verified)
        contacts = report['contacts']
        self.assertEqual(set(contacts), {'B', 'C', 'BAN', 'HEAVY', 'EARLY'})
        self.assertEqual({cid for cid, e in contacts.items() if e['served']}, set(result.served))
        self.assertEqual(set(result.served), {'B', 'C'})
        reasons = {cid: e['reason'] for cid, e in contacts.items() if not e['served']}
        self.assertEqual(reasons['BAN'], 'collateral_ban')
        self.assertEqual(reasons['HEAVY'], 'no_team')
        self.assertTrue(reasons['EARLY'].startswith('scout_'), reasons['EARLY'])
        self.assertEqual(sum(report['unserved'].values()), 3)
        self.assertEqual(sum(report['unserved_points'].values()), 180)

    def test_expired_budget_reports_never_tried(self):
        sc = diagnostic_scenario()
        search = Search(sc, seconds=0)
        report = search.report(search.run())
        reasons = {cid: e.get('reason') for cid, e in report['contacts'].items()}
        self.assertEqual(reasons['BAN'], 'collateral_ban')
        self.assertEqual({r for cid, r in reasons.items() if cid != 'BAN'}, {'never_tried'})

    def test_generated_scenario_accounting_is_complete(self):
        sc = Scenario.from_dict(generate('L2', 3))
        search = Search(sc, seconds=2.0)
        verified = search.run()
        report = search.report(verified)
        served = set(verified[0][1].served)
        targets = {c.id for c in sc.contacts if c.cls in ('hostile', 'unknown')}
        self.assertEqual(set(report['contacts']), targets)
        for cid, entry in report['contacts'].items():
            self.assertEqual(entry['served'], cid in served)
            self.assertEqual('reason' in entry, cid not in served)
            self.assertEqual(sum(entry['outcomes'].values()), entry['attempts'])
        self.assertEqual(sum(report['unserved'].values()), len(targets - served))
        self.assertEqual(set(report['unserved']), set(report['unserved_points']))


def tour_scenario():
    """U2's early window only fits before U1, so appending fails but insertion works."""
    aircraft = [dict(id='scout0', type='scout', subtype=None, home=[0, 0, 0], v_h_mps=30,
                     v_up_mps=10, v_down_mps=15, capacity_wh=400, battery_pct=100,
                     power=dict(p0_w=80, k_v=.1, e_climb_wh_per_m=.01))]
    contacts = [dict(id='U1', **{'class': 'unknown'}, pos=[3000, 0, 0], points=50,
                     window_s=[300, 900]),
                dict(id='U2', **{'class': 'unknown'}, pos=[800, 300, 0], points=30,
                     window_s=[40, 200])]
    return Scenario.from_dict(dict(seed=2, level='test', horizon_s=1000,
                                   aircraft=aircraft, contacts=contacts, zones=[]))


class LnsTests(unittest.TestCase):
    def test_tour_insertion_serves_what_appending_cannot(self):
        sc = tour_scenario()
        search = Search(sc, seconds=10)
        search.matrix = CandidateMatrix(sc)
        u1, u2 = sc.contact_by_id()['U1'], sc.contact_by_id()['U2']
        first, _ = next(search.children(search.root, u1, search.stop))
        self.assertEqual(first.served, {'U1'})
        # Appending U2 after U1 never builds a route...
        self.assertTrue(all(route is None for _, _, route in
                            search.scout_options(first, u2, insert=False)))
        # ...so append mode falls back to insertion, as insert mode does directly.
        for insert in (False, True):
            child, _ = next(search.children(first, u2, search.stop, insert=insert))
            self.assertEqual(child.served, {'U1', 'U2'})
            self.assertEqual([t['cid'] for t in child.tasks['scout0']], ['U2', 'U1'])
            result = evaluate(sc, child.plan(sc.seed))
            self.assertTrue(result.valid)
            self.assertAlmostEqual(result.score, child.score, places=6)

    def test_append_tour_matches_baseline(self):
        sc = Scenario.from_dict(generate('L2', 3))
        p = _Planner(sc)
        scout = next(a for a in sc.aircraft if a.type == 'scout')
        contact = next(c for c in sc.contacts if c.cls == 'unknown' and c.window)
        built = p._add_observation(scout, contact, 70.0, 0.0, 7.0)
        self.assertIsNotNone(built)
        tasks, route = built
        # The kit's speed and observation offset: eco mode, no stretch.
        saved = (search_planner.SCOUT_SPEED_MODE, search_planner.OBS_STRETCH,
                 search_planner.DIAGONAL_CLIMB)
        (search_planner.SCOUT_SPEED_MODE, search_planner.OBS_STRETCH,
         search_planner.DIAGONAL_CLIMB) = "eco", False, False
        try:
            self.assertEqual(scout_tour(sc, scout, tasks, 70.0, 7.0), route)
        finally:
            (search_planner.SCOUT_SPEED_MODE, search_planner.OBS_STRETCH,
             search_planner.DIAGONAL_CLIMB) = saved

    def test_removal_is_valid_and_frees_the_contact(self):
        sc = Scenario.from_dict(generate('L2', 3))
        search = Search(sc, seconds=3.0, lns_share=0.0)
        best = search.run()[0][0]
        self.assertEqual(search.lns['iterations'], 0)
        search.checker.stop = time.monotonic() + 30
        removed = 0
        for cid in sorted(best.served):
            smaller = search.remove(best, cid)
            if smaller is None:
                continue
            removed += 1
            self.assertNotIn(cid, smaller.served)
            self.assertLessEqual(smaller.served, best.served)
            result = evaluate(sc, smaller.plan(sc.seed))
            self.assertTrue(result.valid, [str(v) for v in result.violations][:3])
            self.assertAlmostEqual(result.score, smaller.score, places=6)
        self.assertGreater(removed, 0)

    def test_lns_phase_runs_and_exports_valid_plans(self):
        sc = Scenario.from_dict(generate('L2', 3))
        search = Search(sc, seconds=3.0, lns_share=0.5)
        verified = search.run()
        self.assertGreater(search.lns['iterations'], 0)
        self.assertIn('lns', search.report(verified))
        for state, result in verified:
            self.assertTrue(evaluate(sc, state.plan(sc.seed)).valid)
            self.assertAlmostEqual(result.score, state.score, places=6)
        with self.assertRaises(ValueError):
            Search(sc, seconds=1, lns_share=1.5)

    def test_destroy_operators_pick_served_contacts(self):
        sc = Scenario.from_dict(generate('L2', 3))
        search = Search(sc, seconds=3.0, lns_share=0.0)
        best = search.run()[0][0]
        by_id = sc.contact_by_id()
        served = sorted(best.served)
        priorities = {c: 1.0 for c in served}
        rng = random.Random(1)
        for op in DESTROY_OPERATORS:
            for _ in range(5):
                picked = search.destroy(op, served, by_id, best, priorities, rng)
                self.assertTrue(1 <= len(picked) <= 4, op)
                self.assertEqual(len(set(picked)), len(picked), op)
                self.assertLessEqual(set(picked), best.served, op)

    def test_variants_export_valid_plans(self):
        sc = Scenario.from_dict(generate('L2', 3))
        for flags in ({'interleave': True}, {'adaptive': True}, {'restarts': False}):
            with self.subTest(**flags):
                search = Search(sc, seconds=3.0, lns_share=0.5, **flags)
                bursts = []
                improve = search.improve
                search.improve = lambda c, p, stop=None: (bursts.append(stop), improve(c, p, stop))[1]
                verified = search.run()
                self.assertGreater(search.lns['iterations'], 0)
                for state, result in verified:
                    self.assertTrue(evaluate(sc, state.plan(sc.seed)).valid)
                    self.assertAlmostEqual(result.score, state.score, places=6)
                report = search.report(verified)['lns']
                for name, value in flags.items():
                    self.assertEqual(report['variant'][name], value)
                if 'interleave' in flags:
                    self.assertGreater(len(bursts), 1)
                    self.assertTrue(all(b is not None and b <= search.stop for b in bursts[:-1]))
                else:
                    self.assertEqual(bursts, [None])
                if 'adaptive' in flags:
                    self.assertNotEqual(set(report['weights'].values()), {1.0})



def plain_aircraft(sc):
    return [dict(id=a.id, type=a.type, subtype=a.subtype, home=list(a.home), v_h_mps=a.v_h_mps,
                 v_up_mps=a.v_up_mps, v_down_mps=a.v_down_mps, capacity_wh=a.capacity_wh,
                 battery_pct=a.battery_pct, power=dict(p0_w=a.power.p0_w, k_v=a.power.k_v,
                                                      e_climb_wh_per_m=a.power.e_climb_wh_per_m))
            for a in sc.aircraft]


class PadTimingTests(unittest.TestCase):
    def pads(self, gap):
        aircraft = plain_aircraft(scenario())
        aircraft[0]['home'], aircraft[1]['home'] = [0, 0, 0], [gap, 0, 0]
        return Scenario.from_dict(dict(seed=1, level='test', horizon_s=900, zones=[],
                                       aircraft=aircraft, contacts=[]))

    def test_takeoff_shift_clears_neighbouring_pad(self):
        from search_planner import takeoff_shift, takeoff_need
        sc = self.pads(11.0)
        a, b = sc.aircraft[0], sc.aircraft[1]
        need = takeoff_need(sc, a, b)
        self.assertGreater(need, 0)
        routes = {b.id: [{'t': 100.0}]}
        shift = takeoff_shift(sc, sc.aircraft_by_id(), routes, a, 99.0)
        self.assertGreaterEqual(abs(99.0 + shift - 100.0), need)
        self.assertEqual(takeoff_shift(sc, sc.aircraft_by_id(), routes, a, 300.0), 0.0)
        far = self.pads(100.0)
        self.assertIsNone(takeoff_need(far, far.aircraft[0], far.aircraft[1]))

    def test_pad_conflict_flags_close_takeoffs_only(self):
        from search_planner import pad_conflict
        sc = self.pads(11.0)
        by_id = sc.aircraft_by_id()
        a, b = sc.aircraft[0].id, sc.aircraft[1].id
        self.assertTrue(pad_conflict(sc, by_id, {b: [{'t': 50.0}]}, {a: [{'t': 51.0}]}))
        self.assertFalse(pad_conflict(sc, by_id, {b: [{'t': 50.0}]}, {a: [{'t': 80.0}]}))
        # A route being replaced is not compared with its own old version.
        self.assertFalse(pad_conflict(sc, by_id, {a: [{'t': 51.0}]}, {a: [{'t': 50.0}]}))

    def test_search_with_pad_takeoffs_exports_valid_plans(self):
        sc = Scenario.from_dict(generate('L3', 2))
        for state, result in Search(sc, seconds=4.0).run():
            self.assertTrue(evaluate(sc, state.plan(sc.seed)).valid)



class ClimbTests(unittest.TestCase):
    """DIAGONAL_CLIMB: climbs and descents ride on the first and last cruise legs."""

    def legs_ok(self, sc, routes):
        from fleetkit import validator
        from fleetkit.model import parse_plan
        parsed, errors = parse_plan({'seed': sc.seed, 'routes': [
            {'aircraft': aid, 'waypoints': r} for aid, r in routes.items()]})
        self.assertEqual(errors, [])
        return [v.rule for v in validator._r2(sc, parsed) + validator._r6(sc, parsed)]

    def test_scout_tour_climbs_on_the_way_and_is_cheaper(self):
        from search_planner import raw_energy_wh
        sc = Scenario.from_dict(generate('L2', 3))
        scout = next(a for a in sc.aircraft if a.type == 'scout')
        contact = next(c for c in sc.contacts if c.cls == 'unknown' and c.window)
        tasks, _ = _Planner(sc)._add_observation(scout, contact, 70.0, 0.0, 7.0)
        tours = {}
        for diagonal in (False, True):
            search_planner.DIAGONAL_CLIMB = diagonal
            try:
                tours[diagonal] = scout_tour(sc, scout, tasks, 70.0, depart=0.0)
            finally:
                search_planner.DIAGONAL_CLIMB = True
        flat, diag = tours[False], tours[True]
        self.assertEqual((flat[1]['x'], flat[1]['y']), tuple(scout.home[:2]))
        self.assertNotEqual((diag[1]['x'], diag[1]['y']), tuple(scout.home[:2]))
        self.assertEqual((diag[-1]['x'], diag[-1]['y'], diag[-1]['z']), tuple(scout.home))
        self.assertEqual(self.legs_ok(sc, {scout.id: diag}), [])
        seen = lambda r: next(w['t'] for w in r if w.get('action') == 'observe')
        self.assertLess(seen(diag), seen(flat))
        self.assertLess(raw_energy_wh(diag, scout), raw_energy_wh(flat, scout))

    def test_effector_route_is_earlier_and_falls_back_over_a_zone(self):
        # The kit's builder; search_planner patches fleetkit.baseline to its own.
        from search_planner import _effector_route, effector_route
        data = generate('L2', 3)
        sc = Scenario.from_dict(data)
        eff = next(a for a in sc.aircraft if a.type == 'effector')
        target = next(c for c in sc.contacts if c.cls == 'hostile')
        kit, kit_earliest = _effector_route(eff, target, 0.0, 130.0, 0.0, sc)
        route, earliest = effector_route(eff, target, kit_earliest, 130.0, 0.0, sc)
        self.assertLess(earliest, kit_earliest)
        self.assertNotEqual((route[1]['x'], route[1]['y']), tuple(eff.home[:2]))
        self.assertEqual(self.legs_ok(sc, {eff.id: route}), [])
        # An 80 m zone right after the pad: flown over at 130 m, but the climbing piece
        # would cross it, so the route climbs vertically first.
        hx, hy = eff.home[:2]
        dx, dy = target.pos[0] - hx, target.pos[1] - hy
        norm = (dx * dx + dy * dy) ** 0.5
        data['zones'] = [dict(id='ZX', center=[hx + 60 * dx / norm, hy + 60 * dy / norm],
                              radius_m=20.0, top_m=80.0)]
        zoned = Scenario.from_dict(data)
        route, _ = effector_route(eff, target, kit_earliest + 30, 130.0, 0.0, zoned)
        self.assertEqual((route[1]['x'], route[1]['y']), tuple(eff.home[:2]))
        self.assertEqual(self.legs_ok(zoned, {eff.id: route}), [])

    def test_level_takeoffs_need_a_shorter_gap(self):
        from search_planner import takeoff_need
        sc = PadTimingTests().pads(11.0)
        a, b = sc.aircraft[0], sc.aircraft[1]
        self.assertLess(takeoff_need(sc, a, b, True), takeoff_need(sc, a, b))

    def test_search_with_diagonal_climbs_exports_valid_plans(self):
        sc = Scenario.from_dict(generate('L4', 1))
        for state, result in Search(sc, seconds=6.0).run():
            self.assertTrue(evaluate(sc, state.plan(sc.seed)).valid)



class GroundStrikeTests(unittest.TestCase):
    def plan_with(self, sc, effector_wps, contact, scout):
        tasks = [{"cid": contact.id, "point": search_planner.obs_point(sc, scout, contact, 0.0, 30.0),
                  "window": contact.window, "cpos": contact.pos}]
        tour = scout_tour(sc, scout, tasks, 30.0, depart=0.0)
        obs_end = next(w['t'] + w['dwell_s'] for w in tour if w.get('action') == 'observe')
        return tour, obs_end

    def test_ground_route_is_valid_and_cheaper_than_30_m(self):
        from fleetkit.energy import route_energy_wh
        from fleetkit.model import parse_plan
        sc = scenario()
        scout = next(a for a in sc.aircraft if a.type == 'scout')
        eff = next(a for a in sc.aircraft if a.subtype == 'light')
        target = next(c for c in sc.contacts if c.id == 'B')
        tour, obs_end = self.plan_with(sc, None, target, scout)
        # After the scout has landed (pads are 60 m apart, in line with the contact).
        arrive = max(obs_end + 15.0, tour[-1]['t'] + 60.0)
        ground, earliest = search_planner.effector_route(eff, target, arrive,
                                                         search_planner.GROUND_Z, 0.0, sc)
        self.assertEqual(len(ground), 2)
        self.assertEqual([w['z'] for w in ground], [0.0, 0.0])
        self.assertEqual(ground[-1]['action'], 'engage')
        low, _ = search_planner.effector_route(eff, target, arrive, 30.0, 0.0, sc)
        results = {}
        for name, route in (('ground', ground), ('low', low)):
            plan = {'seed': sc.seed, 'routes': [{'aircraft': scout.id, 'waypoints': tour},
                                                {'aircraft': eff.id, 'waypoints': route}]}
            result = evaluate(sc, plan)
            self.assertTrue(result.valid, result.violations)
            results[name] = result
        self.assertGreater(results['ground'].score, results['low'].score)
        energy = lambda r: route_energy_wh(parse_plan({'seed': sc.seed, 'routes': [
            {'aircraft': eff.id, 'waypoints': r}]})[0][eff.id].waypoints, eff)
        self.assertLess(energy(ground), energy(low) - eff.power.e_climb_wh_per_m * 29.0)
        # Too early for the straight line at cruise speed.
        self.assertIsNone(search_planner.effector_route(eff, target, earliest - 1.0,
                                                        search_planner.GROUND_Z, 0.0, sc)[0])
        # Ground routes are straight: other approach angles add nothing.
        self.assertIsNone(search_planner.effector_route(eff, target, arrive,
                                                        search_planner.GROUND_Z, 35.0, sc)[0])

    def test_blocked_ground_line_falls_back_to_the_30_m_route(self):
        data = json.loads(json.dumps({
            'seed': 1, 'level': 'test', 'horizon_s': 900, 'aircraft': [], 'contacts': [],
            'zones': [dict(id='Z', center=[700, 325], radius_m=60.0, top_m=40.0)]}))
        base = scenario()
        sc = replace(base, zones=Scenario.from_dict(data).zones)
        eff = next(a for a in sc.aircraft if a.subtype == 'light')
        target = next(c for c in sc.contacts if c.id == 'B')
        self.assertIsNone(search_planner.ground_route(eff, target, 500.0, sc))
        route, _ = search_planner.effector_route(eff, target, 500.0, search_planner.GROUND_Z,
                                                 0.0, sc)
        self.assertGreater(len(route), 2)
        self.assertTrue(all(w['z'] >= 30.0 for w in route[1:-1]))
        self.assertEqual(ClimbTests().legs_ok(sc, {eff.id: route}), [])

    def test_layer_sets_follow_the_flag(self):
        self.assertEqual(search_planner.effector_layer_sets()[0], (search_planner.GROUND_Z,))
        search_planner.GROUND_STRIKES = False
        try:
            self.assertEqual(search_planner.effector_layer_sets(), search_planner.EFFECTOR_Z)
        finally:
            search_planner.GROUND_STRIKES = True

    def test_search_exports_valid_plans_with_ground_strikes(self):
        sc = scenario()
        verified = Search(sc, seconds=3.0).run()
        for state, result in verified:
            self.assertTrue(evaluate(sc, state.plan(sc.seed)).valid)
        best = verified[0][0]
        effectors = [r for aid, r in best.routes.items()
                     if next(a for a in sc.aircraft if a.id == aid).type == 'effector']
        self.assertTrue(any(len(r) == 2 and r[-1]['z'] == 0.0 for r in effectors))


class ScoutRankTests(unittest.TestCase):
    def test_cheaper_scout_ranks_first_with_energy_ranking(self):
        sc = scenario()
        cheap = sc.aircraft[1]
        sc.aircraft[1] = replace(cheap, power=replace(cheap.power, p0_w=50.0))
        search = Search(sc, seconds=5)
        contact = next(c for c in sc.contacts if c.id == 'A')
        ranked = [a.id for a in search.ranked_scouts(search.root, contact, False)]
        self.assertEqual(ranked[0], 'scout1')
        search_planner.SCOUT_ENERGY_RANK = False
        try:
            ranked = [a.id for a in search.ranked_scouts(search.root, contact, False)]
        finally:
            search_planner.SCOUT_ENERGY_RANK = True
        # By added distance only, the nearer pad wins.
        self.assertEqual(ranked[0], 'scout0')


class CacheTests(unittest.TestCase):
    def test_memoised_validation_matches_fresh_validation(self):
        sc = Scenario.from_dict(generate('L3', 2))
        search = Search(sc, seconds=4.0)
        best = search.run()[0][0]
        self.assertGreater(search.checker.cache_hits, 0)
        # A fresh validator (empty memo) given the whole best plan at once gives the same
        # state as the memoised search, and the official scorer agrees.
        fresh = IncrementalValidator(sc, time.monotonic() + 30)
        state = fresh.extend(search.root, dict(best.routes))
        self.assertIsNotNone(state)
        self.assertEqual(fresh.cache_hits, 0)
        self.assertEqual(state.served, best.served)
        self.assertAlmostEqual(state.score, best.score, places=6)
        self.assertAlmostEqual(evaluate(sc, best.plan(sc.seed)).score, best.score, places=6)
        search.checker.stop = time.monotonic() + 30
        again = search.checker.extend(search.root, dict(best.routes))
        self.assertAlmostEqual(again.score, best.score, places=6)

    def test_every_memoised_verdict_matches_the_official_validator(self):
        """Summaries (R4/R10/scoring), pad neighbours, effector shapes and the R8 box
        filter: every accepted or rejected extend during a search agrees with validate()."""
        from fleetkit.validator import validate
        sc = Scenario.from_dict(generate('L4', 2))
        search = Search(sc, seconds=5.0)
        inner = search.checker._extend
        seen = {'checked': 0, 'rejected': 0}

        def checked(parent, extra):
            result = inner(parent, extra)
            plan = {'seed': sc.seed, 'routes': [{'aircraft': a, 'waypoints': w}
                                                for a, w in {**parent.routes, **extra}.items()]}
            violations = validate(sc, plan)
            self.assertEqual(result is None, bool(violations), [v.rule for v in violations])
            if result is not None:
                official = evaluate(sc, plan)
                self.assertAlmostEqual(result.score, official.score, places=6)
                self.assertEqual(set(result.served), set(official.served))
            seen['checked'] += 1
            seen['rejected'] += result is None
            return result

        search.checker._extend = checked
        search.run()
        self.assertGreater(seen['checked'], 100)
        self.assertGreater(seen['rejected'], 10)

if __name__ == '__main__':
    unittest.main()


class TripTests(unittest.TestCase):
    """Scout trip label search (LNS operator "trip")."""
    @classmethod
    def setUpClass(cls):
        cls.sc = Scenario.from_dict(generate('L4', 9))
        cls.search = Search(cls.sc, seconds=6.0, lns_share=0.0)
        cls.best = cls.search.run()[0][0]
        cls.search.checker.stop = time.monotonic() + 60.0

    def scouts(self):
        return [a for a in self.sc.aircraft if a.type == 'scout']

    def test_trip_route_is_valid_within_battery_and_horizon(self):
        s, checked = self.search, 0
        for scout in self.scouts():
            partial, _ = s.strip_scout(self.best, scout.id)
            self.assertIsNotNone(partial)
            others = {t['cid'] for sid, ts in self.best.tasks.items() if sid != scout.id for t in ts}
            z = search_planner.SCOUT_LAYERS[0]
            start = time.monotonic()
            trips = s.scout_trips(partial, scout, partial.used, others | set(partial.served), z)
            self.assertLess(time.monotonic() - start, 0.5)
            if not trips or not trips[0][1]:
                continue
            value, order, wh, finishes = trips[0]
            self.assertEqual(list(finishes), sorted(finishes))
            tasks = [{'cid': c, 'point': search_planner.obs_point(self.sc, scout, s.contacts[c], 0.0, z),
                      'window': s.contacts[c].window, 'cpos': s.contacts[c].pos} for c in order]
            route = scout_tour(self.sc, scout, tasks, z)
            self.assertIsNotNone(route, order)
            self.assertLessEqual(route[-1]['t'], self.sc.horizon_s + 1e-6)
            self.assertLessEqual(search_planner.raw_energy_wh(route, scout), scout.usable_wh + 1e-6)
            alone = IncrementalValidator(self.sc, time.monotonic() + 60).extend(s.root, {scout.id: route})
            self.assertIsNotNone(alone, order)
            self.assertLessEqual(set(order) & {c.id for c in self.sc.contacts if c.cls == 'unknown'},
                                 set(alone.served))
            checked += 1
        self.assertGreater(checked, 0)

    def test_best_trip_is_no_worse_than_the_current_tour(self):
        s = self.search
        lam = self.sc.rules.lambda_e
        for scout in self.scouts():
            current = tuple(t['cid'] for t in self.best.tasks[scout.id])
            partial, _ = s.strip_scout(self.best, scout.id)
            others = {t['cid'] for sid, ts in self.best.tasks.items() if sid != scout.id for t in ts}
            route = self.best.routes.get(scout.id)
            z = max(w['z'] for w in route) if route else search_planner.SCOUT_LAYERS[0]
            trips = s.scout_trips(partial, scout, partial.used, others | set(partial.served), z)
            mine = [tr for tr in trips if set(tr[1]) == set(current)]
            if current and mine:
                self.assertGreaterEqual(trips[0][0], mine[0][0] - 1e-9)
            # Value is net value minus scout energy: never above the sum of net values.
            for value, order, wh, _ in trips:
                self.assertLess(value, sum(s.matrix.priority(s.contacts[c]) if s.contacts[c].cls == 'unknown'
                                           else s.contacts[c].points for c in order) + 1e-9)
                self.assertGreater(wh, 0)

    def test_trip_move_keeps_plans_valid(self):
        s = self.search
        rng = random.Random(3)
        moved = 0
        for _ in range(6):
            got = s.trip_move(self.best, rng)
            if got is None:
                continue
            partial, removed = got
            moved += 1
            result = evaluate(self.sc, partial.plan(self.sc.seed))
            self.assertTrue(result.valid, [str(v) for v in result.violations][:3])
            self.assertAlmostEqual(result.score, partial.score, places=6)
        self.assertGreater(moved, 0)

    def test_lns_with_trip_operator_exports_valid_plans(self):
        sc = Scenario.from_dict(generate('L3', 4))
        old = search_planner.TRIP_SHARE
        search_planner.TRIP_SHARE = 0.6
        try:
            search = Search(sc, seconds=5.0, lns_share=0.6)
            verified = search.run()
        finally:
            search_planner.TRIP_SHARE = old
        self.assertGreater(search.lns['trip_tried'], 0)
        self.assertIn('trip_improvements', search.report(verified)['lns'])
        for state, result in verified:
            self.assertTrue(evaluate(sc, state.plan(sc.seed)).valid)
            self.assertAlmostEqual(result.score, state.score, places=6)


class TripPlanTests(unittest.TestCase):
    """All-scouts trip selection (set packing) used as a warm start."""
    def test_pack_trips_matches_brute_force(self):
        rng = random.Random(5)
        for _ in range(30):
            trips = [(rng.uniform(1, 20), f'S{rng.randrange(3)}',
                      frozenset(rng.sample('ABCDEFG', rng.randint(1, 3))), None) for _ in range(9)]
            value, chosen = search_planner.pack_trips(trips)
            self.assertEqual(len({t[1] for t in chosen}), len(chosen))
            seen = set()
            for t in chosen:
                self.assertFalse(seen & t[2])
                seen |= t[2]
            best = 0.0
            for r in range(len(trips) + 1):
                for combo in __import__('itertools').combinations(trips, r):
                    scouts = [t[1] for t in combo]
                    cids = [c for t in combo for c in t[2]]
                    if len(set(scouts)) == len(scouts) and len(set(cids)) == len(cids):
                        best = max(best, sum(t[0] for t in combo))
            self.assertAlmostEqual(value, sum(t[0] for t in chosen))
            # Greedy plus swaps is a heuristic: allow a small gap, never above optimal.
            self.assertLessEqual(value, best + 1e-9)
            self.assertGreaterEqual(value, 0.8 * best)

    def test_cover_adds_a_trip_for_each_reachable_contact(self):
        sc = Scenario.from_dict(generate('L4', 9))
        search = Search(sc, seconds=10)
        search.matrix = CandidateMatrix(sc)
        scout = next(a for a in sc.aircraft if a.type == 'scout')
        z = search_planner.SCOUT_LAYERS[0]
        cands = search.trip_candidates(scout, frozenset(), set(), z)
        plain = search_planner.trip_labels(sc, scout, cands, z, top=5)
        covered = search_planner.trip_labels(sc, scout, cands, z, top=5, cover=True)
        self.assertEqual(covered[:5], plain)
        reached = {c for tr in covered for c in tr[1]}
        self.assertGreater(len(reached), len({c for tr in plain for c in tr[1]}))
        for c in reached:
            self.assertTrue(any(c in tr[1] for tr in covered))

    def test_trip_plan_warm_start_records_valid_plans(self):
        sc = Scenario.from_dict(generate('L3', 4))
        search = Search(sc, seconds=8.0, lns_share=0.3)
        verified = search.run()
        self.assertIn('realised', search.trip_stats)
        self.assertTrue(any(isinstance(n['decision'], dict) and n['decision'].get('kind') == 'trip_plan'
                            for n in search.trace))
        self.assertIn('trip_plan', search.report(verified))
        for state, result in verified:
            self.assertTrue(evaluate(sc, state.plan(sc.seed)).valid)
            self.assertAlmostEqual(result.score, state.score, places=6)

    def test_trip_plan_respects_its_stop(self):
        sc = Scenario.from_dict(generate('L4', 9))
        search = Search(sc, seconds=20)
        search.matrix = CandidateMatrix(sc)
        start = time.monotonic()
        try:
            state = search.trip_plan(start + 0.3)
        except search_planner.DeadlineReached:
            state = None
        self.assertLess(time.monotonic() - start, 1.0)
        if state is not None:
            self.assertTrue(evaluate(sc, state.plan(sc.seed)).valid)

    def test_trip_tree_completes_with_valid_scored_nodes(self):
        sc = Scenario.from_dict(generate('L4', 9))
        search = Search(sc, seconds=20)
        search.matrix = CandidateMatrix(sc)
        search.repair = lambda state, depth: state
        start = time.monotonic()
        state = search.trip_plan(start + 6.0)
        self.assertLess(time.monotonic() - start, 7.0)
        tree = search.trip_stats['tree']
        # Anytime: every scout depth is reached within the stop.
        self.assertEqual(tree['depth'], tree['scouts'])
        self.assertGreater(tree['nodes'], 0)
        self.assertGreaterEqual(state.score, search.trip_stats['realised_score'] - 1e-9)
        nodes = [s for s in search.archive]
        self.assertTrue(any(n['decision'].get('kind') == 'trip_tree' for n in search.trace
                            if isinstance(n['decision'], dict)))
        for s in nodes + [state]:
            result = evaluate(sc, s.plan(sc.seed))
            self.assertTrue(result.valid)
            self.assertAlmostEqual(result.score, s.score, places=6)


class GroupTests(unittest.TestCase):
    """Joint re-plan of n scouts (LNS operator "group")."""
    @classmethod
    def setUpClass(cls):
        cls.sc = Scenario.from_dict(generate('L4', 9))
        cls.search = Search(cls.sc, seconds=6.0, lns_share=0.0)
        cls.best = cls.search.run()[0][0]
        cls.search.checker.stop = time.monotonic() + 60.0

    def test_best_disjoint_matches_brute_force(self):
        import itertools
        rng = random.Random(5)
        for case in range(300):
            n = rng.choice((2, 3))
            lists = []
            for _ in range(n):
                trips = {(round(rng.uniform(-2, 20), 3), rng.randrange(1, 64)) for _ in range(rng.randint(1, 7))}
                lists.append(sorted(trips | {(0.0, 0)}, key=lambda t: -t[0]))
            want = max(sum(t[0] for t in combo) for combo in itertools.product(*lists)
                       if all(a[1] & b[1] == 0 for a, b in itertools.combinations(combo, 2)))
            got = search_planner.best_disjoint(lists)
            self.assertIsNotNone(got, case)
            value, picks = got
            self.assertAlmostEqual(value, want, places=6)
            chosen = [lists[i][k] for i, k in enumerate(picks)]
            self.assertAlmostEqual(sum(t[0] for t in chosen), value, places=6)
            for a, b in itertools.combinations(chosen, 2):
                self.assertEqual(a[1] & b[1], 0)
            # A floor at the optimum means nothing beats it.
            self.assertIsNone(search_planner.best_disjoint(lists, want))

    def test_group_move_keeps_plans_valid(self):
        s = self.search
        old = search_planner.GROUP_SIZES
        search_planner.GROUP_SIZES = (2, 3, 4)
        try:
            rng = random.Random(4)
            for _ in range(8):
                got = s.group_move(self.best, rng)
                if got is None:
                    continue
                partial, removed = got
                result = evaluate(self.sc, partial.plan(self.sc.seed))
                self.assertTrue(result.valid, [str(v) for v in result.violations][:3])
                self.assertAlmostEqual(result.score, partial.score, places=6)
        finally:
            search_planner.GROUP_SIZES = old
        self.assertGreater(s.lns['group_realised'], 0)

    def test_lns_with_group_operator_keeps_or_improves_valid_plans(self):
        sc = Scenario.from_dict(generate('L4', 10))
        old = search_planner.GROUP_SIZES, search_planner.GROUP_SHARE, search_planner.TRIP_SHARE
        search_planner.GROUP_SIZES, search_planner.GROUP_SHARE, search_planner.TRIP_SHARE = (2, 3), 0.8, 0.0
        try:
            search = Search(sc, seconds=10.0, lns_share=0.5)
            contacts, priorities = search.search_phase()
            before = search.archive[0].score
            search.improve_phase(contacts, priorities)
            verified = search.finalize()
        finally:
            search_planner.GROUP_SIZES, search_planner.GROUP_SHARE, search_planner.TRIP_SHARE = old
        self.assertGreater(search.lns['group_tried'], 0)
        self.assertGreaterEqual(verified[0][1].score, before - 1e-6)
        self.assertIn('group_improvements', search.report(verified)['lns'])
        for state, result in verified:
            self.assertTrue(evaluate(sc, state.plan(sc.seed)).valid)
            self.assertAlmostEqual(result.score, state.score, places=6)


class StrikeGroupTests(unittest.TestCase):
    """Strike-side LNS uses route conflicts and retains scout confirmations."""
    def test_conflict_finder_uses_actual_r8_candidate_route(self):
        sc = scenario()
        # Put the two effector pads inside R8 separation distance.  Two identical
        # strike trajectories then genuinely conflict; this is not a target-distance
        # approximation.
        sc = replace(sc, aircraft=[replace(a, home=(0, 0, 0)) if a.type == 'effector' else a
                                   for a in sc.aircraft])
        s = Search(sc, seconds=10)
        contact = sc.contacts[0]
        eff = [a for a in sc.aircraft if a.type == 'effector']
        first, _ = search_planner.effector_route(eff[0], contact, 180.0, 40.0, 0.0, sc)
        second, _ = search_planner.effector_route(eff[1], contact, 180.0, 40.0, 0.0, sc)
        info = s.checker.route_info(eff[0].id, first)
        parent = search_planner.State({eff[0].id: first}, {a.id: [] for a in sc.aircraft if a.type == 'scout'},
                                      frozenset(), {eff[0].id: info[1]}, {eff[0].id: info[3]},
                                      {eff[0].id: info[4]}, frozenset(), 0.0,
                                      infos={eff[0].id: info})
        self.assertIn(eff[0].id, s.strike_conflicts(parent, {eff[1].id: second}))

    def test_drop_and_restrike_retains_observation_and_is_valid(self):
        sc = Scenario.from_dict(generate('L4', 9))
        s = Search(sc, seconds=6.0, lns_share=0.0)
        best = s.run()[0][0]
        s.checker.stop = time.monotonic() + 60.0
        cid = next(c for c in best.served if s.contacts[c].cls == 'hostile')
        partial = s.drop_strike(best, cid)
        self.assertIsNotNone(partial)
        self.assertNotIn(cid, partial.served)
        self.assertTrue(any(t['cid'] == cid for ts in partial.tasks.values() for t in ts))
        scout, route, end = s.observed_strike(partial, cid)
        children = list(s.strikes(partial, s.contacts[cid], s.matrix.groups(s.contacts[cid], partial.used),
                                  scout, route, end, time.monotonic() + 10.0))
        self.assertTrue(children)
        child = max((state for state, _ in children), key=lambda state: state.score)
        result = evaluate(sc, child.plan(sc.seed))
        self.assertTrue(result.valid, [str(v) for v in result.violations][:3])
        self.assertIn(cid, child.served)

    def test_strike_operator_can_be_disabled(self):
        sc = Scenario.from_dict(generate('L3', 4))
        old = search_planner.STRIKE_GROUP, search_planner.TRIP_SHARE, search_planner.GROUP_SHARE
        search_planner.STRIKE_GROUP, search_planner.TRIP_SHARE, search_planner.GROUP_SHARE = False, 0.0, 0.0
        try:
            s = Search(sc, seconds=4.0, lns_share=0.5)
            verified = s.run()
        finally:
            search_planner.STRIKE_GROUP, search_planner.TRIP_SHARE, search_planner.GROUP_SHARE = old
        self.assertEqual(s.lns['strike_tried'], 0)
        self.assertTrue(evaluate(sc, verified[0][0].plan(sc.seed)).valid)
