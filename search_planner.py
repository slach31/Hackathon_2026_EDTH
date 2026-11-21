"""Deadline-bounded beam search using official baseline routes and rule checks.

Nodes are valid, complete-to-home partial fleet plans. No route is edited in place.
The matrix ranks candidates; only official rule checks establish validity.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
import itertools
import math
import random
import time

from fleetkit import baseline
from fleetkit.baseline import (_Planner, _effector_route, _scout_route, EFFECTOR_LAYERS,
                               SCOUT_LAYERS, SPEED_FACTOR)
from fleetkit.energy import best_range_speed, route_energy_wh
from fleetkit.geometry import EPS, hdist
from fleetkit.model import HOSTILE, SCOUT, UNKNOWN, parse_plan
from fleetkit.scorer import evaluate
from fleetkit import validator as rules
from candidate_matrix import CandidateMatrix

# The kit's scout layers (70, 110 m) plus a low one. At 30 m the 3D observation range allows
# observing from ~139 m instead of ~123 m, and the climb is shorter, so early windows are
# reached sooner. Official check_handin, L1-L4 seeds 35-39: 9913.7 -> 10089.6 (+1.8%),
# L4 +3.0%; with seeds 30-34 and the two validation L4s, L4 won 10 of 12 cases.
SCOUT_LAYERS = tuple(SCOUT_LAYERS) + (30.0,)


class DeadlineReached(Exception):
    pass


DESTROY_OPERATORS = ("random", "related", "worst", "tour")


# Scout candidate generation.
SCOUT_FANOUT = 4        # scouts tried first for a contact
SCOUT_FALLBACK = 8      # scouts tried when none of the first SCOUT_FANOUT gives a valid child
FULL_INSERT_TASKS = 4   # tours up to this long try every insertion position, not only 3
REORDER_TASKS = 6       # tours up to this long also try deadline- and start-ordered tours
# Rebuilt tours keep earlier visits within a few seconds of the parent's (other altitude or
# departure shift); the reachability bound subtracts this much from the parent's times.
REACH_SLACK_S = 20.0

_tours = {}

# Scout cruise speed. The kit flies scouts at min(0.95 v_h, best-range speed). Energy per
# metre is flat near the best-range speed (on L4, 0.98 v_h costs +0-1.5% per metre) while
# airtime, hence hover-free p0 energy, R1 (home by the horizon) and window reach, improve
# by 5-35%. For scouts whose best-range speed exceeds 0.95 v_h the kit speed is both slower
# and dearer. R2 allows v_h exactly; 0.98 keeps a 2% margin.
SCOUT_CRUISE = 0.98
# "fast": every scout leg at SCOUT_CRUISE * v_h. "eco": the kit's speed. "hybrid": the kit's
# speed, falling back to fast when the eco tour misses a window, the horizon or the battery.
# Set per run by Search(scout_speed=...); planner.py --scout-speed.
SCOUT_SPEEDS = ("fast", "eco", "hybrid")
SCOUT_SPEED_MODE = "fast"
# Contacts are on the ground; the 3D observation range allows a horizontal offset of
# sqrt(R_obs^2 - z^2): ~133 m at 70 m, ~102 m at 110 m. The kit always uses 100 m.
OBS_MARGIN_M = 8.0
# Set per run by Search(obs_stretch=...); planner.py --obs-stretch / --no-obs-stretch.
OBS_STRETCH = True


def eco_speed(scout):
    return min(SPEED_FACTOR * scout.v_h_mps, best_range_speed(scout.power))


def fast_speed(scout):
    return max(eco_speed(scout), SCOUT_CRUISE * scout.v_h_mps)


def obs_offset(scenario, contact, z):
    """Horizontal observation offset toward home for a scout at altitude z."""
    if not OBS_STRETCH:
        return baseline.OBS_OFFSET_M
    reach = scenario.rules.R_obs_m - OBS_MARGIN_M
    dz = z - contact.pos[2]
    if reach <= abs(dz):
        return baseline.OBS_OFFSET_M
    return max(baseline.OBS_OFFSET_M, math.sqrt(reach * reach - dz * dz))


def obs_point(scenario, scout, contact, angle, z):
    ux, uy = baseline._toward_home(contact, scout.home, angle)
    off = obs_offset(scenario, contact, z)
    return contact.pos[0] + ux * off, contact.pos[1] + uy * off


def build_scout_route(scout, tasks, depart, z, scenario, speed):
    """The kit's _scout_route with an explicit cruise speed (same waypoints at the kit speed)."""
    rules_ = scenario.rules
    detour, length, wp = baseline._detour, baseline._length, baseline._wp
    home = scout.home
    above = (home[0], home[1], z)
    t = depart
    wps = [wp(t, home)]
    t += z / (SPEED_FACTOR * scout.v_up_mps) + 0.5
    wps.append(wp(t, above))
    cur = above
    for task in tasks:
        px, py = task["point"]
        cpos = task.get("cpos")
        if cpos is not None:
            # A point stretched for 70 m may be out of range when the tour flies at 110 m.
            reach = rules_.R_obs_m - OBS_MARGIN_M
            h = math.sqrt(max(0.0, reach * reach - (z - cpos[2]) ** 2))
            d = math.hypot(px - cpos[0], py - cpos[1])
            if d > h:
                px, py = cpos[0] + (px - cpos[0]) * h / d, cpos[1] + (py - cpos[1]) * h / d
        target = (px, py, z)
        path = detour([cur, target], scenario)
        if path is None:
            return None
        total = length(path)
        lo, hi = task["window"]
        arrive = max(t + total / speed, lo)
        if arrive + rules_.obs_dwell_s > hi:
            return None
        for k in range(1, len(path) - 1):
            wps.append(wp(t + (arrive - t) * length(path[:k + 1]) / total, path[k]))
        wps.append(wp(arrive, target, action="observe", target_id=task["cid"],
                      dwell_s=rules_.obs_dwell_s))
        t = arrive + rules_.obs_dwell_s
        cur = target
    path = detour([cur, above], scenario)
    if path is None:
        return None
    total = length(path)
    arrive = t + total / speed
    for k in range(1, len(path) - 1):
        wps.append(wp(t + (arrive - t) * length(path[:k + 1]) / total, path[k]))
    wps.append(wp(arrive, above))
    wps.append(wp(arrive + z / (SPEED_FACTOR * scout.v_down_mps) + 0.5, home))
    for w in wps:
        if "target_id" in w:
            w["target"] = w.pop("target_id")
    return wps


def raw_energy_wh(route, scout):
    """route_energy_wh on raw waypoint dicts (same formula as fleetkit.energy)."""
    p = scout.power
    total = 0.0
    for a, b in zip(route, route[1:]):
        dep = a["t"] + a.get("dwell_s", 0.0)
        d = b["t"] - dep
        if d > 0:
            v = math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / d
            total += (p.p0_w + p.k_v * v * v) * d / 3600.0 + p.e_climb_wh_per_m * max(0.0, b["z"] - a["z"])
        total += p.p0_w * b.get("dwell_s", 0.0) / 3600.0
    return total


def _depart(scout, tasks, z, speed, shift):
    travel = hdist(scout.home, tasks[0]["point"]) / speed + z / scout.v_up_mps + 3.0
    return max(0.0, tasks[0]["window"][0] - travel) + shift


def _build_tour(scenario, scout, tasks, z, shift, depart):
    if SCOUT_SPEED_MODE == "eco":
        speeds = (eco_speed(scout),)
    elif SCOUT_SPEED_MODE == "fast":
        speeds = (fast_speed(scout),)
    else:
        speeds = (eco_speed(scout), fast_speed(scout))
    route = None
    for speed in speeds:
        start = _depart(scout, tasks, z, speed, shift) if depart is None else depart
        route = build_scout_route(scout, tasks, start, z, scenario, speed)
        if route is not None and len(speeds) > 1 and speed != speeds[-1]:
            if (route[-1]["t"] > scenario.horizon_s + EPS
                    or raw_energy_wh(route, scout) > scout.usable_wh + EPS):
                continue
        if route is not None:
            return route
    return route


def scout_tour(scenario, scout, tasks, z, shift=0.0, depart=None):
    """Scout tour for an ordered task list; same departure rule as the baseline.

    Memoised: sibling branches rebuild identical tours many times. Routes are never edited
    after construction, so the waypoint dicts are shared; the list itself is a copy.
    """
    key = (id(scenario), scout.id, z, shift, depart, SCOUT_SPEED_MODE, OBS_STRETCH,
           tuple((t["cid"], tuple(t["point"]), tuple(t["window"])) for t in tasks))
    hit = _tours.get(key)
    if hit is None or hit[0] is not scenario:
        if len(_tours) > 100000:
            _tours.clear()
        hit = _tours[key] = (scenario, _build_tour(scenario, scout, tasks, z, shift, depart))
    return None if hit[1] is None else list(hit[1])


def scout_reachable(scenario, scout, tasks, pos, z, prev_route=None):
    """False only when the tour must fail: the scout tour builder misses the window of
    tasks[pos], or the scout cannot be home before the horizon (R1).

    Lower bounds only: the builder flies at most `speed` along paths no shorter than straight
    lines, departs at t >= 0, and cannot observe before a window opens. prev_route, the
    parent's route for the same earlier tasks, tightens the bound on the previous visit.
    """
    rules_ = scenario.rules
    speed = eco_speed(scout) if SCOUT_SPEED_MODE == "eco" else fast_speed(scout)
    climb = z / (SPEED_FACTOR * scout.v_up_mps) + 0.5
    descend = z / (SPEED_FACTOR * scout.v_down_mps) + 0.5
    task = tasks[pos]
    lo, hi = task["window"]
    # Stretched observation points may be pulled in by up to ~35 m at 110 m altitude.
    slack = 2 * 35.0 / speed if OBS_STRETCH else 0.0
    arrive = climb + hdist(scout.home, task["point"]) / speed - slack
    if pos > 0:
        prev = tasks[pos - 1]
        prev_done = prev["window"][0] + rules_.obs_dwell_s
        if prev_route is not None:
            for w in prev_route:
                if w.get("action") == "observe" and w.get("target") == prev["cid"]:
                    prev_done = max(prev_done, w["t"] + w.get("dwell_s", 0.0) - REACH_SLACK_S)
                    break
        arrive = max(arrive, prev_done + hdist(prev["point"], task["point"]) / speed - slack)
    finish = max(arrive, lo) + rules_.obs_dwell_s
    if finish > hi + EPS:
        return False
    # Every remaining path back home is at least the straight line.
    return (finish + hdist(task["point"], scout.home) / speed - slack + descend
            <= scenario.horizon_s + EPS)


# Pads are about 11 m apart and separation is 40 m: two aircraft climbing from neighbouring
# pads at the same time break R8. Most scout R8 rejections were exactly that.
PAD_TAKEOFFS = True
# Strike arrival delays after confirmation; finer early steps give pad-separated takeoffs
# more chances before the window closes.
STRIKE_DELAYS = (0.0, 4.0, 8.0, 14.0, 20.0, 30.0, 45.0, 90.0)


def takeoff_need(scenario, a, b):
    """Seconds two takeoffs from these pads must be apart, or None if far enough."""
    d_sep = scenario.rules.d_sep_m + 2.0
    gap = hdist(a.home, b.home)
    if gap >= d_sep:
        return None
    climb = SPEED_FACTOR * min(a.v_up_mps, b.v_up_mps)
    return math.sqrt(max(0.0, d_sep * d_sep - gap * gap)) / climb + 1.0


def pad_conflict(scenario, aircraft, routes, extra):
    """True if a new route takes off too close in time to a neighbouring pad's takeoff."""
    for aid, route in extra.items():
        mine = aircraft[aid]
        for oid, other in list(extra.items()) + list(routes.items()):
            if oid == aid or not other or (oid in extra and other is not extra[oid]):
                continue
            need = takeoff_need(scenario, mine, aircraft[oid])
            if need is not None and abs(route[0]["t"] - other[0]["t"]) < need:
                return True
    return False


def takeoff_shift(scenario, aircraft, routes, scout, depart, limit=40.0, step=2.0):
    """Smallest delay so the scout's climb stays clear of takeoffs on neighbouring pads.
    Landings and later legs are left to the validator."""
    others = []
    for aid, route in routes.items():
        if aid == scout.id or not route:
            continue
        need = takeoff_need(scenario, scout, aircraft[aid])
        if need is not None:
            others.append((route[0]["t"], need))
    shift = 0.0
    while shift <= limit:
        if all(abs(depart + shift - t0) >= need for t0, need in others):
            return shift
        shift += step
    return 0.0


# Scout options are ranked and filtered by a straight-leg replay of the whole tour
# (tour_estimate): every window, strike confirmations, landing before the horizon and
# battery. Off: the distance ranking and scout_reachable bound only.
TIME_RANK = True


def tour_estimate(scenario, scout, legs, z):
    """Straight-leg replay of scout_tour: (finish times, landing time, Wh), or None as
    soon as a visit cannot finish inside its window.

    `legs` = [(point, window, reach)] in visit order; `reach` shortens the legs into and
    out of a point that is not chosen yet (it lies within `reach` of its contact). Same
    departure rule, climb, dwell and cruise speed as scout_tour at layer z (hybrid: the
    fast speed), without detours, so every time is a lower bound on the real tour.
    """
    rules_, power = scenario.rules, scout.power
    speed = eco_speed(scout) if SCOUT_SPEED_MODE == "eco" else fast_speed(scout)
    climb = z / (SPEED_FACTOR * scout.v_up_mps) + 0.5
    home = scout.home[:2]
    first = max(0.0, hdist(home, legs[0][0]) - legs[0][2])
    t = max(0.0, legs[0][1][0] - (first / speed + z / scout.v_up_mps + 3.0)) + climb
    wh = power.p0_w * climb / 3600.0 + power.e_climb_wh_per_m * z
    here, slack, finishes = home, 0.0, []
    for point, (lo, hi), reach in legs:
        d = max(0.0, hdist(here, point) - reach - slack)
        arrive = max(t + d / speed, lo)
        if arrive + rules_.obs_dwell_s > hi + EPS:
            return None
        if arrive > t:
            v = d / (arrive - t)
            wh += (power.p0_w + power.k_v * v * v) * (arrive - t) / 3600.0
        t = arrive + rules_.obs_dwell_s
        wh += power.p0_w * rules_.obs_dwell_s / 3600.0
        finishes.append(t)
        here, slack = point, reach
    back = max(0.0, hdist(here, home) - slack) / speed
    descent = z / (SPEED_FACTOR * scout.v_down_mps) + 0.5
    wh += ((power.p0_w + power.k_v * speed * speed) * back + power.p0_w * descent) / 3600.0
    return finishes, t + back + descent, wh


def tour_fits(scenario, scout, current, contact, order, z, deadlines):
    """False only when the tour visiting `order` (contact ids) at layer z must fail:
    a window or a strike's confirmation deadline (R4) missed, landing after the horizon
    (R1), or the battery exceeded (R3, 1% margin for the estimate)."""
    by_id = {t["cid"]: t for t in current}
    legs = []
    for cid in order:
        if cid == contact.id:
            legs.append((contact.pos[:2], tuple(contact.window), obs_offset(scenario, contact, z)))
        else:
            t = by_id[cid]
            lo, hi = t["window"]
            legs.append((t["point"], (lo, min(hi, deadlines.get(cid, math.inf))), 0.0))
    est = tour_estimate(scenario, scout, legs, z)
    return (est is not None and est[1] <= scenario.horizon_s + EPS
            and est[2] <= 1.01 * scout.usable_wh)


def strike_deadlines(scenario, aircraft, routes):
    """Latest observation finish for each struck contact (its first engagement - margin)."""
    out = {}
    for aid, route in routes.items():
        if aircraft[aid].type == SCOUT:
            continue
        for w in route:
            if w.get("action") == "engage":
                cid = w["target"]
                out[cid] = min(out.get(cid, math.inf), w["t"] - scenario.rules.confirm_margin_s)
    return out


def insertion_positions(home, tasks, pos, limit):
    """Tour positions ordered by added horizontal distance (cheapest insertion first)."""
    points = [home[:2]] + [t["point"] for t in tasks] + [home[:2]]
    cost = [(hdist(points[i], pos) + hdist(pos, points[i+1]) - hdist(points[i], points[i+1]), i)
            for i in range(len(tasks) + 1)]
    return [i for _, i in sorted(cost)[:limit]]


# How far one attempt to serve a contact got, from least to most progress.
# An attempt is one call to Search.children for one parent node.
STAGES = ("no_options", "no_team", "scout_window", "scout_route", "scout_rule", "strike_route",
          "strike_rule", "not_improving", "accepted")


def outcome_stage(outcome):
    return outcome.split(":", 1)[0]


def unserved_reason(contact, banned, stats):
    """One primary reason why a contact is missing from the winning plan.

    collateral_ban / no_window: excluded before the search.
    never_tried: no attempt ran (the pass order or the deadline never reached it).
    outranked: some branch served it validly, but that branch did not win.
    Otherwise the most common attempt outcome, e.g. "scout_rule:R4", "time_cap".
    """
    if contact.id in banned:
        return "collateral_ban"
    if contact.window is None:
        return "no_window"
    if stats is None or (not stats["attempts"] and not stats["warm_accepted"]):
        return "never_tried"
    outcomes = stats["outcomes"]
    if stats["warm_accepted"] or outcomes.get("accepted"):
        return "outranked"
    if not outcomes:
        return "never_tried"
    # Most frequent outcome; ties go to the attempt that got furthest.
    order = {name: i for i, name in enumerate(STAGES)}
    return max(outcomes, key=lambda o: (outcomes[o], order.get(outcome_stage(o), -1), o))


_official_detour = baseline._detour
_detours = {}


def _cached_detour(points, scenario):
    """Memoised baseline._detour: route builders repeat the same legs many times."""
    key = (id(scenario), tuple(points))
    if key not in _detours:
        if len(_detours) > 200000:
            _detours.clear()
        path = _official_detour(points, scenario)
        _detours[key] = (scenario, None if path is None else tuple(path))
    path = _detours[key][1]
    return None if path is None else list(path)


# The kit's route builders look _detour up in their module; results are unchanged.
baseline._detour = _cached_detour


@dataclass
class State:
    routes: dict
    tasks: dict
    used: frozenset
    parsed: dict
    separation: dict
    energies: dict
    served: frozenset
    score: float
    node_id: int = 0
    keys: dict = field(default_factory=dict)

    def plan(self, seed):
        return {"seed": seed, "routes": [
            {"aircraft": aid, "waypoints": self.routes[aid]} for aid in sorted(self.routes)]}

    def signature(self):
        return tuple(sorted(self.keys.items()))


def route_key(route):
    """Hashable identity of a waypoint list; routes are never edited after construction."""
    return tuple((w["t"], w["x"], w["y"], w["z"], w.get("dwell_s", 0.0), w.get("action"),
                  w.get("target")) for w in route)


class IncrementalValidator:
    """Reuse valid unchanged routes, including their cached R8 flight pieces.

    Local rules run on replaced/new routes. R4 and R10 run on the entire plan:
    rebuilding a scout tour can invalidate an earlier engagement's confirmation.
    R8 checks every pair containing a changed route. Unchanged pairs were already
    validated in the parent. Scoring uses the same energy and observation helpers.
    """
    def __init__(self, scenario, stop):
        self.sc = scenario
        self.aircraft = scenario.aircraft_by_id()
        self.contacts = scenario.contact_by_id()
        self.stop = stop
        self.calls = 0
        self.seconds = 0.0
        self.rejections = {}
        self.last_rule = None

    def check_time(self, stop=None):
        if time.monotonic() >= min(self.stop, stop if stop is not None else self.stop):
            raise DeadlineReached

    def reject(self, violations):
        key = violations[0].rule
        self.last_rule = key
        self.rejections[key] = self.rejections.get(key, 0) + 1
        return None

    @staticmethod
    def without(parent, removed):
        """Parent minus whole routes; extend() must still recheck R4/R10 globally."""
        def keep(d):
            return {k: v for k, v in d.items() if k not in removed}
        return replace(parent, routes=keep(parent.routes), parsed=keep(parent.parsed),
                       separation=keep(parent.separation), energies=keep(parent.energies))

    def extend(self, parent, extra):
        self.check_time()
        start = time.monotonic()
        self.calls += 1
        try:
            return self._extend(parent, extra)
        finally:
            self.seconds += time.monotonic() - start

    def quick_reject(self, extra):
        """Exact subset of R1 (horizon) and R9 (windows) on raw waypoints, before parsing."""
        horizon = self.sc.horizon_s
        for route in extra.values():
            for wp in route:
                start = wp["t"]
                end = start + wp.get("dwell_s", 0.0)
                if start < -EPS or end > horizon + EPS:
                    return "R1"
                contact = self.contacts.get(wp.get("target"))
                if contact is not None and contact.window is not None and wp.get("action"):
                    lo, hi = contact.window
                    if start < lo - EPS or (end if wp["action"] == "observe" else start) > hi + EPS:
                        return "R9"
        return None

    def _extend(self, parent, extra):
        rule = self.quick_reject(extra)
        if rule:
            self.last_rule = rule
            self.rejections[rule] = self.rejections.get(rule, 0) + 1
            return None
        plan = {"seed": self.sc.seed, "routes": [
            {"aircraft": aid, "waypoints": route} for aid, route in extra.items()]}
        changed, errors = parse_plan(plan)
        if errors or changed is None:
            raise ValueError("route builder produced malformed waypoints: " + str(errors))
        for check in (rules._r1, rules._r2, rules._r3, rules._r5, rules._r6,
                      rules._r7, rules._r9):
            violations = check(self.sc, changed, plan) if check is rules._r1 else check(self.sc, changed)
            if violations:
                return self.reject(violations)
        parsed = {**parent.parsed, **changed}
        for check in (rules._r4, rules._r10):
            violations = check(self.sc, parsed)
            if violations:
                return self.reject(violations)
        new_items = {aid: rules.separation_item(aid, route, self.aircraft[aid])
                     for aid, route in changed.items()}
        old_items = {aid: item for aid, item in parent.separation.items() if aid not in changed}
        pairs = itertools.chain(itertools.combinations(new_items.values(), 2),
                                itertools.product(new_items.values(), old_items.values()))
        for a, b in pairs:
            self.check_time()
            if a is not None and b is not None:
                violation = rules.separation_violation(self.sc.rules, self.contacts, a, b)
                if violation:
                    return self.reject([violation])
        energies = dict(parent.energies)
        energies.update({aid: route_energy_wh(route.waypoints, self.aircraft[aid])
                         for aid, route in changed.items()})
        served = set()
        used = set()
        for aid, route in parsed.items():
            for wp in route.waypoints:
                if wp.action == "engage":
                    served.add(wp.target)
                    used.add(aid)
                elif wp.action == "observe":
                    c = self.contacts[wp.target]
                    if c.cls == UNKNOWN and rules.valid_observation(wp, c, self.sc):
                        served.add(c.id)
        score = (sum(self.contacts[cid].points for cid in served)
                 - self.sc.rules.lambda_e * sum(energies.values())
                 - self.sc.rules.c_eff * len(used))
        keys = {**parent.keys, **{aid: route_key(route) for aid, route in extra.items()}}
        return State({**parent.routes, **extra}, parent.tasks, frozenset(used), parsed,
                     {**old_items, **new_items}, energies, frozenset(served), score, keys=keys)


class WarmPlanner(_Planner):
    """The original greedy choices, with incremental checks and a deadline."""
    def __init__(self, scenario, checker, state, stop):
        super().__init__(scenario)
        self.checker, self.state, self.stop = checker, state, stop
        self.last = None

    def _try(self, extra):
        self.checker.check_time(self.stop)
        self.last = self.checker.extend(self.state, extra)
        return self.last.score if self.last is not None else None

    def _add_observation(self, *args):
        self.checker.check_time(self.stop)
        return super()._add_observation(*args)


class Search:
    def __init__(self, scenario, seconds=55.0, top_n=5, beam_width=6, start=None, lns_share=0.4,
                 interleave=False, adaptive=False, restarts=True, widen=False,
                 scout_speed="fast", obs_stretch=True, variant=0):
        if seconds < 0 or not 1 <= top_n <= 100 or not 1 <= beam_width <= 100:
            raise ValueError("seconds must be nonnegative; top_n and beam_width must be 1..100")
        if not 0 <= lns_share <= 1:
            raise ValueError("lns_share must be 0..1")
        if scout_speed not in SCOUT_SPEEDS:
            raise ValueError(f"scout_speed must be one of {SCOUT_SPEEDS}")
        # Scout tour settings are module-wide (tour cache and helpers); one Search per run.
        global SCOUT_SPEED_MODE, OBS_STRETCH
        SCOUT_SPEED_MODE, OBS_STRETCH = scout_speed, bool(obs_stretch)
        self.scout_speed, self.obs_stretch = scout_speed, bool(obs_stretch)
        self.sc = scenario
        self.start = time.monotonic() if start is None else start
        # Includes input reading when the CLI passes its process start time.
        self.stop = self.start + min(seconds, 55.0)
        self.top_n, self.width = top_n, beam_width
        # The beam ends early so the final share of the budget repairs its best plan.
        self.beam_stop = self.stop - lns_share * max(0.0, self.stop - self.start)
        self.checker = IncrementalValidator(scenario, self.stop)
        self.aircraft = scenario.aircraft_by_id()
        self.contacts = scenario.contact_by_id()
        self.lns = {"iterations": 0, "accepted": 0, "improvements": 0, "gain": 0.0}
        self.lns_share = lns_share
        self.interleave, self.adaptive, self.restarts = interleave, adaptive, restarts
        # Later passes keep a wider beam: more outranked branches survive to the end.
        self.widen = widen
        if interleave:
            # Beam passes alternate with LNS bursts; a short final tail is LNS only.
            self.beam_stop = self.stop - min(lns_share, 0.1) * max(0.0, self.stop - self.start)
        # variant > 0: a parallel worker. It skips the deterministic warm start (the main
        # process runs it), starts the pass-order cycle elsewhere and draws other random
        # orders and LNS moves, so workers grow different subtrees of the same root.
        self.variant = variant
        self.lns_rng = random.Random(scenario.seed + 7919 + 104729 * variant)
        self.lns_state = None
        self.weights = {op: 1.0 for op in DESTROY_OPERATORS}
        self.root = State({}, {a.id: [] for a in scenario.aircraft if a.type == SCOUT},
                          frozenset(), {}, {}, {}, frozenset(), 0.0)
        self.trace = [{"id": 0, "parent": None, "decision": "start", "score": 0.0,
                       "served": [], "depth": 0}]
        self.archive = [self.root]
        self.signatures = {self.root.signature()}
        self.matrix = None
        self.passes = 0
        # Per-contact attempt accounting for "why was this contact not served".
        self.attempts = {}

    def stats(self, cid):
        if cid not in self.attempts:
            self.attempts[cid] = {"attempts": 0, "warm_accepted": 0, "outcomes": {}, "rules": {}}
        return self.attempts[cid]

    def record(self, state, parent, decision, depth):
        # Trace stores metadata, not full copies of every discarded plan.
        state = replace(state, node_id=len(self.trace))
        self.trace.append({"id": state.node_id, "parent": parent.node_id,
                           "decision": decision, "score": round(state.score, 6),
                           "served": sorted(state.served), "depth": depth})
        if len(self.archive) < self.top_n or state.score > self.archive[-1].score:
            signature = state.signature()
            if signature not in self.signatures:
                self.archive.append(state)
                self.archive.sort(key=lambda s: -s.score)
                self.archive = self.archive[:self.top_n]
                self.signatures = {s.signature() for s in self.archive}
        return state

    def warm_start(self):
        stop = min(self.stop, self.start + min(10.0, max(0.0, (self.stop-self.start)*0.25)))
        p = WarmPlanner(self.sc, self.checker, self.root, stop)
        contacts = (sorted((c for c in self.sc.contacts if c.cls == HOSTILE), key=lambda c: -c.points)
                    + sorted((c for c in self.sc.contacts if c.cls == UNKNOWN), key=lambda c: -c.points))
        for depth, c in enumerate(contacts, 1):
            self.checker.check_time(stop)
            accepted = p.serve_hostile(c) if c.cls == HOSTILE else p.serve_unknown(c)
            if accepted:
                self.stats(c.id)["warm_accepted"] += 1
                state = replace(p.last, tasks=dict(p.tasks))
                p.state = self.record(state, p.state, {"kind": "greedy", "contact": c.id}, depth)

    def children(self, parent, contact, stop, limit=2, insert=False):
        """Bounded candidate generation, recording how far each attempt got.

        insert=False appends the visit to a scout tour (beam search); insert=True
        also tries the cheapest earlier tour positions (LNS repair).
        """
        if contact.id in parent.served:
            return
        stats = self.stats(contact.id)
        stats["attempts"] += 1
        progress = {"stage": -1, "outcome": "no_options"}
        outcome = None
        try:
            yield from self._children(parent, contact, stop, limit, stats, progress, insert)
            outcome = progress["outcome"]
        except DeadlineReached:
            outcome = "accepted" if progress["outcome"] == "accepted" else "time_cap"
            raise
        finally:
            if outcome is None:  # Closed early by the consumer.
                outcome = progress["outcome"]
            stats["outcomes"][outcome] = stats["outcomes"].get(outcome, 0) + 1

    @staticmethod
    def _progress(stats, progress, outcome):
        stage = STAGES.index(outcome_stage(outcome))
        if ":" in outcome:
            stats["rules"][outcome] = stats["rules"].get(outcome, 0) + 1
        if stage >= progress["stage"]:
            progress["stage"], progress["outcome"] = stage, outcome

    def ranked_scouts(self, parent, contact, insert):
        """Scouts by distance from their tour (insert) or tour end (append) to the contact.

        If the first SCOUT_FANOUT are all busy, the nearest idle scout takes the last slot:
        all scouts may share a base, so an idle one is often the only one that fits.
        """
        scouts = _Planner(self.sc)._scout_candidates(contact, limit=len(parent.tasks))
        if TIME_RANK:
            return self._time_ranked(parent, contact, insert, scouts)
        if insert:
            def reach(a):
                points = [a.home] + [t["point"] for t in parent.tasks[a.id]]
                return min(hdist(q, contact.pos) for q in points)
            scouts.sort(key=reach)
        else:
            scouts.sort(key=lambda a: hdist(parent.tasks[a.id][-1]["point"] if parent.tasks[a.id]
                                           else a.home, contact.pos))
        head = scouts[:SCOUT_FANOUT]
        if head and all(parent.tasks[a.id] for a in head):
            idle = next((a for a in scouts[SCOUT_FANOUT:] if not parent.tasks[a.id]), None)
            if idle is not None:
                scouts = head[:-1] + [idle] + [a for a in scouts[SCOUT_FANOUT - 1:] if a is not idle]
        return scouts

    def _time_ranked(self, parent, contact, insert, scouts):
        """Scouts with a visit order that fits (tour_fits at the lowest layer), cheapest
        added distance first; one idle scout that fits among the first SCOUT_FANOUT."""
        deadlines = strike_deadlines(self.sc, self.aircraft, parent.routes)
        z = min(SCOUT_LAYERS)
        ranked = []
        for scout in scouts:
            current = parent.tasks[scout.id]
            cids = tuple(t["cid"] for t in current)
            points = [scout.home[:2]] + [t["point"] for t in current] + [scout.home[:2]]
            best = None
            for i in (range(len(current) + 1) if insert else [len(current)]):
                order = cids[:i] + (contact.id,) + cids[i:]
                if not tour_fits(self.sc, scout, current, contact, order, z, deadlines):
                    continue
                added = (hdist(points[i], contact.pos) + hdist(contact.pos, points[i + 1])
                         - hdist(points[i], points[i + 1]))
                best = added if best is None else min(best, added)
            if best is not None:
                ranked.append((best, scout.id, scout))
        ranked.sort(key=lambda r: r[:2])
        out = [r[2] for r in ranked]
        head = out[:SCOUT_FANOUT]
        if head and all(parent.tasks[a.id] for a in head):
            idle = next((a for a in out[SCOUT_FANOUT:] if not parent.tasks[a.id]), None)
            if idle is not None:
                out = head[:-1] + [idle] + [a for a in out[SCOUT_FANOUT - 1:] if a is not idle]
        return out

    @staticmethod
    def tour_orders(scout, current, contact, insert, skip_append=False):
        """Visit orders (contact ids) to try, with whether earlier visits keep the parent's order.

        Append mode: the new visit goes last. Insert mode: the cheapest insertion positions
        (every position for short tours), then deadline-ordered and start-ordered tours.
        """
        cids = tuple(t["cid"] for t in current)
        orders, seen = [], set()

        def add(order, keeps_prefix):
            if order not in seen:
                seen.add(order)
                orders.append((order, keeps_prefix))

        if not insert:
            add(cids + (contact.id,), True)
            return orders
        if skip_append:
            seen.add(cids + (contact.id,))
        k = len(current) + 1 if len(current) <= FULL_INSERT_TASKS else 3
        for pos in insertion_positions(scout.home, current, contact.pos, k):
            add(cids[:pos] + (contact.id,) + cids[pos:], True)
        if 1 <= len(current) <= REORDER_TASKS:
            windows = {t["cid"]: tuple(t["window"]) for t in current}
            windows[contact.id] = tuple(contact.window)
            add(tuple(sorted(windows, key=lambda c: (windows[c][1], windows[c][0], c))), False)
            add(tuple(sorted(windows, key=lambda c: (windows[c][0], windows[c][1], c))), False)
        return orders

    def scout_options(self, parent, contact, insert, ranks=slice(0, SCOUT_FANOUT),
                      skip_append=False):
        """(scout, tasks, route or None) for each scout, visit order and flight setting.

        Orders that cannot reach the new visit in its window, or get home before the
        horizon, yield None without building a route.
        """
        p = _Planner(self.sc)
        deadlines = strike_deadlines(self.sc, self.aircraft, parent.routes) if TIME_RANK else {}
        for scout in self.ranked_scouts(parent, contact, insert)[ranks]:
            current = parent.tasks[scout.id]
            by_id = {t["cid"]: t for t in current}
            prev_route = parent.routes.get(scout.id)
            for order, keeps_prefix in self.tour_orders(scout, current, contact, insert, skip_append):
                pos = order.index(contact.id)
                fits = {z: not TIME_RANK or tour_fits(self.sc, scout, current, contact, order, z,
                                                      deadlines) for z in SCOUT_LAYERS}
                for z, obs_angle, shift in itertools.product(SCOUT_LAYERS, (0.0, 60.0, -60.0),
                                                             (0.0, 7.0)):
                    if not fits[z]:
                        yield scout, None, None
                        continue
                    task = {"cid": contact.id,
                            "point": obs_point(self.sc, scout, contact, obs_angle, z),
                            "window": contact.window, "cpos": contact.pos}
                    tasks = [task if cid == contact.id else by_id[cid] for cid in order]
                    if PAD_TAKEOFFS:
                        # Delay takeoff past neighbours on nearby pads (most scout R8 conflicts).
                        speed = eco_speed(scout) if SCOUT_SPEED_MODE != "fast" else fast_speed(scout)
                        shift += takeoff_shift(self.sc, self.aircraft, parent.routes, scout,
                                               _depart(scout, tasks, z, speed, 0.0))
                    if not scout_reachable(self.sc, scout, tasks, pos, z,
                                           prev_route if keeps_prefix else None):
                        yield scout, tasks, None
                        continue
                    yield scout, tasks, scout_tour(self.sc, scout, tasks, z, shift)

    def _children(self, parent, contact, stop, limit, stats, progress, insert=False):
        """Bounded candidate generation: alternate scouts, teams and flight settings.

        Rounds stop at the first one that yields a child: the nearest scouts as asked
        (append or insert), then (append mode) the same scouts with insertion and
        reordering, then the next SCOUT_FALLBACK - SCOUT_FANOUT scouts with insertion.
        """
        groups = self.matrix.groups(contact, parent.used) if contact.cls == HOSTILE else [()]
        if not groups:
            self._progress(stats, progress, "no_team")
            return
        rounds = [self.scout_options(parent, contact, insert)]
        if not insert:
            rounds.append(self.scout_options(parent, contact, True, skip_append=True))
        rounds.append(self.scout_options(parent, contact, True, slice(SCOUT_FANOUT, SCOUT_FALLBACK)))
        state = {"count": 0, "seen": set()}
        if TIME_RANK and not any(self.ranked_scouts(parent, contact, True)):
            # No scout has any visit order that fits, at any tour position.
            self._progress(stats, progress, "scout_window")
            return
        for options in rounds:
            for item in self._serve(parent, contact, options, groups, stop, limit, stats,
                                    progress, state):
                yield item
                if state["count"] >= limit:
                    return
            if state["count"]:
                return

    def _serve(self, parent, contact, scout_options, groups, stop, limit, stats, progress, state):
        seen = state["seen"]
        # Round-robin teams before spending time on fine timing adjustments.
        for scout, tasks, route in scout_options:
            self.checker.check_time(stop)
            if route is None:
                self._progress(stats, progress, "scout_route")
                continue
            scout_extra = {scout.id: route}
            scout_state = self.checker.extend(parent, scout_extra)
            if scout_state is None:
                self._progress(stats, progress, f"scout_rule:{self.checker.last_rule}")
                continue
            if contact.cls == UNKNOWN:
                options = [(scout_state, ())]
            else:
                obs_end = max(w["t"] + w.get("dwell_s", 0) for w in route
                              if w.get("target") == contact.id and w.get("action") == "observe")
                options = self.strikes(parent, contact, groups, scout, route, obs_end, stop,
                                       stats, progress)
            for candidate, team in options:
                if candidate.score <= parent.score:
                    self._progress(stats, progress, "not_improving")
                    continue
                signature = candidate.signature()
                if signature in seen:
                    continue
                seen.add(signature)
                candidate.tasks = {**parent.tasks, scout.id: tasks}
                decision = {"kind": "serve", "contact": contact.id, "scout": scout.id,
                            "effectors": [a.id for a in team]}
                self._progress(stats, progress, "accepted")
                state["count"] += 1
                yield candidate, decision
                if state["count"] >= limit:
                    return

    def strikes(self, parent, contact, groups, scout, scout_route, obs_end, stop,
                stats=None, progress=None):
        base = max(obs_end + self.sc.rules.confirm_margin_s + 2.0, contact.window[0])
        # Try cheap alternatives to the nearest team. At most one child per team
        # for each scout setting, avoiding a beam filled with timing variants.
        for team in groups:
            found = False
            spacing = min(5.0, self.sc.rules.sync_window_s / max(1, len(team)-1))
            delays = STRIKE_DELAYS if PAD_TAKEOFFS else (0.0, 8.0, 20.0, 45.0, 90.0)
            for delay, angle, rotation in itertools.product(delays, (0.0, 35.0, -35.0), range(3)):
                self.checker.check_time(stop)
                arrive = base + delay
                if arrive + spacing*(len(team)-1) > contact.window[1] + EPS:
                    break
                extra = {scout.id: scout_route}
                for i, eff in enumerate(team):
                    z = EFFECTOR_LAYERS[(i + rotation) % len(EFFECTOR_LAYERS)]
                    route, earliest = _effector_route(eff, contact, arrive + spacing*i, z, angle, self.sc)
                    if route is None:
                        break
                    extra[eff.id] = route
                if len(extra) != len(team)+1:
                    if stats is not None:
                        self._progress(stats, progress, "strike_route")
                    continue
                if PAD_TAKEOFFS and pad_conflict(self.sc, self.aircraft, parent.routes,
                                                 {aid: r for aid, r in extra.items() if aid != scout.id}):
                    # Neighbouring pads climbing together: R8 without a validator call.
                    if stats is not None:
                        self._progress(stats, progress, "strike_rule:R8")
                    continue
                candidate = self.checker.extend(parent, extra)
                if candidate is None and stats is not None:
                    self._progress(stats, progress, f"strike_rule:{self.checker.last_rule}")
                if candidate is not None:
                    yield candidate, team
                    found = True
                if found:
                    break

    def repair(self, state, depth):
        """Greedy fill of what the search left unserved: unknowns join any scout tour at
        their cheapest positions, then hostiles retry with the free effectors."""
        improved = True
        while improved:
            improved = False
            contacts = [c for c in self.sc.contacts if c.cls in (HOSTILE, UNKNOWN)
                        and c.window is not None and c.id not in self.matrix.banned
                        and c.id not in state.served]
            contacts.sort(key=lambda c: (c.cls != UNKNOWN, -self.matrix.priority(c), c.id))
            for contact in contacts:
                self.checker.check_time()
                local_stop = min(self.checker.stop, time.monotonic() + 0.25)
                best = None
                try:
                    for child, decision in self.children(state, contact, local_stop, 4, True):
                        if best is None or child.score > best[0].score:
                            best = (child, decision)
                except DeadlineReached:
                    self.checker.check_time()
                if best is not None:
                    depth += 1
                    state = self.record(best[0], state, {**best[1], "kind": "repair"}, depth)
                    improved = True
        return state

    def search(self, contacts, priorities, rng):
        """Beam passes over varied contact orders until the checker deadline.
        Returns False once the trace cap is reached."""
        while contacts:
            self.checker.check_time()
            # Different orders explore different scout tours and allocations.
            kind = (self.passes + self.variant) % 4
            if kind == 0:
                order = sorted(contacts, key=lambda c: (-priorities[c.id], c.id))
            elif kind == 1:
                order = sorted(contacts, key=lambda c: (c.window[1], -priorities[c.id], c.id))
            elif kind == 2:
                order = sorted(contacts, key=lambda c: (-c.points, c.id))
            else:
                noise = {c.id: rng.uniform(0.65, 1.35) for c in contacts}
                order = sorted(contacts, key=lambda c: (-priorities[c.id]*noise[c.id], c.id))
            pass_start = time.monotonic()
            beam = [self.root]
            for depth, contact in enumerate(order, 1):
                self.checker.check_time()
                next_beam = []
                for parent in beam:
                    self.checker.check_time()
                    skipped = self.record(parent, parent, {"kind": "skip", "contact": contact.id}, depth)
                    next_beam.append(skipped)
                    remaining = max(1, (len(order)-depth+1)*len(beam))
                    allowance = min(0.12, max(0.005, (self.checker.stop-time.monotonic()) / (remaining*1.5)))
                    local_stop = min(self.checker.stop, time.monotonic()+allowance)
                    try:
                        for child, decision in self.children(parent, contact, local_stop):
                            next_beam.append(self.record(child, parent, decision, depth))
                    except DeadlineReached:
                        self.checker.check_time()  # Local cap expires without ending search.
                unique = {}
                for state in sorted(next_beam, key=lambda s: -s.score):
                    unique.setdefault(state.signature(), state)
                width = self.width
                if self.widen and self.passes >= 4:
                    width = min(100, self.width + self.width // 2)
                beam = list(unique.values())[:width]
            self.passes += 1
            if self.interleave and self.lns_share < 1:
                burst = (time.monotonic() - pass_start) * self.lns_share / (1 - self.lns_share)
                self.improve(contacts, priorities, time.monotonic() + burst)
            # Bound trace size for long runs on tiny or impossible scenarios.
            if len(self.trace) >= 20000:
                return False
        return True

    def run(self):
        contacts, priorities = self.search_phase()
        self.improve_phase(contacts, priorities)
        return self.finalize()

    def search_phase(self):
        """Warm start plus beam passes until the beam deadline.
        Returns the searchable contacts and their priorities."""
        if self.variant == 0:
            try:
                self.warm_start()
            except DeadlineReached:
                pass
        self.matrix = CandidateMatrix(self.sc)
        contacts = [c for c in self.sc.contacts if c.cls in (HOSTILE, UNKNOWN)
                    and c.window is not None and c.id not in self.matrix.banned]
        priorities = {c.id: self.matrix.priority(c) for c in contacts}
        rng = random.Random(self.sc.seed + 1009 * self.variant)
        self.checker.stop = min(self.beam_stop, self.stop)
        try:
            self.search(contacts, priorities, rng)
        except DeadlineReached:
            pass
        self.checker.stop = self.stop
        return contacts, priorities

    def improve_phase(self, contacts, priorities):
        """Greedy repair of the best beam plan, then LNS until the deadline."""
        # Greedy repair of the best plan gets at most a quarter of the LNS share.
        self.checker.stop = self.beam_stop + 0.25 * max(0.0, self.stop - self.beam_stop)
        try:
            if contacts:
                best = self.archive[0]
                depth = max((n["depth"] for n in self.trace if n["id"] == best.node_id), default=0)
                self.repair(best, depth)
        except DeadlineReached:
            pass
        self.checker.stop = self.stop
        try:
            self.improve(contacts, priorities)
        except DeadlineReached:
            pass

    def finalize(self):
        # Official full validation is the authority; archive can include partial
        # plans because every scout already has its return-and-land leg.
        verified = []
        for state in self.archive:
            if time.monotonic() >= self.start + 58.0:
                break
            result = evaluate(self.sc, state.plan(self.sc.seed))
            if result.valid:
                verified.append((state, result))
        if not verified:
            verified = [(self.root, evaluate(self.sc, self.root.plan(self.sc.seed)))]
        verified.sort(key=lambda pair: -pair[1].score)
        return verified

    def remove(self, state, cid):
        """State without contact cid, or None if no valid removal was found.

        Drops the strike team and the scout task, rebuilding the tour from its
        original departure so earlier visits keep their times and later ones can
        only move earlier. A hostile whose rebuilt tour fails keeps its harmless
        observation and loses only the strike.
        """
        team = {aid for aid, route in state.parsed.items()
                if any(w.action == "engage" and w.target == cid for w in route.waypoints)}
        attempts = []
        for sid, tasks in state.tasks.items():
            if not any(t["cid"] == cid for t in tasks):
                continue
            rest = [t for t in tasks if t["cid"] != cid]
            if not rest:
                attempts.append((team | {sid}, {}, {**state.tasks, sid: rest}))
                break
            route = state.routes.get(sid)
            if route is None:
                break
            for depart in (route[0]["t"], None):
                tour = scout_tour(self.sc, self.aircraft[sid], rest, route[1]["z"], depart=depart)
                if tour is not None:
                    attempts.append((team, {sid: tour}, {**state.tasks, sid: rest}))
            break
        if self.contacts[cid].cls == HOSTILE and team:
            attempts.append((team, {}, state.tasks))
        for removed, extra, tasks in attempts:
            candidate = self.checker.extend(self.checker.without(state, removed), extra)
            if candidate is not None and cid not in candidate.served:
                candidate.tasks = tasks
                return candidate
        return None

    def improve(self, contacts, priorities, stop=None):
        """Large neighbourhood search: remove a few served contacts, then reinsert.

        Destroy picks random, spatially related, low-value or same-tour served
        contacts (only the first two unless adaptive). Repair tries the removed
        contacts, their nearest unserved neighbours and a few random unserved ones,
        inserting each scout visit at its cheapest tour positions. Worse plans are
        sometimes accepted (annealing); accepted plans enter the archive.
        State persists across calls, so interleaved bursts continue one search.
        """
        if not contacts:
            return
        stop = self.stop if stop is None else min(stop, self.stop)
        st = self.lns_state
        if st is None:
            st = self.lns_state = {
                "by_id": {c.id: c for c in contacts}, "best": self.archive[0],
                "current": self.archive[0], "begin": time.monotonic(),
                "temperature": max(1.0, 0.01 * abs(self.archive[0].score)),
                "stale": 0, "since_best": 0}
        elif self.archive[0].score > st["best"].score + 1e-9:
            st["best"] = st["current"] = self.archive[0]
            st["stale"] = st["since_best"] = 0
        saved, self.checker.stop = self.checker.stop, stop
        try:
            # Small scenarios converge quickly; stopping early lowers the runtime tie-break.
            while st["since_best"] < 400:
                self.checker.check_time()
                self.lns_step(st, contacts, priorities)
        except DeadlineReached:
            if time.monotonic() >= self.stop:
                raise
        finally:
            self.checker.stop = saved

    def destroy(self, op, served, by_id, current, priorities, rng):
        k = rng.randint(1, min(4, len(served)))
        if op == "related":
            seed = by_id[rng.choice(served)]
            return sorted(served, key=lambda c: hdist(by_id[c].pos, seed.pos))[:k]
        if op == "worst":
            return sorted(served, key=lambda c: priorities[c] * rng.uniform(0.8, 1.2))[:k]
        if op == "tour":
            tours = [[t["cid"] for t in tasks if t["cid"] in current.served and t["cid"] in by_id]
                     for tasks in current.tasks.values()]
            tours = [t for t in tours if t]
            if tours:
                tour = rng.choice(tours)
                return rng.sample(tour, min(k, len(tour)))
        return rng.sample(served, k)

    def lns_step(self, st, contacts, priorities):
        rng, by_id = self.lns_rng, st["by_id"]
        current, best = st["current"], st["best"]
        self.lns["iterations"] += 1
        served = sorted(c for c in current.served if c in by_id)
        if self.adaptive:
            ops = list(self.weights)
            op = rng.choices(ops, weights=[self.weights[o] for o in ops])[0]
        else:
            op = "related" if rng.random() < 0.5 else "random"
        removed = []
        partial = current
        if served:
            for cid in self.destroy(op, served, by_id, current, priorities, rng):
                smaller = self.remove(partial, cid)
                if smaller is not None:
                    partial = smaller
                    removed.append(cid)
        open_ = [c for c in contacts if c.id not in partial.served and c.id not in removed]
        centre = [by_id[c].pos for c in removed] or [rng.choice(contacts).pos]
        near = sorted(open_, key=lambda c: min(hdist(c.pos, q) for q in centre))[:6]
        extra = rng.sample(open_, min(3, len(open_)))
        pool = list({c.id: c for c in near + extra}.values())
        noise = {c.id: rng.uniform(0.6, 1.4) for c in pool}
        order = (sorted(pool, key=lambda c: -priorities[c.id] * noise[c.id])
                 + [by_id[c] for c in removed])
        for contact in order:
            self.checker.check_time()
            local_stop = min(self.checker.stop, time.monotonic() + 0.08)
            chosen_child = None
            try:
                for child, _ in self.children(partial, contact, local_stop, insert=True):
                    if chosen_child is None or child.score > chosen_child.score:
                        chosen_child = child
            except DeadlineReached:
                self.checker.check_time()
            if chosen_child is not None:
                partial = chosen_child
        delta = partial.score - current.score
        span = max(1e-6, self.stop - st["begin"])
        heat = st["temperature"] * max(0.0, self.stop - time.monotonic()) / span
        reward = 0.0
        if delta > 1e-9 or (removed and heat > 0 and rng.random() < math.exp(delta / heat)):
            self.lns["accepted"] += 1
            reward = 1.0 if delta > 1e-9 else 0.3
            current = st["current"] = self.record(partial, current, {"kind": "lns", "op": op,
                                                                     "removed": removed},
                                                  len(partial.served))
            if current.score > best.score + 1e-9:
                self.lns["improvements"] += 1
                self.lns["gain"] += current.score - best.score
                st["best"], st["stale"], st["since_best"] = current, 0, 0
                reward = 3.0
        if self.adaptive:
            self.weights[op] = max(0.05, 0.8 * self.weights[op] + 0.2 * reward)
        if reward == 3.0:
            return
        st["stale"] += 1
        st["since_best"] += 1
        if st["stale"] >= 25:
            st["stale"] = 0
            st["current"] = rng.choice(self.archive) if self.restarts else st["best"]

    def diagnostics(self, served):
        """Per-contact accounting against the winner's served set."""
        banned = self.matrix.banned if self.matrix is not None else CandidateMatrix(self.sc).banned
        served = set(served)
        contacts, summary, points = {}, {}, {}
        for c in self.sc.contacts:
            if c.cls not in (HOSTILE, UNKNOWN):
                continue
            stats = self.attempts.get(c.id)
            entry = {"cls": c.cls, "points": c.points, "served": c.id in served,
                     "attempts": stats["attempts"] if stats else 0,
                     "outcomes": dict(stats["outcomes"]) if stats else {},
                     "rules": dict(stats["rules"]) if stats else {}}
            if c.id not in served:
                reason = unserved_reason(c, banned, stats)
                entry["reason"] = reason
                summary[reason] = summary.get(reason, 0) + 1
                points[reason] = points.get(reason, 0) + c.points
            contacts[c.id] = entry
        return {"contacts": contacts,
                "unserved": dict(sorted(summary.items(), key=lambda kv: -points[kv[0]])),
                "unserved_points": dict(sorted(points.items(), key=lambda kv: -kv[1]))}

    def report(self, verified):
        diagnostics = self.diagnostics(verified[0][1].served if verified else [])
        return {"seed": self.sc.seed, "elapsed_s": time.monotonic()-self.start,
                "search_budget_s": max(0, self.stop-self.start), "passes": self.passes,
                "candidate_checks": self.checker.calls,
                "candidate_check_seconds": self.checker.seconds,
                "rejections": self.checker.rejections,
                "beam_width": self.width, "requested_top_n": self.top_n,
                "scouting": {"scout_speed": self.scout_speed, "obs_stretch": self.obs_stretch},
                "lns": {**self.lns, "variant": {"interleave": self.interleave,
                                                "adaptive": self.adaptive,
                                                "restarts": self.restarts},
                        **({"weights": dict(self.weights)} if self.adaptive else {})},
                "unserved": diagnostics["unserved"],
                "unserved_points": diagnostics["unserved_points"],
                "contacts": diagnostics["contacts"],
                "solutions": [{"rank": i, "node": state.node_id, "score": result.score,
                               "served": result.served,
                               **({"worker": state.variant} if isinstance(state, ImportedPlan) else {})}
                              for i, (state, result) in enumerate(verified, 1)],
                **({"parallel": self.parallel} if getattr(self, "parallel", None) else {}),
                "nodes": self.trace}

# Parallel search. Worker processes run independent Search variants (their own beam passes
# from the same root and their own LNS chains) and send back their validated top plans. The
# main process runs the ordinary search, so workers=1 is exactly the single-core planner.

WORKER_MARGIN_S = 1.5    # workers stop this much earlier, so their plans arrive in time
WORKER_IMPORTS = 3       # worker plans the main process will re-validate at most
MAX_WORKERS = 16


def default_workers():
    """One process per spare core, keeping a core for the system; at most 5."""
    import os
    try:
        cores = len(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        cores = os.cpu_count() or 1
    return max(1, min(5, cores - 1))


class ImportedPlan:
    """A plan found by worker `variant`; the main process validates it again before export."""
    node_id = None

    def __init__(self, plan, variant):
        self._plan, self.variant = plan, variant

    def plan(self, seed):
        return self._plan


def _worker(scenario, kwargs, variant, conn):
    try:
        search = Search(scenario, variant=variant, **kwargs)
        verified = search.run()
        conn.send({"variant": variant, "elapsed_s": time.monotonic() - search.start,
                   "passes": search.passes, "lns": dict(search.lns),
                   "candidate_checks": search.checker.calls,
                   "plans": [(state.plan(scenario.seed), result.score) for state, result in verified]})
    except BaseException as exc:  # Report, never hang the main process.
        try:
            conn.send({"variant": variant, "error": repr(exc)})
        except Exception:
            pass
    finally:
        conn.close()


def run_parallel(scenario, workers=1, **kwargs):
    """Search.run() on `workers` processes; returns (main search, verified plans).

    Every candidate the main process exports passed its own official evaluate() call. Worker
    failures, late workers or an unavailable multiprocessing module fall back to the main
    process's result.
    """
    search = Search(scenario, **kwargs)
    if workers <= 1:
        return search, search.run()
    try:
        import multiprocessing as mp
        ctx = mp.get_context("fork" if "fork" in mp.get_all_start_methods() else "spawn")
    except (ImportError, ValueError):
        return search, search.run()
    budget = max(0.0, search.stop - search.start - WORKER_MARGIN_S)
    worker_kwargs = dict(kwargs, start=search.start, seconds=budget)
    procs = []
    for variant in range(1, min(workers, MAX_WORKERS)):
        try:
            recv, send = ctx.Pipe(duplex=False)
            proc = ctx.Process(target=_worker, args=(scenario, worker_kwargs, variant, send),
                               daemon=True)
            proc.start()
        except (OSError, RuntimeError):
            break
        send.close()
        procs.append((proc, recv))
    try:
        verified = search.run()
        reports = []
        collect_until = min(search.start + 57.0, max(time.monotonic(), search.stop) + 1.0)
        for proc, recv in procs:
            try:
                if recv.poll(max(0.0, collect_until - time.monotonic())):
                    reports.append(recv.recv())
            except (EOFError, OSError):
                pass
        best = verified[0][1].score
        candidates = sorted(((score, plan, r["variant"]) for r in reports
                             for plan, score in r.get("plans", ()) if score > best + 1e-9),
                            key=lambda item: -item[0])
        imported = []
        for _, plan, variant in candidates[:WORKER_IMPORTS]:
            if time.monotonic() >= search.start + 58.0:
                break
            result = evaluate(scenario, plan)
            if result.valid:
                imported.append((ImportedPlan(plan, variant), result))
                break
        verified = sorted(imported + verified, key=lambda pair: -pair[1].score)
        winner = verified[0][0]
        search.parallel = {
            "workers": 1 + len(procs), "requested_workers": workers,
            "winner": winner.variant if isinstance(winner, ImportedPlan) else 0,
            "main_score": best,
            "reports": [{k: v for k, v in r.items() if k != "plans"}
                        | {"best": max((score for _, score in r.get("plans", ())), default=None)}
                        for r in reports],
            "missing": len(procs) - len(reports)}
        return search, verified
    finally:
        for proc, recv in procs:
            if proc.is_alive():
                proc.terminate()
            proc.join(0.2)
            recv.close()
