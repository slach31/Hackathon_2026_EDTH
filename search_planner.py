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
from fleetkit.geometry import EPS, INF, hdist
from fleetkit.model import HOSTILE, SCOUT, UNKNOWN, parse_plan
from fleetkit.scorer import evaluate
from fleetkit import validator as rules
from candidate_matrix import CandidateMatrix


class DeadlineReached(Exception):
    pass


DESTROY_OPERATORS = ("random", "related", "worst", "tour")


# Scout candidate generation.
SCOUT_FANOUT = 4        # scouts tried first for a contact
SCOUT_FALLBACK = 8      # scouts tried when none of the first SCOUT_FANOUT gives a valid child
FULL_INSERT_TASKS = 4   # tours up to this long try every insertion position, not only 3
REORDER_TASKS = 6       # tours up to this long also try deadline- and start-ordered tours
# Search limits. The hand-in keeps these; offline board runs (analysis/board_search.py)
# raise them for longer searches.
MAX_SEARCH_S = 55.0     # cap on the search deadline, from startup
LNS_PATIENCE = 400      # LNS iterations without a new best before LNS stops early
TRACE_CAP = 20000       # recorded nodes after which beam passes stop
VARIANT_SEED_STRIDE = 1000003  # random seed offset per search variant
# Scout observation altitudes. Low tours climb less and observe from further away (the 3D
# observation range allows ~138 m horizontally at 35 m, ~125 m at 70 m).
SCOUT_ALTITUDES = (30.5, 70.0)
# Low strike routes: pad straight to the engagement point; detour waypoints (around zones)
# fly at this altitude, since R2 requires z >= z_min at every waypoint but the last.
LOW_DETOUR_Z = 32.0
# Low strikes engage at ground level over the contact: R2 allows z = 0 at the last waypoint
# and R10 only needs the engagement within R_eng (3D) of the contact; no climb energy.
LOW_ENGAGE_Z = 0.0
# Alternative low strikes cross at this altitude (above scouts at 30.5 m by > d_sep) and
# dive over the last LOW_CROSS_M metres.
LOW_CROSS_Z = 72.0
LOW_CROSS_M = 150.0
# LNS scout rebuilds: share of iterations, and time to regrow the emptied tour.
# Measured on L4-402 (90 s from the best plan): rebuilds were accepted 7 times in ~160
# tries with changes of -3..+4 points, and a full repair after each rebuild made it worse
# (3 accepted, half the LNS iterations). Kept rare so it costs little.
SCOUT_REBUILD_P = 0.1
SCOUT_REBUILD_S = 1.5   # per rebuilt scout
SCOUT_REBUILD_SIZES = (1, 2, 2, 3)  # scouts cleared together per rebuild (drawn uniformly)
# Arrival shifts tried when re-flying a strike (the team's 20 s sync window still applies).
REFLY_SHIFTS_S = (0.0, 3.0, -3.0, 7.0, -7.0)
# Joint scout planning: share of the beam phase, candidate tours per scout, contacts tried
# per insertion step, and ranking noise of the randomised tours.
JOINT_SHARE = 0.85
JOINT_BUILDS = 3      # distinct candidate plans built per run; the best result is kept
JOINT_PATIENCE = 150  # restarts without a better plan before the joint planner stops early
JOINT_TOURS_PER_SCOUT = 12
JOINT_NEIGHBOURS = 6
JOINT_NOISE = 0.5
JOINT_SLACK_S = 20.0   # generated tours end this long before the horizon
# Building chosen tours: altitude layers 40 m apart (R8 cannot fail between layers) and
# departure delays, tried in order until the validator accepts the route.
JOINT_BUILD_ALTITUDES = (30.5, 70.5, 110.5)
JOINT_BUILD_SHIFTS = (0.0, 5.0, 10.0, 20.0, 35.0, 50.0)
# Scout-first construction (sector tours) and idle-scout tours.
CONSTRUCT_SHARE = 0.0  # share of the beam phase spent on scout-first construction
GROW_CANDIDATES = 6     # contacts tried per step when growing one scout's tour
GROW_CONTACT_S = 0.25   # time cap per contact tried while growing a tour
REPAIR_SHARE = 0.35     # share of the LNS phase for observation cleanup, idle tours and repair
POLISH_RESERVE_S = 0.4  # kept before the deadline to clean up the best plans' tours
# Rebuilt tours keep earlier visits within a few seconds of the parent's (other altitude or
# departure shift); the reachability bound subtracts this much from the parent's times.
REACH_SLACK_S = 20.0

_tours = {}


def evict(cache):
    """Drop the oldest tenth of a full cache. Clearing a cache of ~200k parsed routes at
    once stalls the process for seconds (deallocation), which can push a run past the
    deadline; evicting in small steps spreads that cost."""
    for key in list(itertools.islice(cache, max(1, len(cache) // 10))):
        del cache[key]

# Scout cruise speed. The kit flies scouts at min(0.95 v_h, best-range speed). Energy per
# metre is flat near the best-range speed (on L4, 0.98 v_h costs +0-1.5% per metre) while
# airtime, hence hover-free p0 energy, R1 (home by the horizon) and window reach, improve
# by 5-35%. For scouts whose best-range speed exceeds 0.95 v_h the kit speed is both slower
# and dearer. R2 allows v_h exactly; 0.998 v_h plus 0.01 s per leg absorbs waypoint
# rounding. The last 2% decides whether early-window contacts ~5 km out can be confirmed.
SCOUT_CRUISE = 0.998
# "fast": every scout leg at SCOUT_CRUISE * v_h. "eco": the kit's speed. "hybrid": the kit's
# speed, falling back to fast when the eco tour misses a window, the horizon or the battery.
# Set per run by Search(scout_speed=...); planner.py --scout-speed.
SCOUT_SPEEDS = ("fast", "eco", "hybrid")
SCOUT_SPEED_MODE = "fast"
# Contacts are on the ground; the 3D observation range allows a horizontal offset of
# sqrt(R_obs^2 - z^2): ~147 m at 30.5 m, ~133 m at 70 m. The kit always uses 100 m. The
# validator checks the 3D distance at the waypoint (+1e-6); 5 cm covers 1 mm rounding.
OBS_MARGIN_M = 0.05
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


def _at(path, z):
    """A detour path with every point after the first at altitude z (R2: z >= z_min)."""
    return [path[0]] + [(q[0], q[1], z) for q in path[1:]]


def build_scout_route(scout, tasks, depart, z, scenario, speed):
    """The kit's _scout_route with an explicit cruise speed and diagonal climb/descent.

    R2 bounds a leg's duration by max(horizontal / v_h, vertical / v_up or v_down), and
    altitude limits apply at waypoints only, so the scout climbs on its way to the first
    observation and descends on its way home instead of hovering straight up and down.
    """
    rules_ = scenario.rules
    detour, length, wp = baseline._detour, baseline._length, baseline._wp
    home = scout.home
    ground = (home[0], home[1], 0.0)
    v_up, v_down = SPEED_FACTOR * scout.v_up_mps, SPEED_FACTOR * scout.v_down_mps
    t = depart
    wps = [wp(t, home)]
    cur = ground
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
        path = _at(path, z)
        total = length(path)
        if total < 1.0:
            return None
        lo, hi = task["window"]
        # Climbing legs (only the first) also need z / v_up, spread over the whole path.
        climb = (z - cur[2]) / v_up + 0.2 if cur[2] < z - EPS else 0.0
        # 0.01 s absorbs waypoint rounding (1 mm, 1 ms) at near-v_h cruise speed.
        arrive = max(t + max(total / speed, climb) + 0.01, lo)
        if arrive + rules_.obs_dwell_s > hi:
            return None
        for k in range(1, len(path) - 1):
            wps.append(wp(t + (arrive - t) * length(path[:k + 1]) / total, path[k]))
        wps.append(wp(arrive, target, action="observe", target_id=task["cid"],
                      dwell_s=rules_.obs_dwell_s))
        t = arrive + rules_.obs_dwell_s
        cur = target
    path = detour([cur, ground], scenario)
    if path is None:
        return None
    path = _at(path[:-1], z) + [ground]
    total = length(path)
    # The descent is spread over the whole way home; the last leg alone must also fit it.
    last = hdist(path[-2], path[-1])
    t_last = max(last / speed, z / v_down + 0.2)
    arrive = t + (total - last) / speed + t_last
    # Detour points come before the last leg, at the cruise speed.
    for k in range(1, len(path) - 1):
        wps.append(wp(t + length(path[:k + 1]) / speed, path[k]))
    wps.append(wp(arrive, home))
    for w in wps:
        if "target_id" in w:
            w["target"] = w.pop("target_id")
    return wps


def scout_range_m(scout, scenario):
    """Farthest horizontal distance a scout can fly out to and back from, with one
    observation dwell, within both its battery and the horizon, at its cruise speed."""
    rules_ = scenario.rules
    speed = fast_speed(scout) if SCOUT_SPEED_MODE != "eco" else eco_speed(scout)
    p = scout.power
    cruise_w = p.p0_w + p.k_v * speed * speed
    fixed_wh = (p.p0_w * rules_.obs_dwell_s / 3600.0
                + p.e_climb_wh_per_m * min(SCOUT_ALTITUDES))
    by_battery = max(0.0, scout.usable_wh - fixed_wh) * 3600.0 / cruise_w * speed / 2.0
    by_horizon = max(0.0, scenario.horizon_s - rules_.obs_dwell_s) * speed / 2.0
    return min(by_battery, by_horizon)


def scout_can_serve(scout, contact, scenario, reach):
    """Lower-bound test: can this scout alone observe the contact (in time for a strike,
    for hostiles) and get home? reach: its range from scout_range_m."""
    rules_ = scenario.rules
    speed = fast_speed(scout) if SCOUT_SPEED_MODE != "eco" else eco_speed(scout)
    off = obs_offset(scenario, contact, min(SCOUT_ALTITUDES))
    d = max(0.0, hdist(scout.home, contact.pos) - off)
    if d > reach:
        return False
    lo, hi = contact.window
    done = max(d / speed, lo) + rules_.obs_dwell_s
    limit = hi - (rules_.confirm_margin_s if contact.cls == HOSTILE else 0.0)
    return done <= limit + EPS


def low_effector_route(eff, contact, arrive, scenario, cross_z=None):
    """(route, earliest arrival): pad straight to the engagement point at best-range speed.

    No climb to a cruise layer: R2 limits altitude at waypoints only, and the engagement
    (the last waypoint) may be low. Zones and protected areas are avoided with the kit's
    detour points, flown at LOW_DETOUR_Z. cross_z: instead climb diagonally to this
    altitude on the way out (one waypoint, LOW_CROSS_M short of the target) and dive to
    the engagement, passing above low scouts at a fraction of the layered routes' climb.
    """
    wp = baseline._wp
    home = (eff.home[0], eff.home[1], 0.0)
    e_pt = (contact.pos[0], contact.pos[1], LOW_ENGAGE_Z)
    path = baseline._detour([home, e_pt], scenario)
    if path is None:
        return None, arrive
    z_detour = max(LOW_DETOUR_Z, scenario.rules.z_min_m + 1.0)
    if cross_z is None:
        path = [path[0]] + [(q[0], q[1], z_detour) for q in path[1:-1]] + [e_pt]
    else:
        path = [path[0]] + [(q[0], q[1], cross_z) for q in path[1:-1]] + [e_pt]
        last, prev = path[-1], path[-2]
        d = hdist(prev, last)
        if d < 2 * LOW_CROSS_M:
            return None, arrive
        f = (d - LOW_CROSS_M) / d
        path.insert(-1, (prev[0] + (last[0] - prev[0]) * f, prev[1] + (last[1] - prev[1]) * f, cross_z))
    speed = min(SCOUT_CRUISE * eff.v_h_mps, best_range_speed(eff.power))
    v_up, v_down = SPEED_FACTOR * eff.v_up_mps, SPEED_FACTOR * eff.v_down_mps
    legs = []
    for a, b in zip(path, path[1:]):
        dz = b[2] - a[2]
        legs.append(max(hdist(a, b) / speed, dz / v_up if dz > 0 else -dz / v_down) + 0.05)
    earliest = sum(legs)
    if arrive < earliest:
        return None, earliest
    t = arrive - earliest
    wps = [wp(t, home)]
    for q, leg in zip(path[1:-1], legs):
        t += leg
        wps.append(wp(t, q))
    wps.append(wp(arrive, e_pt, action="engage", target=contact.id))
    return wps, earliest


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
            evict(_tours)
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
    # Tours climb and descend diagonally, so straight-line flight is the only lower bound.
    climb = descend = 0.0
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


def insertion_positions(home, tasks, pos, limit):
    """Tour positions ordered by added horizontal distance (cheapest insertion first)."""
    points = [home[:2]] + [t["point"] for t in tasks] + [home[:2]]
    cost = [(hdist(points[i], pos) + hdist(pos, points[i+1]) - hdist(points[i], points[i+1]), i)
            for i in range(len(tasks) + 1)]
    return [i for _, i in sorted(cost)[:limit]]


# How far one attempt to serve a contact got, from least to most progress.
# An attempt is one call to Search.children for one parent node.
STAGES = ("no_options", "no_team", "scout_route", "scout_rule", "strike_route",
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
            evict(_detours)
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
    # Contacts whose R4/R10 status must be rechecked: their routes were removed (without()).
    recheck: frozenset = frozenset()

    def plan(self, seed):
        return {"seed": seed, "routes": [
            {"aircraft": aid, "waypoints": self.routes[aid]} for aid in sorted(self.routes)]}

    def signature(self):
        return tuple(sorted(self.keys.items()))


def route_key(route):
    """Hashable identity of a waypoint list; routes are never edited after construction."""
    return tuple((w["t"], w["x"], w["y"], w["z"], w.get("dwell_s", 0.0), w.get("action"),
                  w.get("target")) for w in route)


# Rules that look at one route only, in the order the validator reports them.
LOCAL_CHECKS = (rules._r1, rules._r2, rules._r3, rules._r5, rules._r6, rules._r7, rules._r9)


# Pairs closer than d_sep + this margin go to the official check (rounding safety).
SEPARATION_MARGIN_M = 0.01


def piece_boxes(item):
    """Official separation item plus, per flight piece, its bounding box and velocity."""
    if item is None:
        return None
    extra = []
    for t0, t1, p0, p1 in item[1]:
        span = t1 - t0
        k = 1.0 / span if span > 1e-12 else 0.0
        extra.append((min(p0[0], p1[0]), max(p0[0], p1[0]), min(p0[1], p1[1]), max(p0[1], p1[1]),
                      min(p0[2], p1[2]), max(p0[2], p1[2]),
                      (p1[0] - p0[0]) * k, (p1[1] - p0[1]) * k, (p1[2] - p0[2]) * k))
    return item, extra


def may_be_close(a, b, d):
    """False only if no two time-overlapping pieces of a and b come within d of each other.

    Pieces are straight, constant-speed motions (as in the official geometry): a bounding
    box gap of at least d, or a closest approach over the shared time of at least d,
    proves the official close_intervals() finds nothing for that pair of pieces.
    """
    (_, pa, s_a, e_a, _), ba = a
    (_, pb, s_b, e_b, _), bb = b
    if min(e_a, e_b) - max(s_a, s_b) <= 1e-12:
        return False
    dd = d * d
    i = j = 0
    na, nb = len(pa), len(pb)
    while i < na and j < nb:
        x, y = pa[i], pb[j]
        lo, hi = max(x[0], y[0]), min(x[1], y[1])
        if hi - lo > 1e-12:
            p, q = ba[i], bb[j]
            if (p[0] - q[1] < d and q[0] - p[1] < d and p[2] - q[3] < d and q[2] - p[3] < d
                    and p[4] - q[5] < d and q[4] - p[5] < d):
                # Relative position at lo and relative velocity; minimise |r + w s| on [0, hi-lo].
                ta, tb = lo - x[0], lo - y[0]
                xa, xb = x[2], y[2]
                rx = xb[0] + q[6] * tb - xa[0] - p[6] * ta
                ry = xb[1] + q[7] * tb - xa[1] - p[7] * ta
                rz = xb[2] + q[8] * tb - xa[2] - p[8] * ta
                wx, wy, wz = q[6] - p[6], q[7] - p[7], q[8] - p[8]
                ww = wx * wx + wy * wy + wz * wz
                s = 0.0
                if ww > 1e-18:
                    s = min(max(-(rx * wx + ry * wy + rz * wz) / ww, 0.0), hi - lo)
                cx, cy, cz = rx + wx * s, ry + wy * s, rz + wz * s
                if cx * cx + cy * cy + cz * cz < dd:
                    return True
        if x[1] < y[1]:
            i += 1
        else:
            j += 1
    return False


class IncrementalValidator:
    """Reuse valid unchanged routes, including their cached R8 flight pieces.

    Local rules (one route at a time) run once per distinct route and are cached.
    R4 and R10 run on every route that touches a contact whose observations or
    engagements changed: rebuilding a scout tour can invalidate an earlier engagement's
    confirmation, and the parent was valid for every other contact.
    R8 checks every pair containing a changed route; pairs whose pieces are provably
    40 m apart skip the official check. Unchanged pairs were already validated in the
    parent. Scoring uses the same energy and observation helpers.
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
        self.local = {}
        self.targets = {}
        self.d_sep = scenario.rules.d_sep_m + SEPARATION_MARGIN_M

    def local_check(self, aid, route):
        """(parsed route, (index, rule) of the first local violation or None, separation
        item with boxes, energy), cached per distinct route."""
        key = (aid, route_key(route))
        hit = self.local.get(key)
        if hit is None:
            if len(self.local) > 200000:
                evict(self.local)
            plan = {"seed": self.sc.seed, "routes": [{"aircraft": aid, "waypoints": route}]}
            parsed, errors = parse_plan(plan)
            if errors or parsed is None:
                raise ValueError("route builder produced malformed waypoints: " + str(errors))
            failed = None
            for i, check in enumerate(LOCAL_CHECKS):
                violations = check(self.sc, parsed, plan) if check is rules._r1 else check(self.sc, parsed)
                if violations:
                    failed = (i, violations[0].rule)
                    break
            r = parsed[aid]
            item = energy = None
            if failed is None:
                item = piece_boxes(rules.separation_item(aid, r, self.aircraft[aid]))
                energy = route_energy_wh(r.waypoints, self.aircraft[aid])
            hit = self.local[key] = (r, failed, item, energy)
        return hit

    def route_targets(self, route):
        hit = self.targets.get(id(route))
        if hit is None or hit[0] is not route:
            if len(self.targets) > 400000:
                evict(self.targets)
            hit = self.targets[id(route)] = (route, frozenset(
                w.target for w in route.waypoints if w.action is not None))
        return hit[1]

    def check_time(self, stop=None):
        if time.monotonic() >= min(self.stop, stop if stop is not None else self.stop):
            raise DeadlineReached

    def reject(self, violations):
        key = violations[0].rule
        self.last_rule = key
        self.rejections[key] = self.rejections.get(key, 0) + 1
        return None

    def without(self, parent, removed):
        """Parent minus whole routes; extend() rechecks R4/R10 for the contacts they touched."""
        def keep(d):
            return {k: v for k, v in d.items() if k not in removed}
        touched = set(parent.recheck)
        for aid in removed:
            if aid in parent.parsed:
                touched |= self.route_targets(parent.parsed[aid])
        return replace(parent, routes=keep(parent.routes), parsed=keep(parent.parsed),
                       separation=keep(parent.separation), energies=keep(parent.energies),
                       recheck=frozenset(touched))

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
        checked = {aid: self.local_check(aid, route) for aid, route in extra.items()}
        failed = [hit[1] for hit in checked.values() if hit[1] is not None]
        if failed:
            # The first rule (in validator order) that any changed route breaks.
            rule = min(failed)[1]
            self.last_rule = rule
            self.rejections[rule] = self.rejections.get(rule, 0) + 1
            return None
        changed = {aid: hit[0] for aid, hit in checked.items()}
        parsed = {**parent.parsed, **changed}
        affected = set(parent.recheck)
        for aid, route in changed.items():
            affected |= self.route_targets(route)
            if aid in parent.parsed:
                affected |= self.route_targets(parent.parsed[aid])
        if affected:
            # Every route touching an affected contact: its R4/R10 verdict is exact.
            touching = {aid: route for aid, route in parsed.items()
                        if not affected.isdisjoint(self.route_targets(route))}
            for check in (rules._r4, rules._r10):
                violations = check(self.sc, touching)
                if violations:
                    return self.reject(violations)
        new_items = {aid: hit[2] for aid, hit in checked.items()}
        old_items = {aid: item for aid, item in parent.separation.items() if aid not in changed}
        pairs = itertools.chain(itertools.combinations(new_items.values(), 2),
                                itertools.product(new_items.values(), old_items.values()))
        d_sep = self.d_sep
        for n, (a, b) in enumerate(pairs):
            if n % 64 == 0:
                self.check_time()
            if a is not None and b is not None and may_be_close(a, b, d_sep):
                violation = rules.separation_violation(self.sc.rules, self.contacts, a[0], b[0])
                if violation:
                    return self.reject([violation])
        energies = dict(parent.energies)
        energies.update({aid: hit[3] for aid, hit in checked.items()})
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
        self.stop = self.start + min(seconds, MAX_SEARCH_S)
        # Variant 0 is the default search; others change random seeds and pass order.
        self.variant = variant
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
        self.lns_rng = random.Random(scenario.seed + 7919 + VARIANT_SEED_STRIDE * variant)
        self.lns_state = None
        self.weights = {op: 1.0 for op in DESTROY_OPERATORS}
        self.root = State({}, {a.id: [] for a in scenario.aircraft if a.type == SCOUT},
                          frozenset(), {}, {}, {}, frozenset(), 0.0)
        self.trace = [{"id": 0, "parent": None, "decision": "start", "score": 0.0,
                       "served": [], "depth": 0}]
        self.archive = [self.root]
        self.signatures = {self.root.signature()}
        self.matrix = None
        self.searchable = []
        # Scout ranges and, per contact, the scouts able to serve it (set_capabilities).
        self.scout_range, self.capable = {}, {}
        self.joint_stats = {}
        self._strike_earliest = {}
        self.tour_pool = []
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

    def children(self, parent, contact, stop, limit=2, insert=False, only=None):
        """Bounded candidate generation, recording how far each attempt got.

        insert=False appends the visit to a scout tour (beam search); insert=True
        also tries the cheapest earlier tour positions (LNS repair). only: a single
        scout to use (idle-scout tours and scout-first construction).
        """
        if contact.id in parent.served:
            return
        stats = self.stats(contact.id)
        stats["attempts"] += 1
        progress = {"stage": -1, "outcome": "no_options"}
        outcome = None
        try:
            yield from self._children(parent, contact, stop, limit, stats, progress, insert, only)
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

    def ranked_scouts(self, parent, contact, insert, only=None):
        """Scouts by distance from their tour (insert) or tour end (append) to the contact.

        If the first SCOUT_FANOUT are all busy, the nearest idle scout takes the last slot:
        all scouts may share a base, so an idle one is often the only one that fits.
        Scouts whose tour already visits the contact are left out: a second visit only
        costs time and energy (existing_observations() reuses the first one).
        """
        if only is not None:
            return [only]
        scouts = [a for a in _Planner(self.sc)._scout_candidates(contact, limit=len(parent.tasks))
                  if all(t["cid"] != contact.id for t in parent.tasks[a.id])]
        if insert:
            def reach(a):
                points = [a.home] + [t["point"] for t in parent.tasks[a.id]]
                return min(hdist(q, contact.pos) for q in points)
            scouts.sort(key=reach)
        else:
            scouts.sort(key=lambda a: hdist(parent.tasks[a.id][-1]["point"] if parent.tasks[a.id]
                                           else a.home, contact.pos))
        if self.capable:
            # Keep where idle scouts sit in the ranking, but choose which idle scout by fit.
            slots = [i for i, a in enumerate(scouts) if not parent.tasks[a.id]]
            for i, a in zip(slots, self.fit_order([scouts[i] for i in slots], contact)):
                scouts[i] = a
        head = scouts[:SCOUT_FANOUT]
        if head and all(parent.tasks[a.id] for a in head):
            idle = next((a for a in scouts[SCOUT_FANOUT:] if not parent.tasks[a.id]), None)
            if idle is not None:
                scouts = head[:-1] + [idle] + [a for a in scouts[SCOUT_FANOUT - 1:] if a is not idle]
        return scouts

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

    def existing_observations(self, parent, contact):
        """(scout, tasks, route) for scouts whose current route already observes the contact."""
        for sid, tasks in parent.tasks.items():
            route = parent.routes.get(sid)
            if route is not None and any(t["cid"] == contact.id for t in tasks):
                yield self.aircraft[sid], tasks, route

    def scout_options(self, parent, contact, insert, ranks=slice(0, SCOUT_FANOUT),
                      skip_append=False, only=None):
        """(scout, tasks, route or None) for each scout, visit order and flight setting.

        Orders that cannot reach the new visit in its window, or get home before the
        horizon, yield None without building a route.
        """
        p = _Planner(self.sc)
        for scout in self.ranked_scouts(parent, contact, insert, only)[ranks]:
            current = parent.tasks[scout.id]
            if any(t["cid"] == contact.id for t in current):
                continue
            by_id = {t["cid"]: t for t in current}
            prev_route = parent.routes.get(scout.id)
            for order, keeps_prefix in self.tour_orders(scout, current, contact, insert, skip_append):
                pos = order.index(contact.id)
                for z, obs_angle, shift in itertools.product(SCOUT_ALTITUDES, (0.0, 60.0, -60.0),
                                                             (0.0, 7.0)):
                    task = {"cid": contact.id,
                            "point": obs_point(self.sc, scout, contact, obs_angle, z),
                            "window": contact.window, "cpos": contact.pos}
                    tasks = [task if cid == contact.id else by_id[cid] for cid in order]
                    if not scout_reachable(self.sc, scout, tasks, pos, z,
                                           prev_route if keeps_prefix else None):
                        yield scout, tasks, None
                        continue
                    yield scout, tasks, scout_tour(self.sc, scout, tasks, z, shift)

    def _children(self, parent, contact, stop, limit, stats, progress, insert=False, only=None):
        """Bounded candidate generation: alternate scouts, teams and flight settings.

        Rounds stop at the first one that yields a child: a strike confirmed by an
        observation already in the plan (hostiles), the nearest scouts as asked
        (append or insert), then (append mode) the same scouts with insertion and
        reordering, then the next SCOUT_FALLBACK - SCOUT_FANOUT scouts with insertion.
        """
        groups = self.matrix.groups(contact, parent.used) if contact.cls == HOSTILE else [()]
        if not groups:
            self._progress(stats, progress, "no_team")
            return
        rounds = []
        if contact.cls == HOSTILE and only is None:
            rounds.append(self.existing_observations(parent, contact))
        rounds.append(self.scout_options(parent, contact, insert, only=only))
        if only is None:
            if not insert:
                rounds.append(self.scout_options(parent, contact, True, skip_append=True))
            rounds.append(self.scout_options(parent, contact, True, slice(SCOUT_FANOUT, SCOUT_FALLBACK)))
        state = {"count": 0, "seen": set()}
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
            settings = [None, "cross"] + list(itertools.product((0.0, 35.0, -35.0), range(3)))
            for delay, setting in itertools.product((0.0, 8.0, 20.0, 45.0, 90.0), settings):
                angle, rotation = setting if isinstance(setting, tuple) else (0.0, 0)
                self.checker.check_time(stop)
                arrive = base + delay
                if arrive + spacing*(len(team)-1) > contact.window[1] + EPS:
                    break
                extra = {scout.id: scout_route}
                for i, eff in enumerate(team):
                    z = EFFECTOR_LAYERS[(i + rotation) % len(EFFECTOR_LAYERS)]
                    if not isinstance(setting, tuple):
                        route, earliest = low_effector_route(
                            eff, contact, arrive + spacing*i, self.sc,
                            None if setting is None else LOW_CROSS_Z)
                    else:
                        route, earliest = _effector_route(eff, contact, arrive + spacing*i, z, angle,
                                                          self.sc)
                    if route is None:
                        break
                    extra[eff.id] = route
                if len(extra) != len(team)+1:
                    if stats is not None:
                        self._progress(stats, progress, "strike_route")
                    continue
                candidate = self.checker.extend(parent, extra)
                if candidate is None and stats is not None:
                    self._progress(stats, progress, f"strike_rule:{self.checker.last_rule}")
                if candidate is not None:
                    yield candidate, team
                    found = True
                if found:
                    break

    def set_capabilities(self):
        """Each scout's range and, per contact, the scouts able to serve it alone."""
        scouts = [a for a in self.sc.aircraft if a.type == SCOUT]
        self.scout_range = {a.id: scout_range_m(a, self.sc) for a in scouts}
        self.capable = {
            c.id: frozenset(a.id for a in scouts
                            if scout_can_serve(a, c, self.sc, self.scout_range[a.id]))
            for c in self.sc.contacts if c.cls in (HOSTILE, UNKNOWN) and c.window is not None}

    def fit_order(self, scouts, contact):
        """Idle scouts in best-fit order for a contact: the smallest range that can still
        serve it first (keeps long-range scouts free for contacts only they reach), then
        scouts that cannot serve it alone, by range."""
        if not self.scout_range:
            return list(scouts)
        capable = self.capable.get(contact.id, frozenset())
        return sorted(scouts, key=lambda a: (a.id not in capable, self.scout_range[a.id]
                                             if a.id in capable else -self.scout_range[a.id], a.id))

    def net_value(self, contact, used):
        """Estimated points minus the cheapest free team's cost; None without a team."""
        if contact.cls == UNKNOWN:
            return contact.points
        groups = self.matrix.groups(contact, used, 1)
        if not groups:
            return None
        return contact.points - sum(self.matrix.costs[a.id, contact.id] for a in groups[0])

    def grow(self, state, scout, stop, depth, kind, rng=None):
        """Greedy orienteering for one scout: repeatedly add the contact that raises the
        plan score most. Candidates are ranked by estimated net value per added kilometre
        of tour (cheapest insertion); only GROW_CANDIDATES are tried per step. rng adds
        noise to the ranking (LNS scout rebuilds explore different chains)."""
        failed = set()
        while True:
            self.checker.check_time(stop)
            points = ([scout.home[:2]] + [t["point"] for t in state.tasks[scout.id]]
                      + [scout.home[:2]])
            ranked = []
            for c in self.searchable:
                if c.id in state.served or c.id in failed:
                    continue
                value = self.net_value(c, state.used)
                if value is None or value <= 0:
                    continue
                added = min(hdist(points[i], c.pos) + hdist(c.pos, points[i + 1])
                            - hdist(points[i], points[i + 1]) for i in range(len(points) - 1))
                noise = rng.uniform(0.6, 1.4) if rng is not None else 1.0
                ranked.append((-noise * value / (1.0 + added / 1000.0), c.id, c))
            ranked.sort(key=lambda r: r[:2])
            best = None
            for _, _, contact in ranked[:GROW_CANDIDATES]:
                local_stop = min(stop, time.monotonic() + GROW_CONTACT_S)
                found = False
                try:
                    for child, decision in self.children(state, contact, local_stop, 2, True,
                                                         only=scout):
                        found = True
                        if best is None or child.score > best[0].score:
                            best = (child, decision)
                except DeadlineReached:
                    self.checker.check_time(stop)
                if not found:
                    failed.add(contact.id)
            if best is None or best[0].score <= state.score + 1e-9:
                return state, depth
            depth += 1
            state = self.record(best[0], state, {**best[1], "kind": kind}, depth)

    def fill_idle(self, state, depth, stop):
        """Give every idle scout its own greedy tour over what is still unserved."""
        idle = [a for a in self.sc.aircraft if a.type == SCOUT and not state.tasks[a.id]
                and a.id not in state.routes]
        for scout in sorted(idle, key=lambda a: (-a.usable_wh, a.id)):
            state, depth = self.grow(state, scout, stop, depth, "idle")
        return state, depth

    def construct(self, stop):
        """Scout-first construction ("sector tours"), an alternative to the beam's
        contact-first order. Valuable anchor contacts spread over different bearings
        from the scout bases each get their own scout (bigger batteries first); each
        tour then grows with contacts that are cheap to insert along its way."""
        scouts = sorted((a for a in self.sc.aircraft if a.type == SCOUT),
                        key=lambda a: (-a.usable_wh, a.id))
        if not scouts or not self.searchable:
            return
        cx = sum(a.home[0] for a in scouts) / len(scouts)
        cy = sum(a.home[1] for a in scouts) / len(scouts)
        valued = [(self.net_value(c, frozenset()), c) for c in self.searchable]
        valued = sorted(((v, c) for v, c in valued if v is not None and v > 0),
                        key=lambda vc: (-vc[0], vc[1].id))
        gap = math.pi / len(scouts)
        anchors, bearings = [], []
        for _, c in valued:
            bearing = math.atan2(c.pos[1] - cy, c.pos[0] - cx)
            if all(abs((bearing - b + math.pi) % (2 * math.pi) - math.pi) >= gap for b in bearings):
                anchors.append(c)
                bearings.append(bearing)
                if len(anchors) == len(scouts):
                    break
        state, depth = self.root, 0
        anchors.sort(key=lambda c: (-hdist((cx, cy), c.pos), c.id))
        free = list(scouts)
        for anchor in anchors:
            self.checker.check_time(stop)
            best = None
            for scout in self.fit_order(free, anchor)[:2]:
                local_stop = min(stop, time.monotonic() + GROW_CONTACT_S)
                try:
                    for child, decision in self.children(state, anchor, local_stop, 2, True,
                                                         only=scout):
                        if best is None or child.score > best[0].score:
                            best = (child, decision, scout)
                except DeadlineReached:
                    self.checker.check_time(stop)
            if best is not None:
                depth += 1
                state = self.record(best[0], state, {**best[1], "kind": "anchor"}, depth)
                free.remove(best[2])
        for scout in scouts:
            state, depth = self.grow(state, scout, stop, depth, "sector")

    def deep_repair(self, state, depth, stop, per_try=0.3):
        """Insert each unserved contact with a positive estimated net value at its best
        place over every scout and every tour position (the beam and LNS only try the
        nearest scouts and the cheapest positions). Repeats until nothing improves."""
        global FULL_INSERT_TASKS
        saved, FULL_INSERT_TASKS = FULL_INSERT_TASKS, 1000
        scouts = [a for a in self.sc.aircraft if a.type == SCOUT]
        try:
            improved = True
            while improved:
                improved = False
                ranked = []
                for c in self.searchable:
                    if c.id in state.served:
                        continue
                    value = self.net_value(c, state.used)
                    if value is not None and value > 0:
                        ranked.append((-value, c.id, c))
                for _, _, contact in sorted(ranked, key=lambda r: r[:2]):
                    self.checker.check_time(stop)
                    best = None
                    for scout in scouts:
                        local_stop = min(stop, time.monotonic() + per_try)
                        try:
                            for child, decision in self.children(state, contact, local_stop, 2, True,
                                                                 only=scout):
                                if best is None or child.score > best[0].score:
                                    best = (child, decision)
                        except DeadlineReached:
                            self.checker.check_time(stop)
                    if best is not None and best[0].score > state.score + 1e-9:
                        depth += 1
                        state = self.record(best[0], state, {**best[1], "kind": "deep"}, depth)
                        improved = True
        finally:
            FULL_INSERT_TASKS = saved
        return state, depth

    def refly(self, state, depth):
        """Rebuild existing routes with the cheaper flight profiles, one route at a time:
        strikes as low direct routes (same target and arrival time), scout tours with
        diagonal climbs at each altitude. A change is kept only if the plan stays valid,
        serves at least the same contacts and scores higher."""
        for aid in sorted(state.routes):
            self.checker.check_time()
            route = state.routes[aid]
            aircraft = self.aircraft[aid]
            options = []
            if aircraft.type == SCOUT:
                tasks = state.tasks.get(aid) or []
                if not tasks:
                    continue
                for z in SCOUT_ALTITUDES:
                    for depart in (route[0]["t"], None):
                        tour = scout_tour(self.sc, aircraft, tasks, z, depart=depart)
                        if tour is not None:
                            options.append(tour)
            else:
                last = route[-1]
                contact = self.contacts.get(last.get("target"))
                if last.get("action") != "engage" or contact is None:
                    continue
                for shift in REFLY_SHIFTS_S:
                    for cross in (None, LOW_CROSS_Z):
                        low, _ = low_effector_route(aircraft, contact, last["t"] + shift, self.sc, cross)
                        if low is not None:
                            options.append(low)
            best = None
            for new in options:
                if new == route:
                    continue
                candidate = self.checker.extend(state, {aid: new})
                if (candidate is not None and candidate.served >= state.served
                        and candidate.score > (best or state).score + 1e-9):
                    candidate.tasks = state.tasks
                    best = candidate
            if best is not None:
                depth += 1
                state = self.record(best, state, {"kind": "refly", "aircraft": aid}, depth)
        return state, depth

    def prune(self, state, depth):
        """Drop scout visits that serve nothing (repeat observations, hostiles never
        struck) when the rebuilt tour keeps every served contact and saves energy."""
        improved = True
        while improved:
            improved = False
            counts = {}
            for tasks in state.tasks.values():
                for t in tasks:
                    counts[t["cid"]] = counts.get(t["cid"], 0) + 1
            for sid, tasks in sorted(state.tasks.items()):
                route = state.routes.get(sid)
                if route is None:
                    continue
                for i, task in enumerate(tasks):
                    if task["cid"] in state.served and counts[task["cid"]] == 1:
                        continue
                    rest = tasks[:i] + tasks[i + 1:]
                    options = []
                    if not rest:
                        options.append((self.checker.without(state, {sid}), {}))
                    else:
                        for depart in (route[0]["t"], None):
                            tour = scout_tour(self.sc, self.aircraft[sid], rest, route[1]["z"],
                                              depart=depart)
                            if tour is not None:
                                options.append((state, {sid: tour}))
                    for base, extra in options:
                        self.checker.check_time()
                        candidate = self.checker.extend(base, extra)
                        if (candidate is not None and candidate.served >= state.served
                                and candidate.score > state.score + 1e-9):
                            candidate.tasks = {**state.tasks, sid: rest}
                            depth += 1
                            state = self.record(candidate, state,
                                                {"kind": "prune", "contact": task["cid"]}, depth)
                            improved = True
                            break
                    if improved:
                        break
                if improved:
                    break
        return state

    # Joint scout planning (column generation): candidate tours per scout, then the best
    # combination of one tour per scout, then strikes on the chosen observations.

    def strike_earliest(self, contact):
        """Earliest time the slowest needed effector type can reach the contact."""
        hit = self._strike_earliest.get(contact.id)
        if hit is None:
            hit = 0.0
            for sub, k in contact.needs.items():
                if k > 0:
                    hit = max(hit, min((hdist(a.home, contact.pos)
                                        / min(SCOUT_CRUISE * a.v_h_mps, best_range_speed(a.power))
                                        for a in self.sc.aircraft
                                        if a.type != SCOUT and a.subtype == sub), default=INF))
            self._strike_earliest[contact.id] = hit
        return hit

    def tour_ok(self, scout, tasks, route):
        """Route fits battery and the horizon with JOINT_SLACK_S to spare (so it can be
        delayed at take-off against R8), and every hostile can still be struck: its
        effectors reach it, after observation + confirm margin, with the team's spacing,
        before the window closes."""
        if route is None or route[-1]["t"] > self.sc.horizon_s - JOINT_SLACK_S:
            return False
        if raw_energy_wh(route, scout) > scout.usable_wh + EPS:
            return False
        rules_ = self.sc.rules
        for w in route:
            if w.get("action") == "observe":
                c = self.contacts[w["target"]]
                if c.cls != HOSTILE:
                    continue
                team = sum(k for k in c.needs.values() if k > 0)
                spacing = min(5.0, rules_.sync_window_s / max(1, team - 1)) * (team - 1)
                first = max(w["t"] + w.get("dwell_s", 0.0) + rules_.confirm_margin_s + 2.0,
                            self.strike_earliest(c))
                if first + spacing > c.window[1] - 1.0:
                    return False
        return True

    def random_tour(self, scout, values, rng, z, noise):
        """One candidate tour: randomised nearest-neighbour insertion by value per added
        kilometre, keeping each insertion that adds value net of the scout's energy."""
        lam = self.sc.rules.lambda_e
        pool = [c for c in self.searchable if values.get(c.id, 0.0) > 0
                and scout.id in self.capable.get(c.id, ())]
        tasks, route, energy, value = [], None, 0.0, 0.0
        chosen = set()
        while True:
            points = [scout.home[:2]] + [t["point"] for t in tasks] + [scout.home[:2]]
            ranked = []
            for c in pool:
                if c.id in chosen:
                    continue
                added = min(hdist(points[i], c.pos) + hdist(c.pos, points[i + 1])
                            - hdist(points[i], points[i + 1]) for i in range(len(points) - 1))
                ranked.append((-values[c.id] * rng.uniform(1.0 - noise, 1.0 + noise)
                               / (1.0 + added / 1000.0), c.id, c))
            ranked.sort(key=lambda r: r[:2])
            best = None
            for _, _, c in ranked[:JOINT_NEIGHBOURS]:
                task = {"cid": c.id, "point": obs_point(self.sc, scout, c, 0.0, z),
                        "window": c.window, "cpos": c.pos}
                for pos in insertion_positions(scout.home, tasks, c.pos, min(3, len(tasks) + 1)):
                    new = tasks[:pos] + [task] + tasks[pos:]
                    if not scout_reachable(self.sc, scout, new, pos, z):
                        continue
                    r = scout_tour(self.sc, scout, new, z)
                    if not self.tour_ok(scout, new, r):
                        continue
                    e = raw_energy_wh(r, scout)
                    gain = values[c.id] - lam * (e - energy)
                    if gain > 0 and (best is None or gain > best[0]):
                        best = (gain, new, r, e, c.id)
            if best is None:
                break
            _, tasks, route, energy, cid = best
            chosen.add(cid)
            value += values[cid]
        if not tasks:
            return None
        return {"scout": scout.id, "cids": frozenset(chosen), "tasks": tasks, "z": z,
                "value": value - lam * energy}

    def select_tours(self, columns, rng, stop):
        """At most one tour per scout, no contact twice, within the effector supply:
        randomised greedy picks plus swap improvement, best total kept."""
        supply = {}
        for a in self.sc.aircraft:
            if a.type != SCOUT:
                supply[a.subtype] = supply.get(a.subtype, 0) + 1
        need = {}
        for col in columns:
            n = {}
            for cid in col["cids"]:
                for sub, k in self.contacts[cid].needs.items():
                    if self.contacts[cid].cls == HOSTILE:
                        n[sub] = n.get(sub, 0) + k
            need[id(col)] = n

        def fits(selection, col, dropping=()):
            scouts = {c["scout"] for c in selection if c not in dropping}
            if col["scout"] in scouts:
                return False
            used = set()
            for c in selection:
                if c not in dropping:
                    used |= c["cids"]
            if used & col["cids"]:
                return False
            total = dict(need[id(col)])
            for c in selection:
                if c not in dropping:
                    for sub, k in need[id(c)].items():
                        total[sub] = total.get(sub, 0) + k
            return all(total.get(sub, 0) <= supply.get(sub, 0) for sub in total)

        best, best_total = [], 0.0
        for attempt in range(1000):
            if time.monotonic() >= stop:
                break
            noise = 0.0 if attempt == 0 else 0.3
            order = sorted(columns, key=lambda c: -c["value"] * rng.uniform(1 - noise, 1 + noise))
            selection = []
            for col in order:
                if col["value"] > 0 and fits(selection, col):
                    selection.append(col)
            improved = True
            while improved and time.monotonic() < stop:
                improved = False
                for col in columns:
                    if col in selection or col["value"] <= 0:
                        continue
                    conflicts = [c for c in selection if c["scout"] == col["scout"]
                                 or c["cids"] & col["cids"]]
                    if (col["value"] > sum(c["value"] for c in conflicts) + 1e-9
                            and fits(selection, col, conflicts)):
                        selection = [c for c in selection if c not in conflicts] + [col]
                        improved = True
            total = sum(c["value"] for c in selection)
            if total > best_total:
                best, best_total = selection, total
        return best

    def build_joint(self, chosen, stop):
        """Scout routes of the chosen tours (most valuable first), then the strikes on the
        hostiles they observe; every step is recorded in the archive."""
        state, depth = self.root, 0
        for col in sorted(chosen, key=lambda c: -c["value"]):
            new = self.place_tour(state, col, stop)
            if new is not None:
                depth += 1
                state = self.record(new, state, {"kind": "joint", "scout": col["scout"]}, depth)
        hostiles = [self.contacts[cid] for col in chosen for cid in col["cids"]
                    if self.contacts[cid].cls == HOSTILE]
        hostiles.sort(key=lambda c: (-self.matrix.priority(c), c.id))
        for contact in hostiles:
            if time.monotonic() >= stop:
                break
            self.checker.check_time()
            sid = next((s for s, ts in state.tasks.items()
                        if any(t["cid"] == contact.id for t in ts) and s in state.routes), None)
            if sid is None:
                continue
            route = state.routes[sid]
            obs_end = max((w["t"] + w.get("dwell_s", 0.0) for w in route
                           if w.get("target") == contact.id and w.get("action") == "observe"),
                          default=None)
            groups = self.matrix.groups(contact, state.used)
            if obs_end is None or not groups:
                continue
            local_stop = min(stop, time.monotonic() + 0.4)
            try:
                for candidate, team in self.strikes(state, contact, groups, self.aircraft[sid],
                                                    route, obs_end, local_stop):
                    if candidate.score > state.score:
                        candidate.tasks = state.tasks
                        depth += 1
                        state = self.record(candidate, state, {"kind": "joint-strike",
                                                               "contact": contact.id}, depth)
                        break
            except DeadlineReached:
                self.checker.check_time()
        return state

    def place_tour(self, state, col, stop):
        """State with the tour flown by its scout, or None if no altitude layer and
        take-off delay passes the validator (R8 against the routes already placed)."""
        scout = self.aircraft[col["scout"]]
        for z, shift in itertools.product(JOINT_BUILD_ALTITUDES, JOINT_BUILD_SHIFTS):
            self.checker.check_time(stop)
            route = scout_tour(self.sc, scout, col["tasks"], z, shift)
            if route is None:
                continue
            new = self.checker.extend(state, {scout.id: route})
            if new is not None:
                new.tasks = {**state.tasks, scout.id: col["tasks"]}
                return new
        return None

    def trim_tour(self, col, values):
        """The tour without its lowest-value contact, or None."""
        if len(col["tasks"]) < 2:
            return None
        scout = self.aircraft[col["scout"]]
        drop = min(col["cids"], key=lambda cid: (values.get(cid, 0.0), cid))
        tasks = [t for t in col["tasks"] if t["cid"] != drop]
        route = scout_tour(self.sc, scout, tasks, col["z"])
        if not self.tour_ok(scout, tasks, route):
            return None
        energy = raw_energy_wh(route, scout)
        cids = col["cids"] - {drop}
        return {"scout": scout.id, "cids": cids, "tasks": tasks, "z": col["z"],
                "value": sum(values.get(c, 0.0) for c in cids)
                - self.sc.rules.lambda_e * energy}

    def joint(self, stop):
        """Plan all scout tours jointly, then strike what they confirm (see above)."""
        scouts = [a for a in self.sc.aircraft if a.type == SCOUT]
        if not scouts or not self.searchable:
            return
        rng = random.Random(self.sc.seed + 31 + VARIANT_SEED_STRIDE * self.variant)
        values = {}
        for c in self.searchable:
            v = self.net_value(c, frozenset())
            if v is not None and v > 0:
                values[c.id] = v
        start = time.monotonic()
        gen_stop = start + 0.6 * max(0.0, stop - start)
        supply = {}
        for a in self.sc.aircraft:
            if a.type != SCOUT:
                supply[a.subtype] = supply.get(a.subtype, 0) + 1
        # Restarts: scouts in a random order (longest range first in the first one), each
        # touring only contacts nobody took yet, within the effector supply left. Every
        # tour joins a pool; the best complete solution and the pool feed the selection.
        columns, seen, restarts, since_better = [], set(), 0, 0
        top = []  # (estimate, signature, tours) of the best distinct restart solutions
        best_solution, best_total = [], 0.0
        while time.monotonic() < gen_stop:
            self.checker.check_time(stop)
            order = list(scouts)
            if restarts == 0:
                order.sort(key=lambda a: (-self.scout_range.get(a.id, 0.0), a.id))
            else:
                rng.shuffle(order)
            left = dict(supply)
            free = dict(values)
            solution = []
            for scout in order:
                if time.monotonic() >= gen_stop:
                    break
                for cid in list(free):
                    c = self.contacts[cid]
                    if c.cls == HOSTILE and any(left.get(sub, 0) < k for sub, k in c.needs.items()):
                        del free[cid]
                col = self.random_tour(scout, free, rng, SCOUT_ALTITUDES[0],
                                       0.0 if restarts == 0 else JOINT_NOISE)
                if col is None:
                    continue
                solution.append(col)
                for cid in col["cids"]:
                    free.pop(cid, None)
                    c = self.contacts[cid]
                    if c.cls == HOSTILE:
                        for sub, k in c.needs.items():
                            left[sub] = left.get(sub, 0) - k
                key = (col["scout"], tuple(t["cid"] for t in col["tasks"]))
                if key not in seen:
                    seen.add(key)
                    columns.append(col)
            total = sum(c["value"] for c in solution)
            key = frozenset((c["scout"], c["cids"]) for c in solution)
            if solution and key not in {k for _, k, _ in top}:
                top.append((total, key, solution))
                top.sort(key=lambda t: -t[0])
                del top[JOINT_BUILDS:]
            if total > best_total + 1e-9:
                best_solution, best_total = solution, total
                since_better = 0
            else:
                since_better += 1
            restarts += 1
            if since_better >= JOINT_PATIENCE:
                # Converged: hand the rest of the time (selection, build, beam) back.
                stop = min(stop, time.monotonic() + 0.4 * max(0.0, stop - start))
                break
        # Kept for the opt-in column-swap LNS (column_lns.py).
        self.tour_pool = columns
        self.joint_stats = {"restarts": restarts, "columns": len(columns),
                            "best_restart": round(best_total, 1)}
        chosen = self.select_tours(columns, rng, start + 0.7 * max(0.0, stop - start))
        # Estimates miss take-off conflicts and strike timing by 50-150 points per L4, in
        # varying amounts: build the best few distinct candidates and keep the best result.
        candidates = [chosen] + [sol for _, _, sol in top]
        unique, keys = [], set()
        for cand in sorted(candidates, key=lambda c: -sum(t["value"] for t in c)):
            key = frozenset((t["scout"], t["cids"]) for t in cand)
            if cand and key not in keys:
                keys.add(key)
                unique.append(cand)
        unique = unique[:JOINT_BUILDS]
        self.joint_stats["chosen"] = len(unique[0]) if unique else 0
        self.joint_stats["estimate"] = round(sum(t["value"] for t in unique[0]), 1) if unique else 0.0
        state, depth, built = self.root, 0, []
        # The builds share 70% of what is left; the repair and cleanup get the rest.
        builds_end = time.monotonic() + 0.7 * max(0.0, stop - time.monotonic())
        for i, cand in enumerate(unique):
            # The top candidate is built fully; the others only use time left over.
            build_stop = builds_end
            if i > 0 and builds_end - time.monotonic() < 2.0:
                break
            try:
                result = self.build_joint(cand, build_stop)
            except DeadlineReached:
                self.checker.check_time(stop)
                break
            built.append(round(result.score, 1))
            if result.score > state.score:
                state = result
        depth = len(state.served)
        self.joint_stats["builds"] = built
        self.joint_stats["built"] = round(state.score, 1)
        # Contacts of tours that did not fit (R8) go to whichever scout can take them.
        try:
            state = self.repair(state, depth, stop)
        except DeadlineReached:
            pass
        try:
            state = self.prune(state, depth)
        except DeadlineReached:
            pass
        self.joint_stats["repaired"] = round(state.score, 1)

    def repair(self, state, depth, stop=None):
        """Greedy fill of what the search left unserved: unknowns join any scout tour at
        their cheapest positions, then hostiles retry with the free effectors.
        stop: optional earlier deadline (returns the state reached so far)."""
        stop = self.checker.stop if stop is None else min(stop, self.checker.stop)
        improved = True
        while improved:
            improved = False
            contacts = [c for c in self.sc.contacts if c.cls in (HOSTILE, UNKNOWN)
                        and c.window is not None and c.id not in self.matrix.banned
                        and c.id not in state.served]
            contacts.sort(key=lambda c: (c.cls != UNKNOWN, -self.matrix.priority(c), c.id))
            for contact in contacts:
                self.checker.check_time()
                if time.monotonic() >= stop:
                    return state
                local_stop = min(stop, time.monotonic() + 0.25)
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
            if len(self.trace) >= TRACE_CAP:
                return False
        return True

    def run(self):
        contacts, priorities = self.search_phase()
        self.improve_phase(contacts, priorities)
        return self.finalize()

    def search_phase(self):
        """Warm start plus beam passes until the beam deadline.
        Returns the searchable contacts and their priorities."""
        try:
            self.warm_start()
        except DeadlineReached:
            pass
        self.matrix = CandidateMatrix(self.sc)
        self.set_capabilities()
        contacts = [c for c in self.sc.contacts if c.cls in (HOSTILE, UNKNOWN)
                    and c.window is not None and c.id not in self.matrix.banned]
        priorities = {c.id: self.matrix.priority(c) for c in contacts}
        self.searchable = contacts
        rng = random.Random(self.sc.seed + VARIANT_SEED_STRIDE * self.variant)
        self.checker.stop = min(self.beam_stop, self.stop)
        try:
            self.joint(time.monotonic()
                       + JOINT_SHARE * max(0.0, self.checker.stop - time.monotonic()))
        except DeadlineReached:
            pass
        try:
            self.construct(time.monotonic()
                           + CONSTRUCT_SHARE * max(0.0, self.checker.stop - time.monotonic()))
        except DeadlineReached:
            pass
        try:
            self.search(contacts, priorities, rng)
        except DeadlineReached:
            pass
        self.checker.stop = self.stop
        return contacts, priorities

    def improve_phase(self, contacts, priorities):
        """Cleanup, idle-scout tours and greedy repair of the best plan, then LNS until
        shortly before the deadline, then cleanup of the best plans' tours."""
        repair_stop = self.beam_stop + REPAIR_SHARE * max(0.0, self.stop - self.beam_stop)
        self.checker.stop = repair_stop
        try:
            if contacts:
                best = self.archive[0]
                depth = max((n["depth"] for n in self.trace if n["id"] == best.node_id), default=0)
                state = self.prune(best, depth)
                state, depth = self.refly(state, depth)
                state, depth = self.fill_idle(state, depth, repair_stop)
                self.repair(state, depth)
        except DeadlineReached:
            pass
        reserve = POLISH_RESERVE_S if self.stop - self.start > 5.0 else 0.0
        self.checker.stop = self.stop - reserve
        try:
            self.improve(contacts, priorities, self.stop - reserve)
        except DeadlineReached:
            pass
        self.checker.stop = self.stop
        try:
            for state in list(self.archive[:3]):
                self.refly(self.prune(state, 0), 0)
        except DeadlineReached:
            pass

    def finalize(self):
        # Official full validation is the authority; archive can include partial
        # plans because every scout already has its return-and-land leg.
        verified = []
        for state in self.archive:
            if time.monotonic() >= self.stop + 3.0:
                break
            result = evaluate(self.sc, state.plan(self.sc.seed))
            if result.valid:
                verified.append((state, result))
        if not verified:
            verified = [(self.root, evaluate(self.sc, self.root.plan(self.sc.seed)))]
        verified.sort(key=lambda pair: -pair[1].score)
        return verified

    def rebuild_scout(self, state, scout, rng, stop):
        """Regrow an emptied scout tour: first an anchor drawn by value per capable scout
        (contacts few scouts can reach are this scout's natural anchors), then greedy
        growth with a noisy ranking."""
        anchors = []
        for c in self.searchable:
            capable = self.capable.get(c.id, frozenset())
            if c.id in state.served or scout.id not in capable:
                continue
            value = self.net_value(c, state.used)
            if value is not None and value > 0:
                anchors.append((-value / len(capable) * rng.uniform(0.5, 1.5), c.id, c))
        for _, _, anchor in sorted(anchors)[:4]:
            best_child = None
            local_stop = min(stop, time.monotonic() + 0.4)
            try:
                for child, _ in self.children(state, anchor, local_stop, 2, True, only=scout):
                    if best_child is None or child.score > best_child.score:
                        best_child = child
            except DeadlineReached:
                self.checker.check_time(stop)
            if best_child is not None:
                state = best_child
                break
        state, _ = self.grow(state, scout, stop, 0, "rebuild", rng)
        return state

    def clear_scout(self, state, sid):
        """State without scout sid's tour and without the strikes on the hostiles it
        observes (remove() may keep a hostile's observation when a rebuilt tour fails;
        a scout rebuild needs the scout really free). None if no valid state results."""
        observed = {t["cid"] for t in state.tasks.get(sid, [])}
        removed = {sid} if sid in state.routes else set()
        for aid, route in state.parsed.items():
            if any(w.action == "engage" and w.target in observed for w in route.waypoints):
                removed.add(aid)
        candidate = self.checker.extend(self.checker.without(state, removed), {})
        if candidate is None:
            return None
        candidate.tasks = {**state.tasks, sid: []}
        return candidate

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
            while st["since_best"] < LNS_PATIENCE:
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
        if rng.random() < SCOUT_REBUILD_P:
            # Scout LNS: clear 1-3 scout tours together (with the strikes they confirm) and
            # rebuild them jointly, longest range first, each from an anchor that few scouts
            # can reach. Their old contacts are offered back to every scout below, so a
            # long-range scout can move to a far cluster while others take its old work.
            op = "rebuild"
            scouts = [a for a in self.sc.aircraft if a.type == SCOUT]
            weights = {a.id: 1.0 + sum(1.0 / len(caps) for cid, caps in self.capable.items()
                                       if a.id in caps and cid not in current.served
                                       and cid not in self.matrix.banned)
                       for a in scouts}
            group = []
            for _ in range(rng.choice(SCOUT_REBUILD_SIZES)):
                pool = [a for a in scouts if a not in group]
                if not pool:
                    break
                group.append(rng.choices(pool, weights=[weights[a.id] for a in pool])[0])
            for scout in group:
                cleared = self.clear_scout(partial, scout.id)
                if cleared is not None:
                    partial = cleared
            removed = [cid for cid in current.served if cid not in partial.served and cid in by_id]
            rebuild_stop = min(self.checker.stop,
                               time.monotonic() + SCOUT_REBUILD_S * len(group))
            try:
                for scout in sorted(group, key=lambda a: (-self.scout_range.get(a.id, 0.0), a.id)):
                    partial = self.rebuild_scout(partial, scout, rng, rebuild_stop)
            except DeadlineReached:
                self.checker.check_time()
            removed = [cid for cid in removed if cid not in partial.served]
        elif served:
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
        if self.adaptive and op in self.weights:
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
                               "served": result.served} for i, (state, result) in enumerate(verified, 1)],
                "nodes": self.trace}