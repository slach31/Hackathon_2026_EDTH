"""Deadline-bounded beam search using official baseline routes and rule checks.

Nodes are valid, complete-to-home partial fleet plans. No route is edited in place.
The matrix ranks candidates; only official rule checks establish validity.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
import heapq
import itertools
import math
import random
import time

from fleetkit import baseline
from fleetkit.baseline import (_Planner, _effector_route, _scout_route, EFFECTOR_LAYERS,
                               SCOUT_LAYERS, SPEED_FACTOR)
from fleetkit.energy import best_range_speed, route_energy_wh
from fleetkit.geometry import EPS, dist3, hdist, segment_enters_cylinder
from fleetkit.model import EFFECTOR, HOSTILE, SCOUT, UNKNOWN, parse_plan
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


DESTROY_OPERATORS = ("random", "related", "worst", "tour", "trip", "group", "strike")


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
# and dearer. R2 allows v_h exactly; 0.995 keeps a 0.5% margin (waypoint times are
# rounded to 1 ms, legs last tens of seconds).
SCOUT_CRUISE = 0.995
# "fast": every scout leg at SCOUT_CRUISE * v_h. "eco": the kit's speed. "hybrid": the kit's
# speed, falling back to fast when the eco tour misses a window, the horizon or the battery.
# Set per run by Search(scout_speed=...); planner.py --scout-speed.
SCOUT_SPEEDS = ("fast", "eco", "hybrid")
SCOUT_SPEED_MODE = "fast"
# Contacts are on the ground; the 3D observation range allows a horizontal offset of
# sqrt(R_obs^2 - z^2): ~133 m at 70 m, ~102 m at 110 m. The kit always uses 100 m.
OBS_MARGIN_M = 2.0
# Set per run by Search(obs_stretch=...); planner.py --obs-stretch / --no-obs-stretch.
OBS_STRETCH = True


# Climb along the first leg and descend along the last one instead of vertically over the
# pad. R2 times a leg by max(horizontal / v_h, climb / v_up), so the climb overlaps the
# cruise: no hover power during a vertical climb (p0 for ~5-25 s per aircraft) and, for
# scouts leaving at t = 0, every visit that many seconds earlier. Falls back to the
# vertical climb when the diagonal leg would cross a zone or protected area, or is too
# short to hide the climb. Off: the kit's vertical climb and descent.
DIAGONAL_CLIMB = True
# Effector cruise speed as a share of v_h. The kit flies min(0.95 v_h, best-range speed),
# but on L3/L4 the best-range speed of nearly every effector is above 0.95 v_h, so
# flying closer to v_h is both earlier and cheaper per metre. R2 allows v_h exactly.
EFFECTOR_CRUISE = 0.995
# Effector cruise layer sets, tried in order for a strike team; member i flies at
# layers[i % len(layers)]. First the whole team at the 30 m floor: team members arrive a
# few seconds apart, in trail, and less climb saves energy (e_climb x 10-100 m each). Then
# the kit's three rotations of 40/85/130 m when the low team breaks separation.
EFFECTOR_Z = ((30.0,),) + tuple(tuple(EFFECTOR_LAYERS[(i + r) % len(EFFECTOR_LAYERS)]
                                      for i in range(len(EFFECTOR_LAYERS)))
                                for r in range(len(EFFECTOR_LAYERS)))
# Ground-level strikes. R2 bounds the altitude of waypoints after the first and before the
# last only, and R10 needs the engagement within R_eng (3D) of the contact on the ground,
# so a two-waypoint effector route (home, engagement) may fly the straight line at z = 0:
# no climb energy (e_climb x 30 m: 0.3-0.6 Wh per light, 0.9-1.5 Wh per heavy), no climb
# time and no approach detour. Tried first (layer GROUND_Z, angle 0 only) for every team;
# a member whose straight ground line enters a zone or protected area (R6) flies the 30 m
# route instead. Off: the 30 m team first, as before.
GROUND_STRIKES = True
GROUND_Z = 0.0
# Clearance (m) kept from zone and protected-area cylinders by a ground route: waypoints
# are rounded to 1 mm.
GROUND_CLEARANCE_M = 1.0


# Strike team arrival order. Members arrive `spacing` s apart in team order; on a shared
# line from the common base a faster member arriving after a slower one closes on it and
# breaks separation, pushing the team up to 30/40-130 m. Slowest first: gaps only grow.
SLOWEST_FIRST = True


def arrival_order(team):
    """Team members in the order they arrive at the target."""
    if not SLOWEST_FIRST:
        return tuple(team)
    return tuple(sorted(team, key=lambda a: (a.v_h_mps, a.id)))


def effector_layer_sets():
    """Layer sets tried for a strike team, in order."""
    return (((GROUND_Z,),) + EFFECTOR_Z) if GROUND_STRIKES else EFFECTOR_Z


_cyl = {}


def clear_leg(scenario, a, b):
    """True if the straight leg a-b enters no zone or protected area (R6)."""
    hit = _cyl.get(id(scenario))
    if hit is None or hit[0] is not scenario:
        hit = _cyl[id(scenario)] = (scenario, baseline._cylinders(scenario))
    return not any(segment_enters_cylinder(a, b, c, r, top) for c, r, top in hit[1])


def climb_path(scenario, path, distance, end=False):
    """path (horizontal at z, from over the pad when not end, to over the pad when end)
    with a waypoint `distance` along its first (last) leg, where the climb (descent) ends;
    None when the leg is shorter or the climbing piece enters a zone or protected area."""
    a, b = (path[-1], path[-2]) if end else (path[0], path[1])
    seg = hdist(a, b)
    if seg < distance + 1e-6:
        return None
    f = distance / seg
    mid = (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2])
    ground = (a[0], a[1], 0.0)
    if not (clear_leg(scenario, mid, ground) if end else clear_leg(scenario, ground, mid)):
        return None
    return path[:-1] + [mid, path[-1]] if end else [path[0], mid] + path[1:]


_effector_shapes = {}


def effector_route(eff, contact, arrive, z, angle, scenario):
    """The kit's _effector_route, climbing along the first leg (DIAGONAL_CLIMB).

    The flight path depends only on (effector, contact, layer, angle); it is built once
    and only the times are placed for each arrival."""
    if z == GROUND_Z:
        if angle != 0.0:
            return None, arrive
        route = ground_route(eff, contact, arrive, scenario)
        if route is not None:
            return route
        z = 30.0
    if not DIAGONAL_CLIMB:
        return _effector_route(eff, contact, arrive, z, angle, scenario)
    key = (id(scenario), eff.id, contact.id, z, angle, EFFECTOR_CRUISE)
    shape = _effector_shapes.get(key)
    if shape is None or shape[0] is not scenario:
        if len(_effector_shapes) > 200000:
            _effector_shapes.clear()
        shape = _effector_shapes[key] = (scenario, _effector_shape(eff, contact, z, angle, scenario))
    shape = shape[1]
    if shape is None:
        return None, arrive
    if shape == "kit":
        return _effector_route(eff, contact, arrive, z, angle, scenario)
    home, inner, a_pt, e_pt, t_cruise, t_final = shape
    earliest = t_cruise + t_final
    if arrive < earliest:
        return None, earliest
    t_a = arrive - t_final
    t_o = t_a - t_cruise
    wps = [baseline._wp(t_o, home)]
    for frac, point in inner:
        wps.append(baseline._wp(t_o + t_cruise * frac, point))
    wps.append(baseline._wp(t_a, a_pt))
    wps.append(baseline._wp(arrive, e_pt, action="engage", target=contact.id))
    return wps, earliest


_ground_clear = {}


def ground_route(eff, contact, arrive, scenario):
    """(route, earliest arrival) for the two-waypoint route home -> engagement at the
    contact, flown at ground level; None when the straight line is not clear (R6)."""
    key = (id(scenario), eff.id, contact.id)
    hit = _ground_clear.get(key)
    if hit is None or hit[0] is not scenario:
        home, end = eff.home, contact.pos
        cylinders = baseline._cylinders(scenario)
        clear = not any(segment_enters_cylinder(home, end, c, r + GROUND_CLEARANCE_M, top)
                        for c, r, top in cylinders)
        hit = _ground_clear[key] = (scenario, clear)
    if not hit[1]:
        return None
    earliest = hdist(eff.home, contact.pos) / (EFFECTOR_CRUISE * eff.v_h_mps)
    if arrive < earliest:
        return None, earliest
    return [baseline._wp(arrive - earliest, eff.home),
            baseline._wp(arrive, contact.pos, action="engage", target=contact.id)], earliest


def _effector_shape(eff, contact, z, angle, scenario):
    """(home, [(share of cruise time, point)], approach, engage point, cruise s, final s),
    None when no path exists, or "kit" for the kit's vertical climb."""
    home = eff.home
    reach = min(baseline.APPROACH_M, 0.5 * hdist(home, contact.pos))
    ux, uy = baseline._toward_home(contact, home, angle)
    a_pt = (contact.pos[0] + ux * reach, contact.pos[1] + uy * reach, z)
    e_pt = (contact.pos[0], contact.pos[1], baseline.ENGAGE_Z)
    path = _cached_detour([(home[0], home[1], z), a_pt], scenario)
    if path is None:
        return None
    v_h = EFFECTOR_CRUISE * eff.v_h_mps
    t_final = max(hdist(a_pt, e_pt) / v_h,
                  (z - baseline.ENGAGE_Z) / (SPEED_FACTOR * eff.v_down_mps)) + 0.2
    v_cruise = min(v_h, best_range_speed(eff.power))
    total = baseline._length(path)
    path = climb_path(scenario, path, v_cruise * (z / (SPEED_FACTOR * eff.v_up_mps) + 0.5))
    if path is None:
        return "kit"
    inner = [(baseline._length(path[:k + 1]) / total, path[k]) for k in range(1, len(path) - 1)]
    return home, inner, a_pt, e_pt, total / v_cruise, t_final


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


def build_scout_route(scout, tasks, depart, z, scenario, speed, diagonal=None):
    """The kit's _scout_route with an explicit cruise speed (same waypoints at the kit speed
    without DIAGONAL_CLIMB). Diagonal: the first leg climbs and the last descends."""
    if diagonal is None:
        diagonal = DIAGONAL_CLIMB and bool(tasks)
    rules_ = scenario.rules
    detour, length, wp = baseline._detour, baseline._length, baseline._wp
    home = scout.home
    above = (home[0], home[1], z)
    t = depart
    wps = [wp(t, home)]
    if not diagonal:
        t += z / (SPEED_FACTOR * scout.v_up_mps) + 0.5
        wps.append(wp(t, above))
    cur = above
    for n, task in enumerate(tasks):
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
        if diagonal and n == 0:
            path = climb_path(scenario, path, speed * (z / (SPEED_FACTOR * scout.v_up_mps) + 0.5))
            if path is None:
                return build_scout_route(scout, tasks, depart, z, scenario, speed, False)
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
    if diagonal:
        path = climb_path(scenario, path, speed * (z / (SPEED_FACTOR * scout.v_down_mps) + 0.5),
                          end=True)
        if path is None:
            return build_scout_route(scout, tasks, depart, z, scenario, speed, False)
    for k in range(1, len(path) - 1):
        wps.append(wp(t + (arrive - t) * length(path[:k + 1]) / total, path[k]))
    if diagonal:
        wps.append(wp(arrive, home))
    else:
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
    climb = 0.0 if DIAGONAL_CLIMB else z / scout.v_up_mps
    travel = hdist(scout.home, tasks[0]["point"]) / speed + climb + 3.0
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
    key = (id(scenario), scout.id, z, shift, depart, SCOUT_SPEED_MODE, OBS_STRETCH, DIAGONAL_CLIMB,
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
    # Diagonal: the climb and descent overlap the cruise legs (lower bound 0).
    climb = 0.0 if DIAGONAL_CLIMB else z / (SPEED_FACTOR * scout.v_up_mps) + 0.5
    descend = 0.0 if DIAGONAL_CLIMB else z / (SPEED_FACTOR * scout.v_down_mps) + 0.5
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


def takeoff_need(scenario, a, b, level=False):
    """Seconds two takeoffs from these pads must be apart, or None if far enough.
    level: both leave the pad climbing along a cruise leg, so the later one trails the
    earlier at cruise speed instead of rising beneath it at climb speed."""
    d_sep = scenario.rules.d_sep_m + 2.0
    gap = hdist(a.home, b.home)
    if gap >= d_sep:
        return None
    if level:
        rate = SPEED_FACTOR * min(a.v_h_mps, b.v_h_mps)
    else:
        rate = SPEED_FACTOR * min(a.v_up_mps, b.v_up_mps)
    return math.sqrt(max(0.0, d_sep * d_sep - gap * gap)) / rate + 1.0


def _diagonal(route):
    return len(route) > 1 and (route[1]["x"], route[1]["y"]) != (route[0]["x"], route[0]["y"])


_neighbours = {}


def pad_neighbours(scenario, aircraft, aid):
    """{other id: (gap needed vertically, gap needed level)} for pads closer than d_sep."""
    hit = _neighbours.get(id(scenario))
    if hit is None or hit[0] is not scenario:
        table = {}
        for a in aircraft.values():
            table[a.id] = {b.id: (takeoff_need(scenario, a, b), takeoff_need(scenario, a, b, True))
                           for b in aircraft.values()
                           if b.id != a.id and takeoff_need(scenario, a, b) is not None}
        hit = _neighbours[id(scenario)] = (scenario, table)
    return hit[1].get(aid, {})


def pad_conflict(scenario, aircraft, routes, extra):
    """True if a new route takes off too close in time to a neighbouring pad's takeoff."""
    for aid, route in extra.items():
        level = _diagonal(route)
        for oid, needs in pad_neighbours(scenario, aircraft, aid).items():
            other = extra[oid] if oid in extra else routes.get(oid)
            if not other:
                continue
            need = needs[1] if level and _diagonal(other) else needs[0]
            if abs(route[0]["t"] - other[0]["t"]) < need:
                return True
    return False


def takeoff_shift(scenario, aircraft, routes, scout, depart, limit=40.0, step=2.0):
    """Smallest delay so the scout's climb stays clear of takeoffs on neighbouring pads.
    Landings and later legs are left to the validator."""
    others = []
    for aid, needs in pad_neighbours(scenario, aircraft, scout.id).items():
        route = routes.get(aid)
        if not route:
            continue
        need = needs[1] if DIAGONAL_CLIMB and _diagonal(route) else needs[0]
        others.append((route[0]["t"], need))
    shift = 0.0
    while shift <= limit:
        if all(abs(depart + shift - t0) >= need for t0, need in others):
            return shift
        shift += step
    return 0.0


# Rank scouts by added cruise energy (added distance x Wh per metre at the tour speed)
# instead of added distance. Scout efficiency varies by about +-30% within a scenario
# (L2-202: 7.9 vs 10.4-10.7 J/m), and ranking by distance alone often leaves the
# cheapest scout idle; hand-built plans with the efficient scouts score +10 on L2-202.
SCOUT_ENERGY_RANK = True


def scout_wh_per_m(scout):
    """Cruise energy per metre (Wh) at the speed scout tours fly."""
    v = eco_speed(scout) if SCOUT_SPEED_MODE == "eco" else fast_speed(scout)
    return (scout.power.p0_w + scout.power.k_v * v * v) / v / 3600.0


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
    # Diagonal: the climb and descent overlap the cruise legs (lower bound: no extra time).
    climb = 0.0 if DIAGONAL_CLIMB else z / (SPEED_FACTOR * scout.v_up_mps) + 0.5
    home = scout.home[:2]
    first = max(0.0, hdist(home, legs[0][0]) - legs[0][2])
    lead = 0.0 if DIAGONAL_CLIMB else z / scout.v_up_mps
    t = max(0.0, legs[0][1][0] - (first / speed + lead + 3.0)) + climb
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
    descent = 0.0 if DIAGONAL_CLIMB else z / (SPEED_FACTOR * scout.v_down_mps) + 0.5
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


# Scout trip label search (LNS operator "trip"): one scout's whole sortie rebuilt as the
# best-value chain of contacts it can observe in their windows and still get home within
# battery and horizon. Off: the LNS never picks the "trip" operator.
TRIP_SEARCH = True
TRIP_SHARE = 0.2        # share of LNS steps using "trip" (without --adaptive)
TRIP_LABELS = 6         # labels kept per contact (best by value)
TRIP_BATTERY = 0.995    # usable battery share the straight-leg estimate may use
TRIP_STRIKE_SLACK = 3.0  # seconds kept after confirm_margin + 2 s before a strike window end


_legs = {}


def leg_length(scenario, a, b, z, exact=True):
    """Horizontal length of the scout leg a-b at layer z as the tour builder flies it
    (straight, or the kit's detour around zones); inf when no detour exists.
    exact=False: the straight length (a lower bound) without computing detours."""
    if not exact:
        return hdist(a, b)
    key = (id(scenario), a[0], a[1], b[0], b[1], z)
    hit = _legs.get(key)
    if hit is None or hit[0] is not scenario:
        p, q = (a[0], a[1], z), (b[0], b[1], z)
        if clear_leg(scenario, p, q):
            d = hdist(p, q)
        else:
            path = _cached_detour([p, q], scenario)
            d = math.inf if path is None else baseline._length(path)
        if len(_legs) > 200000:
            _legs.clear()
        hit = _legs[key] = (scenario, d)
    return hit[1]


def trip_labels(scenario, scout, cands, z, keep=TRIP_LABELS, top=30, orders=(), cover=False,
                skip=0, rows=None, exact=True):
    """Best single-sortie trips for `scout` by resource-constrained label setting.

    cands: [(cid, point, (lo, hi), net value, latest finish)]. A label is a chain of visits
    ending at one candidate: (finish time, Wh so far, net value, visited mask). Legs are
    straight, at the scout's cruise speed, with tour_estimate's departure rule, waiting and
    energy model (lower bounds of scout_tour). Labels are dropped when dominated at the same
    contact (earlier or equal finish, less Wh, more value, subset of visits) or when the
    contact already keeps `keep` better ones. `orders`: extra visit orders (contact ids) to
    evaluate the same way, e.g. the current tour.

    Returns up to `top` trips [(value, order, Wh, finishes)] with distinct contact sets,
    best first; value = sum of net values - lambda_e * scout energy (home to home).
    cover: also the best trip found that includes each candidate (beyond `top`), so
    trip selection across scouts can trade contacts. skip: bit mask of candidates to
    leave out. rows: a dict reused across calls with the same cands (leg lengths).
    exact=False: straight legs, no detours (faster; realisation checks the real tour).
    """
    rules_, p = scenario.rules, scout.power
    speed = eco_speed(scout) if SCOUT_SPEED_MODE == "eco" else fast_speed(scout)
    home, dwell, lam = scout.home[:2], rules_.obs_dwell_s, rules_.lambda_e
    cap, horizon = TRIP_BATTERY * scout.usable_wh, scenario.horizon_s
    cruise_w = p.p0_w + p.k_v * speed * speed
    descent = 0.0 if DIAGONAL_CLIMB else z / (SPEED_FACTOR * scout.v_down_mps) + 0.5
    lead = 0.0 if DIAGONAL_CLIMB else z / scout.v_up_mps
    climb = 0.0 if DIAGONAL_CLIMB else z / (SPEED_FACTOR * scout.v_up_mps) + 0.5
    start_wh = p.e_climb_wh_per_m * z + p.p0_w * climb / 3600.0
    n = len(cands)
    if rows is not None and "back" in rows:
        back_d = rows["back"]
    else:
        back_d = [leg_length(scenario, c[1], home, z, exact) for c in cands]
    back_t = [d / speed + descent for d in back_d]
    back_wh = [(cruise_w * d / speed + p.p0_w * descent) / 3600.0 for d in back_d]
    dwell_wh = p.p0_w * dwell / 3600.0
    index = {c[0]: j for j, c in enumerate(cands)}

    def leg(t, wh, d, j):
        _, _, (lo, _), _, latest = cands[j]
        arrive = max(t + d / speed, lo)
        finish = arrive + dwell
        if finish > latest + EPS or finish + back_t[j] > horizon + EPS:
            return None
        dt = arrive - t
        if dt > 0:
            v = d / dt
            wh += (p.p0_w + p.k_v * v * v) * dt / 3600.0
        wh += dwell_wh
        if wh + back_wh[j] > cap:
            return None
        return finish, wh

    def first(j):
        d = rows["out"][j]
        if d == math.inf:
            return None
        lo = cands[j][2][0]
        # _depart times the straight line; a detour only makes the arrival later.
        depart = max(0.0, lo - (hdist(home, cands[j][1]) / speed + lead + 3.0)) + climb
        return leg(depart, start_wh, d, j)

    rows = {} if rows is None else rows

    def dist(j):
        row = rows.get(j)
        if row is None:
            row = rows[j] = [leg_length(scenario, cands[j][1], b[1], z, exact) for b in cands]
        return row
    labels = []          # [finish, wh, net, mask, node, pred, alive]
    buckets = [[] for _ in range(n)]
    heap = []

    def push(finish, wh, net, mask, j, pred):
        value = net - lam * wh
        bucket = buckets[j]
        for o in bucket:
            if (o[0] <= finish + EPS and o[1] <= wh + 1e-9 and o[2] - lam * o[1] >= value - 1e-9
                    and o[3] & ~mask == 0):
                return
        kept = []
        for o in bucket:
            if (finish <= o[0] + EPS and wh <= o[1] + 1e-9 and value >= o[2] - lam * o[1] - 1e-9
                    and mask & ~o[3] == 0):
                o[6] = False
            else:
                kept.append(o)
        if len(kept) >= keep:
            worst = min(kept, key=lambda o: o[2] - lam * o[1])
            if worst[2] - lam * worst[1] >= value:
                buckets[j] = kept
                return
            worst[6] = False
            kept.remove(worst)
        label = [finish, wh, net, mask, j, pred, True]
        kept.append(label)
        buckets[j] = kept
        labels.append(label)
        heapq.heappush(heap, (finish, len(labels) - 1))

    if "back" not in rows:
        rows["back"] = [leg_length(scenario, c[1], home, z, exact) for c in cands]
        rows["out"] = [leg_length(scenario, home, c[1], z, exact) for c in cands]
    for j in range(n):
        if skip >> j & 1:
            continue
        got = first(j)
        if got is not None:
            push(got[0], got[1], cands[j][3], 1 << j, j, None)
    while heap:
        _, i = heapq.heappop(heap)
        label = labels[i]
        if not label[6]:
            continue
        t, wh, net, mask, j = label[:5]
        row = dist(j)
        for k in range(n):
            if (mask | skip) >> k & 1 or row[k] == math.inf:
                continue
            got = leg(t, wh, row[k], k)
            if got is not None:
                push(got[0], got[1], net + cands[k][3], mask | 1 << k, k, label)

    def chain(label):
        out = []
        while label is not None:
            out.append(label)
            label = label[5]
        return out[::-1]

    trips = {}
    covering = {}       # candidate index -> (value, mask) of the best trip including it
    for label in labels:
        value = label[2] - lam * (label[1] + back_wh[label[4]])
        best = trips.get(label[3])
        if best is None or value > best[0]:
            path = chain(label)
            trips[label[3]] = (value, tuple(cands[o[4]][0] for o in path), label[1] + back_wh[label[4]],
                               tuple(o[0] for o in path))
        if cover:
            mask, j = label[3], 0
            while mask:
                if mask & 1 and (j not in covering or value > covering[j][0]):
                    covering[j] = (value, label[3])
                mask >>= 1
                j += 1
    for order in orders:
        if not order or any(cid not in index for cid in order):
            continue
        js = [index[cid] for cid in order]
        got, net, finishes = first(js[0]), cands[js[0]][3], []
        for a, b in zip([None] + js, js):
            if a is not None and got is not None:
                got = None if dist(a)[b] == math.inf else leg(got[0], got[1], dist(a)[b], b)
                net += cands[b][3]
            if got is None:
                break
            finishes.append(got[0])
        if got is None:
            continue
        mask = sum(1 << j for j in js)
        value = net - lam * (got[1] + back_wh[js[-1]])
        if mask not in trips or value > trips[mask][0]:
            trips[mask] = (value, tuple(order), got[1] + back_wh[js[-1]], tuple(finishes))
    out = sorted(trips.values(), key=lambda tr: (-tr[0], tr[1]))[:top]
    if cover:
        have = {frozenset(tr[1]) for tr in out}
        for _, mask in sorted(covering.values(), key=lambda vm: -vm[0]):
            tr = trips[mask]
            if frozenset(tr[1]) not in have:
                have.add(frozenset(tr[1]))
                out.append(tr)
    return out


# Trip plan warm start: top trips per scout from the label search, at most one per scout
# and no contact twice (weighted set packing), realised before the beam passes.
TRIP_PLAN = True
TRIP_PLAN_TRIPS = 40     # candidate trips per scout and layer
TRIP_PLAN_SHARE = 0.2    # at most this share of the beam phase (start to beam end)
TRIP_PLAN_ROUNDS = 2     # re-plan each scout around the others' chosen trips, re-pack
TRIP_PLAN_LABELS = 4     # labels kept per contact while generating plan trips
TRIP_PLAN_EXACT = False  # plan trips use straight legs (detours are ~95% of the label time)
TRIP_PLAN_DROPS = 2      # extra searches per scout without one of its best trip's contacts
# Validated trip tree on top of the packing. Depth = one scout (best packed trip first);
# a node's children are that scout's best disjoint pregenerated trips (ranked by trip
# value + a greedy estimate for the scouts after it), the packed trip, and "no trip".
# Each child is realised through _serve and incrementally validated, so nodes carry real
# scores; the beam keeps TRIP_TREE_WIDTH nodes per depth by real score + the estimate of
# the scouts still to come. The best realised plan (tree or packing) is then repaired.
TRIP_TREE = True
TRIP_TREE_WIDTH = 6      # nodes kept per depth
TRIP_TREE_BRANCH = 5     # trips tried per node (plus the packed trip and "no trip")
TRIP_TREE_POOL = 30      # a scout's best disjoint trips ranked for branching
TRIP_TREE_SHARE = 0.2    # trip plan + tree share of the whole budget (may pass beam_stop)


def pack_trips(trips, stop=None):
    """Weighted set packing: at most one trip per scout, no contact in two trips, maximal
    total value. trips = [(value, scout id, contact ids, payload)] with value > 0.

    Greedy by value and by value per contact, then swaps until none helps: a trip outside
    the packing replaces every chosen trip it conflicts with, the scouts left without a
    trip are refilled greedily from their own trips, and the swap is kept when the total
    grows. Returns (value, chosen trips).
    """
    trips = [tr for tr in trips if tr[0] > 0]
    trips.sort(key=lambda tr: (-tr[0], tr[1], tuple(sorted(tr[2]))))
    by_scout = {}
    for tr in trips:
        by_scout.setdefault(tr[1], []).append(tr)

    def fill(chosen):
        scouts = {tr[1] for tr in chosen}
        taken = set()
        for tr in chosen:
            taken |= tr[2]
        free = sorted((tr for sid, own in by_scout.items() if sid not in scouts for tr in own),
                      key=lambda tr: -tr[0])
        for tr in free:
            if tr[1] not in scouts and not (taken & tr[2]):
                chosen.append(tr)
                scouts.add(tr[1])
                taken |= tr[2]
        return chosen

    def total(chosen):
        return sum(tr[0] for tr in chosen)

    best = []
    for key in (None, lambda tr: -tr[0] / max(1, len(tr[2]))):
        order = trips if key is None else sorted(trips, key=key)
        chosen, scouts, taken = [], set(), set()
        for tr in order:
            if tr[1] not in scouts and not (taken & tr[2]):
                chosen.append(tr)
                scouts.add(tr[1])
                taken |= tr[2]
        improved = True
        while improved:
            improved = False
            current = total(chosen)
            for tr in trips:
                if stop is not None and time.monotonic() >= stop:
                    break
                if tr in chosen:
                    continue
                keep = [c for c in chosen if c[1] != tr[1] and not (c[2] & tr[2])]
                if total(keep) + tr[0] + sum(by_scout[c[1]][0][0] for c in chosen
                                             if c not in keep) <= current + 1e-9:
                    continue  # even each freed scout's best trip could not pay for it
                trial = fill(keep + [tr])
                value = total(trial)
                if value > current + 1e-9:
                    chosen, current, improved = trial, value, True
        if total(chosen) > total(best):
            best = chosen
    return total(best), best


# Joint re-plan of n scouts (LNS operator "group"): pool the scouts' contacts and the
# unserved positive-net contacts near them, generate each scout's top trips over that pool
# with the label search, choose the best combination of disjoint trips and realise it when
# its estimated value beats the scouts' current trips. Empty: the operator is off.
GROUP_SIZES = (2, 3, 4)  # random n per move; () turns the operator off
GROUP_SHARE = 0.2        # share of LNS steps using "group" (without --adaptive)
GROUP_TRIPS = 40         # trips kept per scout (plus the cover trips)
GROUP_RADIUS_M = 2000.0  # unserved contacts this close to a chosen scout's tour join the pool
GROUP_POOL = 40          # at most this many pool contacts
GROUP_MOVE_S = 0.8       # realisation time cap per move
GROUP_NODES = 200000     # search nodes cap in best_disjoint

# Joint strike repair.  Unlike a normal destroy/repair step this deliberately removes
# the aircraft which actually violate R8 against an unserved target's candidate team,
# then retries that target and the displaced strikes together.  Keep the switch here:
# it makes an A/B run possible without changing the rest of LNS.
STRIKE_GROUP = True
STRIKE_SHARE = 0.2
STRIKE_MOVE_S = 0.8
STRIKE_CONFLICT_ROUTES = 18
STRIKE_REMOVE_SIZES = (1, 2, 3)  # random count of conflicting hostile strikes to displace


def best_disjoint(lists, floor=-math.inf, nodes=GROUP_NODES):
    """Best combination of one trip per list with pairwise disjoint contact masks.

    lists: per scout [(value, mask)] sorted by value, best first (include (0, 0) to allow
    an empty trip). Depth-first over scouts in order of their best trip, pruned when the
    partial value plus every remaining scout's best trip cannot beat the best so far
    (initially `floor`). Returns (value, [index per list]) or None if nothing beats floor.
    """
    n = len(lists)
    if n == 0 or any(not lst for lst in lists):
        return None
    order = sorted(range(n), key=lambda i: -lists[i][0][0])
    rest = [0.0] * (n + 1)
    for pos in range(n - 1, -1, -1):
        rest[pos] = rest[pos + 1] + max(0.0, lists[order[pos]][0][0])
    best = [floor, None]
    picks = [0] * n
    count = [0]

    def go(pos, value, mask):
        if pos == n:
            if value > best[0] + 1e-9:
                best[0], best[1] = value, list(picks)
            return
        for k, (v, m) in enumerate(lists[order[pos]]):
            count[0] += 1
            if value + v + rest[pos + 1] <= best[0] + 1e-9 or count[0] > nodes:
                return
            if m & mask == 0:
                picks[order[pos]] = k
                go(pos + 1, value + v, mask | m)

    go(0, 0.0, 0)
    return None if best[1] is None else (best[0], best[1])


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
# The warm start (kit planner) builds effector routes through the module too.
baseline._effector_route = effector_route


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
    # IncrementalValidator.route_info entry of every route (avoids re-hashing keys).
    infos: dict = field(default_factory=dict)

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
        # Routes are never edited after construction, so everything derived from one
        # route (parsed form, local rule result, R8 pieces, energy) and every R8 pair
        # result is a pure function of the routes: sibling branches retry the same scout
        # route with ~100 strike variants, and passes rebuild the same routes.
        self._routes = {}   # (aid, route_key) -> (rid, Route, violations, item, energy)
        self._pairs = {}    # (rid, rid) -> R8 violation or None
        self.cache_hits = 0

    def route_info(self, aid, waypoints, key=None):
        k = (aid, route_key(waypoints) if key is None else key)
        hit = self._routes.get(k)
        if hit is not None:
            self.cache_hits += 1
            return hit
        if len(self._routes) > 200000:
            self._routes.clear()
            self._pairs.clear()
        plan = {"seed": self.sc.seed, "routes": [{"aircraft": aid, "waypoints": waypoints}]}
        changed, errors = parse_plan(plan)
        if errors or changed is None:
            raise ValueError("route builder produced malformed waypoints: " + str(errors))
        route, violations = changed[aid], None
        for check in (rules._r1, rules._r2, rules._r3, rules._r5, rules._r6,
                      rules._r7, rules._r9):
            found = check(self.sc, changed, plan) if check is rules._r1 else check(self.sc, changed)
            if found:
                violations = found
                break
        item = energy = None
        obs, eng = [], []
        if violations is None:
            ac = self.aircraft[aid]
            item = rules.separation_item(aid, route, ac)
            energy = route_energy_wh(route.waypoints, ac)
            if item is not None:
                # Bounding box of the flight (pieces are straight, so the waypoints bound
                # it) and its time span: a quick exact pre-filter for R8 pairs.
                pts = [w.pos for w in route.waypoints]
                item = item + ((min(p[0] for p in pts), max(p[0] for p in pts),
                                min(p[1] for p in pts), max(p[1] for p in pts),
                                min(p[2] for p in pts), max(p[2] for p in pts)),)
            # What R4, R10 and scoring read from this route.
            for wp in route.waypoints:
                c = self.contacts.get(wp.target)
                if wp.action == "observe" and c is not None and rules.valid_observation(wp, c, self.sc):
                    obs.append((c.id, wp.t + wp.dwell_s, ac.type == SCOUT, c.cls == UNKNOWN))
                elif wp.action == "engage":
                    strike = c is not None and c.cls == HOSTILE and ac.type == EFFECTOR
                    eng.append((wp.target, wp.t, strike, ac.subtype or "?",
                                strike and dist3(wp.pos, c.pos) <= self.sc.rules.R_eng_m + EPS))
        hit = self._routes[k] = (len(self._routes), route, violations, item, energy, obs, eng)
        return hit

    def global_rules(self, infos):
        """R4 then R10 over the whole plan from per-route summaries; the same verdicts as
        rules._r4 and rules._r10. Returns a violation list or None."""
        margin = self.sc.rules.confirm_margin_s
        first_obs = {}
        groups = {}
        for info in infos:
            for cid, end, by_scout, _ in info[5]:
                if by_scout and end < first_obs.get(cid, math.inf):
                    first_obs[cid] = end
            for cid, t, strike, sub, close in info[6]:
                if strike:
                    groups.setdefault(cid, []).append((t, sub, close))
        for cid, group in groups.items():
            for t, _, _ in group:
                if not first_obs.get(cid, math.inf) <= t - margin + EPS:
                    return [rules.Violation("R4", "engagement without a confirmed observation",
                                            contact=cid)]
        for cid, group in groups.items():
            counts = {}
            for _, sub, _ in group:
                counts[sub] = counts.get(sub, 0) + 1
            want = {k: v for k, v in self.contacts[cid].needs.items() if v > 0}
            times = [t for t, _, _ in group]
            if (counts != want or max(times) - min(times) > self.sc.rules.sync_window_s + EPS
                    or not all(close for _, _, close in group)):
                return [rules.Violation("R10", "engagement group", contact=cid)]
        return None

    def apart(self, a, b):
        """True when two separation items can never come within d_sep: their time spans
        do not overlap, or their bounding boxes are at least d_sep apart on one axis."""
        if min(a[3], b[3]) - max(a[2], b[2]) <= 1e-12:
            return True
        d = self.sc.rules.d_sep_m
        p, q = a[5], b[5]
        return (p[0] - q[1] >= d or q[0] - p[1] >= d or p[2] - q[3] >= d or q[2] - p[3] >= d
                or p[4] - q[5] >= d or q[4] - p[5] >= d)

    def pair_violation(self, a, b):
        """R8 for two (rid, item) routes, memoised by route ids."""
        key = (a[0], b[0]) if a[0] < b[0] else (b[0], a[0])
        if key in self._pairs:
            self.cache_hits += 1
            return self._pairs[key]
        if a[1] is None or b[1] is None or self.apart(a[1], b[1]):
            violation = None
        else:
            violation = rules.separation_violation(self.sc.rules, self.contacts, a[1][:5], b[1][:5])
        if len(self._pairs) > 2000000:
            self._pairs.clear()
        self._pairs[key] = violation
        return violation

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
                       separation=keep(parent.separation), energies=keep(parent.energies),
                       infos=keep(parent.infos), keys=keep(parent.keys))

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
        keys = {aid: route_key(route) for aid, route in extra.items()}
        infos = {aid: self.route_info(aid, route, keys[aid]) for aid, route in extra.items()}
        for info in infos.values():
            if info[2]:
                return self.reject(info[2])
        changed = {aid: info[1] for aid, info in infos.items()}
        parsed = {**parent.parsed, **changed}
        every = {**parent.infos, **infos}
        if len(every) != len(set(parent.parsed) | set(infos)):
            every = None
        if every is not None:
            violations = self.global_rules(every.values())
            if violations:
                return self.reject(violations)
        else:
            for check in (rules._r4, rules._r10):
                violations = check(self.sc, parsed)
                if violations:
                    return self.reject(violations)
        new = [(info[0], info[3]) for info in infos.values()]
        old = []
        for aid, item in parent.separation.items():
            if aid in changed:
                continue
            info = parent.infos.get(aid)
            # Parent routes normally come through here; otherwise check without the memo.
            old.append((info[0], item) if info is not None and info[3] is item else (-1, item))
        for a, b in itertools.chain(itertools.combinations(new, 2), itertools.product(new, old)):
            self.check_time()
            if a[0] < 0 or b[0] < 0:
                violation = (None if a[1] is None or b[1] is None else
                             rules.separation_violation(self.sc.rules, self.contacts,
                                                        a[1][:5], b[1][:5]))
            else:
                violation = self.pair_violation(a, b)
            if violation:
                return self.reject([violation])
        new_items = {aid: info[3] for aid, info in infos.items()}
        old_items = {aid: item for aid, item in parent.separation.items() if aid not in changed}
        energies = dict(parent.energies)
        energies.update({aid: info[4] for aid, info in infos.items()})
        served = set()
        used = set()
        if every is not None:
            for aid, info in every.items():
                for cid, _, _, unknown in info[5]:
                    if unknown:
                        served.add(cid)
                for cid, *_ in info[6]:
                    served.add(cid)
                    used.add(aid)
        else:
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
        keys = {**parent.keys, **keys}
        return State({**parent.routes, **extra}, parent.tasks, frozenset(used), parsed,
                     {**old_items, **new_items}, energies, frozenset(served), score, keys=keys,
                     infos={**parent.infos, **infos})


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
                 scout_speed="fast", obs_stretch=True):
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
        self.lns = {"iterations": 0, "accepted": 0, "improvements": 0, "gain": 0.0,
                    "trip_tried": 0, "trip_accepted": 0, "trip_improvements": 0,
                    "trip_gain": 0.0, "trip_label_s": 0.0,
                    "group_tried": 0, "group_realised": 0, "group_accepted": 0,
                    "group_improvements": 0, "group_gain": 0.0, "group_label_s": 0.0,
                    "group_sizes": {},
                    "strike_tried": 0, "strike_accepted": 0, "strike_improvements": 0,
                    "strike_gain": 0.0}
        self._trip_cache = {}
        self.trip_stats = {}
        self.lns_share = lns_share
        self.interleave, self.adaptive, self.restarts = interleave, adaptive, restarts
        # Later passes keep a wider beam: more outranked branches survive to the end.
        self.widen = widen
        if interleave:
            # Beam passes alternate with LNS bursts; a short final tail is LNS only.
            self.beam_stop = self.stop - min(lns_share, 0.1) * max(0.0, self.stop - self.start)
        self.lns_rng = random.Random(scenario.seed + 7919)
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
                if SCOUT_ENERGY_RANK:
                    best *= scout_wh_per_m(scout)
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
            team = arrival_order(team)
            found = False
            spacing = min(5.0, self.sc.rules.sync_window_s / max(1, len(team)-1))
            delays = STRIKE_DELAYS if PAD_TAKEOFFS else (0.0, 8.0, 20.0, 45.0, 90.0)
            for delay, angle, layers in itertools.product(delays, (0.0, 35.0, -35.0),
                                                             effector_layer_sets()):
                self.checker.check_time(stop)
                arrive = base + delay
                if arrive + spacing*(len(team)-1) > contact.window[1] + EPS:
                    break
                extra = {scout.id: scout_route}
                for i, eff in enumerate(team):
                    z = layers[i % len(layers)]
                    route, earliest = effector_route(eff, contact, arrive + spacing*i, z, angle, self.sc)
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

    def strike_route_options(self, contact, groups, scout, scout_route, obs_end, stop):
        """A small, concrete sample of the routes ``strikes`` will try.

        This is deliberately route based: the strike LNS move must identify R8
        conflicts from the validator's separation items, not from pad or target
        distance heuristics.  It does not validate or mutate a state.
        """
        base = max(obs_end + self.sc.rules.confirm_margin_s + 2.0, contact.window[0])
        count = 0
        for team in groups:
            team = arrival_order(team)
            spacing = min(5.0, self.sc.rules.sync_window_s / max(1, len(team) - 1))
            for delay, angle, layers in itertools.product(STRIKE_DELAYS, (0.0, 35.0, -35.0),
                                                            effector_layer_sets()):
                self.checker.check_time(stop)
                arrive = base + delay
                if arrive + spacing * (len(team) - 1) > contact.window[1] + EPS:
                    break
                extra = {}
                for i, eff in enumerate(team):
                    route, _ = effector_route(eff, contact, arrive + spacing * i,
                                              layers[i % len(layers)], angle, self.sc)
                    if route is None:
                        break
                    extra[eff.id] = route
                if len(extra) == len(team):
                    yield team, extra
                    count += 1
                    if count >= STRIKE_CONFLICT_ROUTES:
                        return

    def strike_conflicts(self, state, routes):
        """Return existing aircraft which really violate R8/pad spacing with routes.

        The caller supplies only prospective effector routes.  Every candidate is
        parsed through the same route-info builder as incremental validation and
        compared with every unchanged route, including scouts.
        """
        conflicts = set()
        infos = {}
        for aid, route in routes.items():
            info = self.checker.route_info(aid, route)
            if info[2]:
                continue
            infos[aid] = info
        for aid, info in infos.items():
            for oid, old in state.infos.items():
                if oid == aid or oid in infos:
                    continue
                item = old[3]
                if item is not None and info[3] is not None and rules.separation_violation(
                        self.sc.rules, self.contacts, info[3][:5], item[:5]):
                    conflicts.add(oid)
            # The cheap pad test catches the same takeoff geometry before a full R8
            # trajectory check, and identifies the concrete neighbouring aircraft.
            for oid in pad_neighbours(self.sc, self.aircraft, aid):
                old = state.routes.get(oid)
                if old and pad_conflict(self.sc, self.aircraft, state.routes, {aid: routes[aid]}):
                    conflicts.add(oid)
        return conflicts

    def observed_strike(self, state, cid):
        """(scout, route, observation finish) for an already observed hostile."""
        for sid, tasks in state.tasks.items():
            if not any(t["cid"] == cid for t in tasks):
                continue
            route = state.routes.get(sid)
            if route is None:
                continue
            times = [w["t"] + w.get("dwell_s", 0.0) for w in route
                     if w.get("action") == "observe" and w.get("target") == cid]
            if times:
                return self.aircraft[sid], route, max(times)
        return None

    def drop_strike(self, state, cid):
        """Drop just cid's effector routes, retaining its harmless scout observation."""
        team = {aid for aid, route in state.parsed.items()
                if any(w.action == "engage" and w.target == cid for w in route.waypoints)}
        if not team:
            return state
        return self.checker.extend(self.checker.without(state, team), {})

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
            if self.passes % 4 == 0:
                order = sorted(contacts, key=lambda c: (-priorities[c.id], c.id))
            elif self.passes % 4 == 1:
                order = sorted(contacts, key=lambda c: (c.window[1], -priorities[c.id], c.id))
            elif self.passes % 4 == 2:
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
        try:
            self.warm_start()
        except DeadlineReached:
            pass
        self.matrix = CandidateMatrix(self.sc)
        contacts = [c for c in self.sc.contacts if c.cls in (HOSTILE, UNKNOWN)
                    and c.window is not None and c.id not in self.matrix.banned]
        priorities = {c.id: self.matrix.priority(c) for c in contacts}
        rng = random.Random(self.sc.seed)
        self.checker.stop = min(self.beam_stop, self.stop)
        if TRIP_PLAN and contacts:
            if TRIP_TREE:
                # A share of the whole budget: the tree may run past the beam phase
                # (its time then comes out of LNS), never past the search deadline.
                trip_stop = min(self.stop, time.monotonic()
                                + TRIP_TREE_SHARE * max(0.0, self.stop - self.start))
                self.checker.stop = max(self.checker.stop, trip_stop)
            else:
                trip_stop = min(self.checker.stop, time.monotonic()
                                + TRIP_PLAN_SHARE * max(0.0, self.beam_stop - self.start))
            try:
                self.trip_plan(trip_stop)
            except DeadlineReached:
                pass
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

    # --- Scout trip label search -------------------------------------------------------

    def trip_candidates(self, scout, used, exclude, z, deadlines=None):
        """Label-search candidates for one scout: contacts with a net value above zero
        (unknown: points; hostile: points minus its cheapest free team), their observation
        point at layer z, window and latest observation finish (hostiles: early enough to
        confirm and strike, and before any existing strike's deadline)."""
        rules_ = self.sc.rules
        deadlines = deadlines or {}
        out = []
        for c in self.sc.contacts:
            if (c.cls not in (HOSTILE, UNKNOWN) or c.window is None or c.id in exclude
                    or c.id in self.matrix.banned):
                continue
            lo, hi = c.window
            latest = hi
            if c.cls == HOSTILE:
                groups = self.matrix.groups(c, used, 1)
                if not groups:
                    continue
                team = groups[0]
                net = c.points - sum(self.matrix.costs[a.id, c.id] for a in team)
                spacing = min(5.0, rules_.sync_window_s / max(1, len(team) - 1)) * (len(team) - 1)
                ready = max(lo, max(self.effector_earliest(a, c) for a in team))
                if ready + spacing > hi + EPS:
                    continue
                latest = hi - spacing - rules_.confirm_margin_s - 2.0 - TRIP_STRIKE_SLACK
            else:
                net = c.points
            latest = min(latest, deadlines.get(c.id, math.inf))
            if net <= 0 or latest < lo + rules_.obs_dwell_s:
                continue
            out.append((c.id, obs_point(self.sc, scout, c, 0.0, z), (lo, hi), net, latest))
        return out

    def effector_earliest(self, eff, contact):
        key = ("eff_earliest", eff.id, contact.id)
        hit = self._trip_cache.get(key)
        if hit is None:
            route, earliest = effector_route(eff, contact, 0.0, EFFECTOR_Z[0][0], 0.0, self.sc)
            hit = self._trip_cache[key] = 0.0 if route is not None else earliest
        return hit

    def scout_trips(self, state, scout, used, exclude, z, top=30):
        """trip_labels for one scout against `state`, with its current tour as an extra
        order so the result is never worse than what the scout already flies (by the same
        estimate)."""
        cands = self.trip_candidates(scout, used, exclude, z)
        current = tuple(t["cid"] for t in state.tasks.get(scout.id, ()))
        return trip_labels(self.sc, scout, cands, z, top=top, orders=(current,))

    def trip_scout(self, state, rng):
        """A scout for the trip move, favouring high battery use and unserved positive-net
        contacts near its tour or base."""
        unserved = [c for c in self.sc.contacts if c.cls in (HOSTILE, UNKNOWN)
                    and c.window is not None and c.id not in state.served
                    and c.id not in self.matrix.banned and self.matrix.priority(c) > 0]
        scouts, weights = [], []
        for sid in sorted(state.tasks):
            scout = self.aircraft[sid]
            frac = state.energies.get(sid, 0.0) / max(1e-9, scout.usable_wh)
            points = [scout.home[:2]] + [t["point"] for t in state.tasks[sid]]
            near = sum(1 for c in unserved if min(hdist(q, c.pos) for q in points) < 1500.0)
            scouts.append(scout)
            weights.append(0.3 + frac + 0.2 * min(5, near))
        return rng.choices(scouts, weights=weights)[0] if scouts else None

    def strip_scout(self, state, sid):
        """State without scout sid's tour and the strikes its observations confirm."""
        return self.strip_scouts(state, (sid,))

    def strip_scouts(self, state, sids):
        """State without the tours of scouts `sids` and the strikes their observations
        confirm. Returns (state or None, removed contact ids)."""
        cids = {t["cid"] for sid in sids for t in state.tasks.get(sid, ())}
        team = {aid for aid, route in state.parsed.items()
                if self.aircraft[aid].type != SCOUT
                and any(w.action == "engage" and w.target in cids for w in route.waypoints)}
        partial = self.checker.extend(self.checker.without(state, team | set(sids)), {})
        if partial is None:
            return None, cids
        partial.tasks = {**state.tasks, **{sid: [] for sid in sids}}
        return partial, cids

    def realise_trip(self, state, scout, order, z, stop):
        """Append the trip's visits one at a time (see realise_visits). Returns (state,
        realised contact ids)."""
        return self.realise_visits(state, [(scout, cid, z) for cid in order], stop)

    def realise_visits(self, state, visits, stop, on_visit=None):
        """Append visits [(scout, contact id, preferred layer)] in order through _serve
        (strikes included), retrying layers, observation angles and takeoff shifts; visits
        that cannot be realised are skipped. A scout keeps the layer its tour already flies.
        on_visit(previous, new, scout, cid) is called after each realised visit.
        Returns (state, realised contact ids); stops quietly at `stop`."""
        done = []
        try:
            for scout, cid, z in visits:
                self.checker.check_time(stop)
                child = self.realise_visit(state, scout, cid, z, stop)
                if child is not None:
                    if on_visit is not None:
                        child = on_visit(state, child, scout, cid)
                    state = child
                    done.append(cid)
        except DeadlineReached:
            self.checker.check_time()
        return state, done

    def realise_visit(self, state, scout, cid, z, stop):
        """Best child appending contact cid to the scout's tour, or None."""
        contact = self.contacts[cid]
        if cid in state.served:
            return None
        groups = self.matrix.groups(contact, state.used) if contact.cls == HOSTILE else [()]
        if not groups:
            return None
        route = state.routes.get(scout.id)
        layer = max(w["z"] for w in route) if route else z
        tasks = list(state.tasks.get(scout.id, ()))
        layers = [layer] + [l for l in SCOUT_LAYERS if l != layer]
        speed = eco_speed(scout) if SCOUT_SPEED_MODE != "fast" else fast_speed(scout)

        def options():
            for zz, angle, sh in itertools.product(layers, (0.0, 60.0, -60.0), (0.0, 7.0)):
                task = {"cid": cid, "point": obs_point(self.sc, scout, contact, angle, zz),
                        "window": contact.window, "cpos": contact.pos}
                tt = tasks + [task]
                if PAD_TAKEOFFS:
                    sh += takeoff_shift(self.sc, self.aircraft, state.routes, scout,
                                        _depart(scout, tt, zz, speed, 0.0))
                yield scout, tt, scout_tour(self.sc, scout, tt, zz, sh)

        # The first visits of a trip may cost more scout energy than they earn; the trip
        # is judged as a whole by the caller.
        base = replace(state, score=-math.inf)
        progress = {"stage": -1, "outcome": "no_options"}
        best = None
        for child, _ in self._serve(base, contact, options(), groups, stop, 2,
                                    self.stats(cid), progress, {"count": 0, "seen": set()}):
            if best is None or child.score > best.score:
                best = child
        return best

    def trip_plan(self, stop):
        """Warm start from a packing of label-search trips of all scouts.

        Candidate trips per scout and layer: the best ones, the best one including each
        contact (cover) and the best one without each contact of its best trip; then
        TRIP_PLAN_ROUNDS rounds of re-planning every scout without the contacts the other
        chosen trips hold, re-packing after each. Every realised visit is recorded; the
        result is then greedily repaired."""
        start = time.monotonic()
        unique = {}
        scouts = [a for a in self.sc.aircraft if a.type == SCOUT]
        layers = (SCOUT_LAYERS[0], min(SCOUT_LAYERS))
        cands = {(a.id, z): self.trip_candidates(a, frozenset(), set(), z)
                 for a in scouts for z in layers}

        rows = {key: {} for key in cands}

        def add(scout, z, exclude=(), top=TRIP_PLAN_TRIPS, cover=True):
            self.checker.check_time(stop)
            pool = cands[scout.id, z]
            skip = sum(1 << j for j, c in enumerate(pool) if c[0] in exclude)
            found = trip_labels(self.sc, scout, pool, z, keep=TRIP_PLAN_LABELS, top=top,
                                cover=cover, skip=skip, rows=rows[scout.id, z],
                                exact=TRIP_PLAN_EXACT)
            for value, order, wh, finishes in found:
                key = (scout.id, frozenset(order))
                if key not in unique or value > unique[key][0]:
                    unique[key] = (value, scout.id, frozenset(order), (z, order, finishes))
            return found

        for scout in scouts:
            for z in layers:
                found = add(scout, z)
                if found:
                    # The best trip without each of its most valuable contacts.
                    net = {c[0]: c[3] for c in cands[scout.id, z]}
                    for cid in sorted(found[0][1], key=lambda c: -net[c])[:TRIP_PLAN_DROPS]:
                        add(scout, z, {cid}, top=3, cover=False)
        pack_stop = lambda: min(stop, time.monotonic() + 0.3)
        value, chosen = pack_trips(list(unique.values()), pack_stop())
        rounds = 0
        for _ in range(TRIP_PLAN_ROUNDS):
            for scout in scouts:
                held = {cid for tr in chosen if tr[1] != scout.id for cid in tr[2]}
                for z in layers:
                    add(scout, z, held, top=10)
            value, chosen = pack_trips(list(unique.values()), pack_stop())
            rounds += 1
        self.trip_stats.update(candidates=len(unique), chosen=len(chosen), rounds=rounds,
                               estimate=round(value, 3),
                               label_s=round(time.monotonic() - start, 4))
        visits = sorted(((f, self.aircraft[sid], cid, z)
                         for _, sid, _, (z, order, finishes) in chosen
                         for cid, f in zip(order, finishes)), key=lambda v: (v[0], v[2]))
        depth = [0]

        def on_visit(previous, child, scout, cid):
            depth[0] += 1
            return self.record(child, previous, {"kind": "trip_plan", "contact": cid,
                                                 "scout": scout.id}, depth[0])

        state, done = self.realise_visits(self.root, [(a, cid, z) for _, a, cid, z in visits],
                                          stop, on_visit)
        self.trip_stats.update(visits=len(visits), realised=len(done),
                               realised_score=round(state.score, 3))
        if TRIP_TREE:
            ratio = min(1.0, max(0.5, state.score / value)) if value > 0 else 1.0
            tree = self.trip_tree(unique, chosen, stop, depth[0], ratio)
            if tree is not None and tree.score > state.score:
                state = tree
        saved, self.checker.stop = self.checker.stop, min(self.checker.stop, stop)
        try:
            state = self.repair(state, depth[0])
        except DeadlineReached:
            pass
        finally:
            self.checker.stop = saved
        self.trip_stats.update(score=round(state.score, 3),
                               seconds=round(time.monotonic() - start, 3))
        return state

    def trip_tree(self, unique, chosen, stop, depth0, ratio=1.0):
        """Beam over whole pregenerated scout trips with validated nodes (TRIP_TREE).
        Returns the best realised state, or None when nothing was realised."""
        start = time.monotonic()
        by_scout = {}
        for tr in unique.values():
            if tr[0] > 0:
                by_scout.setdefault(tr[1], []).append(tr)
        for trips in by_scout.values():
            trips.sort(key=lambda tr: (-tr[0], tuple(sorted(tr[2]))))
        packed = {tr[1]: tr for tr in chosen}
        order = sorted(by_scout, key=lambda sid: (-packed[sid][0] if sid in packed else 0.0,
                                                  -by_scout[sid][0][0], sid))
        stats = {"nodes": 0, "realised": 0, "depth": 0}

        def rest_value(d, taken):
            """Greedy estimate for scouts order[d:]: each takes its best disjoint trip,
            scaled by the packing's realised/estimated ratio (estimates are optimistic)."""
            total, taken = 0.0, set(taken)
            for sid in order[d:]:
                for tr in by_scout[sid]:
                    if not tr[2] & taken:
                        total += tr[0]
                        taken |= tr[2]
                        break
            return ratio * total

        beam = [(self.root, frozenset())]
        best = None
        seen = set()
        try:
            for d, sid in enumerate(order):
                scout = self.aircraft[sid]
                # Anytime: each depth gets an equal share of the time left, so the tree
                # always reaches a complete plan; options are tried round-robin over the
                # beam, best first, and at least one trip is realised per depth.
                depth_stop = time.monotonic() + max(0.0, stop - time.monotonic()) / (len(order) - d)
                children, queues = [], []
                for state, taken in beam:
                    pool = [tr for tr in by_scout[sid] if not tr[2] & taken][:TRIP_TREE_POOL]
                    pool.sort(key=lambda tr: -(tr[0] + rest_value(d + 1, taken | tr[2])))
                    options = pool[:TRIP_TREE_BRANCH]
                    mine = packed.get(sid)
                    if mine is not None and mine not in options and not mine[2] & taken:
                        options.append(mine)
                    # "No trip": the scout stays home and its contacts stay free.
                    children.append((state.score + rest_value(d + 1, taken), state, taken))
                    queues.append((state, taken, options))
                width = max((len(q[2]) for q in queues), default=0)
                tried = 0
                for k in range(width):
                    for state, taken, options in queues:
                        if k >= len(options):
                            continue
                        if tried and time.monotonic() >= depth_stop:
                            break
                        self.checker.check_time(stop)
                        tried += 1
                        z, trip, _ = options[k][3]
                        child, done = self.realise_visits(state, [(scout, cid, z) for cid in trip],
                                                          stop)
                        stats["realised"] += 1
                        if not done:
                            continue
                        sig = child.signature()
                        if sig in seen:
                            continue
                        seen.add(sig)
                        stats["nodes"] += 1
                        child = self.record(child, state, {"kind": "trip_tree", "scout": sid,
                                                           "contacts": list(done)},
                                            depth0 + d + 1)
                        new_taken = taken | frozenset(done)
                        children.append((child.score + rest_value(d + 1, new_taken), child,
                                         new_taken))
                        if best is None or child.score > best.score:
                            best = child
                    if tried and time.monotonic() >= depth_stop:
                        stats["cut"] = stats.get("cut", 0) + 1
                        break
                children.sort(key=lambda ch: -ch[0])
                beam, keys = [], set()
                for _, state, taken in children:
                    key = state.signature()
                    if key not in keys:
                        keys.add(key)
                        beam.append((state, taken))
                    if len(beam) >= TRIP_TREE_WIDTH:
                        break
                stats["depth"] = d + 1
        except DeadlineReached:
            pass
        for state, _ in beam:
            if best is None or state.score > best.score:
                best = state
        stats.update(scouts=len(order), seconds=round(time.monotonic() - start, 3),
                     score=round(best.score, 3) if best is not None else None)
        self.trip_stats["tree"] = stats
        return best

    def trip_move(self, current, rng):
        """Destroy and rebuild one scout's whole tour from its best label-search trip.
        Returns (partial state, contact ids removed) or None when nothing changes."""
        scout = self.trip_scout(current, rng)
        if scout is None:
            return None
        sid = scout.id
        partial, removed = self.strip_scout(current, sid)
        if partial is None:
            return None
        others = {t["cid"] for s, tasks in current.tasks.items() if s != sid for t in tasks}
        route = current.routes.get(sid)
        z = max(w["z"] for w in route) if route else SCOUT_LAYERS[0]
        start = time.monotonic()
        trips = self.scout_trips(partial, scout, partial.used, others | set(partial.served), z)
        self.lns["trip_label_s"] += time.monotonic() - start
        trips = [tr for tr in trips if set(tr[1]) != removed]
        if not trips:
            return None
        pick = 0 if rng.random() < 0.6 or len(trips) == 1 else rng.randrange(1, min(5, len(trips)))
        stop = min(self.checker.stop, time.monotonic() + 0.5)
        partial, _ = self.realise_trip(partial, scout, trips[pick][1], z, stop)
        return partial, sorted(removed)

    def strike_move(self, current, rng):
        """Jointly re-plan a blocked hostile and one to three R8-conflicting strikes.

        Removing an effector route rather than its contact is important: the scout
        observation remains a valid, reusable confirmation while alternative teams
        and timings are explored.  The incremental checker then rechecks R4/R10 and
        every changed-route separation pair on each re-strike.
        """
        open_ = [c for c in self.sc.contacts if c.cls == HOSTILE and c.window is not None
                 and c.id not in current.served and c.id not in self.matrix.banned
                 and self.matrix.priority(c) > 0]
        if not open_:
            return None
        side = ("strike_rule:", "strike_route", "time_cap", "not_improving", "outranked")
        preferred = [c for c in open_ if any(k.startswith(side) for k in
                     self.attempts.get(c.id, {}).get("outcomes", {}))]
        target = rng.choice(preferred or open_)
        observed = self.observed_strike(current, target.id)
        stop = min(self.checker.stop, time.monotonic() + STRIKE_MOVE_S)
        # Most unserved contacts do not retain an observation: ordinary children are
        # atomic.  Build one candidate scout visit solely to obtain the prospective
        # strike trajectories for conflict discovery; after removal, children() does
        # the normal fully-validated scout-and-strike insertion.
        needs_observation = observed is None
        if observed is None:
            try:
                scout, _, scout_route = next((x for x in self.scout_options(current, target, True)
                                               if x[2] is not None), (None, None, None))
            except DeadlineReached:
                return None
            if scout_route is None:
                return None
            obs = [w["t"] + w.get("dwell_s", 0.0) for w in scout_route
                   if w.get("action") == "observe" and w.get("target") == target.id]
            if not obs:
                return None
            obs_end = max(obs)
        else:
            scout, scout_route, obs_end = observed
        conflicts = set()
        # Use all currently installed effectors as prospective candidates.  The
        # conflict-derived removals below make their actual availability explicit.
        for _, routes in self.strike_route_options(target, self.matrix.groups(target, set(), 3),
                                                    scout, scout_route, obs_end, stop):
            conflicts.update(self.strike_conflicts(current, routes))
            if len(conflicts) >= 3:
                break
        displaced = set()
        for aid in conflicts:
            route = current.parsed.get(aid)
            if route is not None:
                displaced.update(w.target for w in route.waypoints if w.action == "engage")
        displaced.discard(target.id)
        if not displaced:
            return None
        sizes = [n for n in STRIKE_REMOVE_SIZES if n <= len(displaced)]
        if not sizes:
            return None
        count = rng.choice(sizes)
        # Prefer low-value displaced strikes when the scarce effector pool cannot
        # support every member of the group.
        removed = sorted(displaced, key=lambda cid: self.matrix.priority(self.contacts[cid]))[:count]
        partial = current
        for cid in removed:
            partial = self.drop_strike(partial, cid)
            if partial is None:
                return None
        if needs_observation:
            try:
                created = max((state for state, _ in self.children(partial, target, stop, 2, True)),
                              key=lambda s: s.score, default=None)
            except DeadlineReached:
                return None
            if created is None:
                return None
            partial = created
        group = [target.id] + removed
        orders = (group, removed + [target.id],
                  sorted(group, key=lambda cid: -self.matrix.priority(self.contacts[cid])))
        best = None
        for order in orders:
            trial = partial
            for cid in order:
                self.checker.check_time(stop)
                got = self.observed_strike(trial, cid)
                if got is None:
                    continue
                a, route, end = got
                children = self.strikes(trial, self.contacts[cid],
                                        self.matrix.groups(self.contacts[cid], trial.used),
                                        a, route, end, stop)
                child = max((state for state, _ in children), key=lambda s: s.score, default=None)
                if child is not None:
                    trial = child
            if best is None or trial.score > best.score:
                best = trial
        return best, group

    def group_scouts(self, state, n, rng):
        """n scouts for the group move: one picked like the trip move, then neighbours
        weighted by how close their tours (or bases) come to the chosen ones."""
        seed = self.trip_scout(state, rng)
        if seed is None:
            return []
        scouts = [a for a in self.sc.aircraft if a.type == SCOUT]

        def points(a):
            return [a.home[:2]] + [t["point"] for t in state.tasks.get(a.id, ())]
        chosen = [seed]
        near = points(seed)
        while len(chosen) < n:
            rest = [a for a in scouts if a not in chosen]
            if not rest:
                break
            weights = []
            for a in rest:
                d = min(hdist(p, q) for p in points(a) for q in near)
                weights.append(1.0 / (1.0 + d / 1000.0) ** 2)
            pick = rng.choices(rest, weights=weights)[0]
            chosen.append(pick)
            near += points(pick)
        return chosen

    def group_move(self, current, rng):
        """Jointly re-plan n scouts (n from GROUP_SIZES) from the best combination of their
        label-search trips over a shared contact pool. Returns (partial state, contact ids
        removed) or None when the estimate finds nothing better than the current trips."""
        n = rng.choice(GROUP_SIZES)
        key = str(n)
        self.lns["group_sizes"][key] = self.lns["group_sizes"].get(key, 0) + 1
        scouts = self.group_scouts(current, n, rng)
        if len(scouts) < 2:
            return None
        sids = [a.id for a in scouts]
        mine = {t["cid"] for sid in sids for t in current.tasks.get(sid, ())}
        others = {t["cid"] for s, tasks in current.tasks.items() if s not in sids for t in tasks}
        near = [p for a in scouts for p in [a.home[:2]] + [t["point"] for t in current.tasks.get(a.id, ())]]
        open_ = []
        for c in self.sc.contacts:
            if (c.cls in (HOSTILE, UNKNOWN) and c.window is not None and c.id not in current.served
                    and c.id not in others and c.id not in self.matrix.banned
                    and self.matrix.priority(c) > 0):
                d = min(hdist(c.pos, p) for p in near)
                if d < GROUP_RADIUS_M:
                    open_.append((d, c.id))
        pool = set(mine) | {cid for _, cid in sorted(open_)[:max(0, GROUP_POOL - len(mine))]}
        partial, removed = self.strip_scouts(current, sids)
        if partial is None:
            return None
        exclude = {c.id for c in self.sc.contacts if c.id not in pool} | set(partial.served)
        start = time.monotonic()
        bit = {cid: j for j, cid in enumerate(sorted(pool))}
        lists, layers, cur = [], [], 0.0
        for a in scouts:
            route = current.routes.get(a.id)
            z = max(w["z"] for w in route) if route else SCOUT_LAYERS[0]
            cands = self.trip_candidates(a, partial.used, exclude, z)
            order = tuple(t["cid"] for t in current.tasks.get(a.id, ()))
            trips = trip_labels(self.sc, a, cands, z, top=GROUP_TRIPS, orders=(order,), cover=True)
            have = [tr[0] for tr in trips if set(tr[1]) == set(order)]
            cur += max(have) if have and order else 0.0
            entries = [(tr[0], sum(1 << bit[c] for c in tr[1]), tr) for tr in trips if tr[0] > 0]
            entries.append((0.0, 0, None))
            entries.sort(key=lambda e: -e[0])
            lists.append(entries)
            layers.append(z)
        got = best_disjoint([[(v, m) for v, m, _ in lst] for lst in lists], cur)
        self.lns["group_label_s"] += time.monotonic() - start
        if got is None:
            return None
        picked = [lists[i][k][2] for i, k in enumerate(got[1])]
        new = {a.id: set(tr[1]) if tr else set() for a, tr in zip(scouts, picked)}
        if all(new[a.id] == {t["cid"] for t in current.tasks.get(a.id, ())} for a in scouts):
            return None
        visits = sorted(((f, a, cid, z) for a, tr, z in zip(scouts, picked, layers) if tr
                         for cid, f in zip(tr[1], tr[3])), key=lambda v: (v[0], v[2]))
        stop = min(self.checker.stop, time.monotonic() + GROUP_MOVE_S)
        partial, _ = self.realise_visits(partial, [(a, cid, z) for _, a, cid, z in visits], stop)
        self.lns["group_realised"] += 1
        return partial, sorted(removed)

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
        if op in ("tour", "trip"):
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
        roll = None if self.adaptive else rng.random()
        if self.adaptive:
            ops = [o for o in self.weights if (TRIP_SEARCH or o != "trip")
                   and (GROUP_SIZES or o != "group") and (STRIKE_GROUP or o != "strike")]
            op = rng.choices(ops, weights=[self.weights[o] for o in ops])[0]
        elif TRIP_SEARCH and roll < TRIP_SHARE:
            op = "trip"
        elif GROUP_SIZES and roll < TRIP_SHARE * bool(TRIP_SEARCH) + GROUP_SHARE:
            op = "group"
        elif STRIKE_GROUP and roll < (TRIP_SHARE * bool(TRIP_SEARCH) + GROUP_SHARE * bool(GROUP_SIZES)
                                      + STRIKE_SHARE):
            op = "strike"
        else:
            op = "related" if rng.random() < 0.5 else "random"
        removed, trip_centre = [], []
        partial = current
        if op in ("trip", "group", "strike"):
            self.lns[op + "_tried"] += 1
            moved = (self.trip_move(current, rng) if op == "trip" else
                     self.group_move(current, rng) if op == "group" else
                     self.strike_move(current, rng))
            if moved is None:
                op = "related"
            else:
                partial, stripped = moved
                trip_centre = [by_id[c].pos for c in stripped if c in by_id]
                removed = [c for c in stripped if c in by_id and c not in partial.served]
        if served and op not in ("trip", "group", "strike"):
            for cid in self.destroy(op, served, by_id, current, priorities, rng):
                smaller = self.remove(partial, cid)
                if smaller is not None:
                    partial = smaller
                    removed.append(cid)
        open_ = [c for c in contacts if c.id not in partial.served and c.id not in removed]
        centre = [by_id[c].pos for c in removed] or trip_centre or [rng.choice(contacts).pos]
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
        if delta > 1e-9 or ((removed or op in ("trip", "group", "strike")) and heat > 0 and rng.random() < math.exp(delta / heat)):
            self.lns["accepted"] += 1
            if op in ("trip", "group", "strike"):
                self.lns[op + "_accepted"] += 1
            reward = 1.0 if delta > 1e-9 else 0.3
            current = st["current"] = self.record(partial, current, {"kind": "lns", "op": op,
                                                                     "removed": removed},
                                                  len(partial.served))
            if current.score > best.score + 1e-9:
                self.lns["improvements"] += 1
                self.lns["gain"] += current.score - best.score
                if op in ("trip", "group", "strike"):
                    self.lns[op + "_improvements"] += 1
                    self.lns[op + "_gain"] += current.score - best.score
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
                "trip_plan": dict(self.trip_stats),
                "unserved": diagnostics["unserved"],
                "unserved_points": diagnostics["unserved_points"],
                "contacts": diagnostics["contacts"],
                "solutions": [{"rank": i, "node": state.node_id, "score": result.score,
                               "served": result.served} for i, (state, result) in enumerate(verified, 1)],
                "nodes": self.trace}
