"""Polish a finished plan: move scout observations between tours and retime the strikes.

The search (search_planner.py) inserts an observation into a scout tour only if every
existing strike stays confirmed. Inserting a visit early in a tour delays the later
visits, so their strikes lose their confirmation and the move is rejected, even when a
strike could simply start later: an effector waiting on its pad uses no energy.

This tool works on a valid plan and tries, ranked by the scout energy they save:
  - relocate: one observation to another scout's tour (or another position of its own);
  - eliminate: every observation of one scout into the other tours, dropping that scout;
  - add: an unserved unknown contact into a tour (points for scout energy).
After each tour change, a strike group whose confirmation now comes too late is shifted
later as a whole (it keeps its synchronisation) if its window allows. Every candidate is
checked with the official scorer and kept only if it scores higher.

Board use only: plans, not the hand-in planner.

Usage (from the repository root):
  python analysis/polish.py --input output/board-best --output output/board-polished
  python analysis/polish.py --input output/board-best --output output/board-best --scenarios L2-202
"""
import argparse
import copy
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "analysis"))
import _kit  # noqa: E402,F401  (makes fleetkit importable)
from fleetkit.model import Scenario, load_json, save_json  # noqa: E402
from fleetkit.scorer import evaluate  # noqa: E402
import search_planner as sp  # noqa: E402

SCENARIOS = ["L1-101", "L2-201", "L2-202", "L3-301", "L3-302", "L3-303", "L4-401", "L4-402"]
EPS = 1e-6
SPEED_MODES = ("fast", "eco")   # scout tour cruise speeds tried for every tour


class Polisher:
    def __init__(self, scenario, plan, log=print):
        self.sc = scenario
        self.ac = scenario.aircraft_by_id()
        self.cs = scenario.contact_by_id()
        self.rules = scenario.rules
        self.seed = plan["seed"]
        self.routes = {r["aircraft"]: copy.deepcopy(r["waypoints"]) for r in plan["routes"]}
        self.log = log
        result = evaluate(scenario, self.plan())
        if not result.valid:
            raise ValueError("input plan is invalid")
        self.score = result.score

    # ---- plan helpers -------------------------------------------------------------
    def plan(self, routes=None):
        routes = self.routes if routes is None else routes
        return {"seed": self.seed,
                "routes": [{"aircraft": a, "waypoints": w} for a, w in sorted(routes.items())]}

    def scouts(self, routes):
        return [a for a in routes if self.ac[a].type == "scout"]

    def tasks(self, route):
        out = []
        for w in route:
            if w.get("action") == "observe":
                c = self.cs[w["target"]]
                out.append({"cid": c.id, "point": (w["x"], w["y"]), "window": c.window,
                            "cpos": c.pos})
        return out

    @staticmethod
    def altitude(route):
        zs = [w["z"] for w in route if w.get("action") == "observe"]
        return zs[0] if zs else route[1]["z"]

    def energy(self, aid, route):
        return sp.raw_energy_wh(route, self.ac[aid]) if route else 0.0

    def tour(self, sid, tasks, z):
        """Best feasible tour (lowest energy) over a few departure shifts, or None."""
        if not tasks:
            return []
        scout = self.ac[sid]
        best = None
        saved = sp.SCOUT_SPEED_MODE
        try:
            for mode in SPEED_MODES:
                sp.SCOUT_SPEED_MODE = mode
                for shift in (0.0, 5.0, 15.0, 30.0):
                    route = sp.scout_tour(self.sc, scout, tasks, z, shift=shift)
                    if route is None:
                        continue
                    if route[-1]["t"] > self.sc.horizon_s - 0.01:
                        continue
                    e = sp.raw_energy_wh(route, scout)
                    if e > scout.usable_wh - 1e-3:
                        continue
                    if best is None or e < best[0] - 1e-9:
                        best = (e, route)
        finally:
            sp.SCOUT_SPEED_MODE = saved
        return None if best is None else best[1]

    def repoint(self, sid, tasks, z, sweeps=3, angles=24):
        """Same task order, each observation point moved on its contact's observation
        circle to the angle giving the cheapest tour; coordinate descent. None if no gain."""
        if not tasks:
            return None
        reach = self.rules.R_obs_m - sp.OBS_MARGIN_M - 0.3
        tasks = [dict(t) for t in tasks]
        best_route = self.tour(sid, tasks, z)
        if best_route is None:
            return None
        best_e = self.energy(sid, best_route)
        start_e = best_e
        for _ in range(sweeps):
            improved = False
            for i, t in enumerate(tasks):
                c = t["cpos"]
                h = math.sqrt(max(0.0, reach * reach - (z - c[2]) ** 2))
                for k in range(angles):
                    th = 2 * math.pi * k / angles
                    pt = (c[0] + h * math.cos(th), c[1] + h * math.sin(th))
                    trial = tasks[:i] + [{**t, "point": pt}] + tasks[i + 1:]
                    route = self.tour(sid, trial, z)
                    if route is None:
                        continue
                    e = self.energy(sid, route)
                    if e < best_e - 1e-6:
                        best_e, best_route, tasks = e, route, trial
                        t = tasks[i]
                        improved = True
            if not improved:
                break
        return best_route if best_e < start_e - 1e-6 else None

    def retime(self, routes):
        """Shift strike groups later where their confirmation now ends too late.
        Returns the new routes, or None if a window cannot absorb the shift."""
        obs_end = {}
        r_obs = self.rules.R_obs_m
        for aid, route in routes.items():
            if self.ac[aid].type != "scout":
                continue
            for w in route:
                if w.get("action") == "observe" and w.get("dwell_s", 0) >= self.rules.obs_dwell_s - EPS:
                    c = self.cs[w["target"]]
                    if math.dist((w["x"], w["y"], w["z"]), c.pos) <= r_obs + EPS:
                        end = w["t"] + w["dwell_s"]
                        obs_end[c.id] = min(obs_end.get(c.id, math.inf), end)
        groups = {}
        for aid, route in routes.items():
            last = route[-1]
            if self.ac[aid].type == "effector" and last.get("action") == "engage":
                groups.setdefault(last["target"], []).append(aid)
        out = dict(routes)
        for cid, aids in groups.items():
            need = obs_end.get(cid, math.inf) + self.rules.confirm_margin_s + 0.01
            if need == math.inf:
                return None
            first = min(routes[a][-1]["t"] for a in aids)
            if first >= need - 1e-9:
                continue
            delta = need - first
            hi = self.cs[cid].window[1]
            if max(routes[a][-1]["t"] for a in aids) + delta > hi - 0.01:
                return None
            for a in aids:
                out[a] = [{**w, "t": w["t"] + delta} for w in routes[a]]
        return out

    def group_of(self, routes, aid):
        last = routes[aid][-1]
        if self.ac[aid].type != "effector" or last.get("action") != "engage":
            return None, []
        cid = last["target"]
        return cid, [a for a, r in routes.items()
                     if self.ac[a].type == "effector" and r[-1].get("action") == "engage"
                     and r[-1]["target"] == cid]

    def shift_group(self, routes, cid, aids, delta):
        """Routes with strike group `aids` on `cid` shifted by delta, or None if that breaks
        its window, its confirmation or the start of time."""
        lo, hi = self.cs[cid].window
        times = [routes[a][-1]["t"] + delta for a in aids]
        if min(times) < lo + 0.01 or max(times) > hi - 0.01:
            return None
        if min(routes[a][0]["t"] for a in aids) + delta < 0.0:
            return None
        out = dict(routes)
        for a in aids:
            out[a] = [{**w, "t": w["t"] + delta} for w in routes[a]]
        return out

    def check(self, routes, repairs=6):
        """Retime strikes, then repair separation by moving conflicting strike groups."""
        fixed = self.retime(routes)
        if fixed is None:
            return None, None
        result = evaluate(self.sc, self.plan(fixed))
        for _ in range(repairs):
            if result.valid:
                return fixed, result
            if any(v.rule != "R8" for v in result.violations):
                return None, result
            v = result.violations[0]
            words = v.message.split()
            pair = [words[0], words[2]]
            best = None
            for aid in pair:
                cid, aids = self.group_of(fixed, aid)
                if not aids:
                    continue
                for delta in (4.0, 8.0, -4.0, 15.0, -8.0, 25.0, -15.0, 40.0):
                    moved = self.shift_group(fixed, cid, aids, delta)
                    # retime() changes nothing iff every strike stays confirmed
                    if moved is None or self.retime(moved) != moved:
                        continue
                    r = evaluate(self.sc, self.plan(moved))
                    if r.valid:
                        return moved, r
                    n_r8 = len(r.violations) if all(x.rule == "R8" for x in r.violations) else 99
                    if best is None or n_r8 < best[0]:
                        best = (n_r8, moved, r)
            if best is None or best[0] >= 99:
                return None, result
            _, fixed, result = best
        return (fixed, result) if result.valid else (None, result)

    # ---- moves --------------------------------------------------------------------
    def candidates(self):
        """(estimated gain, description, new scout routes) for every feasible move."""
        routes = self.routes
        scouts = self.scouts(routes)
        tasks = {s: self.tasks(routes[s]) for s in scouts}
        z = {s: self.altitude(routes[s]) for s in scouts}
        energy = {s: self.energy(s, routes[s]) for s in scouts}
        idle = [a.id for a in self.sc.aircraft if a.type == "scout" and a.id not in routes]
        lam = self.rules.lambda_e
        out = []

        def insert_all(sid, base, task, zz):
            res = []
            seen = set()
            orders = [base[:p] + [task] + base[p:] for p in range(len(base) + 1)]
            full = base + [task]
            for key in (lambda t: t["window"][1], lambda t: t["window"][0],
                        lambda t: t["window"][0] + t["window"][1]):
                orders.append(sorted(full, key=key))
            for new in orders:
                sig = tuple(t["cid"] for t in new)
                if sig in seen:
                    continue
                seen.add(sig)
                route = self.tour(sid, new, zz)
                if route is not None:
                    res.append((self.energy(sid, route), new.index(task), route, new))
            return res

        # rebuild a tour with the same tasks (cheaper speed or departure)
        for x in scouts:
            route = self.tour(x, tasks[x], z[x])
            if route is not None:
                gain = lam * (energy[x] - self.energy(x, route))
                if gain > 0.01:
                    out.append((gain, f"retour {x}", {x: route}))
        # move observation points around their contacts' observation circles
        for x in scouts:
            route = self.repoint(x, tasks[x], z[x])
            if route is not None:
                gain = lam * (energy[x] - self.energy(x, route))
                if gain > 0.01:
                    out.append((gain, f"repoint {x}", {x: route}))
        # relocate one task
        for x in scouts:
            for i, task in enumerate(tasks[x]):
                rest = tasks[x][:i] + tasks[x][i + 1:]
                x_route = self.tour(x, rest, z[x])
                if x_route is None:
                    continue
                ex = self.energy(x, x_route)
                # same tour, other position
                for e, p, route, order in insert_all(x, rest, task, z[x]):
                    if [t["cid"] for t in order] == [t["cid"] for t in tasks[x]]:
                        continue
                    gain = lam * (energy[x] - e)
                    if gain > 0.01:
                        out.append((gain, f"reorder {task['cid']} in {x} -> pos {p}", {x: route}))
                for y in scouts + idle:
                    if y == x:
                        continue
                    base = tasks.get(y, [])
                    zy = z.get(y, z[x])
                    old_y = energy.get(y, 0.0)
                    for e, p, route, _o in insert_all(y, base, task, zy):
                        gain = lam * (energy[x] + old_y - ex - e)
                        if gain > 0.01:
                            out.append((gain, f"move {task['cid']} {x}->{y} pos {p}",
                                        {x: x_route, y: route}))
        # eliminate a scout: greedy cheapest insertion of each of its tasks elsewhere
        for x in scouts:
            new_tasks = {s: list(tasks[s]) for s in scouts if s != x}
            new_routes = {}
            ok = True
            for task in sorted(tasks[x], key=lambda t: t["window"][1]):
                best = None
                for y in new_tasks:
                    for e, p, route, order in insert_all(y, new_tasks[y], task, z[y]):
                        cur = self.energy(y, new_routes.get(y, routes[y]))
                        if best is None or e - cur < best[0]:
                            best = (e - cur, y, order, route)
                if best is not None:
                    _, y, order, route = best
                    new_tasks[y] = list(order)
                    new_routes[y] = route
                    continue
                # Ejection chain: task goes into y in place of one of y's tasks, which
                # moves on to a third scout w.
                for y in new_tasks:
                    cur_y = self.energy(y, new_routes.get(y, routes[y]))
                    for j, ej in enumerate(new_tasks[y]):
                        base = new_tasks[y][:j] + new_tasks[y][j + 1:]
                        fits = insert_all(y, base, task, z[y])
                        if not fits:
                            continue
                        e_y, p_y, r_y, o_y = min(fits, key=lambda f: f[0])
                        for w in new_tasks:
                            if w == y:
                                continue
                            cur_w = self.energy(w, new_routes.get(w, routes[w]))
                            for e_w, p_w, r_w, o_w in insert_all(w, new_tasks[w], ej, z[w]):
                                d = e_y - cur_y + e_w - cur_w
                                if best is None or d < best[0]:
                                    best = (d, y, o_y, r_y, w, o_w, r_w)
                if best is None:
                    ok = False
                    break
                _, y, o_y, r_y, w, o_w, r_w = best
                new_tasks[y], new_tasks[w] = list(o_y), list(o_w)
                new_routes[y], new_routes[w] = r_y, r_w
            if not ok:
                continue
            gain = lam * (energy[x] + sum(energy[y] for y in new_routes)
                          - sum(self.energy(y, r) for y, r in new_routes.items()))
            if gain > 0.01:
                new_routes[x] = []
                out.append((gain, f"eliminate {x}", new_routes))
        # add an unserved unknown contact
        served = {t["cid"] for s in scouts for t in tasks[s]}
        for c in self.sc.contacts:
            if c.cls != "unknown" or c.id in served or c.window is None:
                continue
            for y in scouts + idle:
                zy = z.get(y, sp.SCOUT_ALTITUDES[0])
                scout = self.ac[y]
                for zz in {zy, sp.SCOUT_ALTITUDES[0]}:
                    pt = sp.obs_point(self.sc, scout, c, 0.0, zz)
                    task = {"cid": c.id, "point": pt, "window": c.window, "cpos": c.pos}
                    if y in tasks and zz != zy:
                        continue
                    for e, p, route, _o in insert_all(y, tasks.get(y, []), task, zz):
                        gain = c.points - lam * (e - energy.get(y, 0.0))
                        if gain > 0.01:
                            out.append((gain, f"add {c.id} to {y} pos {p}", {y: route}))
        # add an unserved hostile: observation in a tour plus a new strike group
        engaged = {r[-1]["target"] for r in routes.values() if r[-1].get("action") == "engage"}
        free = [a for a in self.sc.aircraft if a.type == "effector" and a.id not in routes]
        protected = [c for c in self.sc.contacts if c.cls in ("friendly", "civilian")]
        for c in self.sc.contacts:
            if c.cls != "hostile" or c.id in engaged or c.window is None or not c.needs:
                continue
            if any(math.hypot(c.pos[0] - f.pos[0], c.pos[1] - f.pos[1])
                   < self.rules.collateral_radius_m for f in protected):
                continue
            # cheapest free effectors per subtype (straight-line energy estimate)
            team, e_team = [], 0.0
            for sub, k in c.needs.items():
                pool = sorted(
                    (a for a in free if a.subtype == sub),
                    key=lambda a: math.hypot(c.pos[0] - a.home[0], c.pos[1] - a.home[1])
                    * (a.power.p0_w + a.power.k_v * a.v_h_mps ** 2) / a.v_h_mps)
                if len(pool) < k:
                    team = None
                    break
                team += pool[:k]
            if not team:
                continue
            base_gain = c.points - self.rules.c_eff * len(team)
            if base_gain <= 0:
                continue
            for y in scouts + idle:
                zy = z.get(y, sp.SCOUT_ALTITUDES[0])
                pt = sp.obs_point(self.sc, self.ac[y], c, 0.0, zy)
                task = {"cid": c.id, "point": pt, "window": c.window, "cpos": c.pos}
                for e, p, route, _o in insert_all(y, tasks.get(y, []), task, zy):
                    ends = [w["t"] + w["dwell_s"] for w in route
                            if w.get("action") == "observe" and w["target"] == c.id]
                    t_eng = max(ends[0] + self.rules.confirm_margin_s + 0.05,
                                c.window[0] + 0.05)
                    strikes, e_str = {}, 0.0
                    for a in team:
                        _, earliest = sp.low_effector_route(a, c, 1e9, self.sc)
                        t_eng = max(t_eng, earliest + 0.05)
                    if t_eng > c.window[1] - 0.05:
                        continue
                    for a in team:
                        r, _ = sp.low_effector_route(a, c, t_eng, self.sc)
                        if r is None:
                            strikes = None
                            break
                        strikes[a.id] = r
                        e_str += self.energy(a.id, r)
                    if not strikes:
                        continue
                    gain = base_gain - lam * (e - energy.get(y, 0.0) + e_str)
                    if gain > 0.01:
                        out.append((gain, f"strike {c.id} via {y} pos {p}",
                                    {y: route, **strikes}))
        out.sort(key=lambda m: -m[0])
        return out

    def step(self, deadline):
        moves = self.candidates()
        tried = 0
        for gain, desc, change in moves:
            if time.monotonic() > deadline:
                return False
            routes = dict(self.routes)
            for a, r in change.items():
                if r:
                    routes[a] = r
                else:
                    routes.pop(a, None)
            tried += 1
            fixed, result = self.check(routes)
            if fixed is not None and result.score > self.score + 1e-6:
                self.log(f"   +{result.score - self.score:6.2f} (est {gain:5.2f}) {desc} "
                         f"-> {result.score:.2f}  [{tried}/{len(moves)} tried]")
                self.routes, self.score = fixed, result.score
                return True
        return False

    def trimmed(self, route, cut):
        """Effector route ending `cut` m short of the contact on its last leg, same last-leg
        speed, every earlier waypoint shifted later (the pad wait is free). None if no room."""
        a, b = route[-2], route[-1]
        c = self.cs[b["target"]].pos
        dx, dy = a["x"] - b["x"], a["y"] - b["y"]
        d = math.hypot(dx, dy)
        if d < cut + 1.0:
            return None
        f = cut / d
        nb = {**b, "x": b["x"] + dx * f, "y": b["y"] + dy * f}
        if math.dist((nb["x"], nb["y"], nb["z"]), c) > self.rules.R_eng_m - 0.05:
            return None
        dur = b["t"] - (a["t"] + a.get("dwell_s", 0.0))
        shift = dur * f
        # the last leg must still fit the descent
        dz = a["z"] - b["z"]
        if dz > 0 and dur - shift < dz / self.ac_v_down(route) + 0.05:
            return None
        return [{**w, "t": w["t"] + shift} for w in route[:-1]] + [nb]

    def ac_v_down(self, route):
        return self._v_down

    def trim_strikes(self, cut=24.5):
        groups = {}
        for aid, r in self.routes.items():
            if self.ac[aid].type == "effector" and r[-1].get("action") == "engage":
                groups.setdefault(r[-1]["target"], []).append(aid)
        gained = 0.0
        for cid, aids in groups.items():
            routes = dict(self.routes)
            ok = True
            for a in aids:
                self._v_down = self.ac[a].v_down_mps
                r = self.trimmed(routes[a], cut)
                if r is None:
                    ok = False
                    break
                routes[a] = r
            if not ok:
                continue
            res = evaluate(self.sc, self.plan(routes))
            if res.valid and res.score > self.score + 1e-6:
                gained += res.score - self.score
                self.routes, self.score = routes, res.score
        if gained:
            self.log(f"   +{gained:6.2f} trim strikes -> {self.score:.2f}")

    def run(self, seconds):
        deadline = time.monotonic() + seconds
        self.trim_strikes()
        while time.monotonic() < deadline and self.step(deadline):
            pass
        return self.score


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", type=Path, default=ROOT / "output" / "board-best")
    parser.add_argument("--output", type=Path, default=ROOT / "output" / "board-polished")
    parser.add_argument("--scenarios", nargs="+", default=SCENARIOS, choices=SCENARIOS)
    parser.add_argument("--seconds", type=float, default=600.0, help="time cap per scenario")
    args = parser.parse_args()
    total0 = total1 = 0.0
    for name in args.scenarios:
        scenario = Scenario.from_dict(load_json(ROOT / "validation" / f"{name}.json"))
        plan = load_json(args.input / f"{name}.json")
        pol = Polisher(scenario, plan)
        before = pol.score
        print(f"{name}: {before:.2f}", flush=True)
        after = pol.run(args.seconds)
        total0, total1 = total0 + before, total1 + after
        out = args.output / f"{name}.json"
        current = None
        if out.exists():
            r = evaluate(scenario, load_json(out))
            current = r.score if r.valid else None
        if after > before + 1e-6 and (current is None or after > current + 1e-6):
            save_json(pol.plan(), out)
            print(f"{name}: {before:.2f} -> {after:.2f}  saved {out}", flush=True)
        elif current is None and out.resolve() != (args.input / f"{name}.json").resolve():
            save_json(plan, out)
            print(f"{name}: no gain, copied input", flush=True)
        else:
            print(f"{name}: no gain", flush=True)
    print(f"total {total0:.2f} -> {total1:.2f} ({total1 - total0:+.2f})")


if __name__ == "__main__":
    main()
