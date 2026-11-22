"""usage: PYTHONPATH=<fleet-kit>:. python3 analysis/estimate_accuracy.py SCENARIO.json [--per-scout N]

Visit-level calibration of the trip-search value model against single realised plan steps.

Contexts: the trip packing (tree, pricing and repair off, uncapped packing) is realised in
observation-time order and snapshotted after 1/3 and 2/3 of its visits. In each context,
every scout gets up to N probes: the candidate contacts (trip_candidates, raw net value)
that the model's own straight-leg timing says the scout can still append, best first.
Each probe is one realise_visit call (scout visit + strike, incremental validation), the
same step the trip tree and trip moves take. Realised net = points gained - (lambda_e x
energy + c_eff) of the new effector routes; 0 when the step fails.

Each model variant predicts net x keep, where keep comes from visit_keep with the variant's
flags (VISIT_LEARN only sees outcomes of earlier probes). Reported per variant: bias and
mean absolute error of predicted vs realised net, and the Brier score of keep as the
probability that the step realises. Lower |bias|, MAE and Brier mean a closer model.
"""
import argparse
import math
import time

from fleetkit.geometry import hdist
from fleetkit.model import HOSTILE, SCOUT, EFFECTOR, Scenario, load_json

import search_planner as sp
from candidate_matrix import CandidateMatrix

VARIANTS = (("base", False, False), ("sep_risk", True, False),
            ("visit_learn", False, True), ("both", True, True))


def setup(path):
    sp.TRIP_TREE = False
    sp.TRIP_PRICES = False
    sp.SEP_RISK = sp.VISIT_LEARN = False
    sp.TRIP_PACK_S = 30.0
    sc = Scenario.from_dict(load_json(path))
    s = sp.Search(sc, seconds=55.0)
    s.matrix = CandidateMatrix(sc)
    if sp.EXACT_STRIKE_COSTS:
        s.matrix.refine(s.strike_energy)
    s.checker.stop = time.monotonic() + 3600
    s.repair = lambda state, depth: state
    return sc, s


def contexts(s, fractions=(1 / 3, 2 / 3)):
    s.trip_plan(time.monotonic() + 600)
    _, chosen = s.trip_chosen
    visits = sorted(((f, s.aircraft[sid], cid, z) for _, sid, _, (z, order, finishes) in chosen
                     for cid, f in zip(order, finishes)), key=lambda v: (v[0], v[2]))
    marks = {max(1, int(len(visits) * f)) for f in fractions}
    out, state = [], s.root
    for i, (_, scout, cid, z) in enumerate(visits, 1):
        child = s.realise_visit(state, scout, cid, z, time.monotonic() + 600)
        if child is not None:
            state = child
        if i in marks:
            out.append(state)
    s.visit_outcomes.clear()          # the context build is not evidence for the probes
    return out


def model_can_append(s, scout, state, cand, z):
    """The model's own feasibility: straight leg from the tour's last observation at the
    trip-label speed, inside the window and latest finish, home before the horizon."""
    cid, point, (lo, hi), net, latest = cand
    rules = s.sc.rules
    speed = sp.eco_speed(scout) if sp.SCOUT_SPEED_MODE == "eco" else sp.fast_speed(scout)
    route = state.routes.get(scout.id) or []
    obs = [w for w in route if w.get("action") == "observe"]
    if obs:
        last = obs[-1]
        t, pos = last["t"] + last.get("dwell_s", 0.0), (last["x"], last["y"])
    else:
        t, pos = 0.0, scout.home[:2]
    arrive = max(t + hdist(pos, point) / speed, lo)
    finish = arrive + rules.obs_dwell_s
    back = hdist(point, scout.home[:2]) / speed
    if finish > latest + 1e-6 or finish + back > s.sc.horizon_s:
        return False
    # Battery, as trip_labels models it: cruise legs, hover while waiting and dwelling.
    p = scout.power
    cruise_w = p.p0_w + p.k_v * speed * speed
    old_back = hdist(pos, scout.home[:2]) / speed if obs else 0.0
    wait = max(0.0, arrive - t - hdist(pos, point) / speed)
    wh = (state.energies.get(scout.id, 0.0)
          + (cruise_w * (hdist(pos, point) / speed + back - old_back)
             + p.p0_w * (wait + rules.obs_dwell_s)) / 3600.0)
    return wh <= sp.TRIP_BATTERY * scout.usable_wh


def probes(s, state, per_scout):
    out = []
    for scout in (a for a in s.sc.aircraft if a.type == SCOUT):
        route = state.routes.get(scout.id)
        layers = ((max(w["z"] for w in route),) if route else
                  tuple(dict.fromkeys((sp.SCOUT_LAYERS[0], min(sp.SCOUT_LAYERS)))))
        cands = []
        for z in layers:
            for c in s.trip_candidates(scout, state.used, set(state.served), z):
                if model_can_append(s, scout, state, c, z):
                    cands.append((c[3], c, z))
        cands.sort(key=lambda x: -x[0])
        out += [(scout, c, z) for _, c, z in cands[:per_scout]]
    return out


def realised_net(s, state, child):
    if child is None:
        return 0.0
    rules = s.sc.rules
    pts = sum(s.contacts[c].points for c in child.served - state.served)
    cost = sum(rules.lambda_e * child.energies[a] + rules.c_eff for a in child.routes
               if a not in state.routes and s.aircraft[a].type == EFFECTOR)
    return pts - cost


def keep_for(s, flags, sid, cid, point, z):
    sp.SEP_RISK, sp.VISIT_LEARN = flags
    try:
        return s.visit_keep(sid, cid, point, z)
    finally:
        sp.SEP_RISK = sp.VISIT_LEARN = False


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("scenario")
    ap.add_argument("--per-scout", type=int, default=8)
    args = ap.parse_args()
    sc, s = setup(args.scenario)
    start = time.monotonic()
    rows = []          # (net, realised net, success, {variant: keep}, class)
    failures = {}      # (class, most rejected rule during the failed step) -> count
    for state in contexts(s):
        for scout, cand, z in probes(s, state, args.per_scout):
            cid, point, _, net, _ = cand
            keeps = {name: keep_for(s, (sep, learn), scout.id, cid, point, z)
                     for name, sep, learn in VARIANTS}
            before = dict(s.checker.rejections)
            child = s.realise_visit(state, scout, cid, z, time.monotonic() + 600)
            if child is None:
                delta = {k: v - before.get(k, 0) for k, v in s.checker.rejections.items()
                         if v > before.get(k, 0)}
                rule = max(delta, key=delta.get) if delta else "no option"
                failures[(s.contacts[cid].cls, rule)] = failures.get((s.contacts[cid].cls, rule), 0) + 1
            rows.append((net, realised_net(s, state, child), child is not None, keeps,
                         s.contacts[cid].cls))
    n = len(rows)
    print(f"{args.scenario}: {n} probes in {time.monotonic() - start:.1f} s, "
          f"realised {sum(r[2] for r in rows)}/{n}, "
          f"hostiles {sum(r[4] == HOSTILE for r in rows)}")
    real = [r[1] for r in rows]
    print(f"  mean realised net {sum(real) / n:.2f}")
    print(f"  {'variant':12} {'mean pred':>9} {'bias':>7} {'MAE':>7} {'Brier':>7}")
    for name, _, _ in VARIANTS:
        pred = [r[0] * r[3][name] for r in rows]
        bias = sum(p - q for p, q in zip(pred, real)) / n
        mae = sum(abs(p - q) for p, q in zip(pred, real)) / n
        brier = sum((r[3][name] - (1.0 if r[2] else 0.0)) ** 2 for r in rows) / n
        print(f"  {name:12} {sum(pred) / n:9.2f} {bias:7.2f} {mae:7.2f} {brier:7.3f}")
    print("  failed steps by class and most rejected rule:",
          ", ".join(f"{c}/{r}: {k}" for (c, r), k in sorted(failures.items(), key=lambda x: -x[1])))
    ok = [r for r in rows if r[2]]
    if ok:
        err = sum(r[0] - r[1] for r in ok) / len(ok)
        print(f"  realised steps only: mean (model net - realised net) {err:.2f} "
              f"(strike cost and points error, base model)")


if __name__ == "__main__":
    main()
