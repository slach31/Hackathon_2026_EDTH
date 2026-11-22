"""usage: PYTHONPATH=<fleet-kit>:. python3 analysis/estimate_gap.py SCENARIO.json

Splits the trip plan's estimate-vs-realised gap (trip tree and pricing off, no repair)
into: points of visits that were not realised, scout energy, and strike (effector) cost,
per scout and per hostile.
"""
import sys, time
from fleetkit.model import Scenario, load_json, HOSTILE, EFFECTOR, SCOUT
import search_planner as sp
from candidate_matrix import CandidateMatrix

sp.TRIP_TREE = False
sp.TRIP_PRICES = False
sp.TRIP_PACK_S = 30.0     # deterministic packing (no time cap)
sc = Scenario.from_dict(load_json(sys.argv[1]))
s = sp.Search(sc, seconds=55.0)
s.matrix = CandidateMatrix(sc)
if sp.EXACT_STRIKE_COSTS: s.matrix.refine(s.strike_energy)
s.checker.stop = time.monotonic() + 600
s.repair = lambda st, d: st
state = s.trip_plan(time.monotonic() + 300)
cands, chosen = s.trip_chosen
lam, ceff = sc.rules.lambda_e, sc.rules.c_eff
contacts = sc.contact_by_id()
ac = sc.aircraft_by_id()

est_pts = est_team = est_scout = 0.0
real_pts = real_team = real_scout = 0.0
lost_pts = lost_team = 0.0
rows = []
team_rows = []
for value, sid, cset, (z, order, finishes) in chosen:
    net = {c[0]: c[3] for c in cands[sid, z]}
    wh = (sum(net[c] for c in order) - value) / lam
    pts = sum(contacts[c].points for c in order)
    team = sum(contacts[c].points - net[c] for c in order)
    est_pts += pts; est_team += team; est_scout += lam * wh
    done = [c for c in order if c in state.served]
    lost = [c for c in order if c not in state.served]
    lost_pts += sum(contacts[c].points for c in lost)
    lost_team += sum(contacts[c].points - net[c] for c in lost)
    real_wh = state.energies.get(sid, 0.0)
    real_scout += lam * real_wh
    rows.append((sid, z, len(order), len(lost), round(wh, 1), round(real_wh, 1)))
    for c in done:
        if contacts[c].cls != HOSTILE:
            continue
        effs = [aid for aid, r in state.routes.items() if ac[aid].type == EFFECTOR
                and any(w.get('target') == c and w.get('action') == 'engage' for w in r)]
        real = sum(lam * state.energies[a] + ceff for a in effs)
        team_rows.append((c, round(contacts[c].points - net[c], 1), round(real, 1), len(effs),
                          [round(r[-1]['z']) if False else round(max(w['z'] for w in state.routes[a])) for a in effs]))
        real_team += real
for c in state.served:
    real_pts += contacts[c].points

print(sys.argv[1])
print(f"estimate {sum(v for v, *_ in chosen):.1f}   realised score {state.score:.1f}")
print(f"  points       est {est_pts:8.1f}  real {real_pts:8.1f}   (lost visits {lost_pts:.1f})")
print(f"  strike cost  est {est_team:8.1f}  real {real_team:8.1f}   (est for lost {lost_team:.1f}; est for realised {est_team - lost_team:.1f})")
print(f"  scout energy est {est_scout:8.1f}  real {real_scout:8.1f}   (lambda*Wh)")
print("  per scout (sid, z, visits, lost, est Wh, real Wh):")
for r in rows:
    print("   ", r)
team_rows.sort(key=lambda r: r[1] - r[2])
print("  worst strikes (cid, est cost, real cost, n effectors, max z per effector):")
for r in team_rows[:8]:
    print("   ", r)
print("  hostiles realised:", len(team_rows), " est-real total", round(sum(r[1] - r[2] for r in team_rows), 1))

print("  lost visits (cid, class, points, est net, recorded outcomes / rules):")
for value, sid, cset, (z, order, finishes) in chosen:
    net = {c[0]: c[3] for c in cands[sid, z]}
    for c in order:
        if c not in state.served:
            st = s.stats(c)
            print("   ", sid, c, contacts[c].cls, contacts[c].points, round(net[c], 1),
                  dict(st.get("outcomes", {})), dict(st.get("rules", {})))
