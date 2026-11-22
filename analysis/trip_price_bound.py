"""usage: PYTHONPATH=<fleet-kit>:. python3 analysis/trip_price_bound.py SCENARIO.json

Runs 300 rounds of contact pricing (TRIP_PRICES) without a time limit and prints the
Lagrangian bound on the trip packing value, the best packing found and its realised score.
The bound is in the planner's estimate model (label-search pruning, cheapest-team costs).
"""
import sys, time
from fleetkit.model import Scenario, load_json
import search_planner as sp
from candidate_matrix import CandidateMatrix
sp.TRIP_PRICE_ROUNDS = 300; sp.TRIP_PRICE_SHARE = 1.0; sp.TRIP_TREE = False
sc = Scenario.from_dict(load_json(sys.argv[1]))
s = sp.Search(sc, seconds=55.0); s.matrix = CandidateMatrix(sc)
s.checker.stop = time.monotonic() + 1000
s.repair = lambda st, d: st
s.trip_plan(time.monotonic() + 400)
print(sys.argv[1], s.trip_stats['prices'], 'estimate', s.trip_stats['estimate'], 'realised', s.trip_stats['realised_score'])
