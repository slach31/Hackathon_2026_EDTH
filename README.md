# Fleet search planner

A standard-library planner that turns the baseline's route candidates into a bounded search tree, then spends the last part of the budget repairing the best plan with large neighbourhood search (LNS). It always keeps valid partial fleet plans, returns the best plan found, and can export the best N distinct plans and an interactive tree.

Run from this directory:

```sh
python3 planner.py path/to/scenario.json output/plan.json
```

The default search deadline is **45 seconds from startup**, including input loading. This leaves 15 seconds of the organisers' 60-second limit for full validation, writing the result and a slower grading machine (`--seconds` up to 55 for local experiments). The required plan is written before optional diagnostics. Python imports and individual validator calls cannot be preempted: this is a cooperative deadline, not a guarantee against arbitrary machine stalls.

For a shorter run with five candidate plans and the tree:

```sh
python3 planner.py path/to/scenario.json output/plan.json \
  --seconds 5 --top-n 5 --beam-width 6 --lns-share 0.4 --artifacts output/search
```

Open `output/search/tree.html` in a browser. Drag to pan, scroll to zoom, click nodes to inspect decisions, and click a ranked solution to highlight its path. The directory also contains `tree.json` with parent links, decisions, scores and rejection counts, plus `plan-01.json`, `plan-02.json`, etc. These are ordinary plans accepted by the official validator. There may be fewer than N if the run finds fewer distinct valid plans.

To watch the winning fleet fly, use the official viewer:

```sh
PYTHONPATH=path/to/fleet-kit python3 -m fleetkit view \
  path/to/scenario.json output/plan.json -o output/flight.html
```

## Why it can beat greedy

The baseline processes hostiles in descending point value, chooses the nearest required effectors, and accepts the first valid improvement. It then adds observations of unknown contacts. An early choice can consume aircraft that would earn more points in a later combination, and the baseline does not revisit that choice.

The search reuses the baseline's route builders and explores alternatives at those decision points:

| Decision | Greedy baseline | Search planner |
|---|---|---|
| Contact selection | Commits to the first improving assignment in points order | Keeps serve and skip branches, allowing resources to be saved for later contacts |
| Effector allocation | Chooses the nearest available group of the required subtypes | Tries up to three groups ranked by estimated energy and effector cost |
| Scout and flight settings | Stops at the first valid improvement | Can retain multiple assignments and flight settings across branches |
| Contact order | Hostiles by points, then unknowns by points | Explores scarcity/value, window-close, points and seeded randomised orders |
| Candidate checking | Revalidates the entire plan for each proposal | Reuses unchanged-route checks and checks every separation pair involving a changed route |
| Revisiting choices | Never removes an accepted assignment | LNS removes a few served contacts and reinserts contacts, so resources move to better combinations |
| Scout tours | Appends each visit to the end of a tour | LNS also tries the cheapest earlier positions in each tour |

For example, one test gives a 90-point contact both available light effectors, while two 70-point contacts each need one. Greedy commits both effectors to the 90-point contact. A skip branch keeps them available for the 140-point pair; the test confirms that the resulting valid plan also wins after energy and effector costs.

Incremental validation makes more candidate checks possible within the same deadline. The matrix helps spend those checks on promising groups; the official rules still decide validity. The benchmark below shows higher scores on all four sampled scenarios, including a **43.5% improvement on L4-0** with the full budget. These measurements do not establish that every mechanism contributes equally or that every unseen scenario will improve. The capped greedy warm start can also stop before completing the baseline, so there is no universal guarantee of matching its score.

## How the tree works

Each node holds routes, scout tasks, consumed effectors and a score. At each contact the search can skip it or serve it with different scouts, effector groups and flight settings. Serving a hostile constructs its observation and complete required strike group together. Scouts always have return-and-land legs. Extending a scout task list rebuilds its tour and rechecks earlier confirmations.

- `candidate_matrix.py` ranks effector/contact choices using the distance/energy estimates from the analysis matrix. Estimated time and energy do **not** exclude candidates. Only exact subtype availability and the official collateral restriction exclude groups. At most three shortlisted teams are tried, so the search remains heuristic.
- `search_planner.py` first follows the original greedy choices with faster validation, for up to a quarter of the budget (capped at 10 seconds). If interrupted, it retains the last accepted plan. It then searches beams using scarcity/value, window-close, points and seeded randomised orders.
- Every extension applies the official local rules to changed routes, rechecks confirmation and strike completeness globally, and checks separation between changed routes and all other routes. Unchanged pairs retain their parent's validation result. Energy and separation flight pieces are cached per branch.
- The beam keeps at most `--beam-width` distinct states at a depth. A separate archive keeps up to `--top-n` distinct valid plans across the entire search. The two settings are independent. Distinct means different assignments or timed waypoints, so solutions may serve the same contacts.
- When the beam phase ends, a greedy **repair** pass fills the best plan: unserved unknowns, then hostiles, are inserted at the cheapest scout-tour positions with the free effectors (at most 0.25 s per contact and a quarter of the LNS share). LNS then starts from the best archived plan.
- Route builders reuse memoised zone detours, and candidates whose waypoints fall outside the horizon or a contact's window are rejected before parsing. Duplicate plans are detected from per-route waypoint tuples.
- The last `--lns-share` of the budget (default 0.75; it was 0.4 until 22 Nov: 0.75 scored +1.6% on 12 L4 scenarios, 9 better / 3 worse, +0.35% on L3, identical on L1-L2) runs LNS on the best plan. Each iteration removes 1–4 served contacts, chosen at random or as neighbours of a random contact, rebuilding the affected scout tour from its original departure. It then tries to reinsert the removed contacts, their six nearest unserved neighbours and three random unserved contacts. Each scout visit is tried at the three cheapest tour positions (added distance), and the best valid child per contact is kept. Worse plans are sometimes accepted, with a temperature of 1% of the best score falling to zero. After 25 iterations without a new best, the search restarts from a random plan in the top-N archive (`--no-restarts` returns to the best plan instead) and stops after 400 iterations without a new best. Accepted plans enter the archive, so they receive the same final official validation.
- Expansion has local time caps, preventing an impossible contact from consuming the whole run. Children must improve the parent's actual score. All archive winners receive a full official validation and score before export; final validation stops starting new calls at 58 seconds. The valid empty plan is the last fallback.

The result is the best N plans **found**, not a proof of the global top N. Contact ordering, a bounded beam, fixed flight settings, shortlists and the improving-child restriction can all discard useful alternatives. The trace records accepted nodes and skip branches; failed route attempts are aggregated by rule, rather than stored individually. Trace growth is capped between passes to keep tiny scenarios from producing unlimited diagnostics.

## Checks and benchmarks

```sh
python3 -m unittest discover -s tests -v
python3 analysis/benchmark_search.py --seconds 5 --cases L1:0 L2:7 L3:1 L4:0
python3 analysis/benchmark_search.py --levels L1,L2,L3,L4 --seeds 0-4 --workers 4
```

Tests compare incremental validation with the full official validator on perturbed routes, check conflicts with existing aircraft and broken earlier confirmations, verify branch independence, and demonstrate a skip branch beating greedy allocation. A subprocess test copies only the hand-in modules and checks the CLI and exported plans.

### Batch benchmarks

The runner defaults to **12 scenarios** (L1–L4, seeds 0–2), a **55-second search budget** and one worker. Choose a seed range or explicit cases, and optionally run scenarios concurrently:

```sh
# 20 scenarios with four workers and the normal search budget
python3 analysis/benchmark_search.py --levels L1,L2,L3,L4 --seeds 0-4 --workers 4

# Reproducible sequential comparison: no competing benchmark workers
python3 analysis/benchmark_search.py --levels L1,L2,L3,L4 --seeds 0-4 --workers 1

# Quick smoke batch; short searches can score worse than greedy
python3 analysis/benchmark_search.py --levels L1,L2 --seeds 0-2,7 --seconds 5 --workers 2

# Select particular cases and a new output directory
python3 analysis/benchmark_search.py --cases L3:7 L4:13 --workers 2 \
  --output output/benchmarks/selected-cases
```

Each completed case prints greedy's score, our score, the gain, percentage, runtime and statuses. The final summary includes **total and per-level scores, wins/ties/losses, valid outputs, and failure counts**. The total is the sum of raw scores, matching the competition ranking; aggregate improvement is calculated from the totals.

Open the printed `report.html` path to inspect the same comparison in a browser. It refreshes every five seconds while the batch runs. Each run saves:

- `run.json` and `summary.json`: cases, budgets, worker count and aggregate results.
- `results.jsonl`: one flushed record per completion; `results.json` and `results.csv`: ordered results.
- A directory per case with `scenario.json`, `greedy.plan.json`, `search.plan.json`, and both process logs.

Greedy and our planner run separately on the **same generated input**, each with a **60-second hard process limit**. Both plans are checked by the official scorer. Crashes, timeouts, missing output, malformed JSON and invalid plans score zero and do not abort the remaining cases. Generation/setup errors are recorded and excluded from score comparisons. The runner exits nonzero if either planner or scenario setup failed; losing to greedy is a completed comparison, not an execution error.

Generation and the external scoring calls are outside each planner's measured runtime. Parallel workers share CPU, so their timing and time-bounded search scores can differ from a single-worker run. Use `--workers 1` when assessing performance under the competition's per-scenario budget. A custom candidate can be compared using `--planner path/to/planner.py`. Existing output directories are rejected to prevent overwriting results.

### LNS batch result

Both planners ran at full budget under the same load, four workers each at the same time, alongside greedy. Seeds 3–5 were not used during development. The validation scenarios ran 16 planners at the same time on eight cores; the greedy total comes from PLAN.md. `--lns-share 0.6` scored 4391.13 on the seeds 3–5 batch, within run-to-run noise of the default.

| Batch | Greedy | Beam only (master) | Beam + LNS | Gain over master |
|---|---:|---:|---:|---:|
| L1–L4, seeds 0–2 | 3622.86 | 4556.95 | 5208.17 | +14.3% |
| L4 only, seeds 0–2 | 1354.26 | 1899.92 | 2452.72 | +29.1% |
| L3–L4, seeds 3–5 | 2913.46 | 3881.74 | 4334.80 | +11.7% |
| 8 public validation scenarios | 2664.5 | 3674.65 | 4054.29 | +10.3% |

Beam + LNS matched or beat master on every case and greedy on all 18. Every plan was valid and the slowest run took 55.29 seconds. These are sampled scenarios, not a guarantee for unseen inputs.

### Consolidated branch result

The repair pass and LNS were merged with the upstream opt-in `--alns` phase, then rerun at 55 s with one worker. Each batch below ran concurrently with up to two other benchmark batches on eight cores.

| Batch | Greedy | Beam + repair (fork master) | Beam + repair + LNS (default) | `--alns` |
|---|---:|---:|---:|---:|
| L1–L4, seeds 0–2 | 3622.86 | 4655.96 | 5251.52 (+12.8%) | |
| L4 only, seeds 0–2 (after rebasing on upstream) | 1354.26 | 1998.04 | 2412.22 | 2224.16 |

Every plan was valid and the slowest run took 55.25 seconds. L4-2 scored 665.14 in the first batch and 562.88 after the rebase, so single L4 cases vary by up to about 15% between runs.

A second head-to-head at 45 s with one worker per planner (L1–L4, seeds 15–17, two planners at the same time) gave LNS 5623.67 and `--alns` 5435.37: LNS won all three L4 cases (2883.93 vs 2680.64) and the two tied on L1–L3. The `--alns` option was then removed, so LNS is the only improvement phase.

### Mixing the search strategies

Three variants were compared with the LNS described above (which then always restarted from the best plan):

- `--interleave`: alternate each beam pass with an LNS burst (burst length = pass time × share / (1 − share)), then a final LNS tail of 10% of the budget.
- `--adaptive`: adaptive LNS. Destroy operators `random`, `related`, `worst` (lowest-priority served contacts) and `tour` (contacts of one scout tour) are drawn by weight; weights move 20% towards a reward of 3 (new best), 1 (accepted improvement), 0.3 (accepted worse plan) or 0.
- `--restarts`: when stuck, restart from a random archived plan instead of the best.

All variants ran at full budget with two workers each, four planners at the same time on eight cores. None of these seeds were used during development.

| Batch | Restart from best | Interleave | Adaptive | Archive restarts | Restarts + adaptive | All three |
|---|---:|---:|---:|---:|---:|---:|
| L2–L4, seeds 6–8 | 4568.33 | 4535.74 | 4473.54 | **4619.43** | | |
| L3–L4, seeds 9–11 | 3515.57 | 3594.35 | 3634.68 | **3659.48** | | |
| L3–L4, seeds 12–14 | 4245.88 | | | **4265.76** | 4210.44 | 4219.51 |

Archive restarts scored highest in all three batches (12544.67 vs 12329.78 over the 21 cases, +1.7%) and are now the default. Per case they won 10, tied 2 and lost 9 against restarting from the best plan; the gain comes mostly from a few large L4 wins, so it is small relative to run-to-run noise. Combining restarts with the other variants scored lower than restarts alone. Every run in these batches was valid and finished inside the time limit.

### Parallel batch result

A full-budget batch over **L1–L4, seeds 0–2**, with **four concurrent workers**, completed with **12 wins, no ties or losses, and 12 valid outputs**. Neither planner crashed or timed out. These cases overlap earlier tests and are not a held-out evaluation.

| Level | Greedy total | Our total | Gain |
|---|---:|---:|---:|
| L1 | 291.65 | 332.34 | +14.0% |
| L2 | 731.36 | 840.72 | +15.0% |
| L3 | 1245.60 | 1467.69 | +17.8% |
| L4 | 1354.26 | 1837.14 | +35.7% |
| TOTAL | 3622.86 | 4477.89 | +23.6% |

The slowest candidate run finished in 55.23 seconds. Parallel CPU contention can change these scores; use one worker for a sequential comparison. The local report is `output/benchmarks/batch-12-parallel/report.html`. Reproduce the batch with:

```sh
python3 analysis/benchmark_search.py --levels L1,L2,L3,L4 --seeds 0-2 --workers 4 --seconds 55
```

### Repair and cheaper checks

Same inputs, master vs this change, both with two concurrent benchmark workers per batch running at the same time:

| Batch | Master | With repair | Change |
|---|---:|---:|---:|
| 8 public validation scenarios, 55 s | 3768.65 | 3790.20 | +0.6% |
| L1–L4 seeds 3–4, 55 s | 3164.20 | 3190.15 | +0.8% |
| L1–L4 seeds 0–2, 20 s (four workers, run separately) | 4417.21 | 4514.93 | +2.2% |

All plans were valid. Gains come from L4 (e.g. L4-401 1077.92 → 1118.71, L4-0 at 20 s 834.78 → 861.03); L1–L3 mostly tie but finish sooner. L4-402 regressed (613.23 → 593.99) and L4-2 at 20 s slightly (447.65 → 446.43): faster checks change which branches fit in the local time caps, so individual cases vary.

### Scout trip label search

On L4 the scouts are the bottleneck: they fly 9–16 km for 2–6 observations, and the choice of contacts each one chains matters more than how its route is shaped. `trip_labels` finds the best-value trips for one scout by resource-constrained label setting. A label is a chain of visits, with its finish time, Wh used, net value and visited set. Legs are straight lines using the `tour_estimate` speed, departure and energy model. Hostiles must also finish early enough for confirmation and their cheapest free team. Dominated labels are dropped, and each contact keeps at most 6. A run takes 1–8 ms per scout on L4. The search is used in two ways:

- **LNS operator `trip`** (`TRIP_SEARCH`, 20% of LNS steps): removes one scout's tour and the strikes it confirms, then rebuilds it visit by visit from a better trip through `_serve`, retrying layers, angles and takeoff shifts. It reports `trip_tried`/`trip_accepted`/`trip_improvements`/`trip_gain` in the `lns` block.
- **Trip-plan warm start** (`TRIP_PLAN`, at most 20% of the start-to-beam-end time): generates candidate trips for every scout at 70 m and 30 m. The candidates are the best trips, the best trip including each contact, the best trip without each of its two most valuable contacts, and two rounds of re-planning each scout around the others' chosen trips. Without that variety, every scout's top trips share the same few contacts and nothing can be traded. `pack_trips` then picks at most one trip per scout with no contact twice (weighted set packing: greedy, then swaps). The chosen plan is realised in observation-time order, recorded and repaired. Stats are in the `trip_plan` block.

Same-run comparisons, 17 scenarios (8 board validation, L4 seeds 9–14, L3 seeds 3–5), 4 workers, default 45 s, all plans valid:

| Run | base (035a098) | + trip LNS | + trip plan |
|---|---:|---:|---:|
| run 1 total | 13807.5 | 13889.4 | – |
| run 2 total | 13740.3 | 13750.5 | 13918.9 |
| run 2 L4 subtotal | 9907.7 | 9917.2 | 10074.5 |
| run 2 board-8 subtotal | 4790.5 | 4821.3 | 4823.6 |

Repeat runs of the same code differ by 0.4–1% in total; in a third run the committed version scored 14027.9. Step 2's gain over step 1 comes mostly from two large wins (L4-11 +104, L4-14 +53); it won 9 cases and lost 6. Under 4 parallel workers the trip plan reaches its time cap before realisation finishes on the largest cases. Two variants scored lower in the same run: capping generation at 40% of the share (−37) and generating at one layer only (−133).

#### Validated trip tree (`TRIP_TREE`)

The packing picks one combination of trips by estimated value, and only that one is realised. On the board L4 cases that combination lost 6–12% when realised (L4-401: estimate 1623, realised 1436; L4-402: 1213 to 1084). The trip tree searches combinations of the same pregenerated trips with validated nodes:

- one depth per scout, scouts with the most valuable packed trip first;
- each node's children are the scout's best trips that are disjoint from the node's contacts. These are ranked by trip value plus a greedy estimate for the remaining scouts. The packed trip and "no trip" are always options;
- each child is realised through `_serve` (strikes included) and incrementally validated. The beam (`TRIP_TREE_WIDTH` 6, `TRIP_TREE_BRANCH` 5) ranks nodes by real score plus the remaining-scout estimate, scaled by the packing's realised/estimated ratio;
- it is anytime: each depth gets an equal share of the remaining time, and options are tried round-robin across the beam, so a complete plan is always reached. The best of the tree and the realised packing is repaired as before. Trip generation, the packing and the tree share `TRIP_TREE_SHARE`, 20% of the whole budget. This window may run past the end of the beam phase, taking the time from LNS. With the default LNS share of 0.75, a share of the beam phase left the tree 0.2–0.4 s on L4-401 under load. It then stopped at depth 1–3 and went unused.

Every node is a valid plan with an official score, and node validation is cheap because `route_info` and R8 pair results are memoised across siblings. A full tree on L4 costs about 3.5 s, about 230 realised nodes. Stats are in `trip_plan.tree`.

The first version kept only each scout's top 20 trips. Those overlap the contacts earlier scouts had already taken, so the estimate collapsed to zero and the beam collapsed to 3 nodes. A fixed-width version without time shares reached only depth 5 of 15 under load and lost 25 points on L4-401.

Same-run comparisons, 12 concurrent planners on 12 cores, default 45 s, all plans valid. "Off" is the same code with `TRIP_TREE = False`:

| Case | LNS 0.4: off | LNS 0.4: tree | Δ | LNS 0.75: off | LNS 0.75: tree | Δ |
|---|---:|---:|---:|---:|---:|---:|
| L4-401 (6 off / 3 tree runs) | 1577.3 | 1600.0 | +22.7 | 1568.0 | 1586.5 | +18.5 |
| L4-402 (6 / 3) | 1136.0 | 1147.4 | +11.4 | 1157.0 | 1164.1 | +7.1 |
| L1–L3 board cases, 1 run each | 2215.4 | 2213.4 | −1.9 | 2218.9 | 2222.4 | +3.5 |

Before LNS, the tree beat the realised packing in every 0.75 run: +11 to +48 on L4-401 and +111 to +178 on L4-402. LNS recovers much of that difference. Run-to-run spread on L4 is about ±30, so the L4-402 gains and the L1–L3 differences are within noise.

#### Contact prices (`TRIP_PRICES`, off)

This is a Lagrangian relaxation of "no contact in two trips". Each round, every scout runs `trip_labels` with each contact's net value lowered by that contact's price. Contacts claimed by several scouts become more expensive and unclaimed priced contacts become cheaper, using a subgradient step sized against the best packing (Polyak step). Every trip found joins the packing pool at its true value. The sum of the prices plus each scout's best priced trip bounds the packing value. The bound holds only in the planner's estimate model, and only up to the label search's pruning.

Converged bounds after 300 rounds without a time limit (`analysis/trip_price_bound.py`):

| Case | bound | best packing | gap | that packing, realised |
|---|---:|---:|---:|---:|
| L4-401 | 1703.6 | 1657.9 | 46 (2.7%) | 1394.6 |
| L4-402 | 1272.0 | 1231.8 | 40 (3.2%) | 914.6 |

Choosing trips is already within about 3% of optimal in the estimate model. The estimate itself is the bottleneck: the packing that is best on paper realises 16–26% below its estimate. Optimising harder against an optimistic model selects the trips it overrates most. That is why the validated trip tree helps and better packing does not.

End to end (L4 board cases, 45 s), pricing improved the warm start when it had enough rounds (L4-401 tree before LNS: 1466 → 1588, unloaded). LNS reached the same level without it. Same-batch final-score differences against the trip-tree branch were +12/+9 and +18/+8 at 12 concurrent planners. A fix to the time budget was then tried, giving −13/−4, also at 12 concurrent planners. At 6 concurrent planners (4 runs each): branch 2803.8, prices 2784.2, prices with the hard re-plan rounds 2782.0. All of this is within noise, so it stays off.

#### Where the trip estimate goes wrong (`analysis/estimate_gap.py`)

The script realises the trip packing with the tree, pricing and repair all off, and with no time cap on the packing, so its output is deterministic. It then splits the gap between estimated and realised value into three parts. Board L4 cases, before the changes below:

| | L4-401 | L4-402 |
|---|---:|---:|
| estimate − realised | 187 | 129 |
| visits not realised (points − their estimated strike cost) | 162 | 94 |
| strikes costing more than estimated | 42 | 36 |
| scout energy | ≈ 0 | ≈ 0 |

Lost visits fail on the scout route, mostly R8 and sometimes R1. Most R8 rejections pair a scout with an effector (244 of 421 on L4-401; 91 are scout–scout). The 30 m scout layer was added before ground-level strikes existed. Ground strike lines pass 30 m below such a scout, inside d_sep = 40 m. The strikes are then pushed to the 85/130 m layers, which is the strike-cost excess, and scout visits near strike lines fail. The speed assumption contributes almost nothing: effectors fly at 0.995 × v_h, and their best-range speed is at or above v_h.

Two changes:

- `SCOUT_LOW_Z = 45.0` replaces the 30 m scout layer. It clears every ground strike, observes from about 143 m instead of about 147 m, and costs about 0.3 Wh more climb per scout.
- `EXACT_STRIKE_COSTS`: candidate-matrix costs come from the energy of the route `strikes()` tries first (ground, else 30 m), not from the distance-plus-40 m-climb estimate. `CandidateMatrix.refine` applies them.

A single realised packing is chaotic: one changed R8 conflict early on reorders everything after it. Single runs of this script move by ±200 between settings, so the changes were judged end to end. 45 s default, 8 concurrent planners (4 batches) for the first table:

| L4 total (4 runs each) | 30 m | 45 m |
|---|---:|---:|
| estimated costs | 2753.8 | 2752.0 |
| exact costs | 2757.9 | 2775.1 |

Neither change helps alone. Together, on the board cases:

| Case | before | 45 m + exact | Δ |
|---|---:|---:|---:|
| L4-401 (10 runs each) | 1596.2 | 1627.0 | +30.8 (≈ 5 standard errors) |
| L4-402 (10 runs each) | 1157.6 | 1153.1 | −4.5 (noise) |
| L1–L3, 6 cases, 2 runs each | 2217.5 | 2222.7 | +5.2 (no case lower) |

On L4-402, zones block most ground lines, so its strikes fly at 30 m and a 45 m scout is still only 15 m above them.

#### Effector assignment post-pass (`ASSIGN_PASS`)

After LNS, the best plan's strikes are reassigned per subtype by a min-cost assignment (`min_cost_assignment`, a stdlib Hungarian algorithm). It uses the exact strike costs, and an effector counts as infeasible when it cannot arrive within the strike window. Teams that change are removed and rebuilt through `strikes()` with incremental validation. The result is kept only if the plan's score rises. The pass takes about 0.1 s; LNS stops `ASSIGN_S` (0.5 s) early to leave room for it. Stats are in `assign`.

There is little left to gain. On board L4 final plans the estimated assignment gain was 5.6 (L4-401) and 0.1 (L4-402), and the realised gain was +1.5 on L4-401. Measured against each effector's cheapest first-choice route, the final plans overspend 7.2 (L4-401) and 10.0 (L4-402) on strikes in total. That excess comes mostly from L4-402's effectors at 85/130 m, which zones force there. The effector side, assignment and layers together, holds at most about 10–15 points per L4 case. Earlier estimates of 12–30 were measured against older plans.

#### Visit-level model accuracy (`analysis/estimate_accuracy.py`)

This harness measures the trip-search value model against single realised plan steps, with no planner runs. The trip packing is realised and snapshotted after ⅓ and ⅔ of its visits. In each snapshot, every scout gets up to 8 probes: the candidate contacts that the model's own straight-leg timing and battery check say it can still append. Each probe is one `realise_visit` call, which builds the scout visit and its strike with incremental validation. Realised net is the points gained minus (λ·Wh + c_eff) of the new effector routes, and 0 when the step fails. Each variant predicts net × `visit_keep`; `VISIT_LEARN` sees only earlier probes. The harness reports bias, mean absolute error and the Brier score of keep as the probability that the step realises. Runs take about 6 s per case and are deterministic.

Board L4 cases, default model (exact strike costs, 45 m scouts):

| | L4-401 (184 steps) | L4-402 (190 steps) |
|---|---:|---:|
| steps that fail | 12.5% (20 of 23: hostile, R8) | 3.7% |
| model − realised net, realised steps only | 0.09 | 0.19 |
| bias / MAE / Brier: base | 8.73 / 8.73 / 0.125 | 2.41 / 2.41 / 0.037 |
| `SEP_RISK` | 8.20 / 9.11 / 0.128 | 0.18 / 3.99 / 0.041 |
| `VISIT_LEARN` | 7.74 / 8.83 / 0.121 | 2.41 / 2.41 / 0.037 |
| both | 7.22 / 9.22 / 0.123 | 0.18 / 3.99 / 0.041 |

For single steps that realise, the value model is now almost exact. What remains is feasibility, and on L4-401 that is mostly hostile visits whose strike cannot be routed past the scouts already flying. `SEP_RISK` (observation points near strike lines) does not identify those steps. It discounts many visits that succeed, so its MAE and Brier are worse. `VISIT_LEARN` helps slightly where failures repeat. Both flags stay off. A better separation term would test the strike team's route against the current scout routes, which depends on the state.

The first version of the harness omitted the battery check that the trip model applies. That made 29% of probes fail on scout R3, an artefact of the harness. While checking it, 41 (L4-401) and 123 (L4-402) effector/contact pairs turned out to be too slow for their window yet still priced as feasible. `STRIKE_COST_FALLBACKS` (off) marks those pairs infeasible and prices first-route failures by the first valid layer and angle.

#### LNS stopping early (fixed)

About 5% of L4 runs, in every version including the earlier master, ended LNS 20–30 s before the deadline after only a few iterations. When one move hit its own short local deadline, its `DeadlineReached` escaped `lns_step`. The LNS loop treated that as the end of LNS. Such runs were the low outliers; one was master's 1492 on L4-401. A local deadline now ends only that move, counted as `lns.local_timeouts`. After the fix, 32 L4 runs at 6 concurrent planners had none end early. L4-401 scored 1627.7 ± 10.7 (worst 1608.8) and L4-402 1160.8 ± 12.3. The means are unchanged; the spread on L4-401 halved.

#### Trip tree width and time share (flags off)

In the deterministic tree (uncapped packing, equal build budget, before LNS), the tree's result depends on width chaotically. On L4-401, widths 6, 9, 12 and 16 give 1472.7, 1403.8, 1552.4 and 1419.5. On L4-402 almost every setting gives 1153.6. Variants tried, all off by default:

- `TRIP_TREE_ADAPTIVE = True`: margin scaled by the change in the best key. It matches width 12 (1551.7) with a third fewer builds.
- `TRIP_TREE_ADAPTIVE = "noise"`: margin from the tree's own estimate errors, which have a spread of about 10–13 per trip. It narrows the first depths and loses (1467.7, 1131.5).
- `TRIP_TREE_DIVERSE`: one node per set of held contacts. No effect.
- `TRIP_TREE_SCHEDULE`: width falling with depth. 12 → 3 matches width 12 with 28% fewer builds; 16 → 4 and 24 → 2 are worse.

Wider beams are often worse because nodes are ranked by real score plus an optimistic estimate for the remaining scouts. More width admits more nodes that look good only on that estimate. End to end (45 s, 6 concurrent, 4 runs each), the L4 total by tree share of the run was:

| Tree's share of the run | fixed beam | adaptive (1.0, 10) |
|---|---:|---:|
| 10% | 2765.7 | 2763.7 |
| 20% (default) | 2788.4 | 2764.7 |
| 35% | 2793.9 | 2787.5 |

At 10% the tree often does not finish. Between 20% and 35% the difference is within noise (about ±17). The adaptive beam does not win at any share.

#### Faster separation checks (`FAST_R8`, `R8_HOT_FIRST`)

Profiled on L4-401 (45 s, LNS share 0.75), about 80% of LNS time is validation. About half of LNS time goes on candidates that are rejected for R8, after 84 pair checks on average. The exact geometry for pairs that pass the bounding-box filter took 10.8 s.

- `FAST_R8`: a single-loop clearance test over the pieces of two routes. It computes the minimum distance of the relative linear motion for each piece pair, with limit d_sep + 1 mm against the kit's d_sep − EPS. Only "clear" is decided there; every other pair, including all same-target pairs, goes to the kit's `separation_violation`. On 90,656 route pairs from real runs and 3,000 random ones, it never cleared a pair the kit rejects. It is about twice as fast as the kit check, including piece preparation, which is cached per route id and cleared with the route cache.
- `R8_HOT_FIRST`: aircraft from recent R8 violations are checked first. The verdicts are the same.

The deterministic tree (fixed budget) gives identical plans with both on and off. Validator time per call fell by 25–50%, and LNS completed 32–40% more validator calls (L4-401 and L4-402). End to end, 6 runs each, both on against both off: L4-401 +7.7 (se 10.4), L4-402 −7.9 (se 4.5), with 14–25% more LNS iterations. That is within noise, as the convergence curve predicts.

Convergence (diagnostic 120 s run, beyond the official limit): LNS keeps improving to the end. L4-401's best was 1575.0 at 30 s, 1604.8 at 60 s and 1638.4 at 120 s. L4-402's was 1150.7, 1168.1 and 1176.8. That is about 0.3–0.7 points per second late in the run, so 14–25% more iterations is worth about 3–5 points. A 2× single-core speed-up would be worth about 15–25.

#### Other model experiments (flags off)

- `TRIP_TREE_LOOKAHEAD` ranks each child by the next scout re-planned from the child's own plan: label search over the free contacts and unused effectors, plus a strike-vs-scout R8 check. Before LNS (equal budget, width 6) it gave L4-401 1525.8 against 1472.7, but L4-402 1127.8 against 1153.6. It does not beat width 12 or the 12 → 3 schedule (1552.4, 1153.6) on either case. It roughly doubles tree time, and the strike check alone changed nothing.
- `OBS_SIDE_ANGLE`: angle 0 places a hostile's observation point toward the scout's home, which is the shared base, so the scout waits on the line its own strike flies in on. At 90°, strike-vs-scout R8 rejections drop by 81% (L4-401) and 82% (L4-402). Failed single steps fall from 12.5% to 9.3% on L4-401, and lost visits in the packing from 221 to 93 on L4-402. 60° is much worse. End to end (6 runs each, L1–L3 2 each), the score did not change: L4-401 −0.8, L4-402 −8.6 (about 1 se), L1–L3 −1.6. The planner already worked around those conflicts with delays, angles and layers.
- In final plans, collisions are the primary reason for few unserved contacts: 0 on L4-401 and about 2 per run on L4-402 (96 points before strike cost). Collateral bans (576 and 844 points) and windows no scout can reach (294 and 335) cannot be served by any plan.

#### Strike timing (`ANALYTIC_STRIKE`, `STRIKE_LAYER_FIRST`, both off)

`strikes()` loops delay, then angle, then layer set. When ground fails at delay 0, it tries 30 m and the 40/85/130 m rotations before trying ground at 4 s.

- `ANALYTIC_STRIKE` times the first layer set at angle 0 analytically. For each piece pair that can come within d_sep (bounding boxes), the time shifts that bring it within d_sep form one interval, because the minimum distance is convex in the shift. Golden-section search and bisection find it, and the strike takes the smallest clear shift. Across 80,000 sampled shifts, the kit never found a conflict outside the predicted interval plus margin. In the deterministic tree on L4-401 it scored 1486.1 against 1472.7, with no strikes at 40–130 m (6 before), and 78% of attempts were accepted. End to end (6 runs) it cost LNS about 35% of its iterations: L4-401 +5.9, L4-402 −10.5.
- `STRIKE_LAYER_FIRST` tries every delay and angle per layer set, cheapest first. It gives 10–20% more LNS iterations because fewer builds fail. A first 6-run batch gave L4-401 +23.6, but 10 more runs gave −0.2. Pooled over 16 runs each: L4-401 +8.7 (se 5.1), L4-402 +3.4 (se 3.2). L1–L3 (2 runs each) came to −4.2, including a reproducible −6.6 on L2-202.

Neither is clearly better, so both stay off. Single 6-run batches can overstate a gain by 15 points or more; defaults need at least 16 runs per side.

#### Joint re-plan of n scouts (LNS operator `group`)

Moving one scout at a time stays in a local optimum. A model of the board L4 cases re-planned scouts one by one and reached the same contact set as the planner. Exchanging contacts between two scouts raised the model value by 60–80 per case. Most of that gain came from cheaper scout trips, not extra coverage. The `group` operator (`GROUP_SIZES`, `GROUP_SHARE` = 20% of LNS steps, alongside `trip`) works like this:

- It picks n scouts: one is chosen like the `trip` move, and the rest are neighbours weighted by tour/base distance.
- It pools their contacts with up to `GROUP_POOL` unserved positive-net contacts within 2 km of their tours.
- It removes those tours and the strikes they confirm (`strip_scouts`).
- It runs the label search for each scout over the pool, keeping the top 40 trips plus cover trips and the current tour as an extra order.
- `best_disjoint` picks the best set of disjoint trips with a depth-first search and an upper-bound prune. The move goes ahead only if that set beats the estimated value of the current trips.
- It realises the visits in observation-time order through `realise_visits`, taking at most 0.8 s per move. The normal LNS repair and acceptance follow.

Counters in the `lns` block: `group_tried`, `group_realised`, `group_accepted`, `group_improvements`, `group_gain`, `group_label_s` and `group_sizes`.

Same 17 scenarios, 4 versions per run, 4 workers, default 45 s. Base is master at de78902. All 136 plans were valid.

| Run | base | n = 2 | n = 3 | n random 2–4 |
|---|---:|---:|---:|---:|
| run 1 total | 14203.3 | 14431.3 | 14367.1 | 14423.9 |
| run 2 total | 14351.8 | 14337.1 | 14343.9 | 14368.6 |
| sum of both | 28555.1 | 28768.4 | 28711.0 | 28792.5 |
| L4 subtotal, sum | 20785.4 | 20975.5 | 20925.0 | 20987.4 |
| board-8 subtotal, sum | 9859.7 | 9870.7 | 9879.2 | 9938.9 |

Random n in 2–4 is the default. Its gain over base is +237 summed (+0.8%), but it comes almost entirely from run 1: run 2 is a tie within the 0.4–1% run-to-run noise. Against base, mix won 11 and lost 5 in run 1, and won 8 and lost 8 in run 2. Base itself moved by +149 between the runs. Per-case swings of ±40–60 on L4 go both ways; for example, L4-14 was +102 in run 1 and −64 in run 2. Over the two runs the group move improved the best plan 85 times (out of ~3500 tries). About 70% of tries pass the estimate gate and are realised, and LNS iterations drop by ~12% because a group move costs more than a `trip` move.

### Local benchmark results

### Joint re-plan of conflicting strikes (LNS operator `strike`)

The strike move samples concrete candidate effector routes for an unserved positive-net
hostile, identifies the existing aircraft that actually violate separation (including pad
takeoffs), drops one to three conflicting hostile strike teams while retaining their scout
observations, then retries the target and displaced strikes in several orders.  Every retry
uses the normal incremental extension, so R4, R8 and R10 are rechecked globally.  It reports
`strike_tried`, `strike_accepted`, `strike_improvements` and `strike_gain` in `lns`.

Two 17-scenario, 45-second, four-worker frozen-copy runs compared master with random 1--3
removed conflicts (`mix`), fixed sizes, and a 30% strike share.  All 204 candidate plans were
valid; the slowest wall time was 46.1 s.  The default `mix` gained in both runs and on L4, so it
is enabled at a 20% LNS share.

| Run | Base | mix (1--3) | L4 base | L4 mix |
|---|---:|---:|---:|---:|
| 1 | 14304.5 | 14372.4 | 10401.6 | 10477.6 |
| 2 | 14358.5 | 14447.0 | 10453.4 | 10551.3 |

The move won/lost/tied 7/7/3 cases in run 1 and 8/6/3 in run 2.  It improved the best plan only
six times across 3,477 attempts, so the +156.4 summed gain is small relative to the observed
run noise.  It did not recover C22 or C07 in these winners; C30, C06, C14, C50, C48 and C17 were
already served in the relevant board plans.  Fixed-size and higher-share variants were less
consistent (the 30% share won run 1 but lost run 2), so they are not defaults.

Measured on this machine with five-second search budgets. These are sampled scenarios, not a guarantee of performance on unseen inputs.

| Scenario | Greedy score | Search score | Planner wall time |
|---|---:|---:|---:|
| L1-0 | 123.82 | 130.86 | 5.06 s |
| L2-7 | 266.23 | 276.76 | 5.06 s |
| L3-1 | 463.88 | 475.75 | 5.10 s |
| L4-0 | 590.34 | 622.35 | 5.11 s |

A separate **55-second L4-0** run scored **847.12** and completed in **55.15 seconds**, including process startup, final validation and output. All reported plans passed the official validator. Timing and search outcomes can vary with machine load.

A later audit of 12 additional five-second runs produced 10 wins and two losses against greedy; the combined short-budget score was lower. Repeating the two regressions at 55 seconds produced a win on L4-1 (574.11 vs 432.46) and a tie on L4-13 (1045.75). Both completed within 55.26 seconds. Use the normal budget for quality comparisons; the small table above is not evidence of universal improvement.

### Why contacts go unserved

Every run records, for each hostile and unknown contact, how far each attempt to serve it got. Each contact missing from the winning plan gets one primary reason. The planner's stderr line includes `unserved` (count per reason), `unserved_points` and `rejections`, and `--artifacts` writes per-contact detail to `tree.json`. The benchmark sums these per level in its console summary and `report.html`.

| Reason | Meaning |
|---|---|
| `collateral_ban` | A friendly or civilian is inside the collateral radius. R7 forbids the strike, so these points are never available. |
| `no_window` | The contact has no time window. |
| `never_tried` | No attempt ran: no pass reached it before the deadline. |
| `time_cap` | Attempts ran out of their per-node time before finding a valid child. |
| `no_team` | Not enough free effectors of the needed subtypes in that branch. |
| `scout_window` | No scout has a visit order that fits the window, strike deadlines, battery and horizon (time ranking). |
| `scout_route` | The baseline scout builder could not fit the visit into the window when appended to the scout's tour. |
| `scout_rule:Rn` / `strike_rule:Rn` | The scout or strike routes failed official rule Rn. |
| `not_improving` | A valid serve existed but did not raise the score. |
| `outranked` | A valid serve existed in some branch, but that branch did not win. |

The most common attempt outcome is the reason. Ties go to the attempt that got furthest.

**L4 diagnosis (seeds 0–2, 55 s, one worker; 1798.76 vs greedy 1354.26).** 154 targets were missed, worth 8,542 points:

| Reason | Contacts | Points |
|---|---|---|
| `scout_route` | 46 | 2665 |
| `collateral_ban` | 36 | 2460 |
| `outranked` | 42 | 2117 |
| `scout_rule:R1` | 21 | 775 |
| `time_cap` | 3 | 209 |
| `scout_rule:R3` | 3 | 163 |
| `scout_rule:R8` | 3 | 153 |

Scout failures account for 73 contacts and 3,756 points. A direct-flight check (`python3 analysis/scout_reach.py analysis/results/l4-unserved-diagnosis`) shows only 8 non-banned targets across the three scenarios that no scout can observe in time even flying straight from base at full speed (ignoring zones). Most scout failures therefore come from how tours are built (append-only, four nearest candidates), not from physics. `outranked` is the second lever: valid serves existed but the beam kept other branches. Time limits are a small factor (3 contacts). Collateral bans are unavailable points: they make "available points" totals look larger than what any plan can reach. Each L4 run completed 4–5 passes. The full run (report, scenarios, plans and logs) is committed in [analysis/results/l4-unserved-diagnosis/](analysis/results/l4-unserved-diagnosis/report.html).

A generated L2-7 demo is available locally at `output/search/tree.html`, with its winning fleet animation at `output/flight.html`. Generated outputs are ignored by Git; use `--artifacts` to reproduce them.

### Where the L2/L3 gap to the board leader comes from

Board (sum per level): leader L2 562, L3 1587; us at 035a098 L2 ~547, L3 ~1552. Every non-banned unknown is served on all five L2/L3 board scenarios. The hostile set is the knapsack optimum over the light/heavy supply, and effector assignment is within 1 point of the optimal matching (straight-line energy). **The gap is energy only**, in two parts:

1. **Effector climb** (fixed: `GROUND_STRIKES`). R2 bounds the altitude only of waypoints after the first and before the last, so a two-waypoint effector route (home → engagement at the contact, R_eng 3D) can fly at z = 0. That avoids e_climb × 30 m per effector, worth 5–19 Wh per L3 scenario before the fix. Members whose straight ground line enters a zone or protected area fly the 30 m route.
2. **Scout tours** (partly fixed: `SCOUT_ENERGY_RANK`). Scouts differ by about ±30 % in energy per metre (L2-202: S01 7.9 J/m, S02/S03 10.4–10.7), but candidate scouts were ranked by added *distance*, so the cheapest scout was often left idle. Our scouts flew 24–29 km per L3 scenario. One tour through all observation disks without windows is 12–13 km. An offline VRPTW estimate (windows, R4 deadlines, horizon, battery) gives 51–69 Wh against our 71–82 Wh under the same model.

Hand-built plans that use the planner's own builders (VRPTW tours, strikes re-timed, official scorer) score L2-201 295.87, L2-202 266.06 (one scout instead of two), L3-301 715.06, L3-302 418.77 (two scouts instead of four) and L3-303 444.72. That is **L2 561.9** (leader 562) and **L3 1578.6** (leader 1587), so these two causes account for nearly the whole gap. What remains is scout tour construction: fewer and better-chosen scouts per scenario.

Benchmark, same run, four workers, 45 s, all plans valid:

| Batch | 035a098 | + ground strikes | + energy-ranked scouts |
|---|---:|---:|---:|
| 17 cases (board-8, L4 9–14, L3 3–5) | 13745.67 | 13826.45 | **13928.02** (+1.3 %; 14 W / 3 L) |
| board-8 | 4787.92 | 4803.09 | 4810.50 |
| board L2 / L3 | 547.27 / 1543.35 | 549.12 / 1556.95 | 554.69 / 1570.18 |
| L4 (8 cases) | 9922.37 | 9994.03 | 10052.34 |
| L3 seeds 3–15 (separate run) | 6594.65 | | 6660.01 (+1.0 %; 12 W / 1 L) |

In an earlier run, ground strikes alone gave 13763.04 → 13874.67 on the 17 cases and 6605.11 → 6647.56 on L3 seeds 3–15. Two changes were tested and dropped: trying the 30 m scout layer first (13816.67 against 13874.67 for ground strikes alone), and an LNS operator that removes a whole tour (no gain on L2-202 or L3-302).

## Leaderboard submission

`submit_to_board.py` runs the full pipeline in one command: generate the eight public validation scenarios, plan each one, print scores, and submit to the live board.

```sh
# Full pipeline — generates scenarios, plans, scores, and submits
python3 submit_to_board.py --token <TOKEN>

# Skip regenerating scenarios and replanning (reuse existing board-plans/)
python3 submit_to_board.py --token <TOKEN> --skip-generate --skip-plan

# Score locally without submitting
python3 submit_to_board.py --token <TOKEN> --skip-submit
```

The `--team` (default `ELARIS`) and `--server` (default `http://192.168.2.19:8080/`) flags can be overridden if needed. Plans are written to `board-plans/` and scenarios to `validation/`. The server keeps your best score per scenario, so resubmitting a worse plan never hurts.

## Hand-in

Copy `planner.py`, `search_planner.py`, `candidate_matrix.py` and `tree_view.py` together. The official runner supplies `fleetkit` via `PYTHONPATH`. Local runs discover the bundled kit; `FLEETKIT_DIR` can explicitly select another copy. No third-party packages are required, and no changes to the official kit are needed.

The original analysis and isolated-mission checker remain in [analysis/](analysis/README.md).

Development and agent guidance, including test and benchmark commands, is in [AGENTS.md](AGENTS.md).

### Persistent optimisation history

Run benchmarks through `test_logger.py` to keep an independent record of every batch:

```sh
python test_logger.py --cases L1:0 L2:7 L3:1 L4:0 --seconds 5 --note "Before optimisation"
python test_logger.py --cases L1:0 L2:7 L3:1 L4:0 --seconds 5 --note "After optimisation"
python test_logger.py summary
```

With no options it runs L1-L4, seeds 0-2, at 55 seconds per case with one worker.
Each invocation creates a unique directory in `history/`; older runs are
never overwritten. `result.json` includes official points, per-case and aggregate
planner runtimes, baseline gains, validity/failures, served contacts, search diagnostics,
settings, UTC timestamps, source SHA-256 hashes, Git state and machine/Python details.
The `benchmark/` subdirectory keeps scenarios, both plans, process logs, JSON/CSV
results and the HTML report. `--history` chooses another history directory.

The logger reports score and runtime changes against the latest completed batch
with identical cases, budgets, worker count, environment and official-kit hashes.
Source hashes distinguish code revisions even with uncommitted edits. Positive score
deltas and negative runtime deltas indicate improvement; percentage changes from
zero are null. Failed planners count as zero, and batches with setup errors are not
used for history comparisons. `summary` exports all run records as JSONL to stdout.
Runs terminated before final collection retain a `running` record and any benchmark
artifacts already written. Logging applies when launching through this script;
direct `planner.py` invocations do not enter this history. Use identical settings
and one worker for useful comparisons; machine load can still affect results.


History is intentionally not ignored: commit new run folders with `git add history`
to share them. The two original runs have been moved from `output/test-history/`
without changing their contents; embedded command paths describe their original
execution location. They lack per-contact diagnostics and cannot be backfilled.

New runs preserve `results[].diagnostics.contacts`, keyed by contact ID. Each
hostile/unknown contact includes `cls`, `points`, `served`, `attempts`, `outcomes`,
`rules`, and an unserved `reason`. Its enclosing case identifies the level and seed.
These details are also in benchmark JSON/JSONL results and the planner log.

```sh
python test_logger.py contacts
```

This prints per-class counts, reasons, lost points and scout-failure rates across
saved runs. Each benchmark summary also includes `total.contact_summary` and the
same breakdown per level. Scout failures mean the primary unserved reason is
`scout_route` or `scout_rule:*`; the rate denominator is all diagnosed contacts of
that class, including served contacts. Old runs, failed planners and custom planners
without diagnostics are counted as cases without detail, never as zero failures.
Repeated cases across runs count as repeated observations; use `--history` to select
a separate collection for a controlled comparison. Primary reasons summarise the
search attempts, not proof that a contact is physically impossible to serve.

## Link-loss policy (optional second part)

`policy.py` is the per-aircraft policy for the organisers' link-loss challenge (`LINKLOSS.md` in the official kit). It is scored separately from the planner and is handed in next to `planner.py`.

- **Reclassified target:** an effector senses its target within 300 m, before reaching it, and turns back.
- **Pop-up zone:** every move is checked against the zones an aircraft knows; it never enters one, including when turning back.
- **Confirm before engage (R4):** an effector engages only when it knows the observation ended at least 11.5 s earlier. Observations finished before the cut come from the plan; later ones come from the scout's message, which carries the end time. Otherwise it waits short of the target, then turns back.
- **Separation (R8):** each aircraft predicts its peers from the plan and from their announced (route time, rate), with ±1.5 s for stale messages and drones about to take off, and picks the move that keeps at least 55 m.
- **Messages:** only rate changes (repeated, then refreshed every 8 s while off schedule) and scout observation notices.
- **Wire format:** one JSON string of type-coded records (`V` rate state, `O` observation, `R` reclassified, `Z` zone), base-93 printable ASCII, 1 byte per character; see the `policy.py` docstring. Same decisions as the dict format, fewer bytes; news-distribution columns (coverage, reach 90 %, t90, missed) are in `compare`.
- **News (store and forward, on):** `NEWS_MODE = "forward"`. Each drone keeps every sensed or heard danger (reclassified contact, pop-up zone) and the peers it knows have it (heard it from them, or sent it 3 times while they were within half its estimated range). Each second it predicts peers from the plan and sends a danger only when a peer that may lack it is in reach; the range is estimated from the farthest distance a peer was heard from. `"burst"` (send 3 times on learning) and `"off"` remain available.
- **Measured news distribution** (20 cases, L1-L4 seeds 0-4, one comm seed; drones airborne at the event that were still flying at the end and never knew it): off 12/16/25/19, burst 11/12/11/6, forward 7/3/2/1 at 250/500/1000/2000 m; capacity made no difference (messages are a few bytes). Messages about 8 per aircraft per minute in all three.
- **Score:** with 3 comm seeds at 4 ranges (240 runs each) failed runs were off 19, forward 22, and forward that ignores heard news 22; S_loss 0.865 / 0.855 / 0.852, a difference within noise. Acting on news neither causes nor prevents failures, and safe runs keep identical retention, because the policy's only use of news is aborting earlier. All failures are separation (R8), mostly at 250 m: that is the next thing to fix.

- **Coordination (on):** each drone now uses its picture of the fleet to stay on the plan rather than react to it. Pairs both on schedule trust the validated plan (`TRUST_PLAN`); scouts report the exact observation end (`EXACT_OBS_END`); a confirmed effector waits for late strike partners so the group still engages inside the sync window (`GROUP_SYNC`); an unconfirmed effector waits for a scout it knows is late while the window allows (`SCOUT_WAIT`). In safe runs before this, every contact lost against the oracle was a strike that missed sync because partners at the shared target backed off and reversed into one another.
- **Measured** (20 cases L1-L4 seeds 0-4, 250/500/1000/2000 m at 200 B/s, 3 comm seeds, 240 runs): S_loss 0.855 → 0.926, failed runs 22 → 12, retention of safe runs 0.960 → 0.992; at 1000 and 2000 m 0 failures and retention 0.999-1.000. Unseen seeds 20-29 (1000 and 250 m, 2 comm seeds, 160 runs): S_loss 0.732 → 0.833, failed 28 → 20. 17 runs were fixed and 7 newly broken, all R8 at 250-500 m; the remaining failures are mostly `lost_message` at 250 m. Tried and dropped: assuming an off-plan peer may have stopped anywhere since its last message (fixed 500 m, but frozen "ghosts" of engaged effectors boxed others in: 24 failures).

Run the official check (with the updated official kit on `PYTHONPATH`):

```sh
python3 handin/check_policy.py --policy /path/to/policy.py --levels L1,L2,L3,L4 --seeds 0-1
```

Leaderboard setting (1000 m, 200 B/s), greedy plans, 40 unseen scenarios (L1–L4, seeds 20–29, 2 communication seeds each):

| Policy | S_loss | Retention | Failed runs |
|---|---:|---:|---:|
| hold | 0.361 | 0.361 | 4/80 |
| return | 0.366 | 0.366 | 2/80 |
| broadcast | 0.292 | 0.412 | 47/80 |
| **policy.py** | **0.799** | **0.817** | **9/80** |

About 9 messages per aircraft per minute and under 0.2 ms per call. The remaining failures are all separation (R8) between aircraft that leave the plan close together, mostly on L4.

### Link-loss knowledge store

`analysis/linkloss_knowledge.py` records what every drone knew, second by second, and where each fact came from (`self`, `plan`, `sensed`, `heard` with the sender, `inferred`), plus believed versus real peer positions whenever two drones are within 200 m, event learning time against the time it was needed, and a timeline around each separation failure. The recorder never changes a run (tested).

```sh
python3 analysis/linkloss_knowledge.py --history --note "what changed" --policy my_policy.py \
  --levels L1,L2,L3,L4 --seeds 0-4 --settings 250x200,500x200,1000x200,2000x200 --comm-seeds 3 --workers 12
python3 analysis/linkloss_knowledge.py compare                 # every stored collection, per setting
python3 analysis/linkloss_knowledge.py diff BEFORE_DIR AFTER_DIR  # run by run: fixed, broken
```

Compare policies with at least 3 comm seeds: extra messages change the random delivery draws of the whole run, and with one seed off and store-and-forward looked like 6 against 15 failures, which 3 seeds showed to be 19 against 22 in 240 runs. Capacity made no difference so far, so one capacity per range is enough. A full 20-setting collection is about 25 MB: store in `history/` the collections worth keeping and use `output/` (the default without `--history`) for the rest.

**Using news in your own policy.** Everything is in `policy.py` and can be copied as is: the codec (`encode_records`, `decode_records`, contacts numbered in scenario order, `cw` digits wide), `_learn` (decodes the inbox into `reclassified`, `zones`, `confirmed`, `peer_state`), `_note_news` and `_forward` (store and forward), `_peer_where` (peer prediction from the plan, including drones on the ground). A policy that keeps the same attribute names (`reclassified`, `zones`, `confirmed`, `peer_state`, `aborted`, `rate`, `_peer_pos`, `peers`) is fully recorded by the knowledge store; others can define `knowledge()` returning `{fact: value}`.

`--history` stores the collection in Git-tracked `history/linkloss/<id>/` (one directory per collection, about 5-6 MB for 80 runs): `collection.json` (note, settings, policy SHA-256, kit hash, Git state, per-setting S_loss), the exact `policy.py`, `runs.jsonl`, `report.md`, `cases/*.json.gz` (scenario, plan, events) and `runs/*.json.gz` (`summary`, `facts`, `steps`, `near`). Load with `load_run`, `load_case`, `iter_collections`. Without `--history` it writes under ignored `output/`. Plans are the greedy baseline, as in `check_policy.py`.
