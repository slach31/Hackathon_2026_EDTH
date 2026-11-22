# Fleet search planner

A standard-library planner that turns the baseline's route candidates into a bounded search tree, then spends the last part of the budget repairing the best plan with large neighbourhood search (LNS). It always keeps valid partial fleet plans, returns the best plan found, and can export the best N distinct plans and an interactive tree.

Run from this directory:

```sh
python3 planner.py path/to/scenario.json output/plan.json
```

The default search deadline is **50 seconds from startup** (capped at 55), including input loading. This leaves about 10 seconds inside the organisers' 60-second limit for full validation and writing the result; at 55 s, L4 runs reached 57-58 s of wall time under CPU load. The required plan is written before optional diagnostics. Python imports and individual validator calls cannot be preempted: this is a cooperative deadline, not a guarantee against arbitrary machine stalls.

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
- The last `--lns-share` of the budget (default 0.4) runs LNS on the best plan. Each iteration removes 1–4 served contacts, chosen at random or as neighbours of a random contact, rebuilding the affected scout tour from its original departure. It then tries to reinsert the removed contacts, their six nearest unserved neighbours and three random unserved contacts. Each scout visit is tried at the three cheapest tour positions (added distance), and the best valid child per contact is kept. Worse plans are sometimes accepted, with a temperature of 1% of the best score falling to zero. After 25 iterations without a new best, the search restarts from a random plan in the top-N archive (`--no-restarts` returns to the best plan instead) and stops after 400 iterations without a new best. Accepted plans enter the archive, so they receive the same final official validation.
- Expansion has local time caps, preventing an impossible contact from consuming the whole run. Children must improve the parent's actual score. All archive winners receive a full official validation and score before export; final validation stops starting new calls 3 seconds after the search deadline. The valid empty plan is the last fallback.

## Planner v2 (21 Nov): low flight, joint scouts, faster checks

v2 keeps the tree search above and adds new route shapes, a joint scout planner and faster validation. Every plan is still checked by the official validator before it is written.

### Results

Single runs of the hand-in planner on the 8 public validation scenarios, run the way `submit_to_board.py` runs them (one at a time, official scorer):

| Scenario | Original planner (55 s) | v2 before the joint planner (50 s) | **v2 (50 s)** |
|---|---|---|---|
| L1-101 | 68.26 | 72.29 | 69.38 |
| L2-201 | 288.35 | 297.60 | 293.15 |
| L2-202 | 257.84 | 257.54 | 261.05 |
| L3-301 | 693.50 | 713.87 | 717.56 |
| L3-302 | 393.41 | 401.67 | 409.49 |
| L3-303 | 420.05 | 439.73 | 446.00 |
| L4-401 | 1,452.73 | 1,575.49 | 1,632.55 |
| L4-402 | 937.06 | 1,123.90 | 1,156.38 |
| **Total** | **4,511.19** | **4,882.08** | **4,985.55** |

All 24 plans were valid. The slowest v2 run took 50.6 s of wall time; the original planner took up to 60.0 s on L4-401.

On 13 cases (the 8 above plus generated seeds L3:1 and L4:0–3), 55 s, three benchmarks in parallel, v2 before the joint planner scored **9,974.49** against **9,127.90** for the original planner (+9.3%). It won all 13 cases, and every plan was valid. Single L4 runs vary by about ±50 points between runs, so compare totals, not single cases.

### How one run works

Each step produces complete plans and puts them in one shared archive, ranked by score. The steps compete: the improvement step works on whichever plan is best so far, whatever produced it.

| Step | Algorithm | Time |
|---|---|---|
| 1. Warm start | the kit's greedy baseline, with faster checks | up to 25% of the budget (max 10 s) |
| 2. Joint scout planner | randomised nearest-neighbour scout tours, the best set chosen jointly, up to 3 candidates built with strikes | 85% of the beam phase (stops early when it converges) |
| 3. Sector construction | far anchor contacts on different bearings, one scout each | off since 22 Nov (`CONSTRUCT_SHARE = 0`) |
| 4. Beam search | contact-by-contact serve/skip tree (above) | rest of the beam phase |
| 5. Improvement | cleanup, re-fly, idle-scout tours, greedy repair, then LNS | `--lns-share` of the budget (default 0.4) |
| 6. Polish and output | cleanup and re-fly of the 3 best plans, official validation, write | last 0.4 s, then up to 3 s |

### Low flight: cheaper routes within the rules

The kit's route builders climb to a cruise layer (effectors 40/85/130 m, scouts 70/110 m), hover straight up and down, and fly at 95% of top speed. The rules require less:

- **R2 checks altitude only at waypoints**, and the last waypoint of a route may be at 0 m. A strike route therefore flies from the pad straight to the engagement point, which is the contact at ground level (within R_eng = 25 m, so R10 holds). Detour points around zones fly at 32 m, because every waypoint except the last needs z ≥ z_min (30 m).
- **A leg's minimum duration is max(horizontal / v_h, vertical / climb rate)**, so climbing and cruising overlap. Scouts climb on the way to their first observation and descend on the way home, without hovering.
- **Scouts observe from 30.5 m.** The 3D observation range (150 m) then allows about 147 m horizontally, against about 133 m at 70 m. The validator checks the 3D distance at the waypoint (+1e-6), so a 5 cm margin covers waypoint rounding (1 mm).
- **Scouts cruise at 99.8% of v_h**, plus 0.01 s per leg for rounding. R2 allows v_h exactly. The last 2% decides whether some early-window contacts about 5 km out can still be confirmed.
- **Effectors cruise at their best-range speed**, sqrt(p0 / k_v), capped at v_h. Energy per metre is lowest there.

On L1-101 this alone took the same plan from 68.26 to 71.86: 16.3 Wh of energy instead of 23.5 Wh. A low route can still conflict with other aircraft (R8), so strikes try, in order:
1. the direct low route;
2. a low route crossing at 72 m, which is above scouts at 30.5 m by more than d_sep, then diving over the last 150 m;
3. the kit's layered routes.

The re-fly pass rebuilds existing routes with these profiles, one route at a time. It keeps a change only if the plan stays valid and scores higher. It also tries arrival shifts of ±3 and ±7 s within the team's 20 s sync window.

### Joint scout planner (nearest neighbour)

In L4 the scouts are the bottleneck. All 15 share one base, fly about 16 m/s, and have 27–49 Wh of battery against a 900 s horizon. Most contacts are 5–8 km out, so each scout gets roughly one trip. Which scout takes which chain of contacts decides most of the score, and adding contacts one at a time (beam, LNS) fixes that choice too early.

The joint planner decides all scout tours together:

1. **Ranges.** For each scout, the farthest out-and-back distance its battery and the horizon allow at its own speed and power. From that, the scouts able to serve each contact alone: observation done in time, for hostiles 10 s before the window closes (R4), and home again.
2. **Complete plans by randomised nearest-neighbour insertion.** Scouts are taken in a random order (the longest range first in the first restart). Each grows a tour from contacts nobody has taken yet. The next contact is the one with the best value per added kilometre, with ±50% noise, at its cheapest insertion position. A contact is kept only if it adds value net of the scout's energy. A tour must:
   - fit the battery and end 20 s before the horizon, so it can still be delayed at take-off against R8;
   - leave every hostile time for a strike: effectors able to arrive after the observation, plus the confirm margin, plus the team's 5 s spacing, before the window closes;
   - stay within the effector supply.
   
   Hundreds of restarts run in the time given (about 290–560 on L4).
3. **Joint selection.** All tours generated form a pool. The planner picks at most one tour per scout, with no contact twice and within the effector supply, maximising total estimated value: randomised greedy picks plus swap improvements, against the best single restart.
4. **Build.** The chosen tours are added most valuable first. Each tries altitude layers 30.5, 70.5 and 110.5 m (40 m apart, so scouts on different layers cannot break R8) and take-off delays of 0–50 s. Strikes come next, using the normal strike builder. A greedy repair then offers the contacts of tours that did not fit to any scout.

On its own, in about 15 s, it built 1,114 on L4-402 and 1,585 on L4-401: as much as the whole previous planner in 50 s.

**Time split (22 Nov).** In L4 the beam's plans never won: the winning plan's lineage always started from the joint planner, and the beam got only 2–3 passes. The joint planner now gets 85% of the beam phase, and the sector construction is off (`CONSTRUCT_SHARE = 0`). On small scenarios the joint planner stops early after 150 restarts without a better plan (`JOINT_PATIENCE`), leaving the time to the beam. Over 3 runs per side at 50 s: L4-401 1,595.6 → 1,613.3 on average, L4-402 1,075.3 → 1,120.3. L3-303 and L2-201 were unchanged.

**Several builds (22 Nov).** The estimate of a joint plan misses take-off conflicts and strike timing by 50–150 points on L4, by different amounts per plan. The planner now builds up to 3 distinct candidate plans (`JOINT_BUILDS`): the selection result and the best restarts. The top candidate is always built fully; the others only use leftover time. The best built plan is kept. Over 3 runs per side: L4-402 1,131.7 → 1,140.9, L4-401 unchanged (1,618.5 against 1,617.0). Splitting the build time equally between the candidates instead caused a 1,440 outlier on L4-401, because no build finished its strikes.

### Other new steps

- **Range-aware scout choice.** When a contact goes to an idle scout, the smallest scout that can still serve it is used. This keeps long-range scouts free for contacts only they reach. Before, the choice among idle scouts depended on which pad was a few metres closer.
- **No repeat observations.** A hostile that some scout already observes is first struck from that observation, and a tour never gets a second visit to the same contact. The current plans had 2–4 such repeats per L4, each costing at least 20 s of a scout's only trip.
- **Cleanup (prune).** Drops scout visits that serve nothing (repeats, or hostiles never struck) when the rebuilt tour keeps every served contact and scores higher.
- **Idle-scout tours.** After the beam, each idle scout grows its own greedy tour over what is still unserved.
- **Scout rebuild in LNS.** In 10% of iterations, 1–3 scout tours are cleared together and regrown from anchors few scouts can reach.

### Faster validation (same verdicts)

The incremental checker (`IncrementalValidator`) is about 2× faster and gives the same verdicts as the official validator:

- **Per-route rules are cached.** R1, R2, R3, R5, R6, R7 and R9 look at one route at a time, so each distinct route is checked once, along with its parse, separation data and energy. Before, the same scout route was re-checked for every strike variant built on it.
- **R4 and R10 are rechecked only for affected contacts.** These are contacts touched by a changed or removed route; the check covers every route that touches them. The parent plan was valid for every other contact.
- **Separation pairs that are provably far apart are skipped.** If two flight pieces' bounding boxes, or their closest approach over the shared time, stay at least d_sep + 1 cm apart, the official check cannot find a violation. Every other pair goes to `separation_violation`.

Checked by comparing every verdict during real L2–L4 searches with the official `evaluate()`: about 28,000 verdicts, 0 mismatches and 0 score differences. `tests/test_search.py` (`ScoutTourTests`) repeats this on a smaller sample.

### Timing

The default deadline is now 50 s (it was 55 s). At 55 s, L4 runs reached 57–58 s of wall time while other processes loaded the CPU. Under the same load at 50 s, the slowest run took 51.2 s. The run can still overrun briefly when the operating system stalls the process: gaps of 0.8–2 s between deadline checks were observed a few times. Large caches now evict their oldest 10% instead of clearing all at once.

### What did not help

- **Parallel searches inside the hand-in** (one search per CPU core, best plan wins). On a 2-performance-core, 8-efficiency-core laptop, total search effort stopped growing at 8 processes (2.5× one process). 12 processes overran a 15 s budget by 2.5 s. With an unknown grading machine, the risk outweighs the gain. The same idea works well offline for the board (below).
- **Better observation points, scout tour swaps, slower flight while waiting for windows.** Measured on the best plans: about 4, 15 and 1 points in total.
- **Scout-rebuild LNS as the main move.** Clearing whole tours makes a packed plan much worse, and no regrow recovered it: 7 rebuilds accepted out of about 160, with changes of −3 to +4 points. Adding a full repair after each rebuild halved the LNS iterations for no gain. The move is kept at 10% of iterations.
- **Less LNS in favour of the joint planner and beam.** With `--lns-share 0.1`, the 8 public scenarios scored 4,908.21, against 4,985.55 with the default 0.4. L1–L3 were unchanged (2,196.19 against 2,196.63); L4 lost 77 points (1,580.57 and 1,131.45 against 1,632.55 and 1,156.38). The joint planner finds the scout assignment, and LNS then tunes the details.
- **Tested on 22 Nov, not adopted.** Each was compared over 3 runs per side on L4-401 and L4-402 at 50 s; single L4 runs vary by about ±50 points, so differences below ~20 points are noise.
  - Strike costs without the 40 m climb in `CandidateMatrix`: L4-401 −37.5, L4-402 +6.5.
  - LNS share 0.2 instead of 0.4 with the joint-heavy split: +1.4 and −16.6.
  - Observation points facing the tour's neighbours instead of home: −12.6 and +5.6.
  - Wider tour generation (10 candidate contacts, 5 insertion positions): +17.8 and −9.2.
  - Validating every tour during joint restarts (R8 against the restart's other scouts): the 8 public scenarios fell from 4,985.55 to 4,883.49, because restarts dropped from about 560 to about 50 on L4-402.
  - Column-swap LNS (`column_lns.py`, opt-in `--column-lns`): swaps whole scout tours for pool tours. About 240 swaps were tried, 5 accepted and 1 improved; results moved −36 and +44 from scratch.
- **Which hostiles to serve.** An exact knapsack over the effector supply (lights and heavies) picks the same hostiles as the planner on all L2 and L3 scenarios, and differs by 7 and 26 points on the two L4s. On L2/L3 the remaining gap is costs: effectors are within 2–4 points of their one-way minimum per scenario.
- **Early-window hostiles about 5 km out** (for example L4-402's C01, C43 and C07). A scout can now observe them in time, but R4 needs the observation finished 10 s before the strike, which they miss by 3–8 s.

### Offline board search

The live board accepts plans, not code, and keeps each team's best score per scenario. `analysis/board_search.py` runs many long searches per scenario in parallel. Half start from the best plan so far, half from scratch with other random seeds. A plan replaces `output/board-best/<scenario>.json` only if the official scorer rates it higher. It does not change the hand-in planner.

```sh
python3 analysis/board_search.py --scenarios L4-401 L4-402 --runs 8 --seconds 240 --workers 8
python3 -m fleetkit submit-all output/board-best
```

Change `--first-variant` between invocations to explore new random seeds. The board is a validation set; the final ranking runs `planner.py` on unseen scenarios with 60 s, so only changes to the hand-in planner count there.

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

### Local benchmark results

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
