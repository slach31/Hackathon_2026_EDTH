# Working on the fleet planner

The team implementation is at this repository's root. The bundled `AA-Hackathon-EDTH-202611-main/` tree contains the official starter kit; implement team changes in the root modules, analysis tools and tests. Read [README.md](README.md) for usage, design and measured results, and [PLAN.md](PLAN.md) for team priorities and the acceptance workflow.

## Planner constraints

- Python 3.9+ and the standard library only. The official runner supplies `fleetkit` through `PYTHONPATH`; local discovery also supports `FLEETKIT_DIR`.
- The complete planner process has 60 seconds. The default search deadline is 50 seconds from startup (capped at 55); preserve time for full official validation and output. At 55 s, L4 runs reached 57-58 s of wall time under CPU load.
- The official score is contact points minus energy and consumed-effector penalties. Higher is better. Invalid plans, crashes and timeouts score zero.
- The assignment matrix ranks estimated aircraft/contact costs. Timed waypoints represent aircraft motion. Estimated matrix feasibility is not proof that a fleet plan is valid.
- When replacing routes, check separation for every pair involving a changed route. Recheck confirmation and strike completeness globally; a rebuilt scout tour can break earlier confirmations. Fully validate exported winners with the official scorer.

## Tests

Run from the repository root:

```sh
python3 -m unittest discover -s tests -v
```

The suite covers search behavior, incremental/full validation agreement, standalone hand-in execution, parallel benchmark reports, failure accounting and subprocess timeouts. Add targeted tests for behavioral changes; documentation-only edits do not require rerunning long benchmarks.

## Benchmark against greedy

Use [analysis/benchmark_search.py](analysis/benchmark_search.py) for comparisons. It generates identical inputs for both planners, runs each in a separate process with a 60-second hard timeout, and validates both outputs with the official scorer.

```sh
# Quick smoke comparison
python3 analysis/benchmark_search.py --levels L1,L2 --seeds 0-2 --seconds 5 --workers 2

# Full-budget parallel batch: 20 scenarios
python3 analysis/benchmark_search.py --levels L1,L2,L3,L4 --seeds 0-4 --seconds 55 --workers 4

# Sequential comparison without competing benchmark workers
python3 analysis/benchmark_search.py --levels L1,L2,L3,L4 --seeds 0-4 --seconds 55 --workers 1

# Explicit cases; output must be a new directory
python3 analysis/benchmark_search.py --cases L3:7 L4:13 --seconds 55 --workers 2 \
  --output output/benchmarks/selected-cases
```

Defaults are all four levels, seeds 0-2, 55 seconds of search and one worker. For planner changes, compare representative seeds across levels, including L4. Use short runs for smoke checks and full-budget runs for quality conclusions. Parallel jobs share CPU and can change search quality and timing; record the worker count and budget when reporting results.

The runner prints live progress and total/per-level score summaries, including gains, wins/ties/losses, validity and failures. Open its printed `report.html` path for the browser report, which refreshes while running. Results include JSON, JSONL, CSV, run settings, and per-case scenarios, plans and logs. Aggregate gains use summed raw scores, matching the ranking. Failed planner runs count as zero; generation/setup errors are reported separately and excluded from score comparisons. The runner exits nonzero on execution/validation/setup failures, but a valid score below greedy is a completed comparison.

Keep ad hoc generated reports under the ignored `output/` directory. Persistent logger runs belong in the Git-tracked `history/` directory. Report losses and failures alongside wins. Sampled gains do not establish optimality or universal improvement; record observed results and their limits in the README.
