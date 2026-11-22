"""Offline board search: many long searches per scenario in parallel, keeping the best plan.

The live board receives plans, not code, and keeps each team's best score per scenario.
These runs may therefore search far longer than the hand-in's 60 seconds. They do not
change the hand-in planner (planner.py keeps its 55 s deadline).

Each round runs --runs searches per scenario on --workers processes. Even-numbered runs
start from the best plan so far (long LNS repair); odd-numbered runs search from scratch
with other random seeds. Every result is checked with the official scorer, and a plan
replaces <output>/<scenario>.json only if it scores higher.

Usage (from the repository root):
  python analysis/board_search.py --scenarios L4-401 L4-402 --runs 8 --seconds 300 --workers 8
  python -m fleetkit submit-all output/board-best      # then send the improved plans
"""
import argparse
import multiprocessing
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


def state_from_plan(search, plan):
    """A search state (with scout task lists) for a valid plan, or None."""
    routes = {r["aircraft"]: r["waypoints"] for r in plan["routes"]}
    state = search.checker.extend(search.root, routes)
    if state is None:
        return None
    tasks = {sid: [] for sid in search.root.tasks}
    for sid in tasks:
        for w in routes.get(sid, []):
            if w.get("action") == "observe":
                c = search.contacts[w["target"]]
                tasks[sid].append({"cid": c.id, "point": (w["x"], w["y"]),
                                   "window": c.window, "cpos": c.pos})
    state.tasks = tasks
    return state


def run(job):
    name, variant, seconds, seed_path = job
    sp.MAX_SEARCH_S = seconds
    sp.LNS_PATIENCE = 10 ** 9
    sp.TRACE_CAP = 100000
    scenario = Scenario.from_dict(load_json(ROOT / "validation" / f"{name}.json"))
    start = time.monotonic()
    seeded = seed_path is not None
    # Seeded runs spend the whole budget repairing the seed plan with LNS.
    search = sp.Search(scenario, seconds, start=start, variant=variant,
                       lns_share=1.0 if seeded else 0.4)
    seed_score = None
    if seeded:
        state = state_from_plan(search, load_json(seed_path))
        if state is not None:
            seed_score = state.score
            search.record(state, search.root, {"kind": "seed"}, 0)
    verified = search.run()
    state, result = verified[0]
    return {"scenario": name, "variant": variant, "seeded": seeded, "seed_score": seed_score,
            "score": result.score, "plan": state.plan(scenario.seed),
            "lns": search.lns["iterations"], "passes": search.passes,
            "elapsed": time.monotonic() - start}


def best_score(scenario, path):
    if not path.exists():
        return None
    result = evaluate(scenario, load_json(path))
    return result.score if result.valid else None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenarios", nargs="+", default=["L4-401", "L4-402"], choices=SCENARIOS)
    parser.add_argument("--runs", type=int, default=8, help="searches per scenario per round")
    parser.add_argument("--rounds", type=int, default=1, help="rounds; each reseeds from the best")
    parser.add_argument("--seconds", type=float, default=300.0, help="search time per run")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--first-variant", type=int, default=1,
                        help="variant (random seed offset) of the first run; change it between "
                             "invocations to explore new seeds")
    parser.add_argument("--all-seeded", action="store_true",
                        help="every run starts from the best plan so far (once best plans are "
                             "beyond what a search from scratch reaches)")
    parser.add_argument("--output", type=Path, default=ROOT / "output" / "board-best")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    scenarios = {n: Scenario.from_dict(load_json(ROOT / "validation" / f"{n}.json"))
                 for n in args.scenarios}
    for name in args.scenarios:
        print(f"{name}: best so far {best_score(scenarios[name], args.output / f'{name}.json')}")
    variant = args.first_variant
    for round_ in range(1, args.rounds + 1):
        jobs = []
        for name in args.scenarios:
            seed = args.output / f"{name}.json"
            for r in range(args.runs):
                jobs.append((name, variant, args.seconds,
                             str(seed) if (args.all_seeded or r % 2 == 0) and seed.exists() else None))
                variant += 1
        print(f"round {round_}: {len(jobs)} runs of {args.seconds:g} s on {args.workers} workers",
              flush=True)
        with multiprocessing.Pool(args.workers) as pool:
            for out in pool.imap_unordered(run, jobs):
                name = out["scenario"]
                path = args.output / f"{name}.json"
                check = evaluate(scenarios[name], out["plan"])
                current = best_score(scenarios[name], path)
                mark = ""
                if check.valid and (current is None or check.score > current + 1e-9):
                    save_json(out["plan"], path)
                    mark = "  NEW BEST"
                seed = f" from {out['seed_score']:.1f}" if out["seed_score"] is not None else " scratch"
                print(f"  {name} v{out['variant']:<3}{seed:>14}: {check.score:9.2f} "
                      f"valid={check.valid} lns={out['lns']} passes={out['passes']} "
                      f"{out['elapsed']:.0f}s{mark}", flush=True)
    for name in args.scenarios:
        print(f"{name}: best {best_score(scenarios[name], args.output / f'{name}.json')}")


if __name__ == "__main__":
    main()
