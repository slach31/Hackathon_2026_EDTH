"""Hand-in entry point: python3 planner.py scenario.json plan.json.

Standard library only. Optional search diagnostics: --artifacts output/search.
"""
import argparse
import json
from pathlib import Path
import sys
import time

# Start before input parsing/imports so they count against the search budget.
START = time.monotonic()
try:
    import fleetkit
except ModuleNotFoundError:
    # Local checkout convenience. The official runner supplies fleetkit through
    # PYTHONPATH, so a hand-in needs only this file and its three sibling modules.
    import os
    configured = os.environ.get("FLEETKIT_DIR")
    if configured:
        sys.path.insert(0, configured)
    else:
        root = Path(__file__).resolve().parent
        kits = sorted(root.glob("AA-Hackathon*/**/fleet-kit/fleetkit/__init__.py"))
        if not kits:
            raise SystemExit("Set PYTHONPATH or FLEETKIT_DIR to the folder containing fleetkit/")
        sys.path.insert(0, str(kits[0].parent.parent))

from fleetkit.model import Scenario, load_json, save_json
from search_planner import Search


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario")
    parser.add_argument("plan")
    # 50 s leaves ~10 s inside the organisers' 60 s limit: L4 runs reached 57-58 s of wall
    # time at a 55 s deadline under CPU load (OS stalls, process start-up and exit).
    parser.add_argument("--seconds", type=float, default=50.0,
                        help="search deadline including startup (default 50, capped at 55 seconds)")
    parser.add_argument("--top-n", type=int, default=5)
    parser.add_argument("--beam-width", type=int, default=6)
    parser.add_argument("--lns-share", type=float, default=0.4,
                        help="final share of the search budget spent repairing the best plan (0..1)")
    parser.add_argument("--interleave", action="store_true",
                        help="alternate beam passes with LNS bursts instead of LNS only at the end")
    parser.add_argument("--adaptive", action="store_true",
                        help="LNS favours destroy operators that recently improved the plan")
    parser.add_argument("--restarts", action=argparse.BooleanOptionalAction, default=True,
                        help="stuck LNS restarts from a random archived plan instead of the best "
                             "(default: on)")
    parser.add_argument("--scout-speed", choices=("fast", "eco", "hybrid"), default="fast",
                        help="scout cruise: fast = 0.98 v_h, eco = the kit's speed, "
                             "hybrid = eco with a fast fallback (default: fast)")
    parser.add_argument("--obs-stretch", action=argparse.BooleanOptionalAction, default=True,
                        help="observe from as far as the 3D range allows at the scout's altitude "
                             "(~125 m at 70 m) instead of the kit's 100 m (default: on)")
    parser.add_argument("--column-lns", action="store_true",
                        help="LNS also swaps whole scout tours for tours from the joint planner's "
                             "pool (column_lns.py; experimental)")
    parser.add_argument("--artifacts", type=Path,
                        help="optional directory for ranked plans, tree.json and interactive tree.html")
    args = parser.parse_args()
    if not 0 <= args.seconds <= 55 or not 1 <= args.top_n <= 100 or not 1 <= args.beam_width <= 100:
        parser.error("seconds must be 0..55; top-n and beam-width must be 1..100")
    if not 0 <= args.lns_share <= 1:
        parser.error("lns-share must be 0..1")
    scenario = Scenario.from_dict(load_json(args.scenario))
    if args.column_lns:
        from column_lns import ColumnSearch as SearchClass
    else:
        SearchClass = Search
    search = SearchClass(scenario, args.seconds, args.top_n, args.beam_width, START, args.lns_share,
                    args.interleave, args.adaptive, args.restarts,
                    scout_speed=args.scout_speed, obs_stretch=args.obs_stretch)
    verified = search.run()
    # Required output is written before optional diagnostics.
    save_json(verified[0][0].plan(scenario.seed), args.plan)
    report = search.report(verified)
    if args.artifacts:
        from tree_view import save_tree
        args.artifacts.mkdir(parents=True, exist_ok=True)
        for rank, (state, result) in enumerate(verified, 1):
            save_json(state.plan(scenario.seed), args.artifacts / f"plan-{rank:02d}.json")
        save_json(report, args.artifacts / "tree.json")
        save_tree(report, args.artifacts / "tree.html")
    print(json.dumps({key: report[key] for key in
                      ("elapsed_s", "passes", "lns", "candidate_checks", "rejections",
                       "unserved", "unserved_points", "contacts", "solutions") if key in report}), file=sys.stderr)


if __name__ == "__main__":
    main()
