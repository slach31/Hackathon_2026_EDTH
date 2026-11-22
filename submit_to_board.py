"""Full pipeline: generate scenarios → run planner → score → submit to leaderboard.

Usage:
    python3 submit_to_board.py --token <TOKEN>
    python3 submit_to_board.py --token <TOKEN> --team "ELARIS" --server http://192.168.2.19:8080/
    python3 submit_to_board.py --token <TOKEN> --skip-generate --skip-plan
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
FLEETKIT_DIR = str(
    sorted(
        Path(__file__).resolve().parent.glob(
            "AA-Hackathon*/**/fleet-kit/fleetkit/__init__.py"
        )
    )[0].parent.parent
)

SCENARIOS = ["L1-101", "L2-201", "L2-202", "L3-301", "L3-302", "L3-303", "L4-401", "L4-402"]

VALIDATION_DIR = Path("validation")
PLANS_DIR = Path("board-plans")


def run(cmd, env=None, check=True):
    merged_env = {**os.environ, **(env or {})}
    return subprocess.run(cmd, env=merged_env, check=check)


def fleetkit_cmd(*args, **kwargs):
    return run(
        [sys.executable, "-m", "fleetkit", *args],
        env={"FLEETKIT_DIR": FLEETKIT_DIR},
        **kwargs,
    )


# ---------------------------------------------------------------------------
# Steps
# ---------------------------------------------------------------------------

def step_generate():
    print("\n=== Step 1: Generate validation scenarios ===")
    VALIDATION_DIR.mkdir(exist_ok=True)
    fleetkit_cmd("validation-set", "-o", str(VALIDATION_DIR))


def step_plan():
    print("\n=== Step 2: Run planner on all scenarios ===")
    PLANS_DIR.mkdir(exist_ok=True)
    for name in SCENARIOS:
        scenario = VALIDATION_DIR / f"{name}.json"
        plan = PLANS_DIR / f"{name}.json"
        log = PLANS_DIR / f"{name}.log"
        if not scenario.exists():
            print(f"  SKIP {name}: scenario file not found")
            continue
        print(f"\n  Planning {name} ...")
        with open(log, "w") as logfile:
            result = subprocess.run(
                [sys.executable, "planner.py", str(scenario), str(plan),
                 "--scout-speed", "fast", "--obs-stretch"],
                stderr=logfile,
            )
        if result.returncode != 0:
            print(f"  WARNING: planner exited {result.returncode} for {name} (see {log})")
        else:
            print(f"  OK -> {plan}")


def step_score():
    print("\n=== Step 3: Score all plans ===")
    total = 0.0
    for name in SCENARIOS:
        scenario = VALIDATION_DIR / f"{name}.json"
        plan = PLANS_DIR / f"{name}.json"
        if not plan.exists():
            print(f"  {name}: no plan file, skipping")
            continue
        result = subprocess.run(
            [sys.executable, "-m", "fleetkit", "score", str(scenario), str(plan)],
            env={**os.environ, "FLEETKIT_DIR": FLEETKIT_DIR},
            capture_output=True, text=True,
        )
        line = (result.stdout or result.stderr).strip()
        print(f"  {name}: {line}")
        for token in line.split():
            try:
                total += float(token)
                break
            except ValueError:
                continue
    print(f"\n  Total score: {total:.2f}")


def step_login(server, team, token):
    print(f"\n=== Step 4: Login as '{team}' ===")
    fleetkit_cmd("board-login", "--server", server, "--team", team, "--token", token)


def step_submit():
    print("\n=== Step 5: Submit all plans ===")
    fleetkit_cmd("submit-all", str(PLANS_DIR))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--token", required=True, help="Leaderboard token")
    parser.add_argument("--team", default="ELARIS", help="Team name (default: ELARIS)")
    parser.add_argument("--server", default="https://circus-movers-award-raises.trycloudflare.com",
                        help="Leaderboard server URL")
    parser.add_argument("--skip-generate", action="store_true",
                        help="Skip scenario generation (reuse existing validation/)")
    parser.add_argument("--skip-plan", action="store_true",
                        help="Skip planning (reuse existing board-plans/)")
    parser.add_argument("--skip-submit", action="store_true",
                        help="Score locally but do not submit to the board")
    args = parser.parse_args()

    if not args.skip_generate:
        step_generate()
    else:
        print("Skipping scenario generation.")

    if not args.skip_plan:
        step_plan()
    else:
        print("Skipping planning.")

    step_score()

    if not args.skip_submit:
        step_login(args.server, args.team, args.token)
        step_submit()
    else:
        print("Skipping submission.")


if __name__ == "__main__":
    main()
