# policy_v2.py

`policy_v2.py` is an alternative link-loss policy for the organisers' link-loss challenge (`LINKLOSS.md` in the official kit). It keeps the `Policy` interface and wire format of `policy.py` and changes how aircraft behave after the cut. Standard library only. The first version measured under 1 ms per call; the loop 2–4 versions were not timed separately.

It started as a copy of `policy.py` at commit `06dd80d` (news sharing off). Every change sits behind a flag at the top of the file, so each one can be switched off and measured on its own.

## Results

**Test set.** Greedy plans on L1–L4 seeds 20–29 (40 scenarios), 3 communication seeds, 200 B/s. That is 120 runs per range. S_loss is mean retention − 0.002 × messages per aircraft per minute, and a failed run counts as 0 retention.

Against `policy.py` as of `9164c1c` (store-and-forward news on):

| Range | `policy.py` S_loss | Failed | v2 (loop 1) S_loss | Failed |
|---|---:|---:|---:|---:|
| 250 m | 0.645 | 31/120 | 0.713 | 19/120 |
| 500 m | 0.769 | 16/120 | 0.797 | 10/120 |
| 1000 m | 0.823 | 10/120 | 0.835 | 6/120 |
| 2000 m | 0.830 | 9/120 | 0.850 | 5/120 |

Over these 480 paired runs, v2 was better in 87 and worse in 38. It avoided 47 failures that `policy.py` had, and failed 21 runs that `policy.py` survived.

**Improvement loops.** Five loops followed, each on the same 240 runs (250 m and 1000 m). Each loop recorded what every drone knew with `analysis/linkloss_knowledge.py`, compared the runs against the previous version, and changed one thing:

| Version | Change | S_loss 250 m | S_loss 1000 m | Failed (of 240) | Msg/aircraft/min |
|---|---|---:|---:|---:|---:|
| Loop 1 | v2 as first written | 0.713 | 0.835 | 25 | 7.4 |
| Loop 2 | Far-from-plan aircraft re-announce every 3 s | 0.730 | 0.845 | 17 | 12.2 |
| Loop 3 | Aborted aircraft hover instead of retracing their route | 0.793 | 0.834 | 10 | 13.3 |
| **Loop 4 (current)** | **Hovering aircraft refresh fast only with a peer nearby** | **0.805** | **0.838** | **9** | **11.3** |
| Loop 5 | Reverse only as a last resort | 0.790 | 0.842 | 13 | 11.2 |

Loop 5 was measured worse, so it is switched off (`RETURN_LAST = False`). The file behaves exactly like loop 4.

**Not yet compared against the latest `policy.py`.** Commit `3998376` (TRUST_PLAN, GROUP_SYNC, SCOUT_WAIT, exact observation end) was pushed after these measurements.

## What it changes

Each change came from what drones knew and decided in the recorded runs in `history/linkloss/`.

| Flag | Behaviour | Why |
|---|---|---|
| `ZONE_CONTINUE` | A scout whose route crosses a pop-up zone finishes the observations before the crossing, then turns back. Zone checks look one step ahead with a 4 m margin. | `policy.py` turned scouts back at once and dropped every remaining observation. |
| `PARK` | An effector with no confirmation waits 1.5 s of route time short of its target, inside the separation exemption around the shared target. It engages if the confirmation arrives later. | Turning back flew it head-on into its own strike partners. |
| `TAKEOFF_FIX` | A peer still on its pad is checked for separation even after it has announced a state. A lower-id peer is assumed able to take off at any moment. | Delayed take-offs from pads 15–28 m apart collided. |
| `HOPELESS` | An aircraft that can no longer reach any task inside its contact's window aborts. One still on the ground stays there. | Aircraft held on the ground for minutes then took off into airspace nobody predicted them in. |
| `RESEND_NEAR_M` | An off-plan aircraft re-announces its state when a peer it believes within 350 m was not that close at its last announcement. | A message sent out of range is lost. |
| `RELAY` | An effector that heard its target's confirmation repeats it 10 s and 5 s before engaging. | Strike partners behind it often never heard the scout. |
| `O_DENSE_S`, `O_GAP_S` | Scouts repeat an observation notice every second for 6 s, then every 3 s. | They used to repeat it every second for about 100 s. |
| `LAG_REFRESH_S`, `LAG_M` (loop 2) | An off-plan aircraft more than 60 m from where its plan puts it re-announces every 3 s instead of 8 s. Not while still on the ground. | Peers that never heard it placed it hundreds of metres away. |
| `HOVER_ABORT` (loop 3) | An aborted aircraft hovers where it is and moves only when hovering would break separation. | Flying a route backwards met outbound traffic head-on and ended in the busy base. |
| `QUIET_HOVER_M` (loop 4) | A hovering aircraft keeps the 3 s refresh only while a peer is believed within 800 m, otherwise 8 s. | Hover repeats were 63 % of all messages. |
| `RETURN_LAST` (loop 5, off) | A drone that has not aborted reverses to avoid a peer only as a last resort. | Measured worse: 9 → 13 failed runs. |

`GHOST_S` (keep a peer effector at its target after its planned engagement) is off by default.

The wire format is unchanged from `policy.py`: one JSON string of type-coded base-93 records (`V` state, `O` observation, `R` reclassified, `Z` zone). v2 sends no news of dangers.

## Run it

Use it in place of `policy.py`:

```sh
python3 -m fleetkit linkloss scenario.json plan.json events.json policy_v2.py
python3 handin/check_policy.py --policy policy_v2.py --levels L1,L2,L3,L4 --seeds 0-1
```

Compare policies on identical scenarios, events and comm seeds:

```sh
python3 analysis/compare_policies.py policy.py policy_v2.py --levels L1,L2,L3,L4 --seeds 20-29 \
  --settings 250x200,500x200,1000x200,2000x200 --comm-seeds 3 --json output/compare.json
python3 analysis/linkloss_report_html.py output/compare.json -o output/compare.html   # HTML page
```

Record what each drone knew, then inspect one drone at one moment:

```sh
python3 analysis/linkloss_knowledge.py --policy policy_v2.py --levels L1,L2,L3,L4 --seeds 20-29 \
  --settings 250x200,1000x200 --comm-seeds 3 --workers 8 --output output/v2-knowledge
python3 analysis/linkloss_knowledge.py diff BEFORE_DIR AFTER_DIR       # fixed / newly broken runs
python3 analysis/drone_knowledge.py RUN.json.gz H05 600-620            # what H05 knew each second
```

## Limits

- **Tuned on the test set.** The loop changes were chosen from failures on the same 240 runs they are scored on. Check them on other seeds before relying on the gains.
- **Noise.** With 3 comm seeds, one or two failed runs either way is within noise.
- **Remaining failures.** The 9 remaining failures are all separation. Six of the 11 colliding pairs come from one scenario (L3-29), where four strike partners of the same target leave their plan together.
- **Message cost.** Messages roughly doubled from loop 1 (7.4 → 11.3 per aircraft per minute), which costs about 0.008 S_loss.
