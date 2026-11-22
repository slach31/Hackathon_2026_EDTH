# Link-loss policies: `policy.py` and `policy_v2.py`

## Rundown

**The challenge.** A validated plan is flying. Between a quarter and half of the way through, the link to the ground is cut for the whole fleet. From then on every aircraft decides alone, once per second. It can only continue, slow down (0.5–1×), hover, or retrace its route home. It sees itself, anything within 300 m, and messages from peers. A message is at most 64 bytes, one per second, and gets lost more often with distance and congestion. After the cut, unannounced events happen: a target turns friendly, a no-fly zone pops up, an aircraft fails.

A run scores zero on any of these:
- a near-collision under 40 m;
- entering a zone;
- striking a friendly;
- striking without a scout confirmation at least 10 s earlier;
- the policy crashing.

Otherwise the score is the share of a full-information oracle's mission value we keep, minus 0.002 per message per aircraft per minute.

**Our idea in one line.** Every drone carries the shared plan as a model of the whole fleet, so it only has to talk when reality leaves the plan. Drones on schedule trust the plan, drones off schedule announce it, and safety always wins over mission value.

| | `policy.py` | `policy_v2.py` |
|---|---|---|
| Role | **The hand-in policy** | Experimental branch of fixes |
| Based on | Current line of development | `policy.py` at commit `06dd80d`, *before* the coordination features |
| Main strength | Fleet coordination: strike groups stay in sync, effectors wait for late scouts | Fewer near-collisions at short radio range: safer aborts, take-off checks, faster re-announcing |
| Danger news | Store-and-forward on | Off |
| Headline result | S_loss **0.799** at the leaderboard setting on 40 unseen scenarios (hold 0.361, broadcast 0.292). After coordination: 0.732 → **0.833** on unseen seeds, averaged over 250 m and 1000 m | S_loss 0.805 at 250 m and 0.838 at 1000 m on its test set; failed runs 25 → 9 of 240 |
| Caveat | Remaining failures are all near-collisions, mostly at 250 m | Tuned on its own test set; never compared with the latest `policy.py` |

**What to hand in:** `policy.py`. **How `policy_v2.py` might help:** its fixes target exactly the failures `policy.py` still has, near-collisions at short range. Each fix sits behind its own flag, so they can be ported into `policy.py` one at a time and measured (see [Combining them](#combining-them)).

---

## The problem in detail

### What a drone can do

| Action | Effect |
|---|---|
| `continue` | follow the route |
| `speed` 0.5–1 | follow it slower |
| `hold` | hover in place (costs energy, no progress) |
| `return` | retrace the route backwards and land |

A drone cannot invent a new route. The only ways around a danger are holding, slowing or turning back. Everything is judged on real times, so holding makes a drone permanently late. A late strike can miss its window, or fall out of sync with its partners: all effectors on one target must strike within 20 s of each other.

### What a drone knows

- Its own position, velocity, place on its route clock and energy left.
- Observations it completed itself.
- Events within 300 m: a reclassified contact, a new zone.
- Messages from peers, delivered one step later with probability `0.95 × p_range × p_cap`. Delivery is near-certain up to half the radio range, then falls to zero at the full range. A receiver can take only `C` bytes per second.
- Peers' positions only through messages, or by dead-reckoning from the plan.

### How it is scored

```
retention = min(1, V_policy / V_oracle)   (0 if the run failed)
S_loss    = mean retention − 0.002 × messages per aircraft per minute
```

The organisers also score a grid of 20 radio settings: range 250, 500, 1000 or 2000 m, times capacity 50–800 B/s. A cell passes if mean retention is at least 0.9 with no failed run. The leaderboard setting is 1000 m and 200 B/s.

---

## `policy.py` in detail

### 1. Hard safety rules

| Risk | Rule |
|---|---|
| Striking a reclassified (friendly) target | An effector always senses its own target within 300 m before reaching it, and turns back. |
| Entering a pop-up zone | Every candidate move is checked a few seconds ahead against all known zones, sensed or heard. A move that would enter one is never taken, including when turning back. |
| Striking without confirmation (R4) | An effector strikes only if it *knows* a scout's observation ended at least `confirm_margin_s` earlier. Observations finished before the cut are known from the plan. Later ones are announced by the scout with their exact end time. Without one, the effector stops `HOLD_LEAD_S` short of its target, waits, then gives up. |

### 2. Separation without a controller

- Each drone predicts every peer: from the plan (on schedule unless the peer says otherwise), or from the peer's last announced (route time, rate).
- Predictions carry ±1.5 s of uncertainty, to cover stale messages and drones about to take off.
- Each second the drone ranks its possible moves by how well they serve its intent. It takes the first one that keeps at least 55 m (40 m plus a 15 m margin) from every peer over the next 5 s. If none does, it takes the move with the most room.
- **`TRUST_PLAN`:** two drones that are both on schedule are already separated by the validated plan, so the check skips that pair. Checking them anyway made strike partners at a shared target back off, hold and reverse into one another.

### 3. Coordination: keeping the mission alive

| Flag | Behaviour | Why |
|---|---|---|
| `EXACT_OBS_END` | A scout reports the exact end of its observation from its route clock, not the second it noticed it. | Effectors need only 0.1 s of slack and no longer slow down just short of the target. |
| `GROUP_SYNC` | A confirmed effector holds short while its strike partners are predicted to arrive later than the plan's own spread allows. | The group still strikes inside the 20 s sync window. Before this, every contact lost in safe runs was a strike that missed sync. |
| `SCOUT_WAIT` | An unconfirmed effector waits for a scout it knows is late, for as long as the contact's window allows. | Replaces a fixed 15 s grace period that gave up on strikes that were still possible. |

### 4. Communication

There are only three kinds of message:

| Record | Content | Sent when |
|---|---|---|
| `V` | my route time (0.1 s) + rate | my rate changes (repeated once), then every 8 s while off schedule |
| `O` | contact + observation end time | a scout completes an observation after the cut |
| `R` / `Z` | reclassified contact / pop-up zone (centre, radius) | I know a danger a peer may lack |

- **Wire format:** one JSON string of concatenated records. The first character is the record type, and fields are base-93 digits over printable ASCII, so every character is one byte. A status update is about 5 bytes against the 64-byte limit. A decoder stops at the first record it doesn't understand, so a malformed message can never crash the policy.
- **Store-and-forward news (`NEWS_MODE = "forward"`):**
  - Each drone keeps every danger it knows, plus which peers it knows already have it.
  - Each second it predicts where peers are and sends a danger only when a peer that may lack it is within estimated radio range.
  - The range is learned from the farthest distance a peer has been heard from, since the true range is never observed.
  - Measured effect: drones that never heard about a danger dropped from 12/16/25/19 to 7/3/2/1 at 250/500/1000/2000 m.

### 5. Measured results

At the leaderboard setting (1000 m, 200 B/s): greedy plans, 40 unseen scenarios (L1–L4, seeds 20–29), 2 radio seeds each, 80 runs. These runs predate the coordination features.

| Policy | S_loss | Retention | Failed runs |
|---|---:|---:|---:|
| hold | 0.361 | 0.361 | 4/80 |
| return | 0.366 | 0.366 | 2/80 |
| broadcast (organisers' baseline to beat) | 0.292 | 0.412 | 47/80 |
| **policy.py** | **0.799** | **0.817** | **9/80** |

Then the coordination features were added (`TRUST_PLAN`, `GROUP_SYNC`, `SCOUT_WAIT`, `EXACT_OBS_END`):

| Test set | S_loss before → after | Failed runs |
|---|---|---|
| Development: L1–L4 seeds 0–4, 4 ranges, 3 radio seeds (240 runs) | 0.855 → **0.926** | 22 → 12 |
| Unseen: seeds 20–29, 250 m and 1000 m, 2 radio seeds (160 runs) | 0.732 → **0.833** | 28 → 20 |

- On the development set at 1000 and 2000 m: **0 failures**, and retention of 0.999–1.000.
- About 9 messages per aircraft per minute, and under 0.2 ms per decision (the organisers suggest staying well under 50 ms, on a Raspberry Pi 5).
- **Known limit:** all remaining failures are near-collisions (R8), mostly at 250–500 m. There, drones that leave the plan close together can't hear each other in time.

---

## `policy_v2.py` in detail

### What it is

A branch of `policy.py` taken at commit `06dd80d`, with news sharing off, built to attack near-collision failures. It keeps the same `Policy` interface and wire format. Every change sits behind a flag at the top of the file, so it can be switched off and measured on its own.

It was developed in measured loops:
1. Record what every drone knew each second, with `analysis/linkloss_knowledge.py`.
2. Find the cause of each failure.
3. Change one thing.
4. Re-measure on the same 240 runs (250 m and 1000 m, 3 radio seeds).

### The fixes

| Flag | Behaviour | Failure it addresses |
|---|---|---|
| `ZONE_CONTINUE` | A scout whose route crosses a pop-up zone finishes the observations before the crossing, then turns back. Zone checks look one step ahead with a 4 m margin. | `policy.py` turned the scout back at once and dropped every remaining observation. |
| `PARK` | An unconfirmed effector parks 1.5 s of route time short of its target, inside the separation exemption around the shared target, and strikes if the confirmation arrives later. | Turning back flew it head-on into its own strike partners (3 of the 6 near-collisions at the time). |
| `TAKEOFF_FIX` | A peer still on its pad is checked for separation even after it has announced a state. A lower-id peer is assumed able to take off at any moment. | Delayed take-offs from pads 15–28 m apart collided. |
| `HOPELESS` | A drone that can no longer reach any task inside its contact's window aborts. One still on the ground never takes off. | Drones held on the ground for minutes, then took off into airspace nobody predicted. |
| `RESEND_NEAR_M` | An off-plan drone re-announces its state when a peer it believes within 350 m was not that close at its last announcement. | A message sent while out of range is lost. |
| `RELAY` | An effector that heard its target's confirmation repeats it 10 s and 5 s before striking. | Strike partners behind it often never heard the scout. |
| `O_DENSE_S`, `O_GAP_S` | Scouts repeat an "observed" notice every second for 6 s, then every 3 s. | They used to repeat it every second for about 100 s. |
| `LAG_REFRESH_S`, `LAG_M` (loop 2) | A drone more than 60 m from where its plan puts it re-announces every 3 s instead of every 8 s. | Peers that never heard it placed it hundreds of metres away. |
| `HOVER_ABORT` (loop 3) | An aborting drone hovers in place, and moves only if hovering would break separation. | Flying the route backwards met outbound traffic head-on and ended in the busy base. |
| `QUIET_HOVER_M` (loop 4) | A hovering drone keeps the fast refresh only while a peer is believed within 800 m. | Hover repeats were 63% of all messages. |
| `RETURN_LAST` (loop 5, **off**) | Reverse only as a last resort. | Measured worse (9 → 13 failures), so it is switched off. |
| `GHOST_S` (**off**) | Assume a peer effector stays parked at its target after its planned strike. | Not enabled. |

### Measured results

Against `policy.py` as of commit `9164c1c`, which is *before* the coordination features. Greedy plans, L1–L4 seeds 20–29 (40 scenarios), 3 radio seeds, 200 B/s, 120 runs per range:

| Range | `policy.py` S_loss | Failed | v2 loop 1 S_loss | Failed |
|---|---:|---:|---:|---:|
| 250 m | 0.645 | 31/120 | 0.713 | 19/120 |
| 500 m | 0.769 | 16/120 | 0.797 | 10/120 |
| 1000 m | 0.823 | 10/120 | 0.835 | 6/120 |
| 2000 m | 0.830 | 9/120 | 0.850 | 5/120 |

Improvement loops, each on the same 240 runs (250 m and 1000 m):

| Version | Change | S_loss 250 m | S_loss 1000 m | Failed (of 240) | Msg/aircraft/min |
|---|---|---:|---:|---:|---:|
| Loop 1 | v2 as first written | 0.713 | 0.835 | 25 | 7.4 |
| Loop 2 | Off-plan drones re-announce every 3 s | 0.730 | 0.845 | 17 | 12.2 |
| Loop 3 | Aborting drones hover instead of retracing | 0.793 | 0.834 | 10 | 13.3 |
| **Loop 4 (current)** | **Hover refresh only with a peer nearby** | **0.805** | **0.838** | **9** | **11.3** |
| Loop 5 (off) | Reverse only as a last resort | 0.790 | 0.842 | 13 | 11.2 |

### Limits

- **Tuned on its own test set.** Loop changes were chosen from failures on the same 240 runs they are scored on. They need checking on fresh seeds.
- **Noise.** With 3 radio seeds, one or two failed runs either way is noise.
- **Not compared with the latest `policy.py`**, which already has the coordination features.
- **More messages.** From 7.4 to 11.3 per aircraft per minute, which costs about 0.008 S_loss.
- **Remaining failures.** The 9 remaining failures are all near-collisions. Six of the 11 colliding pairs come from one scenario (L3-29), where four strike partners on the same target leave their plan together.

---

## Combining them

The two files fix different things:

- `policy.py` keeps strike groups together and the mission alive.
- `policy_v2.py` makes drones that leave the plan behave more predictably and announce themselves sooner, which is where `policy.py`'s remaining failures are.

Most promising v2 fixes to port into `policy.py`, in order. Port one at a time and measure each:

1. **`HOVER_ABORT`.** It gave the largest single gain (17 → 10 failures). Retracing a route is the most common way an aborting drone meets traffic head-on.
2. **`TAKEOFF_FIX`.** Cheap, and it targets collisions at the base, where pads are only 15–28 m apart.
3. **`PARK`.** Check it alongside `SCOUT_WAIT`, since both decide what an unconfirmed effector does.
4. **`LAG_REFRESH_S` + `QUIET_HOVER_M`.** These trade a few messages for better peer predictions.

Compare on identical scenarios, events and radio seeds. Check fresh seeds as well, not only the ones the fixes were tuned on:

```sh
python3 analysis/compare_policies.py policy.py policy_v2.py --levels L1,L2,L3,L4 --seeds 20-29 \
  --settings 250x200,500x200,1000x200,2000x200 --comm-seeds 3 --json output/compare.json
python3 analysis/linkloss_report_html.py output/compare.json -o output/compare.html
```

## Running and checking

```sh
# one run, then a replay in the browser
python3 -m fleetkit linkloss scenario.json plan.json events.json policy.py
python3 -m fleetkit view scenario.json plan.json --events events.json --policy policy.py -o run.html --open

# the organisers' check (official kit on PYTHONPATH)
python3 handin/check_policy.py --policy policy.py --levels L1,L2,L3,L4 --seeds 0-1

# what each drone knew, second by second
python3 analysis/linkloss_knowledge.py --policy policy.py --levels L1,L2,L3,L4 --seeds 20-29 \
  --settings 250x200,1000x200 --comm-seeds 3 --workers 8 --output output/knowledge
python3 analysis/drone_knowledge.py RUN.json.gz H05 600-620
```

Both policies use the standard library only.
