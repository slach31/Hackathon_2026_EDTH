# Deck number check: "ELARIS Fleet Planner + Link-Loss Pitch (3 min)"

Instructions for a Claude command that edits the deck. Every number in the 5 slides was checked against the repository's recorded results. Apply only the edits under **Required** and **Recommended**. Leave every number marked **OK** exactly as it is. Do not invent new numbers: each value below is quoted from the source named next to it.

Checked on 22 Nov 2026 against `README.md`, `SCOUTING_V1.md`, `output/linkloss_v2/final.json`, `policy.py`, `fleetkit/linkloss/policies.py` and the live board (`/api/board`).

## Summary

| Slide | Number | Status |
|---|---|---|
| 2 | "up to about 15% between runs" | OK |
| 2 | "3,756 points on L4" | Recommended wording fix |
| 3 | "+45%", "3,623 → 5,252" | OK |
| 3 | "+78%", "1,354 → 2,412" | OK |
| 3 | "100%", "55.25 s", "60 s limit" | OK |
| 3 | footnote "55 s budget, one worker … up to about 15%" | OK |
| 4 | "14 B", "about 60 B" | OK |
| 4 | "9 → 4" (1000 m × 200 B/s) | OK, recommended footnote fix |
| 4 | "22 → 11" (250 m × 50 B/s) | OK, recommended footnote fix |
| 5 | "link-loss policy v2" handed in | **Required: team decision** |
| 5 | "3,756 points on L4" | Recommended wording fix |

---

## Required

### R1. Slide 5: which link-loss policy is handed in

- **Current text:** "Search planner with LNS repair, and link-loss policy v2, both validator-checked."
- **Problem:** the repository has two policies. `policy.py` is the file named in the hand-in instructions and documented as the hand-in in `README.md`. `policy_v2.py` is an experimental branch. The slide must name the file the team actually handed in.
- **Edit:**
  - If `policy.py` is handed in, replace with: "Search planner with LNS repair, and a link-loss policy, both validator-checked."
  - If `policy_v2.py` is handed in, keep the text as it is.
- **Do not apply this edit without the team's answer.** If it is unknown, leave the slide unchanged and report that R1 is pending.

---

## Recommended

### C1. Slide 4 footnote: what "v1" and "v2" are

- **Current text:** "Policy v1 → v2, 40 scenarios (L1–L4, seeds 20–29), greedy plans. Separation failures remain, mostly near the base and at short range."
- **Numbers verified** (`output/linkloss_v2/final.json`; 40 scenarios × 2 communication seeds = 80 runs per setting):

  | Setting | v1 (`policy.py`) failed | v2 (`policy_v2.py`) failed |
  |---|---:|---:|
  | 1000 m × 200 B/s | 9/80 | 4/80 |
  | 250 m × 50 B/s | 22/80 | 11/80 |

- **Problem:** "v1" is `policy.py` as it was on 21 Nov, *before* its fleet-coordination update. That update later cut its own failures from 28 to 20 of 160 runs on the same seeds, at 250 m and 1000 m × 200 B/s. The two current versions were never compared head to head.
- **Replace the footnote with:** "Policy v1 → v2, 40 scenarios (L1–L4, seeds 20–29), 2 radio seeds, greedy plans; v1 measured before its coordination update. Separation failures remain, mostly near the base and at short range."

### C2. Slides 2 and 5: "3,756 points on L4" is an upper bound from one diagnosis

- **Source:** `README.md`, "L4 diagnosis". On L4 seeds 0–2 (three scenarios), one 55 s run of an earlier planner version (1,798.76 points) missed contacts worth 8,542 points. Scout failures account for 73 contacts and 3,756 points. Only 8 of those targets were physically unreachable; the rest failed because of how tours are built.
- **Problem:** the number comes from three scenarios and is an upper bound on what better scout tours could add, not points lost on every L4 run.
- **Slide 2:**
  - Current: "Cost: it shows our weak spot, scout tours, 3,756 points on L4."
  - Replace with: "Cost: it shows our weak spot, scout tours: up to 3,756 points missed on three L4 scenarios."
- **Slide 5:**
  - Current: "Better scout tours: scout failures cost 3,756 points on L4, mostly from tour construction rather than physics."
  - Replace with: "Better scout tours: scout failures cost up to 3,756 points on three L4 scenarios, mostly from tour construction rather than physics."

---

## OK: leave unchanged

| Slide | Text | Source and check |
|---|---|---|
| 2 | "L4 scores vary up to about 15% between runs" | `README.md`: L4-2 scored 665.14, then 562.88 after the rebase (−15.4%). |
| 3 | "+45%", "3,623 → 5,252 points, L1–L4, seeds 0–2" | `README.md`, consolidated branch result: greedy 3,622.86 → 5,251.52, i.e. +44.95%. |
| 3 | "+78%", "1,354 → 2,412 points" (L4) | Same table: 1,354.26 → 2,412.22, i.e. +78.1%. |
| 3 | "100% valid plans", "Slowest run 55.25 s, inside the 60 s limit" | `README.md`: "Every plan was valid and the slowest run took 55.25 seconds." |
| 3 | Footnote: "55 s budget, one worker … overlapping development seeds … up to about 15%" | `README.md`. Seeds 0–2 were development seeds, so the caveat is correct. |
| 4 | "14 B, three facts in one message" | `policy.py` wire format: one byte per character. For example a zone (6) + observation (4) + reclassified contact (2) = 12 characters, plus 2 JSON quotes = 14 B. |
| 4 | "about 60 B for the shipped broadcast" | The kit's `BroadcastSensePolicy` sends `{"p":[x,y,z],"rc":[…],"zn":[[x,y,r,top]]}`. With all three facts that is about 55–60 B of compact JSON. |
| 4 | "9 → 4", "22 → 11", "of 80" | `output/linkloss_v2/final.json`; see C1. |

---

## Optional addition (only if the team wants a live-board number)

The live board measures the same thing as the ranking. As of 22 Nov, 11:30, ELARIS stood at **5,114.2** points against **2,664.6** for the baseline over the 8 validation scenarios: **+92%**, 4th place. The board keeps changing, so if this is used, label it with the time: "Live board, 22 Nov 11:30: 5,114 vs baseline 2,665 (+92%)". Do not put it in the deck unless the team asks.
