"""Column-swap LNS for scout tours (opt-in: planner.py --column-lns).

The joint scout planner (search_planner.Search.joint) generates thousands of candidate
tours. Each is checked against the scout's battery, the horizon, the contact windows and
the strike timing, and all are kept in Search.tour_pool. A column swap:

1. picks a pool tour that is not flown now, weighted by its estimated value;
2. clears its scout and every scout whose tour shares a contact with it (at most
   MAX_CLEARED scouts), together with the strikes their observations confirm;
3. places the pool tour, trying altitude layers and take-off delays against R8, and
   strikes the hostiles it observes;
4. refills the other cleared scouts with their best pool tours compatible with the plan;
5. lets a short greedy repair offer every contact still unserved to any scout.

The result is accepted or rejected with the same annealing rule as the other LNS moves.
The scout rebuild in search_planner regrows cleared tours greedily and rarely recovers
(7 of ~160 accepted on L4-402); pool tours are complete, pre-checked alternatives.
"""
import math
import time

from search_planner import HOSTILE, DeadlineReached, Search

SWAP_P = 0.3        # share of LNS iterations that are column swaps
MAX_CLEARED = 3     # most scouts cleared by one swap
SWAP_S = 3.0        # time cap per swap
REPAIR_S = 1.0      # greedy repair after a swap


class ColumnSearch(Search):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.lns["column"] = {"tried": 0, "built": 0, "accepted": 0, "improvements": 0}

    def strike_all(self, state, cids, stop):
        """Strike the unserved hostiles among cids, most valuable first, from the
        observations already in the plan."""
        hostiles = sorted((self.contacts[c] for c in cids
                           if self.contacts[c].cls == HOSTILE and c not in state.served),
                          key=lambda c: (-self.matrix.priority(c), c.id))
        for contact in hostiles:
            sid = next((s for s, ts in state.tasks.items()
                        if s in state.routes and any(t["cid"] == contact.id for t in ts)), None)
            if sid is None:
                continue
            route = state.routes[sid]
            obs_end = max((w["t"] + w.get("dwell_s", 0.0) for w in route
                           if w.get("target") == contact.id and w.get("action") == "observe"),
                          default=None)
            groups = self.matrix.groups(contact, state.used)
            if obs_end is None or not groups:
                continue
            local_stop = min(stop, time.monotonic() + 0.3)
            try:
                for candidate, _ in self.strikes(state, contact, groups, self.aircraft[sid],
                                                 route, obs_end, local_stop):
                    if candidate.score > state.score:
                        candidate.tasks = state.tasks
                        state = candidate
                        break
            except DeadlineReached:
                self.checker.check_time(stop)
        return state

    def column_swap(self, current, rng, stop):
        """(new state, contacts it no longer serves), or (None, []) if the swap fails."""
        flown = {(sid, tuple(t["cid"] for t in ts)) for sid, ts in current.tasks.items() if ts}
        candidates = [c for c in self.tour_pool if c["value"] > 0
                      and (c["scout"], tuple(t["cid"] for t in c["tasks"])) not in flown]
        if not candidates:
            return None, []
        col = rng.choices(candidates, weights=[c["value"] for c in candidates])[0]
        cleared = {col["scout"]} | {sid for sid, ts in current.tasks.items()
                                    if any(t["cid"] in col["cids"] for t in ts)}
        if len(cleared) > MAX_CLEARED:
            return None, []
        partial = current
        for sid in sorted(cleared):
            if partial.tasks.get(sid):
                smaller = self.clear_scout(partial, sid)
                if smaller is None:
                    return None, []
                partial = smaller
        placed = self.place_tour(partial, col, stop)
        if placed is None:
            return None, []
        partial = self.strike_all(placed, col["cids"], stop)
        for sid in sorted(cleared - {col["scout"]}):
            taken = {t["cid"] for ts in partial.tasks.values() for t in ts}
            options = sorted((c for c in self.tour_pool
                              if c["scout"] == sid and not (c["cids"] & taken)),
                             key=lambda c: -c["value"])
            for option in options[:3]:
                new = self.place_tour(partial, option, stop)
                if new is not None:
                    partial = self.strike_all(new, option["cids"], stop)
                    break
        return partial, [c for c in current.served if c not in partial.served]

    def lns_step(self, st, contacts, priorities):
        rng = self.lns_rng
        if not self.tour_pool or rng.random() >= SWAP_P:
            return super().lns_step(st, contacts, priorities)
        current, best = st["current"], st["best"]
        stats = self.lns["column"]
        self.lns["iterations"] += 1
        stats["tried"] += 1
        stop = min(self.checker.stop, time.monotonic() + SWAP_S)
        try:
            partial, removed = self.column_swap(current, rng, stop)
            if partial is not None:
                stats["built"] += 1
                partial = self.repair(partial, 0, min(stop, time.monotonic() + REPAIR_S))
        except DeadlineReached:
            self.checker.check_time()
            partial, removed = None, []
        if partial is not None:
            # Same acceptance as Search.lns_step: improvements, or annealing.
            delta = partial.score - current.score
            span = max(1e-6, self.stop - st["begin"])
            heat = st["temperature"] * max(0.0, self.stop - time.monotonic()) / span
            if delta > 1e-9 or (heat > 0 and rng.random() < math.exp(delta / heat)):
                self.lns["accepted"] += 1
                stats["accepted"] += 1
                current = st["current"] = self.record(
                    partial, current, {"kind": "lns", "op": "column", "removed": removed},
                    len(partial.served))
                if current.score > best.score + 1e-9:
                    self.lns["improvements"] += 1
                    self.lns["gain"] += current.score - best.score
                    stats["improvements"] += 1
                    st["best"], st["stale"], st["since_best"] = current, 0, 0
                    return
        st["stale"] += 1
        st["since_best"] += 1
        if st["stale"] >= 25:
            st["stale"] = 0
            st["current"] = rng.choice(self.archive) if self.restarts else st["best"]
