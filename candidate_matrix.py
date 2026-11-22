"""Cheap assignment ranking. Estimates are not proofs of feasibility."""
from itertools import combinations, product
import math

from fleetkit.energy import best_range_speed
from fleetkit.geometry import EPS, hdist
from fleetkit.model import CIVILIAN, EFFECTOR, FRIENDLY, HOSTILE


class CandidateMatrix:
    def __init__(self, scenario):
        self.scenario = scenario
        self.effectors = [a for a in scenario.aircraft if a.type == EFFECTOR]
        self.banned = {
            c.id for c in scenario.contacts if c.cls == HOSTILE
            and any(hdist(c.pos, p.pos) < scenario.rules.collateral_radius_m - EPS
                    for p in scenario.contacts if p.cls in (FRIENDLY, CIVILIAN))
        }
        self.costs = {}
        for c in scenario.contacts:
            if c.cls != HOSTILE:
                continue
            for a in self.effectors:
                if c.needs.get(a.subtype, 0) <= 0:
                    continue
                distance = hdist(a.home, c.pos)
                speed = min(a.v_h_mps, best_range_speed(a.power))
                climb = 40.0 / a.v_up_mps
                energy = ((a.power.p0_w + a.power.k_v * speed ** 2)
                          * distance / speed / 3600.0
                          + a.power.e_climb_wh_per_m * 40.0
                          + a.power.p0_w * climb / 3600.0)
                self.costs[a.id, c.id] = scenario.rules.lambda_e * energy + scenario.rules.c_eff

    def refine(self, energy_wh):
        """Replace estimated costs by energy_wh(effector, contact) where it returns Wh
        (e.g. the energy of the route the planner would build); None keeps the estimate,
        math.inf marks the pair infeasible (groups() then ranks it last)."""
        for (aid, cid), cost in list(self.costs.items()):
            wh = energy_wh(aid, cid)
            if wh is not None:
                self.costs[aid, cid] = self.scenario.rules.lambda_e * wh + self.scenario.rules.c_eff

    def groups(self, contact, used, limit=3):
        """Shortlist exact subtype mixes; never prune on estimated energy/time."""
        if contact.id in self.banned:
            return []
        pools = []
        for subtype, count in sorted(contact.needs.items()):
            if count <= 0:
                continue
            free = sorted((a for a in self.effectors if a.subtype == subtype and a.id not in used),
                          key=lambda a: (self.costs[a.id, contact.id], a.id))
            if len(free) < count:
                return []
            # Bound combinations before constructing the Cartesian product.
            choices = list(combinations(free[:count + 2], count))
            choices.sort(key=lambda group: sum(self.costs[a.id, contact.id] for a in group))
            pools.append(choices[:limit])
        if not pools:
            return []
        groups = [tuple(a for part in parts for a in part) for parts in product(*pools)]
        groups.sort(key=lambda group: (sum(self.costs[a.id, contact.id] for a in group),
                                       tuple(a.id for a in group)))
        return groups[:limit]

    def priority(self, contact):
        if contact.cls != HOSTILE:
            return contact.points
        groups = self.groups(contact, set(), 1)
        if not groups:
            return -math.inf
        net = contact.points - sum(self.costs[a.id, contact.id] for a in groups[0])
        # Lights are usually scarce; use the scenario's actual demand/supply.
        demand = {}
        supply = {}
        for c in self.scenario.contacts:
            if c.cls == HOSTILE and c.id not in self.banned:
                for sub, count in c.needs.items():
                    demand[sub] = demand.get(sub, 0) + count
        for a in self.effectors:
            supply[a.subtype] = supply.get(a.subtype, 0) + 1
        scarcity = sum(n * max(1.0, demand.get(sub, 0) / max(1, supply.get(sub, 0)))
                       for sub, n in contact.needs.items())
        return net / max(1.0, scarcity)
