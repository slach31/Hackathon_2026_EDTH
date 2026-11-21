"""Link-loss policy v2: policy.py plus fixes found in history/linkloss (see README).

Changes from policy.py, each behind a flag so it can be measured on its own:

- ZONE_CONTINUE: a scout whose route crosses a pop-up zone keeps flying and completes every
  observation before the crossing, then turns back. policy.py turned back at once and dropped
  them all. The zone check of a move now looks one step ahead (the simulator moves exactly
  rate x 1 s per step) with a radius margin, so a scout can reach the zone's edge.
- PARK: an effector without a confirmation does not turn back after a grace period. It parks
  just short of its target (PARK_LEAD_S of route time), inside the separation exemption around
  the shared target, and engages if the confirmation arrives later. Turning back flew it
  head-on into its own strike partners (3 of the 6 R8 failures). An aborted effector already
  inside that exemption radius also holds instead of turning back.
- TAKEOFF_FIX: a peer still on the ground that will take off within the look-ahead is checked
  for separation even after it has announced a state (policy.py skipped it).
- GHOST_S: a peer effector is assumed to stay parked at the end of its route for this long
  after its planned engagement (it may be parked unconfirmed), instead of vanishing.
- RESEND_NEAR_M: an off-plan aircraft re-announces its state when a peer it believes is
  within this distance was not that close at its last announcement (a message sent while out
  of range is lost, and the fixed 8 s refresh came too late at 250 m range).
- HOPELESS: an aircraft whose remaining observations or engagement can no longer happen inside
  the contact's window aborts; one still on the ground never takes off. Aircraft held on the
  ground for minutes then took off into airspace nobody predicted them in.
- Take-off check: a lower-id peer (which may move any way) is assumed able to take off now.
- LAG_REFRESH_S (loop 2): an off-plan aircraft whose real position is more than LAG_M from the
  plan's position re-announces every LAG_REFRESH_S seconds instead of REFRESH_S. Peers that
  never heard it place it there, and the 8 s refresh was often sent while they were out of range.
- HOVER_ABORT (loop 3): an aborted aircraft hovers where it is and moves only when hovering
  would break separation. Flying the route backwards met outbound traffic head-on and ended in
  the busy base (most loop 2 failures involved a returning aircraft).
- QUIET_HOVER_M (loop 4): a hovering aircraft keeps the fast refresh only while a peer is
  believed within this distance (hover repeats were 63 % of all messages in loop 3).
- RETURN_LAST (loop 5, off: 9 -> 13 failed runs): a drone that has not aborted reverses to avoid a peer only when no
  forward or hold move keeps d_sep + RETURN_FLOOR_M and reversing gains RETURN_GAIN_M more room.
  Reversals were in 8 of the 11 colliding pairs in loop 4, often heard too late by the peer.
- RELAY: an effector that heard its target's confirmation repeats it shortly before its own
  engagement, so strike partners close behind it hear it too.
- O_DENSE_S / O_GAP_S: a scout repeats an "observed" notice every second for O_DENSE_S, then
  every O_GAP_S seconds (policy.py: every second for about 100 s).

Wire format: unchanged from policy.py (see its docstring).
"""
import math

from fleetkit.geometry import segment_enters_cylinder
from fleetkit.linkloss import RouteClock
from fleetkit.model import EFFECTOR, SCOUT, parse_plan

# ------------------------------------------------------------------ v2 switches
ZONE_CONTINUE = True
PARK = True
TAKEOFF_FIX = True
GHOST_S = 0.0           # 0 = off
RESEND_NEAR_M = 350.0   # 0 = off
RESEND_MIN_GAP_S = 2.0
RELAY = True
HOPELESS = True         # give up (or never take off) once no remaining task can be on time
O_DENSE_S = 6.0
O_GAP_S = 3.0

PARK_LEAD_S = 1.5       # route time short of the engagement where an unconfirmed effector waits
ZONE_LOOK_S = 1.5       # route time a move is checked against known zones
ZONE_MARGIN_M = 4.0     # extra radius when checking zones
ZONE_STUCK_S = 10.0     # a scout held short of a zone this long gives up its observations
RELAY_LEADS = (10.0, 5.0)  # route time before engagement at which a relay is sent

# ------------------------------------------------------------------ as policy.py
HOLD_LEAD_S = 3.0
HOLD_GRACE_S = 15.0
ANNOUNCE_AFTER_S = HOLD_GRACE_S + 10.0
HORIZON_S = 5.0
SAMPLES = 10
SEP_MARGIN_M = 15.0
NEAR_M = 400.0
REPEAT_S = 4.0
REFRESH_S = 8.0
LAG_REFRESH_S = 3.0     # loop 2: refresh this often while far from where the plan puts us
LAG_M = 60.0            # "far": real position this far from the plan's position at the real time
HOVER_ABORT = True      # loop 3: an aborted aircraft hovers instead of retracing its route
QUIET_HOVER_M = 800.0   # loop 4: a hovering aircraft keeps the fast refresh only with a peer this close
RETURN_LAST = False     # loop 5 (measured worse, off): a drone that has not aborted reverses only as a last resort
RETURN_FLOOR_M = 5.0    # reverse only when every forward or hold move keeps less than d_sep + this
RETURN_GAIN_M = 10.0    # and reversing keeps at least this much more room
EXEMPT_MARGIN_M = 25.0
SLACK_S = 1.5
NEWS_SENDS = 0
NEWS_GAP_S = 2.0
NEWS_ECHOES = 2
MAX_TEXT = 62

# ---------------------------------------------------------------------- compact wire format
ALPHABET = "".join(chr(c) for c in range(32, 127) if chr(c) not in '"\\')
BASE = len(ALPHABET)    # 93
DIGIT = {ch: i for i, ch in enumerate(ALPHABET)}
COORD_OFFSET = 1000
RADIUS_OFFSET = 40
ZONE_TOP_M = 200.0
ZONE_PAD_M = 1.5


def enc(value, width):
    value = max(0, min(int(value), BASE ** width - 1))
    out = []
    for _ in range(width):
        value, r = divmod(value, BASE)
        out.append(ALPHABET[r])
    return "".join(reversed(out))


def dec(text):
    value = 0
    for ch in text:
        value = value * BASE + DIGIT[ch]
    return value


def _fields(cw):
    return {"V": 4, "O": cw + 2, "R": cw, "Z": 5}


def encode_records(records, cw=1):
    text = ""
    for rec in records:
        kind = rec[0]
        if kind == "V":
            part = "V" + enc(round(rec[1] * 10), 3) + enc(round((rec[2] + 1.0) * 20), 1)
        elif kind == "O" and rec[1] < BASE ** cw:
            part = "O" + enc(rec[1], cw) + enc(rec[2], 2)
        elif kind == "R" and rec[1] < BASE ** cw:
            part = "R" + enc(rec[1], cw)
        elif kind == "Z":
            x, y = round(rec[1]) + COORD_OFFSET, round(rec[2]) + COORD_OFFSET
            r = math.ceil(rec[3]) - RADIUS_OFFSET
            if not (0 <= x < BASE ** 2 and 0 <= y < BASE ** 2 and 0 <= r < BASE):
                continue
            part = "Z" + enc(x, 2) + enc(y, 2) + enc(r, 1)
        else:
            continue
        if len(text) + len(part) <= MAX_TEXT:
            text += part
    return text


def decode_records(text, cw=1):
    out, i = [], 0
    if not isinstance(text, str):
        return out
    fields = _fields(cw)
    while i < len(text):
        kind = text[i]
        n = fields.get(kind)
        body = text[i + 1:i + 1 + n] if n else ""
        if n is None or len(body) < n or any(ch not in DIGIT for ch in body):
            break
        if kind == "V":
            out.append(("V", dec(body[:3]) / 10.0, dec(body[3]) / 20.0 - 1.0))
        elif kind == "O":
            out.append(("O", dec(body[:cw]), dec(body[cw:])))
        elif kind == "R":
            out.append(("R", dec(body)))
        else:
            out.append(("Z", dec(body[:2]) - COORD_OFFSET, dec(body[2:4]) - COORD_OFFSET,
                        dec(body[4]) + RADIUS_OFFSET))
        i += 1 + n
    return out


def _d(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


class Policy:
    def __init__(self, scenario, plan, aircraft_id):
        routes, _ = parse_plan(plan)
        aircraft = scenario.aircraft_by_id()
        rules = scenario.rules
        self.id = aircraft_id
        self.ac = aircraft[aircraft_id]
        self.route = routes[aircraft_id]
        self.clock = RouteClock(self.route)
        self.sep = rules.d_sep_m + SEP_MARGIN_M
        self.d_sep = rules.d_sep_m
        self.exempt = rules.exempt_radius_m - EXEMPT_MARGIN_M
        self.contacts = scenario.contact_by_id()
        self.contact_ids = [c.id for c in scenario.contacts]
        self.contact_index = {c: i for i, c in enumerate(self.contact_ids)}
        self.cw = 1 if len(self.contact_ids) <= BASE else 2
        self.news = {}
        self.margin = rules.confirm_margin_s + 1.5
        self.target = self._target(self.route, self.ac)
        self.engage_t = self.route.waypoints[-1].t if self.target else None
        self.peers = {}
        for aid, r in routes.items():
            if aid != aircraft_id:
                self.peers[aid] = (RouteClock(r), self._target(r, aircraft[aid]))
        self.peer_state = {}
        self.obs_end, self.last_engage = {}, {}
        for aid, r in routes.items():
            for wp in r.waypoints:
                if wp.action == "observe" and aircraft[aid].type == SCOUT:
                    self.obs_end.setdefault(wp.target, []).append(wp.t + wp.dwell_s)
                elif wp.action == "engage" and aircraft[aid].type == EFFECTOR:
                    self.last_engage[wp.target] = max(self.last_engage.get(wp.target, 0.0), wp.t)
        # own observations: route time at which each one is complete (departure from it)
        self.obs_deps = []
        if self.ac.type == SCOUT:
            self.obs_deps = [self.clock.dep[k] for k, wp in enumerate(self.route.waypoints)
                             if wp.action == "observe"]
        # own tasks: (route time of the task, end of the contact's window)
        self.tasks = []
        for wp in self.route.waypoints:
            c = self.contacts.get(wp.target) if wp.action in ("observe", "engage") else None
            if c is not None and c.window is not None:
                self.tasks.append((wp.t + (wp.dwell_s if wp.action == "observe" else 0.0), c.window[1]))
        self.confirmed = None
        self.heard_confirm = set()     # contacts whose confirmation came by message
        self.reclassified, self.zones = set(), []
        self.announce, self.reported = {}, {}
        self.aborted = False
        self.zone_stuck_since = None
        self.rate = 1.0
        self.sent = None
        self.last_sent, self.repeats = -1e9, 0
        self.near_at_send = set()      # peers believed close at the last announcement
        self.relays_left = list(RELAY_LEADS) if RELAY else []

    @staticmethod
    def _target(route, ac):
        last = route.waypoints[-1]
        return last.target if (ac.type == EFFECTOR and last.action == "engage") else None

    # ------------------------------------------------------------------ knowledge
    def _learn(self, obs):
        if self.confirmed is None:
            self.confirmed = {c: min(ends) for c, ends in self.obs_end.items() if min(ends) <= obs["t_loss"]}
        t = obs["t"]
        for e in obs["sensed"]:
            if e["kind"] == "reclassify":
                self._learn_reclassified(e["contact"], t)
            elif e["kind"] == "popup_zone":
                self._learn_zone((e["center"][0], e["center"][1], e["radius_m"], e["top_m"]), t)
        for m in obs["inbox"]:
            for rec in decode_records(m["payload"], self.cw):
                kind = rec[0]
                if kind == "V":
                    self.peer_state[m["from"]] = (rec[1], rec[2], m["sent_t"])
                elif kind == "O" and rec[1] < len(self.contact_ids):
                    c, end = self.contact_ids[rec[1]], rec[2]
                    if c not in self.confirmed or end < self.confirmed[c]:
                        self.confirmed[c] = end
                        self.heard_confirm.add(c)
                elif kind == "R" and rec[1] < len(self.contact_ids):
                    self._learn_reclassified(self.contact_ids[rec[1]], t, m["from"])
                elif kind == "Z":
                    self._learn_zone((rec[1], rec[2], rec[3] + ZONE_PAD_M, ZONE_TOP_M), t, m["from"])

    def _note_news(self, key, rec, t, sender):
        item = self.news.get(key)
        if item is None:
            item = self.news[key] = {"rec": rec, "left": NEWS_SENDS, "next": t, "echoes": set()}
        if sender is not None:
            item["echoes"].add(sender)
            if len(item["echoes"]) >= NEWS_ECHOES:
                item["left"] = 0

    def _learn_reclassified(self, contact, t, sender=None):
        self.reclassified.add(contact)
        if contact in self.contact_index:
            self._note_news(("R", contact), ("R", self.contact_index[contact]), t, sender)

    def _learn_zone(self, z, t, sender=None):
        same = next((k for k in self.zones if abs(k[0] - z[0]) <= 3 and abs(k[1] - z[1]) <= 3
                     and abs(k[2] - z[2]) <= 3), None)
        if same is None:
            self.zones.append(z)
            same = z
        self._note_news(("Z", round(same[0]), round(same[1])), ("Z", same[0], same[1], same[2]), t, sender)

    def _enters_zone(self, a, b):
        return any(segment_enters_cylinder(a, b, (z[0], z[1]), z[2] + ZONE_MARGIN_M, z[3])
                   for z in self.zones)

    def _zone_block_tau(self, tau):
        """Route time at which the remaining route starts the segment that enters a known zone."""
        if not self.zones:
            return None
        k = self.clock.next_waypoint(tau)
        pts = [self.clock.position(tau)] + self.clock.pos[k:]
        starts = [tau] + self.clock.dep[k:]
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            if self._enters_zone(a, b):
                return starts[i]
        return None

    # ------------------------------------------------------------------ separation
    def _peer_pos(self, aid, t, rate_override=None):
        clock, tgt = self.peers[aid]
        tau0, rate, t0 = self.peer_state.get(aid, (None, 1.0, None))
        if rate_override is not None and tau0 is not None:
            rate = rate_override
        tau = t if tau0 is None else tau0 + rate * (t - t0)
        if tau >= clock.end - 1e-9:
            if GHOST_S > 0 and tgt is not None and tau - clock.end <= GHOST_S:
                return clock.position(clock.end - PARK_LEAD_S)   # may be parked short of it
            return None
        if tau <= clock.start + 1e-9:
            return None
        return clock.position(tau)

    def _takes_off_soon(self, aid, t):
        clock = self.peers[aid][0]
        tau0, rate, t0 = self.peer_state.get(aid, (None, 1.0, None))
        now = t if tau0 is None else tau0 + rate * (t - t0)
        if tau0 is not None and aid < self.id:
            rate = 1.0                 # right of way: it may take off at any moment
        later = (t + HORIZON_S + SLACK_S) if tau0 is None else tau0 + max(rate, 0.0) * (t + HORIZON_S + SLACK_S - t0)
        return now <= clock.start + 1e-9 and later >= clock.start

    def _room(self, t, tau, rate):
        here = self.clock.position(tau)
        near = []
        for aid in self.peers:
            p = self._peer_pos(aid, t)
            if p is None:
                clock = self.peers[aid][0]
                if TAKEOFF_FIX:
                    if self._takes_off_soon(aid, t):
                        p = clock.pos[0]
                elif aid not in self.peer_state and t <= clock.start <= t + HORIZON_S + SLACK_S:
                    p = clock.pos[0]
            if p is not None and _d(p, here) < NEAR_M:
                near.append(aid)
        best = math.inf
        for i in range(1, SAMPLES + 1):
            dt = HORIZON_S * i / SAMPLES
            my_tau = min(max(tau + rate * dt, self.clock.start), self.clock.end)
            if my_tau <= self.clock.start + 1e-9:
                continue
            me = self.clock.position(my_tau)
            for aid in near:
                known = aid in self.peer_state
                rates = (None, -1.0, 0.0, 1.0) if (known and aid < self.id) else (None,)
                shifts = (-SLACK_S, 0.0, SLACK_S) if known else (0.0,)
                for ro, sh in ((ro, sh) for ro in rates for sh in shifts):
                    p = self._peer_pos(aid, t + dt + sh, ro)
                    if p is None:
                        continue
                    tgt = self.peers[aid][1]
                    if tgt is not None and tgt == self.target:
                        c = self.contacts[tgt].pos
                        if _d(me, c) <= self.exempt and _d(p, c) <= self.exempt:
                            continue
                    best = min(best, _d(me, p))
        return best

    # ------------------------------------------------------------------ decision
    def _near_target(self, tau):
        if self.target is None:
            return False
        return _d(self.clock.position(tau), self.contacts[self.target].pos) <= self.exempt

    def _options(self, obs):
        tau, t = obs["tau"], obs["t"]
        block = self._zone_block_tau(tau)
        if block is not None and not self.aborted:
            if ZONE_CONTINUE and self.ac.type == SCOUT:
                pending = any(tau < dep <= block + 1e-6 for dep in self.obs_deps)
                stuck = self.zone_stuck_since is not None and t - self.zone_stuck_since > ZONE_STUCK_S
                if not pending or stuck:
                    self.aborted = True
            else:
                self.aborted = True
        if self.target is not None and self.target in self.reclassified:
            self.aborted = True
        if HOPELESS and not self.aborted:
            left = [(x, end) for x, end in self.tasks if x >= tau - 1e-6]
            if left and all(t + (x - tau) > end for x, end in left):
                self.aborted = True
        if (not PARK and self.target is not None and not self.aborted
                and self.target not in self.confirmed and t > self.engage_t + HOLD_GRACE_S):
            self.aborted = True
        if self.aborted:
            if PARK and self.target is not None and self.ac.type == EFFECTOR and self._near_target(tau):
                return [0.0, -1.0], False       # parked inside the exemption: stay out of the way
            if HOVER_ABORT:
                if tau <= self.clock.start + 1e-9:
                    return [-1.0], False            # still on its pad: stay down
                return [0.0, -1.0, 0.5, 1.0], False
            return [-1.0, 0.0, 0.5, 1.0], False
        if self.target is not None and self.target in self.confirmed:
            if t + (self.engage_t - tau) < self.confirmed[self.target] + self.margin:
                return [1.0, 0.5, 0.0, -1.0], False
            return [1.0, 0.5, 0.0, -1.0], True
        if self.target is not None:
            return [1.0, 0.5, 0.0, -1.0], False
        return [1.0, 0.5, 0.0, -1.0], True

    def _allowed(self, tau, rate, may_engage):
        if rate == 0.0:
            return True
        if rate > 0 and self.target is not None and not may_engage:
            if PARK:
                if tau + rate * 1.0 > self.engage_t - PARK_LEAD_S:
                    return False
            elif tau + rate * (HORIZON_S + 1.0) >= self.engage_t - 0.5:
                return False
        if self.zones:
            look = ZONE_LOOK_S if ZONE_CONTINUE else HORIZON_S + 1.0
            end = tau + rate * look
            lo, hi = (tau, end) if rate > 0 else (end, tau)
            lo, hi = max(lo, self.clock.start), min(hi, self.clock.end)
            taus = [lo] + [x for x in self.clock.breakpoints(lo, hi)] + [hi]
            pts = [self.clock.position(x) for x in taus]
            if any(self._enters_zone(a, b) for a, b in zip(pts, pts[1:])):
                return False
        return True

    def step(self, obs):
        self._learn(obs)
        t, tau = obs["t"], obs["tau"]
        options, may_engage = self._options(obs)
        allowed = [r for r in options if self._allowed(tau, r, may_engage)]
        if ZONE_CONTINUE and self.zones and not self.aborted and 1.0 in options and not any(r > 0 for r in allowed):
            if self.zone_stuck_since is None:
                self.zone_stuck_since = t
        else:
            self.zone_stuck_since = None
        options = allowed or [0.0]
        choice, best_room = options[0], -1.0
        rooms = {}
        for r in options:
            room = rooms[r] = self._room(t, tau, r)
            if room >= self.sep:
                choice = r
                break
            if room > best_room:
                choice, best_room = r, room
        if (RETURN_LAST and choice == -1.0 and not self.aborted and rooms.get(-1.0, 0.0) < self.sep):
            ahead = {r: rooms.get(r, self._room(t, tau, r)) for r in options if r >= 0.0}
            if ahead:
                r_best = max(ahead, key=ahead.get)
                if ahead[r_best] >= self.d_sep + RETURN_FLOOR_M or rooms[-1.0] < ahead[r_best] + RETURN_GAIN_M:
                    choice = r_best             # a hold or a slow-down surprises peers less than reversing
        self.rate = choice
        action = ({"kind": "continue"} if choice == 1.0 else {"kind": "hold"} if choice == 0.0
                  else {"kind": "return"} if choice == -1.0 else {"kind": "speed", "scale": choice})
        return action, self._message(obs, tau, choice)

    def _peer_within(self, t, tau, radius):
        here = self.clock.position(tau)
        for aid in self.peers:
            p = self._peer_pos(aid, t)
            if p is None and self._takes_off_soon(aid, t):
                p = self.peers[aid][0].pos[0]
            if p is not None and _d(p, here) <= radius:
                return True
        return False

    def _close_peers(self, t, tau):
        here = self.clock.position(tau)
        out = set()
        for aid in self.peers:
            p = self._peer_pos(aid, t)
            if p is not None and _d(p, here) <= RESEND_NEAR_M:
                out.add(aid)
        return out

    def _message(self, obs, tau, rate):
        t = obs["t"]
        records = []
        if self.ac.type == SCOUT:
            for c in obs["own_observed"]:
                if c not in self.reported:
                    self.reported[c] = int(math.ceil(t))
                    if c in self.last_engage:
                        self.announce[c] = self.last_engage[c] + ANNOUNCE_AFTER_S
            live = sorted(c for c, until in self.announce.items()
                          if t <= until and (t - self.reported[c] <= O_DENSE_S
                                             or int(t - self.reported[c]) % max(1, int(O_GAP_S)) == 0))
            records += [("O", self.contact_index[c], self.reported[c]) for c in live[:2]]
        if (self.relays_left and self.target is not None and self.target in self.heard_confirm
                and self.target in self.confirmed and self.engage_t - tau <= self.relays_left[0]):
            while self.relays_left and self.engage_t - tau <= self.relays_left[0]:
                self.relays_left.pop(0)
            records.append(("O", self.contact_index[self.target], self.confirmed[self.target]))
        changed = self.sent is None and rate != 1.0 or self.sent is not None and rate != self.sent[0]
        off = abs(tau - t) > 0.25 or rate != 1.0
        refresh = REFRESH_S
        if LAG_REFRESH_S > 0 and off and tau > self.clock.start + 1e-9:   # not while still on the ground
            planned = None if t >= self.clock.end else self.clock.position(max(t, self.clock.start))
            if planned is None or _d(planned, self.clock.position(tau)) > LAG_M:
                refresh = LAG_REFRESH_S      # peers that never heard us place us far from here
                if QUIET_HOVER_M > 0 and rate == 0.0 and not self._peer_within(t, tau, QUIET_HOVER_M):
                    refresh = REFRESH_S      # hovering with nobody around: the slow refresh is enough
        due = self.sent is not None and off and t - self.last_sent >= (REPEAT_S if self.repeats < 1 else refresh)
        close = set()
        if RESEND_NEAR_M > 0 and self.sent is not None and off:
            close = self._close_peers(t, tau)
            if close - self.near_at_send and t - self.last_sent >= RESEND_MIN_GAP_S:
                due = True
        if changed:
            self.sent, self.repeats = (rate, t), 0
        elif due:
            self.repeats += 1
        if changed or due:
            records.insert(0, ("V", tau, rate))
            self.last_sent = t
            if RESEND_NEAR_M > 0:
                self.near_at_send = close or self._close_peers(t, tau)
        elif RESEND_NEAR_M > 0 and off:
            self.near_at_send &= close      # forget peers that moved away: announce again if they return
        for item in self.news.values():
            if item["left"] > 0 and t >= item["next"]:
                records.append(item["rec"])
                item["left"] -= 1
                item["next"] = t + NEWS_GAP_S
        return encode_records(records, self.cw) or None
