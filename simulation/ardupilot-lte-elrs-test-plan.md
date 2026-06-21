# ArduPilot Pre-Flight Simulation & Test Plan
## Multi-Link Comms Setup: ELRS 4.0.1 (MAVLink mode) + LTE Modem + RPi Companion Computer

**Status:** Draft / Work in progress — handed off for further development with coding agent.

---

## 1. System Overview

| Component | Role |
|---|---|
| Flight Controller | Matek H743 (ArduPilot) |
| RC Link | ELRS 4.0.1 receiver, switched to **MAVLink mode** (not standard RC mode) |
| Companion Computer | Raspberry Pi |
| Secondary Telemetry Link | LTE modem (via RPi) |
| Comms Protocol | MAVLink over both links simultaneously |

**Core concern driving this test plan:** Two independent MAVLink sources (ELRS link and LTE/RPi link) talking to the same flight controller at the same time. Need to validate failsafe behavior, link arbitration, and degraded-mode handling *before* first flight.

---

## 2. Open Questions / Risks to Resolve

- [ ] Which link has `SYSID_MYGCS` authority — or do both, and if so how does the FC arbitrate competing mode-change commands?
- [ ] Does the FC produce duplicate/conflicting telemetry streams (`SR_*` rates) to both links?
- [ ] What happens when both links report heartbeats at different effective latencies (ELRS low-latency vs. LTE 100s of ms)?
- [ ] Is `FS_GCS_TIMEOUT` (default 5s) appropriate given expected LTE latency/jitter?
- [ ] Does ELRS MAVLink mode register as "RC" for `FS_THR_ENABLE` purposes, or purely as a GCS-type link? (Needs confirmation — this changes which failsafe parameters actually govern it.)
- [ ] Behavior when **one** link drops vs. **both** links drop simultaneously.
- [ ] RPi crash/hang scenario — does FC have any visibility into companion computer health independent of the MAVLink stream itself?

---

## 3. Test Tiers (Recommended Sequence)

```
SITL (logic dev)
   → HITL (hardware integration)
      → Bench fault injection (link kill/degrade tests)
         → Tethered full-system test (props off, everything live)
            → First flight (props on, controlled environment)
```

Each tier de-risks a different layer. Don't skip tiers — HITL catches hardware/wiring issues SITL can't see; bench fault injection catches failsafe logic issues that only show up with real (flaky) radio links.

---

### Tier 1 — SITL (Software In The Loop)

**Goal:** Validate MAVLink routing logic and failsafe parameter behavior in pure software, no hardware involved.

**Setup:**
```bash
sim_vehicle.py -v ArduCopter --console --map
# Connect GCS + RPi-side logic to SITL's MAVLink output (UDP 14550)
```

**What to test:**
- MAVLink routing path: GCS ↔ RPi ↔ SITL
- Failsafe parameter behavior in isolation:
  - `FS_THR_ENABLE`
  - `FS_GCS_ENABLE`
  - `FS_EKF_ACTION`
  - `FS_GCS_TIMEOUT`
- Mode-switching via MAVLink `SET_MODE` / `MAV_CMD_DO_SET_MODE`
- Simulated RC loss behavior
- Simulated GCS link loss behavior

**Limitation:** No real ELRS, no real LTE modem, no real RPi-to-FC serial link. Logic only.

---

### Tier 2 — HITL (Hardware In The Loop)

**Goal:** Real flight controller hardware, real peripherals, simulated flight physics. Motors disconnected.

**Setup:**
- Enable ArduPilot `SIM_*` HITL parameters on the **real Matek H743**
- Connect Mission Planner or QGroundControl in HITL mode
- **Props off, motors disconnected** — HITL does not arm real motors but treat this as a hard safety rule anyway

**What to test:**
- Real ELRS receiver → FC MAVLink stream (actual hardware path, not simulated)
- Real RPi ↔ FC serial/USB MAVLink communication
- RC failsafe triggered by physically powering off the ELRS TX
- Dual-link contention with real timing characteristics (ELRS vs LTE latency profiles)
- `SYSID_MYGCS` behavior with real packet timing
- Stream rate (`SR_*`) behavior under real link load

---

### Tier 3 — Bench Fault Injection (RPi / Link Focus)

**Goal:** Deliberately break each link, individually and together, and observe FC + RPi response. This is the tier most specific to your dual-MAVLink-source setup.

**Test matrix:**

| # | Scenario | Trigger method | Expected / Observed behavior |
|---|---|---|---|
| 1 | LTE link drops | `ifconfig wwan0 down` (or pull modem) | RPi detects loss? FC enters GCS failsafe? |
| 2 | ELRS link drops | Power off ELRS TX | RC/MAVLink failsafe triggers correctly? |
| 3 | Both links drop | Combine #1 + #2 | FC executes configured failsafe action (RTL/Land)? |
| 4 | ELRS floods FC with high-rate MAVLink | Increase ELRS stream rate | LTE-side heartbeats still processed without starvation? |
| 5 | RPi process crash | `kill` the mavlink-router/companion process | FC behavior with companion software dead but link physically up? |
| 6 | LTE latency spike | `tc qdisc add dev wwan0 root netem delay 2000ms` | Does GCS failsafe trigger at the configured timeout, or spuriously early/late? |
| 7 | LTE packet loss | `tc qdisc add dev wwan0 root netem loss 5%` | Heartbeat loss tolerance — does FC flap in/out of failsafe? |
| 8 | LTE link restored after drop | Re-enable `wwan0` | Does GCS failsafe clear cleanly? Any stuck state? |
| 9 | ELRS link restored after drop | Power ELRS TX back on | Does RC/MAVLink failsafe clear cleanly? |
| 10 | Conflicting mode commands from both links simultaneously | Send `DO_SET_MODE` from both GCS instances near-simultaneously | Which wins? Is behavior deterministic? |

**Tools:**
- `mavlink-router` or `MAVProxy` on RPi — for logging, link multiplexing, and fault injection
- `tc netem` — for realistic LTE impairment simulation (delay, jitter, loss, corruption)

**Realistic LTE impairment baseline (starting point, tune from real-world data):**
```bash
sudo tc qdisc add dev wwan0 root netem delay 300ms 100ms loss 5% corrupt 1%
```

---

### Tier 4 — Tethered Full-System Test

**Goal:** Closest pre-flight validation to real conditions. All systems live, props removed, drone restrained/tethered.

**Configuration:**
- FC powered, **props removed** (hard requirement)
- RPi running full production stack (mavlink-router, companion scripts)
- LTE modem connected and active (real cellular link, not simulated)
- ELRS TX on, in MAVLink mode
- GCS connected over LTE

**Sequential checklist:**
1. [ ] Verify simultaneous dual telemetry: ELRS + LTE both delivering valid MAVLink streams
2. [ ] Kill LTE → confirm FC failsafe behavior matches Tier 3 findings
3. [ ] Kill ELRS TX → confirm RC failsafe behavior matches Tier 3 findings
4. [ ] Kill both → confirm combined failsafe (should RTL or Land per configured `FS_*` params)
5. [ ] Restore LTE → confirm GCS failsafe clears correctly, no stuck state
6. [ ] Restore ELRS → confirm RC failsafe clears correctly, no stuck state
7. [ ] Repeat with motors armed (props still off) to validate arming/disarming logic under each fault condition

---

## 4. Parameters to Review / Tune

| Parameter | Purpose | Notes for this setup |
|---|---|---|
| `FS_GCS_ENABLE` | Enables GCS failsafe | Needs to be enabled given LTE is a primary link |
| `FS_GCS_TIMEOUT` | Timeout before GCS failsafe triggers (default 5s) | May need tuning given LTE latency/jitter profile — validate in Tier 3 #6/#7 |
| `FS_THR_ENABLE` | RC throttle failsafe | Confirm how this interacts with ELRS-in-MAVLink-mode (may not behave like standard RC) |
| `FS_EKF_ACTION` | EKF failsafe action | Independent of comms links but worth confirming doesn't interact unexpectedly |
| `SYSID_MYGCS` | Identifies authoritative GCS system ID | Critical — determine intended value given two MAVLink sources |
| `SR_*` (stream rates) | Telemetry stream rate per link | Check for duplication/conflicts across the two links |

---

## 5. Next Steps (for agent / further dev work)

- [ ] Confirm whether ELRS-in-MAVLink-mode is treated by ArduPilot as an RC-type failsafe source or a GCS-type source — this determines which `FS_*` parameters actually govern it. Source: ArduPilot docs / firmware behavior, needs verification against actual `MAV_TYPE`/link config.
- [ ] Write `mavlink-router` config for RPi that explicitly defines priority/arbitration between ELRS and LTE links, if such config is possible.
- [ ] Script the Tier 3 fault injection matrix (table in Section 3) as a repeatable test harness — ideally one script per scenario, logged output, pass/fail criteria.
- [ ] Define explicit pass/fail criteria for each test (e.g. "GCS failsafe must trigger within X±Y seconds of link loss").
- [ ] Capture real-world LTE latency/jitter/loss stats from an actual field test (modem connected, drone stationary) to replace the placeholder `netem` values in Section 3 with realistic figures.
- [ ] Determine RTL vs Land as the default combined-failsafe action and confirm `FS_*` config matches that decision.
- [ ] Build a simple status/log capture mechanism on the RPi so Tier 3/4 test runs are recorded automatically (timestamps of link state changes, FC mode changes, failsafe events) rather than relying on manual observation.

---

## 6. Related Hardware Note (separate workstream)

JC02-1 rangefinder integration with the Matek H743 requires a custom Lua scripting solution, since the sensor's proprietary hex packet protocol isn't compatible with standard ArduPilot drivers. Tracked separately from comms/failsafe testing above, but flagged here since it's part of the same overall pre-flight readiness effort.
