# ArduCopter SITL Simulation — Developer Reference

Tier 1 validation of dual-MAVLink-source (ELRS + LTE) failsafe behaviour before
first flight. ArduCopter runs as a Docker container; a pytest suite talks MAVLink
to it over TCP.

---

## Directory layout

```
simulation/
├── SIMULATION.md               ← this file
├── docker-compose.yml          ← orchestrates SITL + test-runner services
├── ardupilot-lte-elrs-test-plan.md   ← original test plan / open questions
├── docker/
│   ├── Dockerfile              ← ArduCopter SITL image (Ubuntu 22.04 + waf build)
│   └── entrypoint.sh           ← container start script (calls sim_vehicle.py)
├── params/
│   └── sitl_base.param         ← ArduPilot parameters injected at SITL boot
└── scripts/
    ├── common.py               ← shared MAVLink helpers (connect, arm, get_param …)
    ├── requirements.txt        ← pymavlink + pytest
    ├── Dockerfile              ← test-runner image (python:3.11-slim)
    ├── test_connectivity.py    ← Test suite 1: basic ports & param verification
    ├── test_dual_link.py       ← Test suite 2: dual-link command contention
    └── test_gcs_failsafe.py    ← Test suite 3: GCS failsafe trigger/clear
```

---

## Architecture overview

```
  Windows host (your PC)
  ┌──────────────────────────────────────────────────────────────┐
  │  Mission Planner (optional, for visual monitoring)           │
  │    connects to localhost:5760 via TCP                        │
  │                                                              │
  │  WSL2 Ubuntu 24.04 (Noble)                                   │
  │  ┌──────────────────────────────────────────────────────┐   │
  │  │  Docker Engine (native, no Docker Desktop)           │   │
  │  │                                                      │   │
  │  │  Docker bridge network: simulation_mavnet            │   │
  │  │  ┌────────────────────────┐  ┌──────────────────┐   │   │
  │  │  │  ardupilot-sitl        │  │  tests (ephemeral)│   │   │
  │  │  │                        │  │                   │   │   │
  │  │  │  sim_vehicle.py        │  │  pytest           │   │   │
  │  │  │  └─ arducopter (SITL)  │  │  └─ common.py     │   │   │
  │  │  │      SERIAL0 :5760 ←──────── connect_elrs()   │   │   │
  │  │  │      SERIAL1 :5761 ←──────── connect_lte()    │   │   │
  │  │  └────────────────────────┘  └──────────────────┘   │   │
  │  │         │           │                                │   │
  │  │    port 5760    port 5761  (bound to 0.0.0.0)        │   │
  │  └─────────┼───────────┼────────────────────────────────┘   │
  │            │           │  (localhostForwarding=true in       │
  │       localhost:5760  localhost:5761  .wslconfig)            │
  └──────────────────────────────────────────────────────────────┘
```

---

## Serial → TCP port mapping

| ArduPilot serial | Real hardware role | SITL TCP port | Who connects |
|------------------|--------------------|---------------|--------------|
| SERIAL0 (uartA)  | ELRS receiver MAVLink | **5760** | test_connectivity, test_dual_link, test_gcs_failsafe (ELRS link) |
| SERIAL1 (uartB)  | RPi / LTE MAVLink  | **5761** | test_dual_link, test_gcs_failsafe (LTE link) |

SERIAL0 defaults to TCP 5760 in SITL. SERIAL1 is activated via
`-A "--serial1=tcp:5761"` passed through `sim_vehicle.py` to the arducopter binary.

---

## Docker configuration

### SITL image — `docker/Dockerfile`

Built on Ubuntu 22.04. Key build steps:

1. Creates user `ardupilot` with passwordless sudo (required by ArduPilot prereqs script).
2. Pre-configures tzdata so `apt-get` never prompts for timezone.
3. Clones ArduPilot at `ARDUPILOT_TAG` (default: `master`) with shallow submodules.
4. Runs `install-prereqs-ubuntu.sh -y` (ArduPilot's official toolchain installer).
5. Compiles the SITL binary: `./waf configure --board sitl && ./waf copter`.
6. Exposes ports 5760 and 5761. Mounts `/params` as a volume for the param file.

Build time: ~30 minutes on first run. **Fully cached after the first build** — only
the `entrypoint.sh` layer rebuilds when you edit that script.

### Container startup — `docker/entrypoint.sh`

```
sim_vehicle.py
  -v ArduCopter
  --no-mavproxy       # no MAVProxy process — its heartbeats would prevent GCS failsafe
  --no-rebuild        # skip waf recompile on every container start
  --speedup 1
  --add-param-file=/params/sitl_base.param
  -A "--serial1=tcp:5761"   # -A passes args directly to arducopter binary
```

`--no-mavproxy` is critical: MAVProxy would inject its own GCS heartbeats, which
would prevent the GCS failsafe from ever triggering in tests.

### Test-runner image — `scripts/Dockerfile`

python:3.11-slim + pymavlink + pytest. Scripts are **volume-mounted** from
`scripts/` so you can edit tests without rebuilding the image.

### docker-compose.yml — key settings

```yaml
sitl:
  restart: "no"              # does NOT auto-restart on crash — must be started manually
  ports:
    - "5760:5760"
    - "5761:5761"
  volumes:
    - ./params:/params:ro    # param file injected read-only
    - sitl-logs:/home/ardupilot/ardupilot/logs

tests:
  profiles: [test]           # only starts when --profile test is supplied
  environment:
    SITL_HOST: sitl          # DNS name of the SITL container on mavnet
    ELRS_PORT: "5760"
    LTE_PORT:  "5761"
```

---

## Parameters — `params/sitl_base.param`

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `FS_GCS_ENABLE` | 2 | GCS failsafe: RTL when armed and all GCS links lost |
| `FS_GCS_TIMEOUT` | 5 | Seconds of silence before GCS failsafe fires |
| `FS_THR_ENABLE` | 2 | Throttle failsafe: RTL |
| `FS_EKF_ACTION` | 2 | EKF failsafe: AltHold |
| `MAV_GCS_SYSID` | 255 | GCS system ID tracked for heartbeat failsafe (ArduPilot ≥2024) |
| `SYSID_MYGCS` | 255 | Same param, old name (pre-2024 ArduPilot) — kept for compatibility |
| `SERIAL1_PROTOCOL` | 2 | Enable MAVLink2 on SERIAL1 (LTE port) |
| `SERIAL1_BAUD` | 115 | Baud rate for SERIAL1 (115 = 115200) |
| `ARMING_CHECK` | 0 | Disable pre-arm checks (old param name) |
| `ARMING_SKIPCHK` | 65535 | Skip ALL pre-arm checks (new param name, ArduPilot ≥2024) |
| `FRAME_CLASS` | 1 | Quadcopter |
| `FRAME_TYPE` | 1 | X-frame |

---

## How to start and stop everything

All commands below are run in **PowerShell on Windows** (they invoke WSL2 internally).
Working directory: `C:\src\dron\10inch\simulation`.

### First-time setup (once)

```powershell
# Open WSL2 shell to verify Docker Engine is running
wsl -u root -- docker info
```

### Build the SITL image (only needed once, or after Dockerfile changes)

```powershell
wsl -u root -- bash -c "cd /mnt/c/src/dron/10inch/simulation && docker compose build sitl"
```

Takes ~30 minutes on first build. Subsequent builds are instant (cached layers).
To pin a specific ArduPilot release instead of `master`:

```powershell
wsl -u root -- bash -c "cd /mnt/c/src/dron/10inch/simulation && docker compose build --build-arg ARDUPILOT_TAG=ArduCopter-4.5.7 sitl"
```

### Start SITL (background)

```powershell
wsl -u root -- bash -c "cd /mnt/c/src/dron/10inch/simulation && docker compose up -d sitl"
```

SITL is ready when port 5760 starts accepting connections (~8 seconds after start).
Check readiness:

```powershell
Test-NetConnection -ComputerName 127.0.0.1 -Port 5760 -InformationLevel Quiet
# Returns True when ready
```

### Connect Mission Planner (optional, for visual monitoring)

1. Open Mission Planner.
2. Top-right dropdown → select **TCP**.
3. Click **Connect**.
4. Host: `127.0.0.1`, Port: `5760`.

You should see the simulated drone appear in STABILIZE mode.

### Run the full test suite

```powershell
wsl -u root -- bash -c "cd /mnt/c/src/dron/10inch/simulation && docker compose --profile test run --rm tests"
```

To run a single test file:

```powershell
wsl -u root -- bash -c "cd /mnt/c/src/dron/10inch/simulation && docker compose --profile test run --rm tests pytest test_gcs_failsafe.py -v"
```

### View live SITL logs

```powershell
wsl -u root -- bash -c "docker logs ardupilot-sitl -f"
```

### Stop SITL

```powershell
wsl -u root -- bash -c "cd /mnt/c/src/dron/10inch/simulation && docker compose down"
```

`docker compose down` removes the container and the `mavnet` bridge network.
Use `docker compose stop sitl` to stop without removing (faster restart next time,
but leaves container in exited state).

### Restart SITL (after it crashed or was stopped)

```powershell
wsl -u root -- bash -c "cd /mnt/c/src/dron/10inch/simulation && docker compose down && docker compose up -d sitl"
```

`down` + `up` (not `restart`) is important: it forces a fresh container with a
clean eeprom state. `restart` reuses the same container, which can replay corrupt
parameter state if a test left SITL in a bad condition.

---

## Windows networking note

SITL ports (5760, 5761) are bound inside WSL2, not directly on Windows. They reach
`localhost` on Windows via WSL2's localhost-forwarding feature, which requires:

```ini
# C:\Users\<you>\.wslconfig
[wsl2]
localhostForwarding=true
```

If you edit `.wslconfig`, restart WSL2 first:

```powershell
wsl --shutdown
# then start SITL again
```

---

## Test suite reference

All tests live under `scripts/`. Scripts are volume-mounted into the test container
so edits take effect immediately without rebuilding.

### `common.py` — shared helpers

| Function | Description |
|----------|-------------|
| `connect(port)` | Open TCP MAVLink connection, wait for FC heartbeat |
| `connect_elrs()` | Connect to ELRS port (5760) |
| `connect_lte()` | Connect to LTE port (5761) |
| `send_heartbeat(conn)` | Send one GCS heartbeat (MAV_TYPE_GCS, sysid=255) |
| `get_param(conn, name)` | Fetch a single parameter by name (10 s timeout) |
| `set_mode(conn, name)` | Send SET_MODE command |
| `get_fc_mode(conn)` | Read current flight mode from next HEARTBEAT |
| `arm(conn, force=False)` | Send ARM command; `force=True` bypasses all pre-arm checks |
| `disarm(conn)` | Send DISARM command, wait for confirmation |
| `wait_for_statustext(conn, keyword)` | Block until STATUSTEXT containing keyword arrives |

Environment variables (set automatically by docker-compose for the tests service):

| Variable | Default | Meaning |
|----------|---------|---------|
| `SITL_HOST` | `localhost` | Hostname of the SITL container |
| `ELRS_PORT` | `5760` | SERIAL0 / ELRS TCP port |
| `LTE_PORT` | `5761` | SERIAL1 / LTE TCP port |

---

### `test_connectivity.py` — Suite 1: basic connectivity

**Purpose:** Verifies both TCP ports accept MAVLink connections and that key
failsafe parameters match `sitl_base.param`.

| Test | What it checks |
|------|----------------|
| `test_elrs_port_heartbeat` | SERIAL0 / port 5760 delivers a FC HEARTBEAT |
| `test_lte_port_heartbeat` | SERIAL1 / port 5761 delivers a FC HEARTBEAT independently |
| `test_both_ports_same_sysid` | Both ports report the same FC system ID (same autopilot, two links) |
| `test_param[FS_GCS_ENABLE-2.0]` | GCS failsafe is set to RTL |
| `test_param[FS_GCS_TIMEOUT-5.0]` | 5-second heartbeat timeout configured |
| `test_param[FS_THR_ENABLE-2.0]` | Throttle failsafe is set to RTL |
| `test_param[MAV_GCS_SYSID-255.0]` | GCS sysid matches our test connections (255) |
| `test_param[SERIAL1_PROTOCOL-2.0]` | MAVLink2 active on the LTE port |
| `test_initial_mode` | FC is in a valid mode (STABILIZE) at startup |

---

### `test_dual_link.py` — Suite 2: dual-link behaviour

**Purpose:** Validates that both links carry independent telemetry and that either
link can send commands. Documents contention behaviour.

| Test | What it checks | Finding |
|------|----------------|---------|
| `test_both_links_receive_telemetry` | FC sends HEARTBEAT on both SERIAL0 and SERIAL1 | ArduPilot multiplexes heartbeats across all active serial ports |
| `test_mode_change_via_elrs` | SET_MODE(GUIDED) from ELRS link is accepted | ELRS link has full command authority |
| `test_mode_change_via_lte` | SET_MODE(LOITER) from LTE link is accepted | LTE link has full command authority |
| `test_simultaneous_mode_commands` | Conflicting SET_MODE from both links simultaneously | Last-writer-wins; neither link has inherent priority |
| `test_sysid_authority_observation` | SET_MODE from a non-SYSID_MYGCS sender (sysid=200) | FC accepts commands from any sysid (MAV_GCS_SYSID only restricts failsafe tracking, not command authority) |

---

### `test_gcs_failsafe.py` — Suite 3: GCS failsafe

**Purpose:** Answers the primary open question: does GCS failsafe trigger when only
one link drops, or only when all links drop?

| Test | What it checks | Result |
|------|----------------|--------|
| `test_no_failsafe_while_both_active` | No failsafe with both links sending heartbeats for 8 s | **PASS** — as expected |
| `test_no_failsafe_when_only_lte_drops` | Drop LTE, ELRS still active — no failsafe expected | **PASS** — FC does NOT failsafe when one link survives |
| `test_failsafe_triggers_when_both_drop` | Close both connections, drone armed — failsafe must fire within FS_GCS_TIMEOUT + 2 s | **PASS** — failsafe fires as configured |
| `test_failsafe_clears_on_link_restore` | After failsafe, restore ELRS — expect "GCS Heartbeat Restored" STATUSTEXT | **XFAIL** — ArduPilot may not emit this specific text in all versions; treated as expected skip |

**Key operational finding:** The FC uses last-seen-any-GCS-heartbeat logic. Losing
one link while the other is alive does NOT trigger failsafe. All links must go silent
for `FS_GCS_TIMEOUT` seconds before RTL is commanded.

**Important implementation detail:** To reliably trigger the GCS failsafe in tests,
the TCP connections must be fully **closed** (not just stopped sending heartbeats).
Some ArduPilot versions detect TCP presence as link-alive even with no heartbeats.
The tests close both sockets and reconnect a silent observer to monitor STATUSTEXT.

---

## Known issues and workarounds

| Issue | Cause | Workaround |
|-------|-------|------------|
| SITL stays in crash loop after test run | Tests close TCP connections abruptly; arducopter gets SIGPIPE; sim_vehicle.py restarts it but eeprom state may be corrupted | Always use `docker compose down && docker compose up -d sitl` (not `restart`) between runs |
| `ARMING_CHECK=0` has no effect | Parameter was renamed in ArduPilot master 2024+ | `ARMING_SKIPCHK=65535` is the current name; both are in `sitl_base.param` |
| `SYSID_MYGCS` param test fails | Parameter renamed to `MAV_GCS_SYSID` in ArduPilot master 2024+ | Tests now use `MAV_GCS_SYSID`; `SYSID_MYGCS` kept in param file for older firmware |
| pytest cache warnings (read-only FS) | Test scripts are volume-mounted read-only | Cosmetic only; all tests still run correctly |

---

## Rebuilding after code changes

| What changed | Command needed |
|--------------|---------------|
| `params/sitl_base.param` | Just restart SITL — params are volume-mounted |
| `scripts/*.py` | No rebuild needed — scripts are volume-mounted |
| `docker/entrypoint.sh` | `docker compose build sitl` (fast, only last layer rebuilds) |
| `docker/Dockerfile` | `docker compose build sitl` (may be slow depending on which layer changed) |
| ArduPilot source code | `docker compose build sitl` with new `ARDUPILOT_TAG` |

---

## Relationship between simulation params and hardware params

Two separate param files exist in this repo:

| File | Used by | How loaded |
|------|---------|------------|
| `simulation/params/sitl_base.param` | SITL Docker container | Injected at container start via `--add-param-file` |
| `arduconfig.param` (repo root) | Matek H743 flight controller | Loaded via Mission Planner: Full Parameter List → Load from file |

`arduconfig.param` is a **full dump** of every ArduPilot parameter as it exists in
the real FC's flash (1500+ lines, one per parameter). It was exported from Mission
Planner and is the ground truth for what the physical drone is configured to do.

`sitl_base.param` is a **minimal override file** — it sets only the ~12 parameters
needed for the simulation scenario. The rest of SITL's parameters use ArduPilot's
built-in defaults.

---

### Serial port mapping: hardware vs simulation

The physical Matek H743 uses different UART numbers than the simulation.
The SERIALn numbers are just labels — the MAVLink behaviour being tested is
identical regardless of which port number carries it.

**Hardware (from `arduconfig.param`):**

| Port | Protocol | Baud | Role |
|------|----------|------|------|
| SERIAL0 | MAVLink2 (2) | 115200 | USB — Mission Planner via cable |
| SERIAL1 | Scripting (28) | 9600 | Lua scripting port (e.g. rangefinder driver) |
| SERIAL2 | MAVLink2 (2) | 460800 | **ELRS telemetry** (ELRS receiver MAVLink stream) |
| SERIAL3 | GPS (5) | 38400 | GPS receiver |
| SERIAL4 | MAVLink1 (1) | 115200 | Secondary/diagnostic port |
| SERIAL5 | Disabled (-1) | — | — |
| SERIAL6 | Scripting (28) | 9600 | Second Lua scripting port |
| SERIAL7 | MAVLink2 (2) | 460800 | **LTE / RPi MAVLink** |

**Simulation (`sitl_base.param` + `entrypoint.sh`):**

| Port | Protocol | TCP | Role |
|------|----------|-----|------|
| SERIAL0 | MAVLink2 | **5760** | Simulates ELRS MAVLink stream |
| SERIAL1 | MAVLink2 (2) | **5761** | Simulates LTE / RPi MAVLink stream |

SITL defaults SERIAL0 to TCP port 5760. SERIAL1 is activated by passing
`--serial1=tcp:5761` to the arducopter binary via the `-A` flag in `entrypoint.sh`.

The simulation uses SERIAL0/1 because they are the two simplest ports to activate
in SITL. The hardware uses SERIAL2/7 because those UARTs are physically wired to
the ELRS receiver and the RPi. The port number difference is irrelevant to the
failsafe logic being validated.

---

### Parameter differences: failsafe

This is the most safety-critical comparison.

| Parameter | Hardware value | Simulation value | Meaning |
|-----------|---------------|------------------|---------|
| `FS_GCS_ENABLE` | **0 — DISABLED** | **2 — RTL** | Whether GCS heartbeat loss triggers failsafe |
| `FS_GCS_TIMEOUT` | 5 | 5 | Seconds of silence before failsafe fires |
| `FS_THR_ENABLE` | 1 (Land) | 2 (RTL) | Action when RC throttle signal is lost |
| `FS_EKF_ACTION` | 1 (Land) | 2 (AltHold) | Action when EKF health fails |
| `FS_DR_ENABLE` | 2 (enabled) | not set | Dead-reckoning failsafe |
| `FS_OPTIONS` | 16 | not set | Bitmask: bit 4 = continue GUIDED on GCS failsafe |

**The most important difference: `FS_GCS_ENABLE=0` on the real drone.**

The hardware currently has GCS failsafe **completely disabled**. The simulation
tests a future configuration where it is enabled (`=2`, RTL). The simulation
validated that enabling it will behave correctly — losing both links causes RTL,
losing only one link does not cause RTL. Those results are only meaningful once the
hardware is also updated.

**Action required before first flight:**
Set `FS_GCS_ENABLE` to `1` (warn only) or `2` (RTL) on the hardware via Mission
Planner before flying with the LTE/ELRS dual-link setup. The Tier 1 simulation
results justify setting it to `2`.

The `FS_OPTIONS=16` on hardware is already a good choice: it means if the drone
is in GUIDED mode (e.g. executing a waypoint mission commanded over LTE), it will
continue rather than immediately RTL on a brief GCS heartbeat gap. When you enable
`FS_GCS_ENABLE`, consider also keeping `FS_OPTIONS=16`.

---

### Parameter differences: arming

| Parameter | Hardware | Simulation | Reason for difference |
|-----------|----------|------------|-----------------------|
| `ARMING_CHECK` | 1 (all checks) | 0 + `ARMING_SKIPCHK=65535` | Tests force-arm to skip GPS/EKF requirement |

The simulation disables arming checks so the test suite can arm the drone without
a GPS fix (SITL has simulated GPS but it may not satisfy all checks). The hardware
keeps full arming checks enabled, which is the correct and safe setting for flight.

---

### Parameter differences: GCS identity

| Parameter | Hardware | Simulation |
|-----------|----------|------------|
| `SYSID_MYGCS` | 255 | 255 (+ `MAV_GCS_SYSID=255`) |
| `SYSID_ENFORCE` | 0 | not set |

Both use sysid 255 for the GCS, so failsafe heartbeat tracking is consistent.
`SYSID_ENFORCE=0` on hardware means the FC accepts commands from any sender sysid
(not just sysid 255). The simulation tests confirmed this: a sender with sysid 200
can still issue mode-change commands. This is the expected ArduPilot default.

The hardware param file still uses `SYSID_MYGCS` (the old name), which tells us
the physical FC is running an older ArduPilot release than the SITL `master` branch
used in simulation. The simulation therefore includes both `SYSID_MYGCS` and
`MAV_GCS_SYSID` in `sitl_base.param` to cover both naming conventions.

---

### RC_CHANNELS_OVERRIDE and sysid — no change needed

When ELRS operates in MAVLink mode, the receiver generates `RC_CHANNELS_OVERRIDE`
messages to deliver stick inputs to the FC (sysid=255 by default, configurable in
ELRS firmware). Mission Planner joystick also sends `RC_CHANNELS_OVERRIDE` from
sysid=255 when joystick mode is active.

Both sources use the same sysid, so `SYSID_ENFORCE=1` cannot be used to give ELRS
exclusive RC authority without also blocking all MP commands (mode changes, param
writes, etc.). This is too coarse.

**It doesn't matter.** `RC_CHANNELS_OVERRIDE` is last-writer-wins, and:
- ELRS sends at ~50 Hz with ~5 ms latency
- MP joystick sends at ~10 Hz with 100–400 ms LTE latency

ELRS values are always fresher. When ELRS is alive it continuously wins. When ELRS
dies the RC failsafe fires (50 Hz stream stops), not MP interference.

**`RC_OPTIONS=32` is load-bearing.** This bit enables GCS RC override globally.
ELRS in MAVLink mode depends on it — clearing it would silence both MP joystick
AND the ELRS transmitter sticks simultaneously. Do not clear this bit.

---

### Summary: what needs to change on hardware before first flight

| # | Change | Parameter | From | To | Why |
|---|--------|-----------|------|----|-----|
| 1 | **Enable GCS failsafe** | `FS_GCS_ENABLE` | 0 | 2 | Currently disabled; simulation validated this is safe to enable |
| 2 | *(optional)* Keep GUIDED on GCS failsafe | `FS_OPTIONS` | 16 | 16 | Already set correctly — no change needed |
| 3 | *(optional)* Verify RTL altitude | `RTL_ALT` | 5000 (50 m) | — | Currently 50 m; appropriate for first flights |
| 4 | **Do not touch** | `RC_OPTIONS` | 32 | 32 | Load-bearing for ELRS MAVLink RC — clearing it removes stick control entirely |

Everything else in `arduconfig.param` — PID tuning, sensor calibrations, RC
channel mapping, OSD layout, motor PWM ranges — is **irrelevant to the simulation**
and must not be altered based on simulation results. Those values reflect real
hardware calibration done on the bench.
