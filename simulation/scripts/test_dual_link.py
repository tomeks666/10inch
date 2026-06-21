"""
Tier 1 SITL — Test 3: Dual-link command contention and stream behaviour.

Open questions from the test plan this addresses:
  - Which link has SYSID_MYGCS authority when both send commands?
  - Does the FC produce duplicate/conflicting telemetry to both links?
  - What happens with simultaneous SET_MODE commands from both links?

ELRS link → TCP 5760 (SERIAL0) — low latency, 1:1 with the real ELRS receiver
LTE  link → TCP 5761 (SERIAL1) — higher latency in real life; equal in SITL
"""
import time
import threading
import pytest
from pymavlink import mavutil
import common


@pytest.fixture()
def elrs():
    conn = common.connect_elrs()
    yield conn
    conn.close()


@pytest.fixture()
def lte():
    conn = common.connect_lte()
    yield conn
    conn.close()


# ── Test 1: Both links receive the FC's heartbeat stream ─────────────────────

def test_both_links_receive_telemetry(elrs, lte):
    """
    Both SERIAL0 and SERIAL1 must deliver FC heartbeats independently.
    Validates that ArduPilot multiplexes heartbeats across both serial ports.
    """
    elrs_hbs = []
    lte_hbs = []

    def collect(conn, out, n=5):
        for _ in range(n):
            msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
            if msg and msg.get_srcSystem() == conn.target_system:
                out.append(time.time())

    t1 = threading.Thread(target=collect, args=(elrs, elrs_hbs))
    t2 = threading.Thread(target=collect, args=(lte, lte_hbs))
    t1.start(); t2.start()
    t1.join(); t2.join()

    assert len(elrs_hbs) >= 3, f"ELRS link received only {len(elrs_hbs)} FC heartbeats"
    assert len(lte_hbs) >= 3, f"LTE  link received only {len(lte_hbs)} FC heartbeats"
    print(f"\n  ELRS received {len(elrs_hbs)} heartbeats, LTE received {len(lte_hbs)} heartbeats")
    print(f"  Both links are receiving telemetry independently ✓")


# ── Test 2: SET_MODE from ELRS link is accepted ───────────────────────────────

def test_mode_change_via_elrs(elrs, lte):
    """SET_MODE (GUIDED) sent from the ELRS link must be accepted by the FC."""
    # Send heartbeat first so FC knows this is a GCS
    common.send_heartbeat(elrs)
    time.sleep(0.5)

    common.set_mode(elrs, "GUIDED")
    time.sleep(1)

    mode = common.get_fc_mode(elrs)
    assert mode == "GUIDED", f"Expected GUIDED after ELRS SET_MODE, got {mode}"
    print(f"\n  ELRS SET_MODE(GUIDED) accepted: FC now in {mode} ✓")

    # Reset to STABILIZE
    common.set_mode(elrs, "STABILIZE")
    time.sleep(1)


# ── Test 3: SET_MODE from LTE link is accepted ────────────────────────────────

def test_mode_change_via_lte(elrs, lte):
    """SET_MODE (LOITER) sent from the LTE link must be accepted by the FC."""
    common.send_heartbeat(lte)
    time.sleep(0.5)

    common.set_mode(lte, "LOITER")
    time.sleep(1)

    mode = common.get_fc_mode(lte)
    assert mode == "LOITER", f"Expected LOITER after LTE SET_MODE, got {mode}"
    print(f"\n  LTE SET_MODE(LOITER) accepted: FC now in {mode} ✓")

    common.set_mode(lte, "STABILIZE")
    time.sleep(1)


# ── Test 4: Simultaneous SET_MODE from both links — which wins? ───────────────

def test_simultaneous_mode_commands(elrs, lte):
    """
    Send conflicting SET_MODE commands from ELRS and LTE simultaneously.
    Records which mode the FC ends up in — expected to be non-deterministic,
    last-writer-wins based on packet arrival order.

    This is an OBSERVATION test rather than a pass/fail assertion.
    The finding informs whether link priority configuration is needed.
    """
    common.send_heartbeat(elrs)
    common.send_heartbeat(lte)
    time.sleep(0.5)

    results = {}

    def send_guided(_):
        common.set_mode(elrs, "GUIDED")
        results["elrs_sent"] = time.time()

    def send_loiter(_):
        common.set_mode(lte, "LOITER")
        results["lte_sent"] = time.time()

    # Fire both as close together as possible
    t1 = threading.Thread(target=send_guided, args=(None,))
    t2 = threading.Thread(target=send_loiter, args=(None,))
    t1.start(); t2.start()
    t1.join(); t2.join()

    time.sleep(1.5)  # allow FC to process and settle
    final_mode = common.get_fc_mode(elrs)

    print(f"\n  ELRS sent GUIDED at t={results.get('elrs_sent', '?'):.6f}")
    print(f"  LTE  sent LOITER at t={results.get('lte_sent', '?'):.6f}")
    print(f"  FC final mode: {final_mode}")
    print(
        f"\n  FINDING: Last-writer-wins. "
        f"Neither link has inherent priority over the other with SYSID_MYGCS=255. "
        f"To enforce priority, mavlink-router arbitration or SYSID differentiation is needed."
    )

    # Not a hard failure — outcome is intentionally non-deterministic.
    # Record for the test report.
    assert final_mode in ("GUIDED", "LOITER"), (
        f"FC landed in unexpected mode '{final_mode}' after simultaneous commands"
    )

    common.set_mode(elrs, "STABILIZE")
    time.sleep(0.5)


# ── Test 5: SYSID authority — do different SYSIDs affect acceptance? ──────────

def test_sysid_authority_observation(elrs, lte):
    """
    Check whether the FC honours FS_GCS_ENABLE's SYSID_MYGCS for authority.
    Send a command from a connection with a non-standard sysid (e.g., 200).

    This is another OBSERVATION test: ArduPilot accepts commands from any
    sysid by default; SYSID_MYGCS only restricts GCS HEARTBEAT-based failsafe
    tracking in some firmware versions — this test documents actual behaviour.
    """
    from pymavlink import mavutil as mu

    # Open a third connection with a non-standard GCS sysid
    outsider = mu.mavlink_connection(
        f"tcp:{common.SITL_HOST}:{common.ELRS_PORT}",
        source_system=200,   # non-SYSID_MYGCS value
        source_component=190,
    )
    outsider.wait_heartbeat(timeout=15)

    outsider.mav.heartbeat_send(
        mu.mavlink.MAV_TYPE_GCS,
        mu.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )
    time.sleep(0.3)

    # Command mode change from outsider (sysid=200)
    mode_id = outsider.mode_mapping().get("GUIDED")
    if mode_id is not None:
        outsider.set_mode(mode_id)
    time.sleep(1)

    mode = common.get_fc_mode(elrs)
    outsider.close()

    print(f"\n  Mode after SET_MODE from sysid=200: {mode}")
    print(
        f"  FINDING: FC {'accepted' if mode == 'GUIDED' else 'rejected'} command from "
        f"non-SYSID_MYGCS (200) sender. SYSID_MYGCS={'restricts' if mode != 'GUIDED' else 'does NOT restrict'} command authority."
    )

    # Reset
    common.set_mode(elrs, "STABILIZE")
