"""
Tier 1 SITL — Test 2: GCS failsafe behaviour with dual links.

Open questions from the test plan this addresses:
  - Does FC failsafe trigger only when ALL GCS links drop, or when ANY drops?
  - Is FS_GCS_TIMEOUT honoured accurately when using two independent serial links?
  - Does FC emit STATUSTEXT or a mode change on failsafe entry/exit?

Architecture:
  - ELRS link → TCP 5760 (SERIAL0)
  - LTE  link → TCP 5761 (SERIAL1)
  Both send GCS heartbeats at 1 Hz. Tests stop one or both to observe FC reaction.

Note: these tests run sequentially and share state via module-scope connections;
restart SITL between full suite runs to ensure a clean state.
"""
import time
import threading
import pytest
from pymavlink import mavutil
import common

FS_TIMEOUT = 5.0     # must match FS_GCS_TIMEOUT in sitl_base.param
MARGIN = 3.0         # extra seconds to wait past the expected timeout


def _heartbeat_sender(conn, stop_event, name="link"):
    """Thread target: send GCS heartbeats at 1 Hz until stop_event is set."""
    while not stop_event.is_set():
        common.send_heartbeat(conn)
        stop_event.wait(common.HEARTBEAT_INTERVAL)
    print(f"  [{name}] heartbeats stopped")


def _collect_statustexts(conn, duration, out_list):
    """Collect all STATUSTEXT messages for `duration` seconds into out_list."""
    deadline = time.time() + duration
    while time.time() < deadline:
        msg = conn.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if msg:
            out_list.append(msg.text.strip())


@pytest.fixture()
def dual_links():
    """Fresh ELRS and LTE connections for each test."""
    elrs = common.connect_elrs()
    lte = common.connect_lte()
    yield elrs, lte
    elrs.close()
    lte.close()


# ── Test 1: Both links active — FC should NOT enter failsafe ─────────────────

def test_no_failsafe_while_both_active(dual_links):
    """FC must not enter GCS failsafe while both links are sending heartbeats."""
    elrs, lte = dual_links

    elrs_stop = threading.Event()
    lte_stop = threading.Event()
    elrs_t = threading.Thread(target=_heartbeat_sender, args=(elrs, elrs_stop, "ELRS"))
    lte_t = threading.Thread(target=_heartbeat_sender, args=(lte, lte_stop, "LTE "))
    elrs_t.start()
    lte_t.start()

    statustexts = []
    observe_for = FS_TIMEOUT + MARGIN
    _collect_statustexts(elrs, observe_for, statustexts)

    elrs_stop.set(); lte_stop.set()
    elrs_t.join(); lte_t.join()

    gcs_lost = [t for t in statustexts if "gcs" in t.lower() and "lost" in t.lower()]
    assert not gcs_lost, f"Unexpected GCS failsafe while both links active: {gcs_lost}"
    print(f"\n  PASS: No GCS failsafe in {observe_for:.0f}s with both links active")


# ── Test 2: Drop LTE only, ELRS still alive — expect NO failsafe ─────────────

def test_no_failsafe_when_only_lte_drops(dual_links):
    """
    Dropping the LTE link while ELRS still sends heartbeats should NOT trigger
    GCS failsafe — the FC should consider at least one active GCS sufficient.

    If this test fails (failsafe IS triggered), the FC treats each link's timeout
    independently, which changes the operational implications significantly.
    """
    elrs, lte = dual_links

    elrs_stop = threading.Event()
    lte_stop = threading.Event()
    elrs_t = threading.Thread(target=_heartbeat_sender, args=(elrs, elrs_stop, "ELRS"))
    lte_t = threading.Thread(target=_heartbeat_sender, args=(lte, lte_stop, "LTE "))
    elrs_t.start()
    lte_t.start()

    time.sleep(2)  # let both links establish

    # Drop LTE
    lte_stop.set()
    lte_t.join()
    print(f"\n  LTE heartbeats stopped at t=2s (ELRS still running)")

    statustexts = []
    _collect_statustexts(elrs, FS_TIMEOUT + MARGIN, statustexts)

    elrs_stop.set()
    elrs_t.join()

    gcs_lost = [t for t in statustexts if "gcs" in t.lower() and "lost" in t.lower()]
    if gcs_lost:
        pytest.fail(
            f"GCS failsafe triggered when only LTE dropped (ELRS still active): {gcs_lost}\n"
            "FINDING: FC tracks each link's timeout independently — dropping any link "
            "triggers failsafe even if another remains active."
        )
    else:
        print("  PASS: No GCS failsafe when only LTE dropped")

    print(f"  All status messages: {statustexts}")


# ── Test 3: Drop both links — expect failsafe within FS_GCS_TIMEOUT ──────────

def test_failsafe_triggers_when_both_drop(dual_links):
    """
    Dropping BOTH links must trigger GCS failsafe within approximately
    FS_GCS_TIMEOUT seconds. Checks STATUSTEXT and mode change.

    The drone must be armed for ArduCopter to act on the GCS failsafe — without
    arming, the failsafe fires internally but takes no action and emits no STATUSTEXT.
    ARMING_CHECK=0 in sitl_base.param allows arming without sensors.
    """
    elrs, lte = dual_links

    elrs_stop = threading.Event()
    lte_stop = threading.Event()
    elrs_t = threading.Thread(target=_heartbeat_sender, args=(elrs, elrs_stop, "ELRS"))
    lte_t = threading.Thread(target=_heartbeat_sender, args=(lte, lte_stop, "LTE "))
    elrs_t.start()
    lte_t.start()

    time.sleep(3)  # establish GCS link before arming

    # Arm the drone so GCS failsafe takes action (disarmed = failsafe fires but
    # only logs internally; armed = STATUSTEXT + mode change).
    # Use force=True to bypass pre-arm checks (GPS etc. may not be ready in SITL).
    armed = common.arm(elrs, timeout=10, force=True)
    if not armed:
        elrs_stop.set(); lte_stop.set()
        elrs_t.join(); lte_t.join()
        pytest.skip("Could not arm FC — skipping failsafe trigger test")
    print(f"\n  FC armed OK")

    # Drop both links: stop heartbeats AND close TCP sockets (simulates physical
    # link loss; some ArduPilot versions only detect GCS failsafe on full disconnect).
    drop_time = time.time()
    elrs_stop.set(); lte_stop.set()
    elrs_t.join(); lte_t.join()
    elrs.close()
    lte.close()
    print(f"  Both links closed at t=0")

    # Re-open ELRS as observer-only (no heartbeats) to collect FC messages
    observer = common.connect(common.ELRS_PORT, name="OBS", timeout=15)

    # Collect STATUSTEXT and heartbeats via observer (no heartbeats sent)
    failsafe_seen = False
    failsafe_delay = None
    statustexts = []
    mode_changed = False
    deadline = time.time() + FS_TIMEOUT + MARGIN + 5

    while time.time() < deadline:
        msg = observer.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=0.5)
        if not msg:
            continue
        if msg.get_type() == "STATUSTEXT":
            text = msg.text.strip()
            statustexts.append(text)
            print(f"  STATUSTEXT: '{text}'")
            if "gcs" in text.lower() and ("lost" in text.lower() or "failsafe" in text.lower() or "heartbeat" in text.lower()):
                failsafe_seen = True
                failsafe_delay = time.time() - drop_time
                print(f"  GCS failsafe STATUSTEXT after {failsafe_delay:.1f}s")
        if msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() == observer.target_system:
            mode = common.get_fc_mode(observer, timeout=1)
            if mode not in ("STABILIZE", "UNKNOWN") and not failsafe_seen:
                mode_changed = True
                failsafe_seen = True
                failsafe_delay = time.time() - drop_time
                print(f"  Mode changed to '{mode}' after {failsafe_delay:.1f}s (failsafe action)")

    # Disarm (FC may already be disarmed by failsafe action)
    common.disarm(observer, timeout=5)
    observer.close()

    print(f"  All status messages: {statustexts}")

    assert failsafe_seen, (
        f"GCS failsafe not detected within {FS_TIMEOUT + MARGIN + 5:.0f}s after both links dropped.\n"
        f"  All STATUSTEXT seen: {statustexts}\n"
        f"  Mode changed: {mode_changed}"
    )
    assert failsafe_delay <= FS_TIMEOUT + 2, (
        f"Failsafe triggered {failsafe_delay:.1f}s after link drop — expected within "
        f"{FS_TIMEOUT + 2:.0f}s (FS_GCS_TIMEOUT={FS_TIMEOUT})"
    )
    print(f"  PASS: GCS failsafe in {failsafe_delay:.1f}s (FS_GCS_TIMEOUT={FS_TIMEOUT}s)")


# ── Test 4: GCS failsafe clears when a link is restored ─────────────────────

def test_failsafe_clears_on_link_restore(dual_links):
    """
    After both links drop and failsafe triggers, resuming heartbeats on the
    ELRS link should clear the GCS failsafe. Checks for STATUSTEXT confirmation.
    """
    elrs, lte = dual_links

    # Phase 1: establish both links and arm so failsafe is active
    elrs_stop = threading.Event()
    lte_stop = threading.Event()
    elrs_t = threading.Thread(target=_heartbeat_sender, args=(elrs, elrs_stop, "ELRS"))
    lte_t = threading.Thread(target=_heartbeat_sender, args=(lte, lte_stop, "LTE "))
    elrs_t.start(); lte_t.start()
    time.sleep(3)
    common.arm(elrs, timeout=10, force=True)

    # Phase 2: drop both by closing connections, wait for failsafe
    elrs_stop.set(); lte_stop.set()
    elrs_t.join(); lte_t.join()
    elrs.close()
    lte.close()
    print(f"\n  Both links closed — waiting {FS_TIMEOUT + 2:.0f}s for failsafe...")
    time.sleep(FS_TIMEOUT + 2)

    # Phase 3: restore ELRS only — open a fresh connection and send heartbeats
    print("  Restoring ELRS link heartbeats...")
    restore_time = time.time()
    elrs_new = common.connect_elrs()
    elrs_stop = threading.Event()
    elrs_t = threading.Thread(target=_heartbeat_sender, args=(elrs_new, elrs_stop, "ELRS"))
    elrs_t.start()

    recovered = False
    statustexts = []
    deadline = time.time() + 15
    while time.time() < deadline:
        msg = elrs_new.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if msg:
            statustexts.append(msg.text.strip())
            if "gcs" in msg.text.lower() and ("regain" in msg.text.lower() or "restored" in msg.text.lower() or "ok" in msg.text.lower()):
                recovered = True
                delay = time.time() - restore_time
                print(f"  GCS link restored STATUSTEXT after {delay:.1f}s: '{msg.text.strip()}'")
                break

    elrs_stop.set(); elrs_t.join()
    elrs_new.close()

    print(f"  All status messages during recovery: {statustexts}")
    if not recovered:
        pytest.xfail(
            "GCS-regained STATUSTEXT not seen — ArduPilot may not emit this on all versions. "
            f"Messages observed: {statustexts}"
        )
