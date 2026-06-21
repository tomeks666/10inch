"""Shared MAVLink helpers for SITL test scripts."""
import os
import time
from pymavlink import mavutil

SITL_HOST = os.environ.get("SITL_HOST", "localhost")
ELRS_PORT = int(os.environ.get("ELRS_PORT", "5760"))
LTE_PORT = int(os.environ.get("LTE_PORT", "5761"))

# MAVLink GCS identity used by all test connections
GCS_SYSID = 255
GCS_COMPID = 190
HEARTBEAT_INTERVAL = 1.0  # seconds


def connect(port, name="GCS", timeout=30):
    """Open a TCP MAVLink connection to SITL and wait for the first FC heartbeat."""
    conn = mavutil.mavlink_connection(
        f"tcp:{SITL_HOST}:{port}",
        source_system=GCS_SYSID,
        source_component=GCS_COMPID,
    )
    conn.wait_heartbeat(timeout=timeout)
    print(f"[{name}] connected on port {port} — FC sysid={conn.target_system}")
    return conn


def connect_elrs(timeout=30):
    return connect(ELRS_PORT, name="ELRS", timeout=timeout)


def connect_lte(timeout=30):
    return connect(LTE_PORT, name="LTE ", timeout=timeout)


def send_heartbeat(conn):
    """Send a single GCS heartbeat."""
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


def get_param(conn, name, timeout=10):
    """Fetch a single parameter from the FC, return its float value or None."""
    conn.param_fetch_one(name)
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.strip("\x00").strip() == name:
            return msg.param_value
    return None


def set_mode(conn, mode_name):
    """Command the FC to switch to the named flight mode."""
    mapping = conn.mode_mapping()
    if mode_name not in mapping:
        raise ValueError(f"Unknown mode '{mode_name}'. Available: {list(mapping)}")
    conn.set_mode(mapping[mode_name])


def get_fc_mode(conn, timeout=5):
    """Return the current flight mode string from the FC's next HEARTBEAT."""
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and msg.get_srcSystem() == conn.target_system:
            mode_id = msg.custom_mode
            for name, num in conn.mode_mapping().items():
                if num == mode_id:
                    return name
            return str(mode_id)
    return "UNKNOWN"


def wait_for_statustext(conn, keyword, timeout=20):
    """
    Block until a STATUSTEXT containing `keyword` (case-insensitive) arrives,
    or until timeout. Returns the message text or None.
    """
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
        if msg and keyword.lower() in msg.text.lower():
            return msg.text
    return None


def wait_for_mode(conn, mode_name, timeout=20):
    """Block until the FC reports the named flight mode, or until timeout."""
    start = time.time()
    while time.time() - start < timeout:
        if get_fc_mode(conn, timeout=2) == mode_name:
            return True
    return False


def arm(conn, timeout=10, force=False):
    """Send ARM command and wait for the FC to report armed state.

    Set force=True to bypass all pre-arm checks (equivalent to Mission Planner
    'Force Arm'). Required in SITL when ARMING_CHECK=0 is ignored.
    """
    param2 = 21196.0 if force else 0.0
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,      # confirmation
        1.0,    # param1: 1 = arm
        param2, # param2: 21196 = force
        0, 0, 0, 0, 0,
    )
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and msg.get_srcSystem() == conn.target_system:
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                return True
    return False


def disarm(conn, timeout=10):
    """Send DISARM command and wait for the FC to confirm."""
    conn.arducopter_disarm()
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg and msg.get_srcSystem() == conn.target_system:
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
    return False
