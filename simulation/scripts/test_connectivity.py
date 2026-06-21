"""
Tier 1 SITL — Test 1: Basic connectivity and parameter verification.

Checks:
  - Both TCP ports (ELRS and LTE) accept a MAVLink connection.
  - FC sends a valid heartbeat on each port independently.
  - Key failsafe parameters match the values in sitl_base.param.
"""
import pytest
from pymavlink import mavutil
import common


EXPECTED_PARAMS = {
    "FS_GCS_ENABLE":    2.0,
    "FS_GCS_TIMEOUT":   5.0,
    "FS_THR_ENABLE":    2.0,
    "MAV_GCS_SYSID":    255.0,   # renamed from SYSID_MYGCS in ArduPilot master 2024+
    "SERIAL1_PROTOCOL": 2.0,
}


@pytest.fixture(scope="module")
def elrs():
    conn = common.connect_elrs()
    yield conn
    conn.close()


@pytest.fixture(scope="module")
def lte():
    conn = common.connect_lte()
    yield conn
    conn.close()


def test_elrs_port_heartbeat(elrs):
    """ELRS port (5760) delivers a FC heartbeat."""
    hb = elrs.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
    assert hb is not None, "No HEARTBEAT received on ELRS port"
    assert hb.get_srcSystem() == elrs.target_system
    print(f"\n  FC type={hb.type}  autopilot={hb.autopilot}  sysid={hb.get_srcSystem()}")


def test_lte_port_heartbeat(lte):
    """LTE port (5761) delivers a FC heartbeat independently."""
    hb = lte.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
    assert hb is not None, "No HEARTBEAT received on LTE port"
    assert hb.get_srcSystem() == lte.target_system


def test_both_ports_same_sysid(elrs, lte):
    """Both ports report the same FC system ID (same autopilot, two links)."""
    assert elrs.target_system == lte.target_system, (
        f"ELRS sysid={elrs.target_system} != LTE sysid={lte.target_system}"
    )


@pytest.mark.parametrize("param_name,expected", list(EXPECTED_PARAMS.items()))
def test_param(elrs, param_name, expected):
    """Verify each critical failsafe parameter matches sitl_base.param."""
    value = common.get_param(elrs, param_name)
    assert value is not None, f"Parameter {param_name} not received"
    assert value == pytest.approx(expected), (
        f"{param_name}: expected {expected}, got {value}"
    )
    print(f"\n  {param_name} = {value}  ✓")


def test_initial_mode(elrs):
    """FC should be in a sane initial mode (STABILIZE or similar) at startup."""
    mode = common.get_fc_mode(elrs)
    assert mode != "UNKNOWN", "Could not determine flight mode"
    print(f"\n  Initial flight mode: {mode}")
