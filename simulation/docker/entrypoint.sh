#!/bin/bash
set -e

VEHICLE="${VEHICLE:-ArduCopter}"
SPEEDUP="${SPEEDUP:-1}"

cd /home/ardupilot/ardupilot

PARAM_ARG=""
if [ -f /params/sitl_base.param ]; then
    PARAM_ARG="--add-param-file=/params/sitl_base.param"
fi

# Run SITL without MAVProxy so that GCS failsafe behaviour is governed by our
# test connections only — MAVProxy's own heartbeat would otherwise prevent the
# failsafe from triggering.
#
# Two independent TCP serial ports mirror the real hardware topology:
#   uartA / SERIAL0 → port 5760  (simulates ELRS MAVLink stream)
#   uartB / SERIAL1 → port 5761  (simulates LTE / RPi MAVLink stream)
#
# Extra sim_vehicle.py flags can be passed as CMD arguments via docker-compose.
# -A passes extra args directly to the arducopter binary (sim_vehicle.py's -- prefix goes to MAVProxy, not arducopter)
# uartA (SERIAL0) defaults to tcp:5760 in SITL; uartB (SERIAL1) must be set explicitly for the second MAVLink link
exec python3 Tools/autotest/sim_vehicle.py \
    -v "${VEHICLE}" \
    --no-mavproxy \
    --no-rebuild \
    --speedup "${SPEEDUP}" \
    ${PARAM_ARG} \
    -A "--serial1=tcp:5761" \
    "$@"
