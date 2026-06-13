"""
JC02-1 Laser Rangefinder — live terminal monitor
Usage:  python jc02_monitor.py [PORT] [BAUD]
        python jc02_monitor.py COM23
        python jc02_monitor.py COM23 115200
Defaults: COM23, 9600 baud
"""

import serial
import sys
import time

# ── Config ────────────────────────────────────────────────────────────────────
PORT   = sys.argv[1] if len(sys.argv) > 1 else "COM23"
BAUD   = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

# ── Protocol constants ────────────────────────────────────────────────────────
HDR          = bytes([0xAE, 0xA7])
WRP          = bytes([0xBC, 0xBE])
CMD_CONT     = bytes([0xAE, 0xA7, 0x04, 0x00, 0x0E, 0x12, 0xBC, 0xBE])
CMD_STOP     = bytes([0xAE, 0xA7, 0x04, 0x00, 0x0F, 0x13, 0xBC, 0xBE])
MEAS_CMD     = 0x85

# ── Parser ────────────────────────────────────────────────────────────────────
STATE_H1, STATE_H2, STATE_BODY = 0, 1, 2

def parse_stream(port):
    state   = STATE_H1
    buf     = []
    pkt_len = 0

    while True:
        b = port.read(1)
        if not b:
            continue
        v = b[0]

        if state == STATE_H1:
            if v == 0xAE:
                buf   = [v]
                state = STATE_H2

        elif state == STATE_H2:
            if v == 0xA7:
                buf.append(v)
                state = STATE_BODY
            else:
                state = STATE_H1

        else:  # STATE_BODY
            buf.append(v)
            if len(buf) == 3:                 # data_length byte
                pkt_len = v + 4
                if pkt_len not in (8, 27):    # reject invalid length
                    state = STATE_H1
                    buf   = []
                    pkt_len = 0

            elif pkt_len > 0 and len(buf) == pkt_len:
                if buf[-2] == 0xBC and buf[-1] == 0xBE:
                    if pkt_len == 27 and buf[4] == MEAS_CMD:
                        # Packet layout (0-indexed):
                        #   [0..1]  AE A7 header
                        #   [2]     data_length
                        #   [3]     address
                        #   [4]     command (0x85)
                        #   [5..23] MMSG (19 bytes); dist at [7][8]
                        #   [24]    checksum = sum(buf[2..23]) mod 256
                        #   [25]    BC
                        #   [26]    BE
                        ck = sum(buf[2:pkt_len-3]) % 256   # sum indices 2..23
                        if ck == buf[pkt_len-3]:            # compare to buf[24]
                            raw = buf[7] * 256 + buf[8]
                            if raw > 32767:
                                raw -= 65536
                            dist_m = raw * 0.1
                            yield ("ok", dist_m, bytes(buf))
                        else:
                            yield ("badck", None, bytes(buf))
                    else:
                        yield ("fail", None, bytes(buf))   # 8-byte echo / measurement failed
                state   = STATE_H1
                buf     = []
                pkt_len = 0

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print(f"JC02 Monitor  —  {PORT} @ {BAUD} baud")
    print("Press Ctrl+C to exit\n")

    try:
        port = serial.Serial(PORT, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: cannot open {PORT}: {e}")
        sys.exit(1)

    time.sleep(0.2)
    port.write(CMD_CONT)
    print("Continuous measurement started...\n")

    cnt_ok    = 0
    cnt_fail  = 0
    cnt_badck = 0
    t_start   = time.time()
    last_dist = None
    last_raw  = None

    # Bar chart characters
    BAR_FULL  = "█"
    BAR_EMPTY = "░"
    BAR_MAX   = 40   # characters for max display distance
    DIST_MAX  = 20.0 # metres shown at full bar

    try:
        for kind, dist, raw_pkt in parse_stream(port):
            now     = time.time()
            elapsed = now - t_start
            last_raw = raw_pkt.hex(" ").upper()

            if kind == "ok":
                cnt_ok   += 1
                last_dist = dist
            elif kind == "fail":
                cnt_fail += 1
            else:
                cnt_badck += 1

            rate = cnt_ok / elapsed if elapsed > 0 else 0

            if last_dist is not None and last_dist >= 5.0:
                bar_len  = min(BAR_MAX, int(last_dist / DIST_MAX * BAR_MAX))
                bar      = BAR_FULL * bar_len + BAR_EMPTY * (BAR_MAX - bar_len)
                dist_str = f"{last_dist:6.1f} m"
            else:
                bar      = BAR_EMPTY * BAR_MAX
                dist_str = " NO DATA"

            stats    = (f"ok={cnt_ok}  fail={cnt_fail}  badck={cnt_badck}  "
                        f"rate={rate:.1f}Hz  t={elapsed:.0f}s")
            hex_line = f"last pkt: {last_raw}"

            # Three-line live display
            print(f"\r  {dist_str}  |{bar}| 0-{DIST_MAX:.0f}m    ", end="")
            print(f"\n  {stats:<60}", end="\033[F")
            # Uncomment below for raw hex on a third line (needs 3-line scroll guard):
            # print(f"\n\n  {hex_line}\033[2A", end="", flush=True)
            print(f"\n\n  {hex_line:<80}\033[2A", end="", flush=True)

    except KeyboardInterrupt:
        print("\n\n\nStopping...", flush=True)
        port.write(CMD_STOP)
        time.sleep(0.1)
        port.close()
        elapsed = time.time() - t_start
        print(f"\nSession: {elapsed:.1f}s  ok={cnt_ok}  fail={cnt_fail}  badck={cnt_badck}")
        if cnt_ok > 0:
            print(f"Average rate: {cnt_ok/elapsed:.1f} measurements/sec")

if __name__ == "__main__":
    main()
