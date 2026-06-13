-- JC02-1 Laser Rangefinder driver for ArduPilot
--
-- Wiring:
--   White  -> GND
--   LtBlue -> VCC (3.3V–3.8V)
--   Yellow -> FC UART Rx  (module Tx)
--   Black  -> FC UART Tx  (module Rx)
--   Red    -> 3.3V or GPIO (Power ON, keep HIGH)
--   Green  -> NC
--
-- ArduPilot parameters (current setup):
--   SERIAL_n_PROTOCOL = 28     (Scripting)
--   SERIAL_n_BAUD     = 9      (9600 bps)
--   RNGFND1_TYPE      = 10     (MAVLink - existing short-range sensor, 0-8m, DO NOT CHANGE)
--   RNGFND2_TYPE      = 36     (Lua - this script feeds RNGFND2)
--   RNGFND2_MIN_CM    = 500    (spec: Min Range >= 5m)
--   RNGFND2_MAX_CM    = 60000  (600m version) or 100000 (1000m version)
--   RNGFND2_ORIENT    = 25     (downward)
--   SCR_ENABLE        = 1

local BAUD_RATE     = 9600
local UPDATE_MS     = 50      -- polling interval
local LOG_INTERVAL  = 5000   -- ms between periodic status messages

-- Wire protocol constants
local HDR1, HDR2    = 0xAE, 0xA7
local WRP1, WRP2    = 0xBC, 0xBE
local MEAS_CMD_BYTE = 0x85    -- command word in measurement response

-- Commands
local CMD_CONTINUOUS = string.char(0xAE,0xA7,0x04,0x00,0x0E,0x12,0xBC,0xBE)
-- local CMD_STOP    = string.char(0xAE,0xA7,0x04,0x00,0x0F,0x13,0xBC,0xBE)

-- Packet layout (27 bytes total):
--  [1]    AE  header 1
--  [2]    A7  header 2
--  [3]    17  data-length = 23  (covers bytes 3..25)
--  [4]    00  address
--  [5]    85  command word
--  [6..7]     Elevation Angle  (int16 big-endian, unit 0.1 deg)
--  [8..9]     Straight Distance (int16 big-endian, unit 0.1 m)  <- we use this
--  [10..11]   Height           (int16 big-endian, unit 0.1 m)
--  [12..13]   Horizontal Distance (int16 big-endian, unit 0.1 m)
--  [14..23]   Reserved (0x00)
--  [24]   01  unit indicator (0x01 = metres)
--  [25]       checksum = sum(bytes[3..24]) mod 256
--  [26]   BC  wrap 1
--  [27]   BE  wrap 2

-- ── Serial port init ─────────────────────────────────────────────────────────
-- ── Rangefinder backend probe ─────────────────────────────────────────────────
-- RNGFND1_TYPE=10 (MAVLink, existing short-range sensor, 0-8m)
-- RNGFND2_TYPE=36 (Lua scripted, JC02, 5-100m)  <-- we feed this one
--
-- ArduPilot 4.1-4.4: rangefinder:handle_script_msg(dist_m)  feeds first TYPE=36
-- ArduPilot 4.5+:    rangefinder:get_backend(1)              index 1 = RNGFND2
local RNGFND_INSTANCE = 1   -- 0-based index: RNGFND2 = instance 1

local rngfnd_backend = nil
if not rangefinder then
    gcs:send_text(3, "JC02: ERROR - rangefinder global is nil!")
elseif type(rangefinder.handle_script_msg) == "function" then
    -- ArduPilot 4.1-4.4: singleton routes to first TYPE=36 backend automatically
    rngfnd_backend = rangefinder
    gcs:send_text(6, "JC02: rangefinder API v1 (singleton)")
elseif type(rangefinder.get_backend) == "function" then
    -- ArduPilot 4.5+: must specify backend index explicitly
    rngfnd_backend = rangefinder:get_backend(RNGFND_INSTANCE)
    if rngfnd_backend then
        gcs:send_text(6, "JC02: rangefinder API v2 (get_backend(" .. RNGFND_INSTANCE .. "))")
    else
        gcs:send_text(3, "JC02: ERROR - get_backend(" .. RNGFND_INSTANCE .. ") nil")
        gcs:send_text(3, "JC02: check RNGFND2_TYPE=36 is saved and FC rebooted")
    end
else
    gcs:send_text(3, "JC02: ERROR - no rangefinder scripting API in this build!")
end

local uart = serial:find_serial(0)
if not uart then
    gcs:send_text(3, "JC02: ERROR - no serial port with PROTOCOL=28 found!")
    gcs:send_text(3, "JC02: Set SERIALn_PROTOCOL=28 and SERIALn_BAUD=9")
    return  -- abort, no point continuing
end

uart:begin(BAUD_RATE)
uart:set_flow_control(0)

-- Report which serial index was found (find_serial(0) = first PROTOCOL=28 port)
gcs:send_text(6, "JC02: serial port found, opening at 9600 baud")
gcs:send_text(6, "JC02: if wrong port, check which SERIALn_PROTOCOL=28")

-- ── Parser state ─────────────────────────────────────────────────────────────
local STATE_H1, STATE_H2, STATE_BODY = 0, 1, 2
local state    = STATE_H1
local buf      = {}
local pkt_len  = 0

-- ── Counters for diagnostics ─────────────────────────────────────────────────
local cnt_bytes_rx    = 0   -- raw bytes received from UART
local cnt_pkt_ok      = 0   -- valid measurement packets decoded
local cnt_pkt_bad_ck  = 0   -- packets with wrong checksum
local cnt_pkt_fail    = 0   -- 8-byte fail/echo packets
local last_dist_m     = -1
local last_log_ms     = 0
local last_ok_ms      = 0   -- timestamp of last valid measurement

-- Raw hex dump of first 16 bytes (one-shot, helps diagnose wiring/baud issues)
local dump_buf        = {}
local dump_done       = false

local function send_continuous()
    for i = 1, #CMD_CONTINUOUS do
        uart:write(CMD_CONTINUOUS:byte(i))
    end
    gcs:send_text(6, "JC02: sent continuous-measurement command")
end

local started = false

local function update()
    local now_ms = millis()  -- uint32_t, supports arithmetic natively

    -- ── Startup: send command and confirm ────────────────────────────────────
    if not started then
        send_continuous()
        started = true
    end

    -- ── Drain UART buffer (capped to avoid exceeding Lua time limit) ─────────
    local n = uart:available()  -- uint32_t
    if n > 54 then n = 54 end   -- max 2 full packets (27 bytes each) per call
    cnt_bytes_rx = cnt_bytes_rx + n

    while n > 0 do
        local b = uart:read()  -- uint32_t, == comparisons with integers work fine
        n = n - 1

        -- One-shot raw dump of first 16 bytes to identify what is on this port
        if not dump_done then
            dump_buf[#dump_buf + 1] = string.format("%02X", b)
            if #dump_buf >= 16 then
                gcs:send_text(5, "JC02 raw[0..15]: " .. table.concat(dump_buf, " "))
                dump_done = true
                dump_buf  = {}
            end
        end

        if state == STATE_H1 then
            if b == HDR1 then
                buf   = { b }
                state = STATE_H2
            end

        elseif state == STATE_H2 then
            if b == HDR2 then
                buf[2] = b
                state  = STATE_BODY
            else
                state = STATE_H1
            end

        else -- STATE_BODY
            buf[#buf + 1] = b

            if #buf == 3 then
                -- buf[3] is the data_length byte; total = data_length + 2 hdr + 2 wrap
                pkt_len = b + 4
                -- Sanity check: only 8-byte (ack/fail) and 27-byte (measurement) are valid
                if pkt_len ~= 8 and pkt_len ~= 27 then
                    state   = STATE_H1
                    buf     = {}
                    pkt_len = 0
                end
            end

            if pkt_len > 0 and #buf == pkt_len then
                local last = pkt_len
                if buf[last] == WRP2 and buf[last - 1] == WRP1 then
                    if pkt_len == 27 and buf[5] == MEAS_CMD_BYTE then
                        -- Verify checksum
                        local cksum = 0
                        for k = 3, pkt_len - 3 do cksum = (cksum + buf[k]) % 256 end
                        if cksum == buf[pkt_len - 2] then
                            local raw = buf[8] * 256 + buf[9]  -- uint32_t arithmetic
                            if raw > 32767 then raw = raw - 65536 end
                            local dist_m = raw * 0.1  -- multiply by float forces float result
                            if dist_m >= 5.0 and dist_m <= 1000.0 then
                                if rngfnd_backend then
                                    rngfnd_backend:handle_script_msg(dist_m)
                                end
                                last_dist_m = dist_m
                                last_ok_ms  = now_ms
                                cnt_pkt_ok  = cnt_pkt_ok + 1
                            end
                        else
                            cnt_pkt_bad_ck = cnt_pkt_bad_ck + 1
                        end
                    else
                        -- 8-byte ACK or measurement-failed echo
                        cnt_pkt_fail = cnt_pkt_fail + 1
                    end
                end
                state   = STATE_H1
                buf     = {}
                pkt_len = 0
            end
        end
    end

    -- ── Keepalive: signal OutOfRangeLow when no valid measurement ────────────
    -- 4.99m < MIN_CM (5.0m) → status OutOfRangeLow → prearm_healthy() passes.
    -- AP_RANGEFINDER_LUA_TIMEOUT_MS = 500ms: if last_reading_ms is older than
    -- 500ms, the backend resets to NoData which blocks arming.  Must call
    -- handle_script_msg more frequently than that — 200ms gives safe margin.
    if rngfnd_backend and (now_ms - last_ok_ms) > 200 then
        rngfnd_backend:handle_script_msg(0.02)  -- 2cm = OutOfRangeLow
        last_ok_ms = now_ms
    end

    -- ── Periodic status report (every LOG_INTERVAL ms) ───────────────────────
    if now_ms - last_log_ms >= LOG_INTERVAL then
        last_log_ms = now_ms
        local dist_str = (last_dist_m >= 0) and (tostring(last_dist_m) .. "m") or "none"
        gcs:send_text(6, "JC02: bytes=" .. tostring(cnt_bytes_rx) ..
            " ok=" .. cnt_pkt_ok ..
            " fail=" .. cnt_pkt_fail ..
            " badck=" .. cnt_pkt_bad_ck ..
            " last=" .. dist_str)

        -- Warn if no bytes at all (wiring / PROTOCOL / BAUD problem)
        if cnt_bytes_rx == 0 then
            gcs:send_text(4, "JC02: WARNING - no bytes from sensor!")
            gcs:send_text(4, "JC02: check wiring, SERIALn_BAUD=9, Power-ON pin HIGH")
        end

        -- Warn if bytes arriving but no good packets (framing / power issue)
        if cnt_bytes_rx > 0 and cnt_pkt_ok == 0 and cnt_pkt_fail == 0 then
            gcs:send_text(4, "JC02: WARNING - bytes received but no valid packets!")
            gcs:send_text(4, "JC02: possible baud mismatch or partial startup")
        end

        -- Warn if only fail packets (sensor running but can't measure)
        if cnt_pkt_fail > 0 and cnt_pkt_ok == 0 then
            gcs:send_text(5, "JC02: sensor responding but all measurements failed")
            gcs:send_text(5, "JC02: target may be out of range or too close (<5m)")
        end
    end

    return update, UPDATE_MS
end

gcs:send_text(6, "JC02 rangefinder script started")
return update()
