--------------------------------------------------
--------------------------------------------------
---------- VTX LUA for IRC/TRAMP Protocol --------
------------- Based on SmartAudio.lua  -----------
---- (Craig Fitches / H. Wurzburg / P. Hall) -----

-----------------HARDWARE------------------
-- Any FC with a free UART and ArduPilot 4.1+
-- VTX with IRC Tramp protocol (TBS Unify Pro, etc.)

------------ Instructions --------------------
-- 1. Set SERIALx_PROTOCOL = 28 (Scripting) on the port wired to the VTX Tramp input.
--    No half-duplex option required (TX-only is fine for Tramp set commands).
-- 2. Set SCR_USER1 = desired startup power level index (1 = lowest level you define below).
-- 3. Set SCR_USER2 = scripting serial port index to use (0 = first scripting port, 1 = second, ...).
-- 4. Set an RC channel's RCx_OPTION = 300 (Scripting1) to control power in flight.
-- 5. Wire that UART's TX pin to the VTX Tramp data wire.
-- 6. Edit POWER_LEVELS below to match your VTX and desired power steps.

---@diagnostic disable: need-check-nil

-- ============================================================
-- USER CONFIGURATION — edit this section only
-- ============================================================

-- Define 4, 5, or 6 power levels. Each entry is {mw = <milliwatts>, label = "<display name>"}.
-- Common Tramp-capable VTX power values: 25, 100, 200, 400, 600, 800, 1000 mW.
-- The RC switch LOW position → level 1, HIGH position → last level.
local POWER_LEVELS = {
    { mw = 1,  label = "VTX 25mW"  },
    { mw = 2, label = "VTX 800mW" },
    { mw = 3, label = "VTX 1600mW" },
    { mw = 4, label = "VTX 2500mW" },
}
-- Add or remove rows above to get 4, 5, or 6 levels. Script enforces this range.

-- ============================================================

local NUM_LEVELS    = #POWER_LEVELS
local startup_pwr   = param:get('SCR_USER1')        -- 1-based level index at boot
local serial_idx    = param:get('SCR_USER2') or 0   -- which scripting serial port (0 = first)

local scripting_rc  = rc:find_channel_for_option(300)
local port          = serial:find_serial(math.floor(serial_idx))

local _current_level = -1  -- tracks last sent level to avoid redundant writes

-- ============================================================
-- IRC Tramp protocol helpers
-- ============================================================
-- Packet format (16 bytes, all offsets 0-based):
--   [0]     0x0F      sync byte
--   [1]     cmd       0x72='r' query, 0x50='P' set power, 0x46='F' set freq
--   [2]     val_lo    16-bit parameter, little-endian low byte
--   [3]     val_hi    16-bit parameter, high byte
--   [4..13] 0x00      padding (10 bytes)
--   [14]    crc       sum of bytes [1..13], truncated to uint8
--   [15]    0x00      terminator
--
-- In 1-based Lua the same positions are pkt[1]..pkt[16].

local TRAMP_CMD_POWER = 0x50  -- 'P'  set power (mW, little-endian)
local TRAMP_CMD_QUERY = 0x72  -- 'r'  read settings (no payload)

local function tramp_build_packet(cmd, value)
    local pkt = {}
    pkt[1]  = 0x0F                   -- sync
    pkt[2]  = cmd
    pkt[3]  = value & 0xFF           -- val_lo
    pkt[4]  = (value >> 8) & 0xFF    -- val_hi
    for i = 5, 14 do
        pkt[i] = 0x00                -- 10 bytes padding (bytes [4]..[13])
    end
    -- CRC = SUM of pkt[2..14]  (bytes [1]..[13] in 0-based = 13 bytes)
    local crc = 0
    for i = 2, 14 do
        crc = crc + pkt[i]
    end
    pkt[15] = crc & 0xFF             -- CRC at byte[14]
    pkt[16] = 0x00                   -- terminator at byte[15]
    return pkt
end

local function tramp_send(pkt)
    for i = 1, #pkt do
        port:write(pkt[i])
    end
end

-- ============================================================
-- RC → level mapping
-- ============================================================
-- norm_input() returns -1.0 (full low) .. +1.0 (full high).
-- Maps linearly onto level indices 1 .. NUM_LEVELS.

local function get_level_from_rc()
    local input = scripting_rc:norm_input()
    local idx   = math.floor((input + 1) / 2 * (NUM_LEVELS - 1) + 0.5) + 1
    return math.max(1, math.min(NUM_LEVELS, idx))
end

-- ============================================================
-- Power control
-- ============================================================

local function set_power(level)
    if level == _current_level then
        return
    end
    local entry = POWER_LEVELS[level]
    local pkt   = tramp_build_packet(TRAMP_CMD_POWER, entry.mw)
    tramp_send(pkt)
    gcs:send_text(4, "Tramp VTX: " .. entry.label)
    _current_level = level
end

-- ============================================================
-- Main loop (runs every 500 ms)
-- ============================================================

function update()
    set_power(get_level_from_rc())
    return update, 500
end

-- ============================================================
-- Initialization (runs once after 2 s delay)
-- ============================================================

function init()
    if not port then
        gcs:send_text(0, "Tramp VTX: No scripting serial port found (set SERIALx_PROTOCOL=28)")
        return
    end
    if not scripting_rc then
        gcs:send_text(0, "Tramp VTX: No RC channel with RCx_OPTION=300")
        return
    end
    if NUM_LEVELS < 4 or NUM_LEVELS > 6 then
        gcs:send_text(0, "Tramp VTX: POWER_LEVELS must have 4, 5, or 6 entries (has " .. NUM_LEVELS .. ")")
        return
    end

    port:begin(9600)  -- IRC Tramp baud rate

    -- Send a 'r' (read settings) query so the VTX registers the FC and starts
    -- accepting set commands. Many VTXes silently ignore 'P' without seeing this first.
    tramp_send(tramp_build_packet(TRAMP_CMD_QUERY, 0))

    -- Apply startup power level from SCR_USER1
    if startup_pwr then
        local lvl = math.floor(startup_pwr)
        if lvl >= 1 and lvl <= NUM_LEVELS then
            set_power(lvl)
            -- Seed _current_level from RC so first update() call does not instantly override
            _current_level = get_level_from_rc()
        else
            gcs:send_text(4, "Tramp VTX: SCR_USER1 out of range, defaulting to level 1")
            set_power(1)
        end
    end

    gcs:send_text(6, "Tramp VTX: ready, " .. NUM_LEVELS .. " levels configured")
    return update, 500
end

return init, 2000  -- wait 2 s before init (RC input settles, easier to see boot errors)
