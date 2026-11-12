-- gps_depth_gate.lua  (ArduSub 4.7.2, Navigator/BlueOS)
-- Uses GPS for EKF XY position only at the surface. EKF source set 1 (index 0) = no GPS, set 2 (index 1) = GPS.
-- Required params:
--   EK3_SRC1_POSXY=6 VELXY=6 POSZ=1 VELZ=0 YAW=1   (boot default, dive set, no GPS)
--   EK3_SRC2_POSXY=3 VELXY=6 POSZ=1 VELZ=0 YAW=1   (surface set, GPS)
--   EK3_SRC_OPTIONS=0, EK3_OPTIONS=1 (JammingExpected backstop), SCR_ENABLE=1 (reboot)
-- The vehicle boots in set 1 (no GPS), so if this script is missing or never switches, GPS is never fused.
-- If the scripting VM kills the script while set 2 is active, set 2 stays active until reboot
-- (pcall cannot catch the time-limit abort). EK3_OPTIONS=1 is the backstop for that case.

local DIVE_DEPTH_M, DIVE_HOLD_MS       = 0.5, 1000   -- leave GPS when deeper than this for this long
local SURFACE_DEPTH_M, SURFACE_HOLD_MS = 0.5, 5000   -- re-admit GPS only after this long shallow + good
local GPS_MIN_SATS, GPS_MAX_HDOP       = 6, 400      -- entry criteria (hdop is *100): >= 6 sats, HDOP <= 4
local KEEP_MIN_SATS, KEEP_MAX_HDOP     = 6, 400      -- same limits to stay on GPS (no flapping between them)
local GPS_STABLE_M                     = 25          -- entry: all fixes within this of the first one
local GPS_MAX_SPEED                    = 3.0         -- m/s
local GPS_MAX_STEP_M                   = 10          -- max jump between consecutive checks (100 ms)
local GPS_MAX_AGE_MS                   = 1000        -- GPS1_RATE_MS=200, so 1 s = 5 missed messages
local LOOP_MS                          = 100

local SET_UNDERWATER, SET_SURFACE = 0, 1             -- binding indices: 0 = EK3_SRC1_*, 1 = EK3_SRC2_*
local MAV_WARNING, MAV_INFO = 4, 6
local TAG = "GPSGate: "

local state, since_ms, anchor, last_loc = nil, nil, nil, nil

local function depth_m()
  local alt = baro:get_altitude()
  if alt == nil or alt ~= alt then return nil end
  return -alt
end

local function gps_ok(i, now, min_sats, max_hdop)
  return gps:status(i) >= gps.GPS_OK_FIX_3D
     and (now - gps:last_message_time_ms(i):toint()) <= GPS_MAX_AGE_MS
     and gps:num_sats(i) >= min_sats
     and gps:get_hdop(i) <= max_hdop
     and gps:ground_speed(i) <= GPS_MAX_SPEED
end

local function gps_good_and_stable(now)
  local i = gps:primary_sensor()
  if not gps_ok(i, now, GPS_MIN_SATS, GPS_MAX_HDOP) then return false end
  local loc = gps:location(i)
  if anchor == nil then anchor = loc; return true end
  return anchor:get_distance(loc) <= GPS_STABLE_M
end

local function gps_still_ok(now)
  local i = gps:primary_sensor()
  if not gps_ok(i, now, KEEP_MIN_SATS, KEEP_MAX_HDOP) then return false end
  local loc = gps:location(i)
  local ok = (last_loc == nil) or (last_loc:get_distance(loc) <= GPS_MAX_STEP_M)
  last_loc = loc
  return ok
end

local function go(new_state, d, why)
  if new_state == "UNDER" then
    ahrs:set_posvelyaw_source_set(SET_UNDERWATER)
    gcs:send_text(MAV_WARNING, string.format("%s%s d=%.2fm: GPS ignored (src set 1)", TAG, why or "", d or -1))
  else
    ahrs:set_posvelyaw_source_set(SET_SURFACE)
    gcs:send_text(MAV_INFO, string.format("%sd=%.2fm: GPS in use (src set 2)", TAG, d or -1))
  end
  state, since_ms, anchor, last_loc = new_state, nil, nil, nil
end

local function update()
  local now = millis():toint()
  local d = depth_m()
  if state == nil then go("UNDER", d, "boot"); return end
  if d == nil then
    if state ~= "UNDER" then go("UNDER", d, "no depth") end
    return
  end
  if state == "SURFACE" then
    if not gps_still_ok(now) then go("UNDER", d, "gps degraded"); return end
    if d > DIVE_DEPTH_M then
      since_ms = since_ms or now
      if now - since_ms >= DIVE_HOLD_MS then go("UNDER", d, "dive") end
    else
      since_ms = nil
    end
  else
    if d < SURFACE_DEPTH_M and gps_good_and_stable(now) then
      since_ms = since_ms or now
      if now - since_ms >= SURFACE_HOLD_MS then go("SURFACE", d) end
    else
      since_ms, anchor = nil, nil
    end
  end
end

local function protected_wrapper()
  local ok, err = pcall(update)
  if not ok then
    gcs:send_text(MAV_WARNING, TAG .. "error: " .. tostring(err))
    pcall(function() ahrs:set_posvelyaw_source_set(SET_UNDERWATER) end)
    state, since_ms, anchor, last_loc = "UNDER", nil, nil, nil
    return protected_wrapper, 1000
  end
  return protected_wrapper, LOOP_MS
end

gcs:send_text(MAV_INFO, TAG .. "loaded")
return protected_wrapper, 1000
