# AUTO yaw oscillation fix: field test, 2026-11-11

Vehicle: ArduSub 4.7.2 at 192.168.2.2. Navigation in these logs: **DVL** odometry sent as VISION_POSITION_DELTA (~5 Hz), no GPS underwater. (TrackShot's own log shows it was not running VIO during logs 17 and 19; its last VIO run ended at 14:07.)
Mission: lawnmower survey, ~9.7 m legs, ~1.9 m lane spacing, DO_CHANGE_SPEED 0.4 m/s, depths 4–8 m.
Logs: `00000017.BIN` (before), `00000019.BIN` (after; ~48 min, seven AUTO runs).

## Problem

In AUTO, the heading swung back and forth: commanded heading ±12–25° over 4–12 s, and 90° turns at every corner.
The yaw controller was not the cause; it followed its target within 120–220 ms and never saturated. The *target* was moving:
with `WP_YAW_BEHAVIOR = 4` (correct crosstrack error) ArduSub points the vehicle along the position controller's velocity target,
and at survey speed (0.4 m/s) the controller's small position corrections (~0.1 m × `PSC_NE_POS_P`) swing that direction by
±10–25°, allowed up to `XTRACK_ANG_LIM` = 45°.

## Change

| Parameter | Before | After |
|---|---|---|
| `WP_YAW_BEHAVIOR` | 4 | **0** (never change yaw: hold one heading and strafe the legs) |
| `PSC_NE_POS_P` | 2.0 | 1.5 |
| `ATC_INPUT_TC` | 0.1 | 0.2 |

Everything else was unchanged (diffed from the logs' parameter dumps).

## Result

| Metric (AUTO) | Before (log 17) | After (log 19, per run) |
|---|---|---|
| Commanded-heading wiggle, sd | 14.3° | 0.0–1.1° |
| Heading tracking error, p95 | 10.9° | 1.3–2.7° |
| Yaw rate, rms | 26.9 °/s | 1.2–4.1 °/s |
| Heading turns > 30° per run | 18 | 0 (one exception, see below) |
| Horizontal position error, median / p95 | 0.11 / 0.20 m | 0.06–0.08 / 0.12–0.27 m |
| Speed along legs (commanded 0.4 m/s) | ~0.27 m/s | ~0.26 m/s |
| DVL odometry into ArduSub | 4.8 Hz, confidence ~81 | 4.3–6.2 Hz, confidence 83–96 |

The oscillation is gone, and position holding is the same or slightly better. `WP_YAW_BEHAVIOR = 0` also suits photogrammetry:
every image keeps the same orientation. No further yaw tuning is needed.

## Other findings in log 19 (not caused by the change)

1. **A stray GPS fix caused an EKF heading reset in AUTO (2676.5 s).** The NMEA GPS (`GPS1_TYPE` 5) reported one 3D fix at
   8.5 m depth: 3 satellites, ~23 km from the site, 39 m/s. `EK3_SRC1_POSXY` is 3 (GPS), so the EKF reacted: position test
   ratio 172, then a heading reset. The heading target stepped from 174.7° to 140.2° and the vehicle swung to ~123°. The pilot took
   ALT_HOLD at 2683 s, and the EKF restarted relative aiding at 2691 s.
   *Fix:* set `EK3_SRC1_POSXY` to 6 (ExternalNav), as TrackShot's parameter check recommends, so the EKF ignores GPS while
   diving. Alternatively disable the GPS (`GPS1_TYPE` 0) if the GPS isn't needed.
2. **Compass 2 is unreliable.** MAG1 reads ~1,490 mG against MAG0's ~500 mG. Twice (290–419 s and 2692–2782 s) the EKF
   alternated between compasses every 10 s, and in 240–480 s it rejected the compass 70–100% of the time. Holding a fixed heading
   now depends on a good compass. *Fix:* recalibrate, or set `COMPASS_USE2` 0.
3. **The pilot-input failsafe disarmed the vehicle in AUTO (3011.5 s, "Lost manual control").** `FS_PILOT_INPUT` = 2 with a 3 s
   timeout, so an AUTO survey depends on Cockpit sending joystick input continuously. This was probably the end of the session; worth
   knowing if a topside hiccup should not end a survey.
4. **`VISO_DELAY_MS` is 10.** Set it to suit whichever odometry source is sending: the DVL's latency
   (DVL navigation), or TrackShot's suggested value (TrackShot VIO).
5. **Odometry reaches ArduSub at ~5 Hz** (delta span 0.19 s), the DVL's update rate. (An earlier version of this note
   attributed this to TrackShot; TrackShot was not running.) `VISO_ORIENT` is 0, matching the DVL's mounting: its angle
   deltas line up with the vehicle's gyro axes.
6. Minor:
   - Odometry dropouts of 1–2.6 s happened five times in AUTO; there was no visible effect.
   - Selecting AUTO while disarmed (770.8 s) marked all waypoints reached at once. This was harmless; the mission restarted
     normally at 809 s.
   - The vehicle travels at ~65% of the commanded speed, as it did before the change. The thrusters are not saturated. This is
     fine for imaging, and a separate item if faster surveys are wanted.

## Analysis scripts

Kept in the session scratchpad; not part of the app:
- `survey.py`: modes, messages, parameter changes
- `parmdiff.py`: parameter diff between two logs
- `analyze.py`: per-AUTO-run yaw and track metrics
- `checks*.py` and `gps.py`: compass, EKF, GPS and VIO checks
