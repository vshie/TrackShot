"""Runs TrackShot's fake BlueOS (mavlink2rest + ArduSub + MAVLink UDP) for trying the app without a ROV.

    .venv/bin/python tools/fake_autopilot.py              # mavlink2rest on 127.0.0.1:8088, UDP 14550
    .venv/bin/python tools/fake_autopilot.py --flavor 1.1 --skew 3600

Point TrackShot at it with vehicle IP "127.0.0.1:8088" (mavlink2rest transport) or the printed
UDP URL. Received VISION_POSITION_DELTA traffic is summarised once a second.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

from loguru import logger

from trackshot.testing.fake_autopilot import FLAVORS, FakeAutopilot


def summarise(messages: list[dict], interval_s: float, path: list[float]) -> None:
    deltas = [m for m in messages if m["type"] == "VISION_POSITION_DELTA"]
    for m in deltas:
        for i, d in enumerate(m.get("position_delta", [0.0, 0.0, 0.0])[:3]):
            path[i] += d
    if deltas:
        last = deltas[-1]
        dx, dy, dz = last["position_delta"][:3]
        vias = sorted({m["_via"] for m in deltas})
        logger.info(
            f"VISION_POSITION_DELTA {len(deltas) / interval_s:5.1f} Hz via {'/'.join(vias)} | "
            f"last d=({dx:+.3f}, {dy:+.3f}, {dz:+.3f}) m over {last['time_delta_usec'] / 1000:.0f} ms, "
            f"conf {last['confidence']:.0f} | summed FRD path ({path[0]:+.2f}, {path[1]:+.2f}, {path[2]:+.2f}) m")
    health = {m["name"]: m["value"] for m in messages if m["type"] == "NAMED_VALUE_FLOAT"}
    if health:
        logger.info("Health " + ", ".join(f"{k}={v:g}" for k, v in sorted(health.items())))
    others = sorted({m["type"] for m in messages} - {"VISION_POSITION_DELTA", "NAMED_VALUE_FLOAT"})
    if others:
        logger.info(f"Also received: {', '.join(others)}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="address to listen on (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8088, help="mavlink2rest HTTP/WS port (0 = any free port)")
    parser.add_argument("--udp-port", type=int, default=14550, help="MAVLink UDP port (0 = any free port)")
    parser.add_argument("--flavor", choices=FLAVORS, default="0.11",
                        help="mavlink2rest JSON dialect: 0.11 (BlueOS today) or 1.1")
    parser.add_argument("--armed", action="store_true", help="start armed (parameter writes are refused)")
    parser.add_argument("--hide-viso", action="store_true",
                        help="hide VISO_* parameters until VISO_TYPE=1 and a reboot, as ArduPilot does "
                             "(kill -USR1 <pid> simulates the reboot)")
    parser.add_argument("--skew", type=float, default=0.0,
                        help="seconds to offset the fake Pi's clock in mavlink2rest timestamps")
    args = parser.parse_args(argv)

    fake = FakeAutopilot(port=args.port, udp_port=args.udp_port, flavor=args.flavor,
                         clock_skew_s=args.skew, host=args.host, hide_viso_until_reboot=args.hide_viso)
    fake.vehicle.armed = args.armed
    # A plain `kill` should shut down as cleanly as Ctrl-C (the with-block stops the servers)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    # `kill -USR1 <pid>`: the autopilot reboots (boot-time parameters such as VISO_TYPE take effect)
    signal.signal(signal.SIGUSR1, lambda *_: (fake.reboot(), logger.info("Autopilot rebooted")))
    with fake:
        logger.info(f"Fake BlueOS running (mavlink2rest {args.flavor}{', armed' if args.armed else ''})")
        logger.info(f"  vehicle IP : {fake.vehicle_ip}")
        logger.info(f"  HTTP base  : {fake.http_base}")
        logger.info(f"  WebSocket  : {fake.ws_url}")
        logger.info(f"  UDP        : {fake.udp_url}")
        logger.info("Ctrl-C to stop")
        path = [0.0, 0.0, 0.0]
        interval = 1.0
        try:
            while True:
                time.sleep(interval)
                messages = fake.clear_received()
                summarise(messages, interval, path)
        except KeyboardInterrupt:
            logger.info("Stopping")
    return 0


if __name__ == "__main__":
    sys.exit(main())
