#!/usr/bin/env python3
"""
Discover Stringman components on the local subnet and write config.json.

Strategy: TCP-probe every host in the subnet on port 8765 (the component
websocket port), then briefly connect to each responding host's websocket to
confirm it is a cranebot component (its JSON updates contain known keys such
as nf_robot_v, grip_sensors, line_record, spoolN or video_ready) and
classify it:

  - gripper: updates contain "grip_sensors"
  - anchor:  updates contain "line_record", "last_raw_encoder", or "spoolN"

Note: connecting to a component's websocket starts its camera (this is how
the component is designed - the camera runs only while a client is
attached). The probe is brief and the camera stops when we disconnect.

Requires: `pip install websockets`

Usage:
  .venv/bin/python discover_components.py                      # scan local /24
  .venv/bin/python discover_components.py --subnet 192.168.68.0/24
  .venv/bin/python discover_components.py --ips 192.168.68.139 192.168.68.140 192.168.68.141
  .venv/bin/python discover_components.py -o config.json   # default output
"""

import argparse
import asyncio
import ipaddress
import json
import re
import socket
import sys
import time
from pathlib import Path

import websockets

WS_PORT = 8765
CONNECT_TIMEOUT_S = 1.5   # per-host TCP probe timeout
PROBE_ATTEMPTS = 3        # wifi Pi Zeros drop packets; retry before giving up
IDENTIFY_WINDOW_S = 4.0   # how long to read updates when classifying
SCAN_CONCURRENCY = 128


def local_subnet():
    """Best-effort /24 of this machine's primary IPv4 address."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 80))  # no traffic is actually sent
        ip = s.getsockname()[0]
    finally:
        s.close()
    return str(ipaddress.ip_network(f"{ip}/24", strict=False))


async def probe_port(ip):
    """Return ip if it accepts a TCP connection on WS_PORT, else None."""
    for attempt in range(PROBE_ATTEMPTS):
        try:
            _reader, writer = await asyncio.wait_for(
                asyncio.open_connection(str(ip), WS_PORT), timeout=CONNECT_TIMEOUT_S
            )
            writer.close()
            await writer.wait_closed()
            return str(ip)
        except (OSError, asyncio.TimeoutError):
            if attempt < PROBE_ATTEMPTS - 1:
                await asyncio.sleep(0.3)
    return None


async def identify(ip):
    """Connect to a candidate's websocket and classify the component.

    Returns dict(kind, nf_robot_v) or None if it doesn't look like a cranebot.
    Not every component type sends nf_robot_v (arpeggio anchors jump straight
    to spool telemetry), so any known cranebot update key counts as proof.
    """
    kind = None
    version = None
    is_cranebot = False
    try:
        async with websockets.connect(f"ws://{ip}:{WS_PORT}") as ws:
            deadline = time.monotonic() + IDENTIFY_WINDOW_S
            while time.monotonic() < deadline:
                try:
                    msg = await asyncio.wait_for(
                        ws.recv(), timeout=deadline - time.monotonic()
                    )
                except asyncio.TimeoutError:
                    break
                try:
                    update = json.loads(msg)
                except json.JSONDecodeError:
                    continue
                keys = set(update)
                if "nf_robot_v" in keys:
                    is_cranebot = True
                    version = update["nf_robot_v"]
                if "grip_sensors" in keys:
                    is_cranebot = True
                    kind = "gripper"
                if (
                    keys & {"line_record", "last_raw_encoder"}
                    or any(re.fullmatch(r"spool\d+", k) for k in keys)
                ):
                    is_cranebot = True
                    kind = kind or "anchor"
                if "video_ready" in keys:
                    is_cranebot = True  # kind-neutral: every camera emits this
                if is_cranebot and kind:
                    break
    except OSError:
        return None
    if not is_cranebot:
        return None
    return {"kind": kind or "unknown", "nf_robot_v": version}


async def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--subnet", help="CIDR to scan (default: local /24)")
    parser.add_argument("--ips", nargs="+", help="probe these IPs instead of scanning")
    parser.add_argument("-o", "--output", default="config.json", help="output JSON file (default: config.json)")
    args = parser.parse_args()

    if args.ips:
        hosts = args.ips
        subnet = None
    else:
        subnet = args.subnet or local_subnet()
        hosts = [str(ip) for ip in ipaddress.ip_network(subnet).hosts()]
        print(f"Scanning {len(hosts)} hosts in {subnet} on port {WS_PORT}...")

    sem = asyncio.Semaphore(SCAN_CONCURRENCY)

    async def bounded_probe(ip):
        async with sem:
            return await probe_port(ip)

    candidates = [ip for ip in await asyncio.gather(*(bounded_probe(h) for h in hosts)) if ip]
    print(f"{len(candidates)} host(s) with port {WS_PORT} open: {', '.join(candidates) or '-'}")

    components = []
    for ip in sorted(candidates, key=lambda a: tuple(int(o) for o in a.split("."))):
        info = await identify(ip)
        if info is None:
            print(f"[{ip}] not a cranebot component, skipping")
            continue
        print(f"[{ip}] {info['kind']} (nf_robot {info['nf_robot_v']})")
        components.append({"ip": ip, **info})

    # Assign labels: gripper first, then anchors by IP, then anything unknown.
    labelled = []
    anchor_n = 0
    cam_n = 0
    for c in sorted(components, key=lambda c: (c["kind"] == "unknown", c["kind"] != "gripper", c["ip"])):
        if c["kind"] == "gripper":
            label = "gripper"
        elif c["kind"] == "anchor":
            anchor_n += 1
            label = f"anchor{anchor_n}"
        else:
            cam_n += 1
            label = f"cam{cam_n}"
        labelled.append({"label": label, **c})

    doc = {
        "subnet": subnet,
        "discovered_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "components": labelled,
    }
    Path(args.output).write_text(json.dumps(doc, indent=2) + "\n")
    print(f"Wrote {len(labelled)} component(s) to {args.output}")
    if not labelled:
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
