#!/usr/bin/env python3
"""
Capture one full-resolution frame from each Stringman camera.

How it works: each robot component (anchor / gripper) runs a websocket
server on port 8765. When a client connects, the component starts its
camera (rpicam-vid) and serves an mpegts stream on TCP port 8888 for as
long as the client stays connected. This script:

  1. connects to all given components, starting their cameras,
  2. waits for each to report "video_ready" in its JSON updates,
  3. grabs one frame from each stream at (approximately) the same time,
  4. saves them as JPEGs and disconnects (which stops the cameras).

IMPORTANT: stop stringman-headless before running this. Each component
accepts a single websocket client, and the video port accepts a single
TCP connection - both of which stringman-headless holds while running.

Requires: ffmpeg on PATH, and `pip install websockets`.

Usage:
  python capture_frames.py gripper=192.168.1.101 anchor0=192.168.1.102 anchor1=192.168.1.103
  python capture_frames.py 192.168.1.101 192.168.1.102 192.168.1.103 -o captures/
  python capture_frames.py --components config.json   # written by discover_components.py
"""

import argparse
import asyncio
import json
import subprocess
import sys
import time
from pathlib import Path

import websockets

WS_PORT = 8765
READY_TIMEOUT_S = 30  # time to wait for a component's camera to come up
GRAB_TIMEOUT_S = 30


async def wait_video_ready(ip):
    """Connect to a component and wait until its camera stream is up.

    Returns (websocket, port). The websocket must stay open while capturing;
    disconnecting stops the camera on the component.
    """
    ws = await websockets.connect(f"ws://{ip}:{WS_PORT}")
    deadline = time.monotonic() + READY_TIMEOUT_S
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            await ws.close()
            raise TimeoutError(f"[{ip}] timed out waiting for video_ready")
        try:
            msg = await asyncio.wait_for(ws.recv(), timeout=remaining)
        except asyncio.TimeoutError:
            await ws.close()
            raise TimeoutError(f"[{ip}] timed out waiting for video_ready")
        try:
            update = json.loads(msg)
        except json.JSONDecodeError:
            continue
        if "video_ready" in update:
            port = int(update["video_ready"][0])
            print(f"[{ip}] camera ready on port {port}")
            return ws, port


def grab_frame(ip, port, out_path):
    """Grab a single frame from tcp://<ip>:<port> with ffmpeg."""
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-fflags", "nobuffer", "-flags", "low_delay",
        "-i", f"tcp://{ip}:{port}",
        "-frames:v", "1",
        "-q:v", "1",
        str(out_path),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=GRAB_TIMEOUT_S)
    if result.returncode != 0:
        raise RuntimeError(f"[{ip}] ffmpeg failed:\n{result.stderr.strip()}")
    print(f"[{ip}] saved {out_path}")


async def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "cameras",
        nargs="*",
        help="One entry per camera: label=<ip> (or bare <ip>, labelled cam1, cam2, ...)",
    )
    parser.add_argument(
        "--components",
        help="JSON file written by discover_components.py; cameras are taken from it",
    )
    parser.add_argument("-o", "--out-dir", default="captures", help="output directory (default: captures)")
    args = parser.parse_args()

    cameras = []
    if args.components:
        doc = json.loads(Path(args.components).read_text())
        cameras.extend((c["label"], c["ip"]) for c in doc["components"])
    for i, entry in enumerate(args.cameras, start=1):
        if "=" in entry:
            label, ip = entry.split("=", 1)
        else:
            label, ip = f"cam{i}", entry
        cameras.append((label, ip))
    if not cameras:
        parser.error("provide cameras as label=<ip> arguments or via --components")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    # One timestamp per run: the grabs are near-simultaneous, so all files
    # share it and sort/group together.
    ts = time.strftime("%Y%m%d_%H%M%S")

    # Start all cameras first so the frame grabs happen close together in time.
    connections = {}  # label -> (ws, port)
    failures = []
    for label, ip in cameras:
        try:
            connections[label] = (ip, *await wait_video_ready(ip))
        except Exception as e:
            print(e, file=sys.stderr)
            failures.append(label)

    try:
        # Grab one frame from each ready camera concurrently.
        async def grab(label):
            ip, _ws, port = connections[label]
            await asyncio.to_thread(grab_frame, ip, port, out_dir / f"{ts}_{label}.jpg")

        results = await asyncio.gather(
            *(grab(label) for label in connections), return_exceptions=True
        )
        for label, result in zip(connections, results):
            if isinstance(result, Exception):
                print(result, file=sys.stderr)
                failures.append(label)
    finally:
        for label, (ip, ws, _port) in connections.items():
            await ws.close()

    if failures:
        print(f"Failed cameras: {', '.join(failures)}", file=sys.stderr)
        sys.exit(1)
    print("Done.")


if __name__ == "__main__":
    asyncio.run(main())
