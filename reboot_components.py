#!/usr/bin/env python3
"""
Reboot Stringman components over ssh and wait for them to come back online.

For each component: if its websocket port (8765) isn't serving, try to ssh in
(user 'pi', key auth - see ~/.ssh/config) and issue `sudo reboot`. Then poll
until every component accepts TCP connections on 8765 again, or give up.

Components that already serve 8765 are left alone (no pointless reboot).
Components that are fully off-network (no ping, no ssh) can't be rebooted
remotely - they need a physical power cycle; the poll keeps waiting for them
in case they come back on their own.

No third-party dependencies; run with the system python3.

Usage:
  python3 reboot_components.py                          # IPs from config.json
  python3 reboot_components.py --ips 192.168.68.139 192.168.68.140 192.168.68.141
  python3 reboot_components.py --no-reboot              # just wait until all up
  python3 reboot_components.py --timeout 900 --interval 15
"""

import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

WS_PORT = 8765
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=5"]


def port_open(ip, port=WS_PORT, timeout=3.0):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def pings(ip):
    # -c 1: one packet, -t 2: 2s timeout (macOS/BSD ping flags)
    return subprocess.run(
        ["ping", "-c", "1", "-t", "2", ip],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0


def ssh_ok(ip, user):
    return subprocess.run(
        ["ssh", *SSH_OPTS, f"{user}@{ip}", "true"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0


def ssh_reboot(ip, user):
    return subprocess.run(
        ["ssh", *SSH_OPTS, f"{user}@{ip}", "sudo", "-n", "reboot"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0


def load_ips(args):
    ips = list(args.ips or [])
    if not ips and args.components and Path(args.components).exists():
        doc = json.loads(Path(args.components).read_text())
        ips = [c["ip"] for c in doc["components"]]
    if not ips:
        sys.exit("no IPs: pass --ips or have discover_components.py write config.json first")
    return ips


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--ips", nargs="+", help="component IPs (default: read config.json)")
    parser.add_argument("--components", default="config.json", help="JSON from discover_components.py")
    parser.add_argument("--user", default="pi", help="ssh user (default: pi)")
    parser.add_argument("--timeout", type=float, default=600, help="seconds to wait for all components (default: 600)")
    parser.add_argument("--interval", type=float, default=15, help="seconds between polls (default: 15)")
    parser.add_argument("--no-reboot", action="store_true", help="only poll, don't send reboot commands")
    args = parser.parse_args()

    ips = load_ips(args)

    # Phase 1: reboot whatever is reachable but not serving.
    if args.no_reboot:
        print("Skipping reboots (--no-reboot)")
    else:
        for ip in ips:
            if port_open(ip):
                print(f"[{ip}] already serving on {WS_PORT}, leaving it alone")
            elif ssh_ok(ip, args.user):
                ok = ssh_reboot(ip, args.user)
                print(f"[{ip}] reboot {'sent' if ok else 'FAILED (sudo -n reboot returned error)'}")
            elif pings(ip):
                print(f"[{ip}] pings but ssh unreachable - cannot reboot remotely")
            else:
                print(f"[{ip}] off-network (no ping) - needs a physical power cycle")

    # Phase 2: poll until every component serves 8765.
    deadline = time.monotonic() + args.timeout
    print(f"Waiting up to {args.timeout:.0f}s for all {len(ips)} component(s) on port {WS_PORT}...")
    while True:
        states = {ip: port_open(ip) for ip in ips}
        line = "  ".join(f"{ip.rsplit('.', 1)[-1]}:{'up' if up else 'DOWN'}" for ip, up in states.items())
        print(f"{time.strftime('%H:%M:%S')}  {line}", flush=True)
        if all(states.values()):
            print("All components online.")
            sys.exit(0)
        if time.monotonic() >= deadline:
            missing = [ip for ip, up in states.items() if not up]
            print(f"Timed out. Still down: {', '.join(missing)}", file=sys.stderr)
            sys.exit(1)
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
