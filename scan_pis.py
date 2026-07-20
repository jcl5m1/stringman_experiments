#!/usr/bin/env python3
"""
Scan the local subnet for Raspberry Pi devices by MAC address.

Pings every host in the subnet to fill the ARP cache, then reads `arp -a`
and reports entries whose MAC starts with a known Raspberry Pi OUI:

  B8:27:EB  Raspberry Pi Foundation
  DC:A6:32  Raspberry Pi Trading Ltd
  E4:5F:01  Raspberry Pi Trading Ltd
  28:CD:C1  Raspberry Pi Trading Ltd
  2C:CF:67  Raspberry Pi Trading Ltd
  D8:3A:DD  Raspberry Pi Trading Ltd

Unlike discover_components.py (which probes websocket port 8765), this finds
Pis regardless of what software they run - including freshly imaged or
misconfigured boards. Caveat: a device that drops ping won't enter the ARP
cache, so silent hosts are missed.

No third-party dependencies; run with the system python3.

Usage:
  python3 scan_pis.py                          # scan local /24
  python3 scan_pis.py --subnet 192.168.68.0/24
  python3 scan_pis.py --ips 192.168.68.139 192.168.68.140
"""

import argparse
import ipaddress
import re
import socket
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

# IEEE-registered OUIs for Raspberry Pi Foundation / Raspberry Pi Trading Ltd.
PI_OUIS = ("b8:27:eb", "dc:a6:32", "e4:5f:01", "28:cd:c1", "2c:cf:67", "d8:3a:dd")

PING_WORKERS = 128
# macOS arp -a line: "host (192.168.68.1) at b8:27:eb:12:34:56 on en0 ifscope [ethernet]"
ARP_LINE = re.compile(r"^(\S+) \((\d+\.\d+\.\d+\.\d+)\) at ([0-9a-f:]+) ", re.IGNORECASE)


def local_subnet():
    """Best-effort /24 of this machine's primary IPv4 address."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.1.1", 80))  # no traffic is actually sent
        ip = s.getsockname()[0]
    finally:
        s.close()
    return str(ipaddress.ip_network(f"{ip}/24", strict=False))


def ping(ip):
    # -c 2: two packets (wifi Pi Zeros drop packets), -W 400: wait at most
    # 400ms per reply (macOS/BSD flags). Even an unanswered ping triggers an
    # ARP exchange, which is what actually fills the cache.
    subprocess.run(
        ["ping", "-c", "2", "-W", "400", str(ip)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def arp_table():
    """Parse `arp -a` into [(ip, mac, hostname)], skipping incomplete entries."""
    out = subprocess.run(["arp", "-a"], capture_output=True, text=True).stdout
    entries = []
    for line in out.splitlines():
        m = ARP_LINE.match(line)
        if m:
            host, ip, mac = m.groups()
            # macOS prints octets unpadded (e.g. 0:11:32); pad so OUI compare is safe
            mac = ":".join(o.zfill(2) for o in mac.split(":"))
            entries.append((ip, mac.lower(), "" if host == "?" else host))
    return entries


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--subnet", help="CIDR to scan (default: local /24)")
    parser.add_argument("--ips", nargs="+", help="ping these IPs instead of scanning a subnet")
    args = parser.parse_args()

    if args.ips:
        hosts = args.ips
        subnet = None
    else:
        subnet = args.subnet or local_subnet()
        hosts = [str(ip) for ip in ipaddress.ip_network(subnet).hosts()]
        print(f"Ping-sweeping {len(hosts)} hosts in {subnet} to fill the ARP cache...")

    with ThreadPoolExecutor(max_workers=PING_WORKERS) as pool:
        list(pool.map(ping, hosts))

    entries = arp_table()
    if subnet:
        net = ipaddress.ip_network(subnet)
        entries = [e for e in entries if ipaddress.ip_address(e[0]) in net]

    pis = sorted(
        (e for e in entries if e[1].startswith(PI_OUIS)),
        key=lambda e: tuple(int(o) for o in e[0].split(".")),
    )
    if not pis:
        print("No Raspberry Pi devices found.", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(pis)} Raspberry Pi device(s):")
    for ip, mac, host in pis:
        print(f"  {ip:<15}  {mac}  {host}")


if __name__ == "__main__":
    main()
