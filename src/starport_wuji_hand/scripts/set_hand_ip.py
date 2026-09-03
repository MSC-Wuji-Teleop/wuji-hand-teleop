#!/usr/bin/env python3
"""Move a Wuji Hand 2 to a different static IP.

WHY A SCRIPT. The change is three SDK calls, and the third one reboots the hand onto an address
you can no longer reach if any of the three was wrong. Discovery is a UDP BROADCAST, so a host on
192.168.1.0/24 cannot see a hand that has just moved to 192.168.40.0/24 -- there is no "scan the
whole network" fallback, and recovering a hand with a bad address needs a factory reset or a
direct link from a host that happens to share its subnet. So this reads back what is there, makes
you look at both addresses, and refuses to do anything without --execute.

BEFORE YOU RUN IT, give the host NIC an address on the DESTINATION subnet as well as the current
one, so there is never a moment when the hand is unreachable:

    sudo ip addr add 192.168.40.50/24 dev <nic>     # alongside the existing 192.168.1.x
    ip -brief addr                                  # confirm both are up

Then:

    python scripts/set_hand_ip.py --ip 192.168.40.111              # dry run
    python scripts/set_hand_ip.py --ip 192.168.40.111 --execute

Afterwards the hand reboots. Re-scan to confirm it came back before removing the old host address:

    python scripts/set_hand_ip.py --list
"""

from __future__ import annotations

import argparse
import ipaddress
import sys
import time

from wuji_sdk import DeviceType, SdkManager


def _value(obj, name):
    """The SDK mixes plain properties, methods and futures; normalize all three."""
    value = getattr(obj, name)
    value = value() if callable(value) else value
    return value.get() if hasattr(value, "get") else value


def _scan(manager):
    return [d for d in manager.scan() if d.device_type == DeviceType.WujiHand2]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="", help="the new static address, e.g. 192.168.40.111")
    ap.add_argument("--serial", default="", help="which hand, when more than one answers the scan")
    ap.add_argument("--list", action="store_true", help="just report what is on the network and exit")
    ap.add_argument("--execute", action="store_true", help="actually write it; without this nothing changes")
    ap.add_argument("--settle", type=float, default=20.0, help="seconds to wait for the reboot before re-scanning")
    args = ap.parse_args(argv)

    manager = SdkManager.instance()
    found = _scan(manager)
    if not found:
        print(
            "no Wuji Hand 2 answered the scan. Discovery is a UDP broadcast, so the host NIC must "
            "already be on the hand's current subnet.",
            file=sys.stderr,
        )
        return 1

    print(f"{len(found)} hand(s) on the network:")
    for device in found:
        print(f"  {device.sn}  at {device.address}  ({device.transport_type})")
    if args.list:
        return 0

    if not args.ip:
        print("nothing to do without --ip (or --list)", file=sys.stderr)
        return 1
    try:
        new_ip = str(ipaddress.IPv4Address(args.ip))
    except ipaddress.AddressValueError as exc:
        print(f"--ip {args.ip!r} is not a valid IPv4 address: {exc}", file=sys.stderr)
        return 1

    if args.serial:
        found = [d for d in found if d.sn == args.serial]
        if not found:
            print(f"no hand with serial {args.serial}", file=sys.stderr)
            return 1
    elif len(found) > 1:
        print("more than one hand answered; name one with --serial", file=sys.stderr)
        return 1

    hand = manager.connect(sn=found[0].sn, device_name="set_ip")
    try:
        serial = str(_value(hand, "serial_number"))
        side = str(_value(hand, "handedness"))
        current = str(hand.ip().get())
        print(f"\nhand {serial} ({side})")
        print(f"  current IP: {current}")
        print(f"  new IP:     {new_ip}")
        if current == new_ip:
            print("already there; nothing to do.")
            return 0
        if not args.execute:
            print("\nDRY RUN -- nothing written. Re-run with --execute.")
            print(f"Make sure this host already has an address on {ipaddress.IPv4Address(new_ip)}'s subnet.")
            return 0

        hand.ip().set(new_ip)
        # Persist before rebooting: a set that was never saved comes back as the old address, and
        # the reboot would then look like it silently ignored the change.
        hand.save_params()
        print(f"  wrote and saved {new_ip}; rebooting ...")
        try:
            hand.reboot()
        except Exception as exc:  # the link drops as the device goes down -- that is the success path
            print(f"  (link dropped during reboot, as expected: {type(exc).__name__})")
    finally:
        try:
            hand.disconnect()
        except Exception:
            pass

    print(f"  waiting {args.settle:.0f}s for it to come back ...", flush=True)
    time.sleep(args.settle)
    back = _scan(SdkManager.instance())
    for device in back:
        if device.sn == serial:
            print(f"  {serial} is back at {device.address}")
            return 0 if new_ip in device.address else 1
    print(
        f"  {serial} did not answer a scan after the reboot. If this host has no address on "
        f"{new_ip}'s subnet, add one and re-run with --list before assuming the worst.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
