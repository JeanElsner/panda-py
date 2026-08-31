#!/usr/bin/env python3
"""Reads the live robot state stream, choosing the layout from the robot's version.

Requires only Python 3.8 or newer from the standard library. No libfranka.

This is the proof that a single client can talk to any generation. It asks the
control unit which research interface version it speaks, picks the matching
RobotState layout, connects, and decodes the 1 kHz stream. Nothing about the
layout is hardcoded to one firmware: state_layouts.py carries all of them, and
every offset in it was checked against a C++ compiler reading the real
libfranka-common headers.

  ** It does not move the robot. **

The robot streams state to us; we send nothing back on the UDP socket. There is
no Move, no MotionGeneratorCommand and no torque anywhere in this file, and a
robot only moves in response to a Move command that is never sent. This is the
same thing libfranka's readOnce does.

The brakes may stay locked. The FCI does have to be active, since otherwise the
port is not listening, which prepare_and_probe.py arranges.

Usage:
    python3 read_state.py <hostname> [--seconds 3]
"""

import argparse
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_protocol_version import ROBOT, ROBOT_SPEAKERS, probe
from state_layouts import LAYOUTS

CONNECT_SUCCESS = 0


def connect(hostname, version, udp_port, timeout):
    """Open a real session by offering the version the robot actually speaks."""
    body = struct.Struct("<HH").pack(version, udp_port)
    request = ROBOT.header.pack(0, 1, ROBOT.header.size + len(body)) + body
    sock = socket.create_connection((hostname, ROBOT.port), timeout=timeout)
    sock.sendall(request)

    want = ROBOT.header.size + ROBOT.response.size
    reply = b""
    while len(reply) < want:
        chunk = sock.recv(want - len(reply))
        if not chunk:
            raise SystemExit("  connect: control unit closed the connection")
        reply += chunk
    status, reported = ROBOT.response.unpack_from(reply, ROBOT.header.size)
    if status != CONNECT_SUCCESS:
        raise SystemExit(
            f"  connect: rejected, status={status}, robot speaks {reported}"
        )
    print(f"  connect: accepted, session open on protocol {reported}")
    return sock


def unpack(layout, raw):
    fmt = "<" + "".join(f"{count}{code}" for _, code, count in layout)
    if len(raw) != struct.calcsize(fmt):
        raise ValueError(f"expected {struct.calcsize(fmt)} bytes, got {len(raw)}")
    values = struct.unpack(fmt, raw)
    state, index = {}, 0
    for name, _, count in layout:
        state[name] = (
            values[index] if count == 1 else list(values[index : index + count])
        )
        index += count
    return state


def sanity(state):
    """Cheap checks that catch a wrong layout, which would give plausible garbage."""
    problems = []
    q = state["q"]
    if len(q) != 7 or not all(-4.0 < value < 4.0 for value in q):
        problems.append(f"joint positions out of range: {q}")
    bottom = state["O_T_EE"][12:16]
    # Column major 4x4, so elements 3, 7, 11, 15 are the bottom row.
    homogeneous = [state["O_T_EE"][i] for i in (3, 7, 11, 15)]
    if abs(homogeneous[3] - 1.0) > 1e-3 or any(abs(v) > 1e-3 for v in homogeneous[:3]):
        problems.append(f"O_T_EE bottom row is not [0,0,0,1]: {homogeneous}")
    if not 0.0 <= state["control_command_success_rate"] <= 1.0001:
        problems.append(
            f"success rate out of range: {state['control_command_success_rate']}"
        )
    return problems, bottom


def run(hostname, version, seconds, timeout):
    """Decode the state stream for a robot known to speak `version`."""
    args = argparse.Namespace(
        hostname=hostname, seconds=seconds, timeout=timeout, version=version
    )
    return _read(args, version)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hostname")
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--version", type=int, default=None, help="skip discovery")
    args = parser.parse_args()

    version = args.version
    if version is None:
        print("Discovering the protocol version")
        version = probe(ROBOT, args.hostname, args.timeout)
        if version is None:
            raise SystemExit("  could not reach the control unit")
        print()
    return _read(args, version)


def _read(args, version):
    if version not in LAYOUTS:
        raise SystemExit(f"  no layout known for protocol version {version}")

    layout = LAYOUTS[version]
    size = struct.calcsize("<" + "".join(f"{c}{t}" for _, t, c in layout))
    print(f"Using the version {version} layout: {len(layout)} fields, {size} bytes")
    print(f"  spoken by libfranka {ROBOT_SPEAKERS.get(version, 'unknown')}")
    print()

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("", 0))
    udp.settimeout(args.timeout)
    udp_port = udp.getsockname()[1]
    print(f"Listening for state on UDP {udp_port}, sending nothing back")

    tcp = connect(args.hostname, version, udp_port, args.timeout)
    try:
        deadline = time.monotonic() + args.seconds
        count, first, last = 0, None, None
        while time.monotonic() < deadline:
            try:
                raw, _ = udp.recvfrom(4096)
            except socket.timeout:
                break
            if len(raw) != size:
                print(
                    f"  state: unexpected datagram of {len(raw)} bytes, wanted {size}"
                )
                continue
            last = unpack(layout, raw)
            first = first or last
            count += 1
        print()
        if not count:
            raise SystemExit("  no state received; is the FCI active?")

        problems, _ = sanity(last)
        print(f"Received {count} state packets in {args.seconds:g}s")
        print(f"  message_id   {first['message_id']} -> {last['message_id']}")
        print(f"  robot_mode   {last['robot_mode']}")
        print(f"  q            {[round(v, 4) for v in last['q']]}")
        print(f"  dq           {[round(v, 4) for v in last['dq']]}")
        print(
            f"  O_T_EE xyz   {[round(v, 4) for v in (last['O_T_EE'][12], last['O_T_EE'][13], last['O_T_EE'][14])]}"
        )
        print(f"  tau_J        {[round(v, 2) for v in last['tau_J']]}")
        print(f"  success rate {last['control_command_success_rate']:.3f}")
        print()
        if problems:
            print("  LAYOUT LOOKS WRONG:")
            for problem in problems:
                print(f"    {problem}")
            return 1
        print("  Sanity checks passed: joint positions in range, O_T_EE is a valid")
        print("  homogeneous transform, success rate in [0, 1].")
        print(
            f"  A version {version} robot was decoded with the version {version} layout,"
        )
        print("  selected at runtime from a table covering every generation.")
    finally:
        tcp.close()
        udp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
