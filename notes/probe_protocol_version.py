#!/usr/bin/env python3
"""Asks a Franka control unit which research interface versions it speaks.

Requires only Python 3.8 or newer from the standard library. No libfranka, no
build, no pip install, no dependencies at all.

This is a read-only probe. For each of the robot and the gripper it opens one
TCP connection, sends a single Connect request carrying a deliberately invalid
version, reads the reply and closes. The control unit answers "incompatible
library version" and includes its own version, which is the whole point.

It cannot move the robot. Connect is the only message it knows how to build;
there is no Move, Grasp, Homing or torque command anywhere in this file. It
creates no UDP socket, so even a control unit that accepted the handshake would
receive no motion and stream nowhere. The version it offers is 0xFFFF, which is
not a real protocol version, so the handshake is guaranteed to be rejected and
no control session is established.

The robot needs only to be powered on and reachable. Brakes may stay locked,
the FCI does not need to be active, and nobody needs to be near the robot.

Wire formats, measured from the libfranka-common headers. Both are packed with
no padding, little endian, and are stable across every generation panda-py
supports.

    robot, port 1337
      header    12 bytes  uint32 command (kConnect = 0), uint32 id, uint32 size
      request    4 bytes  uint16 version, uint16 udp_port
      response   3 bytes  uint8  status,  uint16 version   <- the robot's

    gripper, port 1338
      header    10 bytes  uint16 command (kConnect = 0), uint32 id, uint32 size
      request    4 bytes  uint16 version, uint16 udp_port
      response   4 bytes  uint16 status,  uint16 version   <- the gripper's

Usage:
    python3 probe_protocol_version.py <hostname>
    python3 probe_protocol_version.py <hostname> --robot-only
"""

import argparse
import socket
import struct
import sys

# Not a real protocol version, so the control unit must reject it and tell us
# its own. This is what keeps the probe incapable of opening a session.
INVALID_VERSION = 0xFFFF

COMMAND_CONNECT = 0

# Which libfranka releases speak each robot protocol version. Taken from the
# libfranka-common commit that each libfranka release pins.
ROBOT_SPEAKERS = {
    3: "0.7.x",
    4: "0.8.x",
    5: "0.9.x",
    6: "0.10.0 - 0.13.2",
    7: "0.13.3 - 0.13.6",
    8: "0.14.x",
    9: "0.15.0 - 0.20.x",
    10: "0.21.x",
}

# panda-py publishes a wheel per libfranka version. This maps what the robot
# says to the build to install.
PANDA_PY_WHEEL = {
    3: "0.7.1",
    4: "0.8.0",
    5: "0.9.2",
    6: "0.13.2",
    7: "0.13.6",
    8: "0.14.2",
    9: "0.17.0",
    10: "0.21.3",
}


class Endpoint:
    """One of the two services, which differ in their header and status width."""

    def __init__(self, name, port, header, response, expected_version):
        self.name = name
        self.port = port
        self.header = header
        self.response = response
        self.expected_version = expected_version

    def request_bytes(self):
        body = struct.Struct("<HH").pack(INVALID_VERSION, 0)
        size = self.header.size + len(body)
        return self.header.pack(COMMAND_CONNECT, 1, size) + body


ROBOT = Endpoint("robot", 1337, struct.Struct("<III"), struct.Struct("<BH"), None)
GRIPPER = Endpoint("gripper", 1338, struct.Struct("<HII"), struct.Struct("<HH"), 3)

STATUS = {0: "kSuccess", 1: "kIncompatibleLibraryVersion"}


def probe(endpoint, hostname, timeout):
    request = endpoint.request_bytes()
    want = endpoint.header.size + endpoint.response.size

    print(f"  {endpoint.name}: connecting to {hostname}:{endpoint.port}")
    print(f"  {endpoint.name}: sending {len(request)} bytes {request.hex(' ')}")
    try:
        with socket.create_connection(
            (hostname, endpoint.port), timeout=timeout
        ) as sock:
            sock.sendall(request)
            reply = b""
            while len(reply) < want:
                chunk = sock.recv(want - len(reply))
                if not chunk:
                    break
                reply += chunk
    except OSError as error:
        print(f"  {endpoint.name}: UNREACHABLE, {error}")
        return None

    print(f"  {endpoint.name}: received {len(reply)} bytes {reply.hex(' ')}")
    if len(reply) < want:
        print(f"  {endpoint.name}: SHORT REPLY, expected {want} bytes")
        return None

    command, command_id, size = endpoint.header.unpack_from(reply, 0)
    status, version = endpoint.response.unpack_from(reply, endpoint.header.size)
    print(
        f"  {endpoint.name}: header command={command} id={command_id} size={size}, "
        f"status={status} ({STATUS.get(status, 'unknown')})"
    )
    print(f"  {endpoint.name}: SPEAKS RESEARCH INTERFACE VERSION {version}")
    if status != 1:
        print(
            f"  {endpoint.name}: NOTE status was not kIncompatibleLibraryVersion, "
            "which is unexpected for an invalid version"
        )
    if endpoint.expected_version is not None:
        verdict = (
            "as expected" if version == endpoint.expected_version else "UNEXPECTED"
        )
        print(f"  {endpoint.name}: expected {endpoint.expected_version}, {verdict}")
    return version


def main():
    parser = argparse.ArgumentParser(
        description="Report the research interface versions a Franka control unit speaks."
    )
    parser.add_argument("hostname", help="control unit IP or hostname, e.g. 172.16.0.2")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--robot-only", action="store_true", help="skip the gripper on port 1338"
    )
    args = parser.parse_args()

    print(f"Probing {args.hostname}. Read-only: this sends only Connect requests.")
    print()
    robot_version = probe(ROBOT, args.hostname, args.timeout)
    print()
    gripper_version = None
    if not args.robot_only:
        gripper_version = probe(GRIPPER, args.hostname, args.timeout)
        print()

    print("Summary")
    print(f"  host                     {args.hostname}")
    print(f"  robot protocol version   {robot_version}")
    if robot_version is not None:
        print(
            f"  spoken by libfranka      {ROBOT_SPEAKERS.get(robot_version, 'unknown')}"
        )
        wheel = PANDA_PY_WHEEL.get(robot_version)
        if wheel:
            print(f"  panda-py wheel to use    the libfranka {wheel} build")
    if not args.robot_only:
        print(f"  gripper protocol version {gripper_version}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
