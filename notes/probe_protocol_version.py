#!/usr/bin/env python3
"""Asks a Franka control unit which research interface version it speaks.

This is a read-only probe. It opens one TCP connection to the command port,
sends a single Connect request carrying a deliberately invalid version, reads
the reply, and closes. The robot answers "incompatible library version" and
includes its own version, which is the whole point.

It cannot move the robot. The only message it knows how to build is Connect;
there is no Move, no torque command, and no UDP socket is created, so even a
robot that accepted the handshake would receive no motion and stream nowhere.
The version sent is 0xFFFF, which is not a real protocol version, so the
handshake is guaranteed to be rejected and no control session is established.

Wire format, stable across protocol versions 3 to 10:

    CommandHeader   12 bytes   uint32 command (kConnect = 0)
                               uint32 command_id
                               uint32 size (total, including this header)
    Connect::Request 4 bytes   uint16 version
                               uint16 udp_port

    Connect::Response 3 bytes  uint8  status (0 success, 1 incompatible)
                               uint16 version   <- the robot's version

Usage:
    ./probe_protocol_version.py <hostname> [--port 1337] [--timeout 5]
"""

import argparse
import socket
import struct
import sys

COMMAND_PORT = 1337
COMMAND_CONNECT = 0
# Not a real protocol version, so the robot must reject it and tell us its own.
INVALID_VERSION = 0xFFFF

HEADER = struct.Struct("<III")
REQUEST_BODY = struct.Struct("<HH")
RESPONSE_BODY = struct.Struct("<BH")

STATUS = {0: "kSuccess", 1: "kIncompatibleLibraryVersion"}

# Which libfranka releases speak each version, for reporting.
SPEAKERS = {
    3: "0.7.x",
    4: "0.8.x",
    5: "0.9.x",
    6: "0.10.0 - 0.13.2",
    7: "0.13.3 - 0.13.6",
    8: "0.14.x",
    9: "0.15.0 - 0.20.x",
    10: "0.21.x",
}


def probe(hostname, port, timeout):
    request = HEADER.pack(
        COMMAND_CONNECT, 1, HEADER.size + REQUEST_BODY.size
    ) + REQUEST_BODY.pack(INVALID_VERSION, 0)

    print(f"  sending {len(request)} bytes to {hostname}:{port}: {request.hex(' ')}")
    with socket.create_connection((hostname, port), timeout=timeout) as sock:
        sock.sendall(request)
        reply = b""
        want = HEADER.size + RESPONSE_BODY.size
        while len(reply) < want:
            chunk = sock.recv(want - len(reply))
            if not chunk:
                break
            reply += chunk
    print(f"  received {len(reply)} bytes: {reply.hex(' ')}")
    if len(reply) < want:
        raise SystemExit(f"  short reply, expected {want} bytes")

    command, command_id, size = HEADER.unpack_from(reply, 0)
    status, version = RESPONSE_BODY.unpack_from(reply, HEADER.size)
    print(f"  header: command={command} command_id={command_id} size={size}")
    print(f"  status: {status} ({STATUS.get(status, 'unknown')})")
    print()
    print(f"  robot speaks research interface version {version}")
    print(f"  which is libfranka {SPEAKERS.get(version, 'unknown to this script')}")
    return version


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hostname", help="robot control unit IP or hostname")
    parser.add_argument("--port", type=int, default=COMMAND_PORT)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    try:
        probe(args.hostname, args.port, args.timeout)
    except OSError as error:
        raise SystemExit(f"  could not reach {args.hostname}:{args.port}: {error}")


if __name__ == "__main__":
    sys.exit(main())
