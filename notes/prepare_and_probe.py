#!/usr/bin/env python3
"""Prepares a freshly booted Franka robot, then reports the protocol versions.

Requires only Python 3.8 or newer from the standard library. No libfranka, no
pip install, no build.

A robot that has just booted has its brakes locked and the FCI switched off, so
the research interface port is not answering yet. This logs into the Desk, takes
a control token, unlocks the brakes, activates the FCI, and then runs the
read-only version probe from probe_protocol_version.py.

  ** Unlocking the brakes makes the robot move. **

The joints settle a little as the brakes release, and the arm becomes
back-drivable, so anything resting on it or against it can shift. Stand clear,
keep the emergency stop within reach, and confirm the prompt. Nothing here
commands a trajectory: there is no Move, no torque and no motion generator
anywhere in these two files. The motion is the brake release itself.

Afterwards it puts the robot back as it was found: FCI off, brakes locked and
the control token released. That happens even if the probe fails or is
interrupted. Releasing the token matters most, because one left held locks out
the next user until someone forces it from the Pilot.

Usage:
    python3 prepare_and_probe.py <hostname> <desk-user> <desk-password>

    --yes             skip the confirmation prompt
    --leave-prepared  leave it unlocked, FCI on and control held
    --platform        panda or fr3, otherwise both are tried
"""

import argparse
import base64
import getpass
import hashlib
import json
import os
import ssl
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_protocol_version import GRIPPER, ROBOT, ROBOT_SPEAKERS, PANDA_PY_WHEEL, probe

# The FER and the FR3 serve mutually exclusive endpoints for the brakes and
# answer 404 for the other's, so the right one can be discovered by trying.
BRAKE_ENDPOINTS = {
    "panda": {
        "lock": "/desk/api/robot/close-brakes",
        "unlock": "/desk/api/robot/open-brakes",
    },
    "fr3": {"lock": "/desk/api/joints/lock", "unlock": "/desk/api/joints/unlock"},
}


def encode_password(username, password):
    """The Desk wants sha256("<password>#<username>@franka") as a base64 digest list."""
    digest = hashlib.sha256(f"{password}#{username}@franka".encode("utf-8")).digest()
    joined = ",".join(str(b) for b in digest)
    return base64.encodebytes(joined.encode("utf-8")).decode("utf-8")


def multipart(fields):
    """Reproduce byte for byte what requests sends for files={"force": True}.

    panda-py posts the brake requests with requests' files= parameter, which
    emits multipart with a filename attribute even though this is not a file
    upload. The Desk accepts that form, so this matches it exactly rather than
    sending something tidier that may not be accepted.
    """
    boundary = "e6b7b1a6f9d94b0a8f7c2d3e4f5a6b7c"
    body = ""
    for name, value in fields.items():
        body += f"--{boundary}\r\n"
        body += (
            f'Content-Disposition: form-data; name="{name}"; filename="{name}"\r\n\r\n'
        )
        body += f"{value}\r\n"
    body += f"--{boundary}--\r\n"
    return body.encode("utf-8"), f"multipart/form-data; boundary={boundary}"


class Desk:
    """A minimal Desk client over urllib, enough to unlock and enable the FCI."""

    def __init__(self, hostname, username, password):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.cookie = None
        self.token = None
        self.legacy = False
        self.platform = None
        # The Desk uses a self-signed certificate.
        self.ssl_context = ssl.create_default_context()
        self.ssl_context.check_hostname = False
        self.ssl_context.verify_mode = ssl.CERT_NONE

    def request(self, method, path, payload=None, headers=None, form=None, check=True):
        url = f"https://{self.hostname}{path}"
        data, content_type = None, None
        if form is not None:
            data, content_type = multipart(form)
        elif payload is not None:
            data = json.dumps(payload).encode("utf-8")
            content_type = "application/json"

        request = urllib.request.Request(url, data=data, method=method.upper())
        if content_type:
            request.add_header("Content-Type", content_type)
        if self.cookie:
            request.add_header("Cookie", f"authorization={self.cookie}")
        for key, value in (headers or {}).items():
            request.add_header(key, value)

        try:
            with urllib.request.urlopen(
                request, timeout=20, context=self.ssl_context
            ) as response:
                return response.status, response.read().decode("utf-8", "replace")
        except urllib.error.HTTPError as error:
            text = error.read().decode("utf-8", "replace")
            if check:
                raise ConnectionError(
                    f"{method.upper()} {path} -> {error.code}: {text[:200]}"
                )
            return error.code, text

    def login(self):
        _, body = self.request(
            "post",
            "/admin/api/login",
            payload={
                "login": self.username,
                "password": encode_password(self.username, self.password),
            },
        )
        self.cookie = body.strip()
        print("  desk: login successful")

    def take_control(self):
        status, body = self.request("get", "/admin/api/control-token", check=False)
        if status == 404 or "File not found" in body:
            self.legacy = True
            print("  desk: legacy Desk, no control tokens needed")
            return
        active = json.loads(body).get("activeToken")
        if active is not None:
            raise SystemExit(
                f"  desk: {active.get('ownedBy')} already holds the control token. "
                "Release it in the Desk UI, or use panda-py's take_control(force=True), "
                "which needs a physical button press."
            )
        _, body = self.request(
            "post",
            "/admin/api/control-token/request",
            payload={"requestedBy": self.username},
        )
        self.token = json.loads(body)["token"]
        print("  desk: control token acquired")

    def unlock(self, requested_platform=None):
        headers = {"X-Control-Token": self.token} if self.token else None
        order = (
            [requested_platform]
            if requested_platform
            else ["fr3", "panda"]  # FR3 first; the FER line is discontinued.
        )
        for platform in order:
            path = BRAKE_ENDPOINTS[platform]["unlock"]
            status, body = self.request(
                "post", path, headers=headers, form={"force": True}, check=False
            )
            if status == 404 or "no handler accepted" in body.lower():
                print(f"  desk: {path} not served here, trying the other layout")
                continue
            if not 200 <= status < 300:
                raise ConnectionError(f"POST {path} -> {status}: {body[:200]}")
            self.platform = platform
            print(f"  desk: brakes unlocked via {path} (platform {platform})")
            return
        raise ConnectionError("  desk: neither brake endpoint exists on this Desk")

    def lock(self):
        headers = {"X-Control-Token": self.token} if self.token else None
        path = BRAKE_ENDPOINTS[self.platform]["lock"]
        self.request("post", path, headers=headers, form={"force": True})
        print(f"  desk: brakes locked via {path}")

    def activate_fci(self):
        if self.legacy:
            print("  desk: legacy Desk has no FCI activation endpoint, skipping")
            return
        self.request(
            "post", "/admin/api/control-token/fci", payload={"token": self.token}
        )
        print("  desk: FCI activated")

    def deactivate_fci(self):
        if self.legacy:
            return
        self.request(
            "delete", "/admin/api/control-token/fci", payload={"token": self.token}
        )
        print("  desk: FCI deactivated")

    def release_control(self):
        """Hand the control token back, so the next user is not locked out.

        Worth doing even more than re-locking the brakes: a token left held
        blocks everyone else until it is forced, which needs someone to press
        the circle button on the Pilot.
        """
        if self.legacy or not self.token:
            return
        status, body = self.request(
            "delete",
            "/admin/api/control-token",
            payload={"token": self.token},
            check=False,
        )
        if 200 <= status < 300:
            print("  desk: control released")
        elif "ControlTokenUnknown" in body:
            print("  desk: control was not held, nothing to release")
        else:
            print(f"  desk: WARNING could not release control, {status}: {body[:120]}")
        self.token = None

    def restore(self, unlocked_by_us):
        """Put the robot back the way it was found, best effort.

        Each step is independent so that one failure does not strand the rest,
        and the control token goes last because locking the brakes needs it.
        """
        print()
        print("  restoring the previous state")
        for label, action in (
            ("deactivate FCI", self.deactivate_fci),
            *((("lock brakes", self.lock),) if unlocked_by_us else ()),
            ("release control", self.release_control),
        ):
            try:
                action()
            except Exception as error:  # noqa: BLE001 - keep going regardless
                print(f"  desk: WARNING {label} failed: {error}")


def confirm(assume_yes):
    if assume_yes:
        return
    print()
    print("  Unlocking the brakes WILL make the robot move.")
    print(
        "  The joints settle as the brakes release and the arm becomes back-drivable."
    )
    print("  Stand clear and keep the emergency stop within reach.")
    print()
    if input("  Type YES to continue: ").strip() != "YES":
        raise SystemExit("  aborted, nothing was changed")


def main():
    parser = argparse.ArgumentParser(
        description="Prepare a robot and report its protocol versions."
    )
    parser.add_argument("hostname", help="control unit IP or hostname, e.g. 172.16.0.2")
    parser.add_argument("username", help="Desk username")
    parser.add_argument(
        "password", nargs="?", help="Desk password, prompted if omitted"
    )
    parser.add_argument("--platform", choices=["panda", "fr3"], default=None)
    parser.add_argument(
        "--yes", action="store_true", help="skip the confirmation prompt"
    )
    parser.add_argument(
        "--leave-prepared",
        action="store_true",
        help="leave the brakes unlocked, the FCI on and the control token held",
    )
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    password = args.password or getpass.getpass("  Desk password: ")

    print(f"Preparing {args.hostname}")
    desk = Desk(args.hostname, args.username, password)
    desk.login()
    desk.take_control()

    robot_version = gripper_version = None
    unlocked = False
    try:
        confirm(args.yes)
        desk.unlock(args.platform)
        unlocked = True
        desk.activate_fci()

        # The Desk may answer before the research interface port is actually
        # listening, so retry rather than reporting a false negative.
        host = args.hostname.split(":")[0]
        print()
        print(f"Probing {host}. Read-only: this sends only Connect requests.")
        for attempt in range(1, 7):
            print()
            robot_version = probe(ROBOT, host, args.timeout)
            if robot_version is not None:
                break
            if attempt < 6:
                print(f"  robot: not answering yet, retrying ({attempt}/5)")
                time.sleep(2.0)
        print()
        gripper_version = probe(GRIPPER, host, args.timeout)
    finally:
        # Restore even if the probe threw or was interrupted. Leaving the
        # control token held locks out the next user until someone forces it
        # from the Pilot, so this is not something to skip on the error path.
        if args.leave_prepared:
            print()
            print("  --leave-prepared: brakes stay unlocked, FCI on, control held")
        else:
            desk.restore(unlocked_by_us=unlocked)

    print()
    print("Summary")
    print(f"  host                     {args.hostname}")
    print(f"  desk platform            {desk.platform}  (legacy={desk.legacy})")
    print(f"  robot protocol version   {robot_version}")
    if robot_version is not None:
        print(
            f"  spoken by libfranka      {ROBOT_SPEAKERS.get(robot_version, 'unknown')}"
        )
        wheel = PANDA_PY_WHEEL.get(robot_version)
        if wheel:
            print(f"  panda-py wheel to use    the libfranka {wheel} build")
    print(f"  gripper protocol version {gripper_version}   (expected 3)")
    if args.leave_prepared:
        print()
        print("  The brakes are still unlocked, the FCI is active and control is held.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
