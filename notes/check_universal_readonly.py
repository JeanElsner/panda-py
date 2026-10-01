"""Read-only check of the universal libfranka on a protocol version 5 robot.

Connects with the fork's own C++ client, test/universal/read_state.cpp, run in
the panda-py-build:0.21.3-universal image with host networking, and checks:

  - it falls back from protocol 10 and connects in 5
  - the state decodes: joint positions in range, O_T_EE a valid transform
  - the model is the bundled FER description
  - O_ddP_O: libfranka 0.21.3's Model::gravity(state) takes the gravity vector
    from it. If the FER firmware leaves it at zero, gravity from the state is
    zero, and the fork needs a fallback below protocol 10.

Nothing moves: only FCI is activated and the brakes stay locked. Afterwards
FCI is deactivated and the control token released, also on failure.

    python notes/check_universal_readonly.py <robot-ip> <desk-user> [password]
        [--libfranka ~/dev/libfranka]
"""

import argparse
import getpass
import json
import pathlib
import subprocess
import sys

import numpy as np

import panda_py
from panda_py import constants

IMAGE = "panda-py-build:0.21.3-universal"
CLIENT = r"""
set -e
POCO=$(dirname $(find / -name "libPocoNet.so.80" -not -path "/proc/*" 2>/dev/null | head -1))
export LD_LIBRARY_PATH=$POCO:/usr/local/lib64
g++ -std=c++17 -O2 /t/read_state.cpp -I/usr/include/eigen3 -L/usr/local/lib64 -lfranka \
    -o /tmp/read_state
/tmp/read_state "$1"
"""


def read_state(hostname, libfranka):
    command = [
        "docker",
        "run",
        "--rm",
        "--network",
        "host",
        "-v",
        f"{libfranka / 'test' / 'universal'}:/t:ro",
        IMAGE,
        "bash",
        "-c",
        CLIENT,
        "read_state",
        hostname,
    ]
    result = subprocess.run(command, capture_output=True, text=True, timeout=120, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"read_state failed ({result.returncode}):\n{result.stderr[-2000:]}")
    return json.loads(result.stdout)


def report(state):
    q = np.array(state["q"])
    pose = np.array(state["O_T_EE"]).reshape(4, 4, order="F")
    lower, upper = constants.JOINT_LIMITS_LOWER, constants.JOINT_LIMITS_UPPER
    gravity = np.array(state["O_ddP_O"])
    checks = [
        ("connected in protocol 5", state["server_version"] == 5, state["server_version"]),
        (
            "joint positions within the FER limits",
            bool(np.all((q > np.array(lower) - 0.01) & (q < np.array(upper) + 0.01))),
            np.round(q, 4).tolist(),
        ),
        (
            "O_T_EE is a homogeneous transform",
            bool(
                np.allclose(pose[3], [0, 0, 0, 1])
                and np.allclose(pose[:3, :3] @ pose[:3, :3].T, np.eye(3), atol=1e-6)
            ),
            np.round(pose[:3, 3], 4).tolist(),
        ),
        ("bundled FER description loaded", 5000 < state["urdf_bytes"] < 7000, state["urdf_bytes"]),
        (
            "O_ddP_O holds a gravity vector",
            9.0 < np.linalg.norm(gravity) < 10.5,
            np.round(gravity, 4).tolist(),
        ),
    ]
    print("\nChecks")
    for name, ok, value in checks:
        print(f"  {'ok  ' if ok else 'FAIL'} {name}: {value}")
    floor = np.array(state["gravity_floor_mounted"])
    print(f"\n  gravity(state)           {np.round(state['gravity'], 3).tolist()} Nm")
    print(f"  gravity, floor mounted   {np.round(floor, 3).tolist()} Nm")
    print(f"  measured tau_J           {np.round(state['tau_J'], 3).tolist()} Nm")
    print(
        f"  velocity limits (upper)  {np.round(state['upper_velocity_limits'], 6).tolist()} rad/s"
    )
    return all(ok for _, ok, _ in checks)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("hostname")
    parser.add_argument("username")
    parser.add_argument("password", nargs="?")
    parser.add_argument(
        "--libfranka",
        default=pathlib.Path.home() / "dev" / "libfranka",
        type=pathlib.Path,
        help="checkout of the universal fork",
    )
    args = parser.parse_args()
    if not (args.libfranka / "test" / "universal" / "read_state.cpp").exists():
        sys.exit(f"no fork checkout at {args.libfranka}")
    password = args.password or getpass.getpass("  Desk password: ")

    desk = panda_py.Desk(args.hostname, args.username, password, platform="panda")
    print("  desk: logged in, control token acquired")
    try:
        desk.activate_fci()
        print("  desk: FCI activated, brakes locked")
        state = read_state(args.hostname, args.libfranka)
    finally:
        print("\n  restoring the previous state")
        for name, call in (
            ("FCI deactivated", desk.deactivate_fci),
            ("control released", desk.release_control),
        ):
            try:
                call()
                print(f"  desk: {name}")
            except Exception as error:  # pylint: disable=broad-except
                print(f"  desk: FAILED before '{name}': {error}")
    print("\n" + json.dumps(state, indent=2))
    return 0 if report(state) else 1


if __name__ == "__main__":
    sys.exit(main())
