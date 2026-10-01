"""Check the move_to_* fixes of #61 and #63 on a robot.

On panda-py 1.0.0 every move_to_* call segfaults (#61). This runs each of them
once with a build of the fix, in small, slow moves:

  1. move_to_start(speed_factor=0) must raise ValueError without moving (#63).
  2. move_to_start at speed factor 0.1, from at most 1 rad away in any joint.
  3. move_to_joint_position: joint 7 by +0.3 rad and back.
  4. move_to_pose: the end effector 5 cm down and back.

The motion runs in a child process, so that the parent can restore the robot
even if it crashes: FCI off, brakes locked, control released.

    python notes/check_move_fix.py <robot-ip> <desk-user> [password]
"""

import argparse
import faulthandler
import getpass
import signal
import subprocess
import sys

import numpy as np

import panda_py
from panda_py import constants

SPEED_FACTOR = 0.1
MAX_START_DISTANCE = 1.0  # rad


def step(name, call):
    print(f"\n  {name}", flush=True)
    result = call()
    print(f"    ok ({result})", flush=True)


def worker(hostname):
    faulthandler.enable()
    panda = panda_py.Panda(hostname)
    start = np.asarray(constants.JOINT_POSITION_START)
    distance = np.abs(np.asarray(panda.q) - start).max()
    if distance > MAX_START_DISTANCE:
        print(f"  ! the start pose is {distance:.2f} rad away in one joint; guide the robot closer")
        return 3

    def rejects_zero_speed():
        q_before = np.asarray(panda.q)
        try:
            panda.move_to_start(speed_factor=0)
        except ValueError as error:
            moved = np.abs(np.asarray(panda.q) - q_before).max()
            assert moved < 1e-3, f"moved {moved:.4f} rad"
            return f"ValueError: {error}"
        raise AssertionError("move_to_start(speed_factor=0) did not raise")

    step("move_to_start(speed_factor=0) raises without moving", rejects_zero_speed)
    step(
        f"move_to_start, speed factor {SPEED_FACTOR}",
        lambda: panda.move_to_start(speed_factor=SPEED_FACTOR),
    )
    wrist = start.copy()
    wrist[6] += 0.3
    step(
        "move_to_joint_position, joint 7 +0.3 rad",
        lambda: panda.move_to_joint_position(wrist, speed_factor=SPEED_FACTOR),
    )
    step(
        "move_to_joint_position, back",
        lambda: panda.move_to_joint_position(start, speed_factor=SPEED_FACTOR),
    )
    pose = panda.get_pose()
    lowered = pose.copy()
    lowered[2, 3] -= 0.05
    step("move_to_pose, 5 cm down", lambda: panda.move_to_pose(lowered, speed_factor=SPEED_FACTOR))
    step("move_to_pose, back", lambda: panda.move_to_pose(pose, speed_factor=SPEED_FACTOR))
    print("\n  All move_to_* calls returned.")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("hostname")
    parser.add_argument("username")
    parser.add_argument("password", nargs="?")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        return worker(args.hostname)
    password = args.password or getpass.getpass("  Desk password: ")

    print(f"panda-py {panda_py.__version__} from {panda_py.__file__}")
    desk = panda_py.Desk(args.hostname, args.username, password, platform="panda")
    print("  desk: logged in, control token acquired")
    status = 1
    try:
        print("\n  Unlocking the brakes WILL make the robot move, and this test then")
        print("  moves it: to the start pose, joint 7 by 0.3 rad, the hand 5 cm down.")
        print("  Keep the workspace clear and the user stop within reach.\n")
        if input("  Type YES to continue: ") != "YES":
            print("  aborted")
            return 1
        desk.unlock()
        print("  desk: brakes unlocked")
        desk.activate_fci()
        print("  desk: FCI activated")
        command = [sys.executable, __file__, args.hostname, args.username, "-", "--worker"]
        status = subprocess.run(command, check=False).returncode
        if status < 0:
            print(f"\n  ! crashed: {signal.Signals(-status).name}")
    finally:
        print("\n  restoring the previous state")
        for name, call in (
            ("FCI deactivated", desk.deactivate_fci),
            ("brakes locked", desk.lock),
            ("control released", desk.release_control),
        ):
            try:
                call()
                print(f"  desk: {name}")
            except Exception as error:  # pylint: disable=broad-except
                print(f"  desk: FAILED before '{name}': {error}")
    return status


if __name__ == "__main__":
    sys.exit(main())
