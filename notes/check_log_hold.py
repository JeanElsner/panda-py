"""Hardware check for panda-py PR #65: a 60 s logged hold, then get_log() and stop_controller().

On 1.1.0 this hung: get_log() stalled the 1 kHz loop until the robot aborted,
and stop_controller() then deadlocked on the GIL. With the fix the hold should
survive the read and the stop should return.

Holds position only; the brake release is the only other motion. The robot is
always restored: FCI off, brakes locked, control released.

    .venv-gilfix/bin/python notes/check_log_hold.py <robot-ip> <desk-user>
"""

import argparse
import getpass
import signal
import subprocess
import sys
import time

import numpy as np

import panda_py
from panda_py import controllers


def worker(hostname):
    panda = panda_py.Panda(hostname)
    print(f"  panda-py {panda_py.__version__}, server version {panda.get_robot().server_version()}")
    panda.enable_logging(65000)
    controller = controllers.JointPosition()
    q0 = panda.q
    panda.start_controller(controller)
    print("  holding position for 60 s with logging", flush=True)
    with panda.create_context(frequency=100, max_runtime=60) as ctx:
        while ctx.ok():
            controller.set_control(q0, np.zeros(7))
    t0 = time.monotonic()
    log = panda.get_log()
    took = time.monotonic() - t0
    print(f"  get_log: {len(log['q'])} samples in {took * 1000:.0f} ms", flush=True)
    time.sleep(1.0)
    panda.raise_error()  # raises if the read stalled the loop into an abort
    print("    ok   control survived the read", flush=True)
    t0 = time.monotonic()
    panda.stop_controller()
    print(f"    ok   stop_controller returned after {time.monotonic() - t0:.2f} s", flush=True)
    rate = np.array(log["control_command_success_rate"]).ravel()
    print(f"    ok   success rate in the log: mean {rate.mean():.4f}, lowest {rate.min():.4f}")
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
    desk = panda_py.Desk(args.hostname, args.username, password, platform="panda")
    print("  desk: logged in, control token acquired")
    status = 1
    try:
        print("\n  Unlocking the brakes WILL make the robot move; it then holds position.")
        if input("  Type YES to continue: ") != "YES":
            return 1
        desk.unlock()
        desk.activate_fci()
        print("  desk: brakes unlocked, FCI activated")
        command = [sys.executable, __file__, args.hostname, args.username, "-", "--worker"]
        try:
            status = subprocess.run(command, check=False, timeout=120).returncode
        except subprocess.TimeoutExpired:
            print("\n  ! hung for 120 s: the bug is still there")
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
