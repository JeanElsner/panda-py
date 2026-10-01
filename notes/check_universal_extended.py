"""Extended hardware check of the universal libfranka on a protocol 5 robot (FER).

check_move_fix.py covered panda-py's torque controllers. This covers the rest:

  1. move_to_start, to begin from a known pose.
  2. libfranka's joint position motion generator, rate limited with the FER
     limits: joint 7 out by 0.1 rad and back in 4 s.
  3. libfranka's Cartesian velocity motion generator: the hand about 2.5 cm
     down and back up in 4 s.
  4. StopMove: a motion whose callback raises after 0.5 s. libfranka must
     cancel it on the robot, hand back the exception, and leave the robot
     ready for the next motion.
  5. 60 s holding position under torque control, with the control command
     success rate logged throughout.
  6. The user stop (skip with --skip-user-stop): joint 7 oscillates slowly
     until you press it. libfranka must report the stop, and after you release
     it, automatic error recovery must bring the robot back.

Small, slow moves only. Keep the workspace clear and the user stop within
reach. The first unexpected failure ends the run, and the robot is always
restored: FCI off, brakes locked, control released.

    .venv-universal/bin/python notes/check_universal_extended.py <robot-ip> <desk-user>
"""

import argparse
import getpass
import signal
import subprocess
import sys
import time

import numpy as np

import panda_py
from panda_py import controllers, libfranka

SPEED_FACTOR = 0.1


def ok(message):
    print(f"    ok   {message}", flush=True)


def step(title):
    print(f"\n  {title}", flush=True)


def joint_seven_out_and_back(robot, amplitude, period, repeat=1):
    """Joint 7 by `amplitude` and back, with zero velocity at both ends."""
    q0 = np.array(robot.read_once().q)
    elapsed = [0.0]
    worst = {"tracking": 0.0, "success": 1.0}

    def callback(state, duration):
        elapsed[0] += duration.to_sec()
        t = elapsed[0]
        q = q0.copy()
        q[6] += amplitude * (1 - np.cos(2 * np.pi * min(t, period * repeat) / period)) / 2
        if t > 0.1:
            worst["tracking"] = max(worst["tracking"], abs(state.q[6] - state.q_d[6]))
            worst["success"] = min(worst["success"], state.control_command_success_rate)
        command = libfranka.JointPositions(q.tolist())
        return libfranka.motion_finished(command) if t >= period * repeat else command

    robot.control_joint_position(callback)
    return q0, np.array(robot.read_once().q), worst


def hand_down_and_up(robot, speed=0.02, period=4.0):
    """vz = -speed sin(2 pi t / T): down about speed T / pi and back to the start."""
    start = np.array(robot.read_once().O_T_EE).reshape(4, 4, order="F")[:3, 3]
    elapsed = [0.0]

    def callback(state, duration):
        elapsed[0] += duration.to_sec()
        t = min(elapsed[0], period)
        command = libfranka.CartesianVelocities(
            [0.0, 0.0, -speed * np.sin(2 * np.pi * t / period), 0.0, 0.0, 0.0]
        )
        return libfranka.motion_finished(command) if elapsed[0] >= period else command

    robot.control_cartesian_velocity(callback)
    end = np.array(robot.read_once().O_T_EE).reshape(4, 4, order="F")[:3, 3]
    return start, end


class Abort(Exception):
    pass


def cancelled_motion(robot):
    """A hold whose callback raises after 0.5 s, so libfranka cancels the motion."""
    q0 = robot.read_once().q
    elapsed = [0.0]

    def callback(state, duration):
        elapsed[0] += duration.to_sec()
        if elapsed[0] > 0.5:
            raise Abort("deliberate abort from the motion callback")
        return libfranka.JointPositions(q0)

    try:
        robot.control_joint_position(callback)
    except Abort as error:
        return str(error)
    raise AssertionError("the motion was not interrupted")


def hold(panda, seconds=60.0):
    panda.enable_logging(int(seconds * 1000) + 5000)
    controller = controllers.JointPosition()
    q0 = panda.q
    try:
        panda.start_controller(controller)
        with panda.create_context(frequency=100, max_runtime=seconds) as ctx:
            while ctx.ok():
                controller.set_control(q0, np.zeros(7))
        log = panda.get_log()
    finally:
        panda.stop_controller()
        panda.disable_logging()
    rate = np.array(log["control_command_success_rate"]).ravel()
    drift = np.abs(np.array(panda.q) - q0).max()
    return rate, drift


def user_stop(robot):
    print("    Joint 7 now oscillates slowly. Press the user stop when it moves.", flush=True)
    try:
        joint_seven_out_and_back(robot, amplitude=0.2, period=6.0, repeat=5)
    except Exception as error:  # pylint: disable=broad-except
        reported = f"{type(error).__name__}: {error}"
    else:
        return None, None
    mode = robot.read_once().robot_mode
    input("    Release the user stop, then press Enter: ")
    robot.automatic_error_recovery()
    return reported, (mode, robot.read_once().robot_mode)


def worker(hostname, skip_user_stop):
    panda = panda_py.Panda(hostname)
    robot = panda.get_robot()
    print(f"  panda-py {panda_py.__version__}, server version {robot.server_version()}")

    step("1. move_to_start")
    assert panda.move_to_start(speed_factor=SPEED_FACTOR), "did not reach the start pose"
    ok("start pose reached")

    step("2. joint position motion generator, joint 7 out 0.1 rad and back")
    q0, q1, worst = joint_seven_out_and_back(robot, amplitude=0.1, period=4.0)
    ok(f"back within {np.abs(q1 - q0).max() * 1000:.2f} mrad of the start")
    ok(
        f"worst tracking error {worst['tracking'] * 1000:.2f} mrad, lowest success rate "
        f"{worst['success']:.3f}"
    )

    step("3. Cartesian velocity motion generator, hand down ~2.5 cm and back")
    start, end = hand_down_and_up(robot)
    ok(f"back within {np.linalg.norm(end - start) * 1000:.2f} mm of the start")

    step("4. StopMove: cancelling a motion from its callback")
    message = cancelled_motion(robot)
    ok(f"the callback's exception came back: {message}")
    mode = robot.read_once().robot_mode
    ok(f"robot mode afterwards: {mode}")
    assert panda.move_to_start(speed_factor=SPEED_FACTOR), "no motion possible after the cancel"
    ok("next motion works")

    step("5. holding position under torque control for 60 s")
    rate, drift = hold(panda)
    ok(
        f"control command success rate: mean {rate.mean():.4f}, lowest {rate.min():.4f} "
        f"over {len(rate)} samples"
    )
    ok(f"drift from the held pose {drift * 1000:.2f} mrad")

    if skip_user_stop:
        print("\n  6. user stop: skipped")
    else:
        step("6. user stop")
        reported, modes = user_stop(robot)
        if reported is None:
            print("    the oscillation ran out without a stop; nothing to check")
        else:
            ok(f"reported: {reported}")
            ok(f"robot mode when stopped {modes[0]}, after recovery {modes[1]}")
            assert panda.move_to_start(speed_factor=SPEED_FACTOR), "no motion after recovery"
            ok("motion works again after recovery")
    print("\n  All steps passed.")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("hostname")
    parser.add_argument("username")
    parser.add_argument("password", nargs="?")
    parser.add_argument("--skip-user-stop", action="store_true")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        return worker(args.hostname, args.skip_user_stop)
    password = args.password or getpass.getpass("  Desk password: ")

    print(f"panda-py {panda_py.__version__} from {panda_py.__file__}")
    desk = panda_py.Desk(args.hostname, args.username, password, platform="panda")
    print("  desk: logged in, control token acquired")
    status = 1
    try:
        print("\n  Unlocking the brakes WILL make the robot move, and this test then moves")
        print("  it slowly: joint 7 by 0.1 rad, the hand 2.5 cm, a 60 s hold, and joint 7")
        print("  oscillating until you press the user stop. Keep the workspace clear and")
        print("  the user stop within reach.\n")
        if input("  Type YES to continue: ") != "YES":
            print("  aborted")
            return 1
        desk.unlock()
        print("  desk: brakes unlocked")
        desk.activate_fci()
        print("  desk: FCI activated")
        command = [sys.executable, __file__, args.hostname, args.username, "-", "--worker"]
        command += ["--skip-user-stop"] if args.skip_user_stop else []
        status = subprocess.run(command, check=False).returncode
        if status < 0:
            print(f"\n  ! crashed: {signal.Signals(-status).name}")
        elif status:
            print(f"\n  ! stopped with exit status {status}")
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
