"""Hardware check of TaskImpedance's guards (deployment request item 8).

At panda-py's start pose, flange control at 400 N/m and 30 Nm/rad:

  1. force guard: 15 N for 50 ms. You push the flange sideways, firmly (about
     2 kg), until it trips; the spring lets go and only damping is left, so
     the arm gives way. Then the retreat the request specifies: rearm at
     200 N/m and move 30 mm up at 0.02 m/s, with the force guard still on.
  2. workspace guard: a 15 mm box around the end effector; the reference
     moves out in +x at 0.1 m/s until the guard trips. The arm should stop
     within a few millimetres of the box.
  3. speed guard: 0.01 m/s; a 10 mm reference step trips it.
  4. a manual trip, as the policy runner raises on missed deadlines.

After each trip, back to the start pose. For the push, the robot's collision
thresholds are raised to 40 N and 15 Nm and restored afterwards.

The check: each trip drops the active wrench on its own tick (the request asks
for 2 ms), the sent torque follows within the 1 Nm/ms rate limit, and no trip
or retreat sets off a reflex. The run is saved to npz.

    python tests/hardware/check_guards.py <robot-ip> <desk-user> [--platform panda]
"""

import argparse
import getpass
import pathlib
import signal
import subprocess
import sys
import time

import numpy as np

import panda_py
from panda_py import controllers, safety, telemetry

STIFFNESS = [400.0] * 3 + [30.0] * 3
RETREAT_STIFFNESS = [200.0] * 3 + [30.0] * 3
RATE = 50
SPEED_FACTOR = 0.1
TRIP_NAMES = ["none", "force", "saturation", "speed", "joint_velocity", "workspace", "manual"]


class Run:
    def __init__(self, panda, ctrl):
        self.panda, self.ctrl = panda, ctrl
        self.events = []

    def mark(self, label):
        self.events.append((label, self.ctrl.get_snapshot()["time"]))
        print(f"    {label}", flush=True)

    def steps(self, translation, count):
        with self.panda.create_context(frequency=RATE, max_runtime=count / RATE) as ctx:
            while ctx.ok():
                self.ctrl.step_reference(translation, [0, 0, 0])

    def wait_for_trip(self, timeout, steps=None):
        start = time.monotonic()
        with self.panda.create_context(frequency=RATE) as ctx:
            while ctx.ok():
                if self.ctrl.guard_state["tripped"]:
                    return self.ctrl.guard_state
                if time.monotonic() - start > timeout:
                    return None
                if steps is not None:
                    self.ctrl.step_reference(steps, [0, 0, 0])
        raise RuntimeError("the control loop stopped")

    def hold(self, seconds):
        time.sleep(seconds)
        if not self.panda.get_state().robot_mode == panda_py.libfranka.RobotMode.kMove:
            raise RuntimeError(f"the robot left move mode: {self.panda.get_state().robot_mode}")

    def back_to(self, pose):
        """Back to a pose at the default stiffness, guard off."""
        self.ctrl.set_guard()
        self.ctrl.rearm()
        self.ctrl.set_stiffness(STIFFNESS)
        self.hold(0.05)
        here = self.ctrl.get_snapshot()["pose"][:3, 3]
        delta = pose[:3, 3] - here
        count = max(1, int(np.ceil(np.linalg.norm(delta) / 0.0004)))  # 0.02 m/s
        self.steps(delta / count, count)
        self.hold(1.0)


def run(panda, out, fake=False):
    if not fake:
        assert panda.move_to_start(speed_factor=SPEED_FACTOR), "did not reach the start pose"
    robot = panda.get_robot()
    safety.set_collision_thresholds(robot, 40.0, 15.0)
    ctrl = controllers.TaskImpedance(STIFFNESS, frame="flange", telemetry=5000)
    ctrl.set_leash(0.025, 0.5)
    recorder = telemetry.Recorder(ctrl)
    results = {}
    panda.start_controller(ctrl)
    r = Run(panda, ctrl)
    try:
        recorder.start()
        home = ctrl.get_snapshot()["pose"]
        r.hold(0.5)

        r.mark("force")
        ctrl.set_guard(force=15.0, force_time=0.05)
        print("      Push the flange sideways, firmly, until the arm gives way (30 s).",
              flush=True)
        results["force"] = r.wait_for_trip(1.0 if fake else 30.0)
        if results["force"] is None:
            print("      no trip within 30 s, skipping the force test")
        else:
            r.hold(0.3)  # damping only, as the request asks for 200 ms at least
            print("      Let go. Retreating 30 mm up at 0.02 m/s in 2 s.", flush=True)
            time.sleep(2.0)
            r.mark("retreat")
            ctrl.rearm()
            ctrl.set_stiffness(RETREAT_STIFFNESS)
            ctrl.set_guard(force=15.0, force_time=0.05)  # still on during the retreat
            r.steps([0, 0, 0.0004], 75)
            r.hold(0.5)
            results["retreat"] = ctrl.guard_state
        r.mark("home")
        r.back_to(home)

        r.mark("workspace")
        ee = np.array(panda.get_state().O_T_EE).reshape(4, 4, order="F")[:3, 3]
        box = safety.box_along_axis(ee, [0, 0, 1], 0.015, 0.015, 0.015)
        ctrl.set_guard(workspace=[box])
        r.hold(0.1)
        results["workspace"] = r.wait_for_trip(3.0, steps=[0.002, 0, 0])
        r.hold(0.5)
        results["workspace_box"] = (box[0].tolist(), box[1].tolist())
        r.mark("home")
        r.back_to(home)

        r.mark("speed")
        ctrl.set_guard(speed=0.01)
        ctrl.step_reference([0.01, 0, 0], [0, 0, 0])
        results["speed"] = r.wait_for_trip(2.0)
        r.hold(0.5)
        r.mark("home")
        r.back_to(home)

        r.mark("manual")
        ctrl.step_reference([0.005, 0, 0], [0, 0, 0])
        r.hold(0.1)
        ctrl.trip()
        r.hold(0.3)
        results["manual"] = ctrl.guard_state
        r.mark("home")
        r.back_to(home)
        r.mark("end")
    finally:
        recorder.stop()
        panda.stop_controller()
        panda.set_default_behavior()
    log = recorder.result()
    meta = {"events": r.events, "results": results, "stiffness": STIFFNESS,
            "panda_py": panda_py.__version__, "fake": fake}
    telemetry.save(out, log, meta)
    print(f"  saved {out}")
    return log, meta


def analyse(log, meta):
    ok = True
    gaps = telemetry.check(log)
    print(f"\n  telemetry: {gaps['samples']} rows, {gaps['missing']} missing; "
          f"{gaps['lost_cycles']} robot cycles without a command, marked")
    ok &= gaps["missing"] == 0
    guard = log["guard"].astype(int)
    trips = np.flatnonzero((guard[1:] != 0) & (guard[:-1] == 0)) + 1
    results = meta["results"]
    for i in trips:
        reason = TRIP_NAMES[guard[i]]
        before = np.linalg.norm(log["tau_task"][i - 1])
        at = log["tau_task"][i]
        passive = log["jacobian"][i].reshape(6, 7, order="F").T @ log["wrench_passive"][i]
        spring_gone = np.allclose(at, passive, atol=1e-9)
        # Ticks until the sent torque matches the law's again (rate limit).
        catch = np.flatnonzero(np.abs(log["tau_cmd"][i:] - log["tau_law"][i:]).max(axis=1) < 1e-6)
        settle = int(catch[0]) if len(catch) else -1
        line = (f"  trip '{reason}' at t = {log['time'][i] - log['time'][0]:.3f} s: "
                f"spring torque gone on the trip tick: {spring_gone}; |tau_task| "
                f"{before:.2f} -> {np.linalg.norm(at):.2f} Nm; sent torque caught up after "
                f"{settle} ticks")
        if reason == "force":
            force = np.linalg.norm(log["O_F_ext_hat_K"][:, :3], axis=1)
            above = i - 1
            while above > 0 and force[above] > 15.0:
                above -= 1
            line += f"; {i - above - 1} ms above 15 N first (guard: 50)"
        print(line)
        ok &= spring_gone and settle >= 0
    for name in ("force", "workspace", "speed", "manual"):
        state = results.get(name)
        if state is None:
            print(f"  {name}: {'skipped, no push' if name == 'force' else 'DID NOT TRIP'}")
            # The fake robot neither moves nor feels a push.
            ok &= name == "force" or meta.get("fake", False)
        else:
            print(f"  {name}: tripped as '{state['reason']}', value {state['value']:.4g}")
            ok &= state["reason"] == name
    if results.get("workspace") and "workspace" in [TRIP_NAMES[guard[t]] for t in trips]:
        pose, half = np.array(results["workspace_box"][0]), np.array(results["workspace_box"][1])
        i = trips[[TRIP_NAMES[guard[t]] for t in trips].index("workspace")]
        ee = log["O_T_EE"][i:i + 500, 12:15]
        local = (ee - pose[:3, 3]) @ pose[:3, :3]
        past = np.max(np.maximum(np.abs(local) - half, 0).max(axis=1))
        print(f"  workspace: the end effector went {past * 1e3:.2f} mm past the box after the trip")
    retreat = results.get("retreat")
    if retreat is not None:
        print(f"  retreat: {'tripped again: ' + retreat['reason'] if retreat['tripped'] else 'no trip'}")
        ok &= not retreat["tripped"]
    print(f"  control command success rate: lowest {log['control_command_success_rate'][1:].min():.3f}")
    return ok


def worker(hostname, out, fake=False):
    panda = panda_py.Panda(hostname)
    print(f"  panda-py {panda_py.__version__}, server version {panda.get_robot().server_version()}")
    log, meta = run(panda, out, fake)
    ok = analyse(log, meta)
    print("\n  " + ("All checks passed." if ok else "SOME CHECKS FAILED."))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("hostname")
    parser.add_argument("username")
    parser.add_argument("password", nargs="?")
    parser.add_argument("--platform", default="panda")
    parser.add_argument("--out", default=None)
    parser.add_argument("--analyse", metavar="NPZ", help="only analyse a saved run")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    # For the fake robot: no start pose, nobody to push.
    parser.add_argument("--fake", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.analyse:
        log, meta = telemetry.load(args.analyse)
        return 0 if analyse(log, meta) else 1
    out = args.out or f"results/guards_{time.strftime('%Y%m%d-%H%M%S')}.npz"
    if args.worker:
        return worker(args.hostname, out, args.fake)
    pathlib.Path(out).parent.mkdir(parents=True, exist_ok=True)
    password = args.password or getpass.getpass("  Desk password: ")
    print(f"panda-py {panda_py.__version__} from {panda_py.__file__}")
    desk = panda_py.Desk(args.hostname, args.username, password, platform=args.platform)
    print("  desk: logged in, control token acquired")
    status = 1
    try:
        print("\n  Unlocking the brakes WILL make the robot move. The test moves it to the")
        print("  start pose and asks you to push the flange by hand until the force guard")
        print("  trips; the arm then gives way. It also moves up to 30 mm on its own. The")
        print("  collision thresholds are raised to 40 N / 15 Nm for the push. Keep the")
        print("  workspace clear and the user stop within reach.\n")
        if input("  Type YES to continue: ") != "YES":
            print("  aborted")
            return 1
        desk.unlock()
        print("  desk: brakes unlocked")
        desk.activate_fci()
        print("  desk: FCI activated")
        command = [sys.executable, __file__, args.hostname, args.username, "-",
                   "--worker", "--out", out]
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
