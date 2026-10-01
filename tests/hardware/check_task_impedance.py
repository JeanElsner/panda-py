"""Hardware check of TaskImpedance, the insertion controller, on a robot.

Runs the deployment request's first controller acceptance tests in free space,
at panda-py's start pose, with the insertion gains (400 N/m, 30 Nm/rad) at the
flange and the dynamically consistent nullspace (k = 10, holding the start
posture):

  1. hold 1 s
  2. a 10 mm reference step in base x, held 2 s, and back (item 1)
  3. a 0.05 rad reference step about base z, held 2 s, and back (item 1)
  4. 4 s of 50 Hz reference steps through step_reference(), a 20 mm sine in
     base y, with the 25 mm / 0.5 rad leash on (item 3)
  5. hold 1 s

The whole run is recorded at 1 kHz (item 4) and saved to npz, then:

  - telemetry: one row per tick, no gaps, no lost cycles
  - steps: overshoot, settling time to 5 %, and the residual error
  - reference updates: exactly one per 50 Hz step, applied at a tick
  - parity (item 13): every logged tick replayed through vic_reference.py,
    the simulator's law, against the torque the controller computed

The moves are 10 mm, 0.05 rad and 20 mm, slow. Keep the workspace clear and
the user stop within reach. The robot is always restored: FCI off, brakes
locked, control released.

    python tests/hardware/check_task_impedance.py <robot-ip> <desk-user> [--platform panda]
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
from panda_py import controllers, telemetry

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import vic_reference as vic  # noqa: E402  pylint: disable=wrong-import-position

STIFFNESS = [vic.FIXED_K_TRANS] * 3 + [vic.FIXED_K_ROT] * 3  # --stiffness overrides
RATE = 50
SPEED_FACTOR = 0.1


def schedule():
    """(seconds, translation per step, rotation per step, label) phases."""
    stream = []
    steps = 4 * RATE
    for i in range(steps):
        # y(t) = A sin(2 pi t / T), as per-step increments
        t0, t1 = i / RATE, (i + 1) / RATE
        dy = 0.02 * (np.sin(2 * np.pi * t1 / 4) - np.sin(2 * np.pi * t0 / 4))
        stream.append(([0.0, dy, 0.0], [0.0, 0.0, 0.0]))
    return [
        ("hold", [(None, None)] * RATE),
        ("x +10 mm", [([0.01, 0, 0], [0, 0, 0])] + [(None, None)] * (2 * RATE - 1)),
        ("x -10 mm", [([-0.01, 0, 0], [0, 0, 0])] + [(None, None)] * (2 * RATE - 1)),
        ("z +0.05 rad", [([0, 0, 0], [0, 0, 0.05])] + [(None, None)] * (2 * RATE - 1)),
        ("z -0.05 rad", [([0, 0, 0], [0, 0, -0.05])] + [(None, None)] * (2 * RATE - 1)),
        ("stream", stream),
        ("hold", [(None, None)] * RATE),
    ]


def run(panda, out, start=True):
    if start:
        assert panda.move_to_start(speed_factor=SPEED_FACTOR), "did not reach the start pose"
    ctrl = controllers.TaskImpedance(
        STIFFNESS, frame="flange", nullspace="dynamic",
        nullspace_stiffness=vic.NULLSPACE_KP, telemetry=4000,
    )
    ctrl.set_leash(vic.LEASH_POS, vic.LEASH_ROT)
    phases, marks, sent = schedule(), [], 0
    recorder = telemetry.Recorder(ctrl)
    panda.start_controller(ctrl)
    try:
        q0 = np.array(panda.q)
        ctrl.set_nullspace_target(q0)
        recorder.start()
        with panda.create_context(frequency=RATE) as ctx:
            for label, steps in phases:
                print(f"    {label}", flush=True)
                marks.append((label, ctrl.get_snapshot()["time"]))
                for translation, rotation in steps:
                    if not ctx.ok():
                        raise RuntimeError("the control loop stopped")
                    if translation is not None:
                        ctrl.step_reference(translation, rotation)
                        sent += 1
        time.sleep(0.05)
    finally:
        recorder.stop()
        panda.stop_controller()
    log = recorder.result()
    meta = {
        "q_nullspace": q0.tolist(), "stiffness": STIFFNESS, "frame": "flange",
        "nullspace": "dynamic", "nullspace_stiffness": vic.NULLSPACE_KP,
        "leash": [vic.LEASH_POS, vic.LEASH_ROT], "marks": marks, "steps_sent": sent,
        "panda_py": panda_py.__version__,
    }
    telemetry.save(out, log, meta)
    print(f"  saved {out}")
    return log, meta


# -- analysis ----------------------------------------------------------------


def wxyz(xyzw):
    return np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]])


def step_metrics(t, signal_, target, start):
    """Overshoot past the target, time to stay within 5 % of the step, residual."""
    size = target - start
    progress = (signal_ - start) / size
    overshoot = max(0.0, progress.max() - 1.0) * abs(size)
    outside = np.flatnonzero(np.abs(progress - 1.0) > 0.05)
    settle = t[outside[-1]] - t[0] if len(outside) else 0.0
    return overshoot, settle, (signal_[-50:].mean() - target)


def analyse(log, meta):
    ok = True
    t = log["time"] - log["time"][0]
    gaps = telemetry.check(log)
    span = log["time"][-1] - log["time"][0]
    lost = np.round(log["duration"][1:] / 1e-3) > 1
    print(f"\n  telemetry: {gaps['samples']} rows over {span:.2f} s, "
          f"{gaps['missing']} missing, {int(log['dropped'])} dropped; "
          f"{gaps['lost_cycles']} robot cycles without a command in {int(lost.sum())} events, "
          "marked in the log (a real-time kernel avoids most)")
    # Item 4: no row missing. Lost robot cycles are marked, not a failure.
    ok &= gaps["missing"] == 0 and int(log["dropped"]) == 0

    marks = dict()
    times = [m[1] for m in meta["marks"]] + [log["time"][-1]]
    for i, (label, start) in enumerate(meta["marks"]):
        marks.setdefault(label, []).append((start, times[i + 1]))

    def window(label):
        start, end = marks[label][0]
        return (log["time"] >= start) & (log["time"] < end)

    # Each step is measured against the reference's own jump: the window opens
    # one tick before the step is applied.
    for label in ("x +10 mm", "x -10 mm"):
        w = window(label)
        x, ref = log["position"][w, 0], log["position_ref"][w, 0]
        over, settle, residual = step_metrics(t[w], x, ref[-1], ref[0])
        stall = log["wrench_active"][w][-200:, 0].mean()
        print(f"  {label}: overshoot {over * 1e3:.2f} mm, settles (5 %) in {settle:.3f} s, "
              f"residual {residual * 1e3:+.2f} mm, spring force left {stall:+.2f} N")
    error = controllers.TaskImpedance.orientation_error
    for label in ("z +0.05 rad", "z -0.05 rad"):
        w = window(label)
        ref = log["orientation_ref"][w]
        # Rotation about z of each orientation relative to the final reference.
        angle = np.array([-error(ref[-1], o)[2] for o in log["orientation"][w]])
        over, settle, residual = step_metrics(t[w], angle, 0.0, -error(ref[-1], ref[0])[2])
        print(f"  {label}: overshoot {over * 1e3:.2f} mrad, settles (5 %) in {settle:.3f} s, "
              f"residual {residual * 1e3:+.2f} mrad")

    updates = np.flatnonzero(log["reference_update"] > 0)
    spacing = np.diff(log["tick"][updates])
    print(f"  reference updates: {len(updates)} applied for {meta['steps_sent']} sent, "
          f"spacing {np.median(spacing):.0f} ticks (min {spacing.min():.0f}, max {spacing.max():.0f})")
    ok &= len(updates) == meta["steps_sent"]
    # The leash binds at the tick a reference is applied, as in the simulator.
    lead = np.linalg.norm(log["position_ref"] - log["position"], axis=1)[updates]
    print(f"  leash: largest |x_ref - x| where applied {lead.max() * 1e3:.2f} mm "
          f"(leash {vic.LEASH_POS * 1e3:.0f} mm)")
    ok &= lead.max() <= vic.LEASH_POS + 1e-9

    # Parity: the controller's own law on the logged inputs, and vic_reference.
    ref_ = vic.VicReference(clip_torque=False, nullspace_projection="dynamic",
                            nullspace_kp=meta["nullspace_stiffness"],
                            q0=np.array(meta["q_nullspace"]))
    own, sim, norm = [], [], []
    for i in range(len(log["tick"])):
        J = log["jacobian"][i].reshape(6, 7, order="F")
        M = log["mass"][i].reshape(7, 7, order="F")
        pose = np.eye(4)
        pose[:3, 3] = log["position"][i]
        q_xyzw = log["orientation"][i]
        x, w, z, y = q_xyzw[0], q_xyzw[3], q_xyzw[2], q_xyzw[1]
        pose[:3, :3] = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        replay = controllers.TaskImpedance.compute(
            q=log["q"][i], dq=log["dq"][i], pose=pose, jacobian=J, mass=M,
            position_ref=log["position_ref"][i], orientation_ref=log["orientation_ref"][i],
            stiffness=log["stiffness"][i], damping=log["damping"][i],
            q_nullspace=meta["q_nullspace"], nullspace_stiffness=meta["nullspace_stiffness"],
        )
        ref_.K = log["stiffness"][i]
        ref_.x_ref, ref_.q_ref = log["position_ref"][i], wxyz(log["orientation_ref"][i])
        v6 = J @ log["dq"][i]
        tau_sim, _ = ref_.torque(log["position"][i], wxyz(q_xyzw), v6[:3], v6[3:],
                                 log["q"][i], log["dq"][i], J, M, 1e-3)
        tau = log["tau_law"][i]
        own.append(np.linalg.norm(replay["tau"] - tau))
        sim.append(np.linalg.norm(tau_sim - tau))
        norm.append(np.linalg.norm(tau))
    own, sim, norm = map(np.array, (own, sim, norm))
    rel = sim / np.maximum(norm, 1e-12)
    loaded = norm > 0.5
    print(f"  replay through compute(): largest difference {own.max():.2e} Nm")
    print(f"  parity with vic_reference (item 13): run {np.linalg.norm(sim) / np.linalg.norm(norm):.2e} "
          f"of the torque norm; per tick with |tau| > 0.5 Nm median {np.median(rel[loaded]):.2e}, "
          f"worst {rel[loaded].max():.2e}")
    # vic_reference zeroes rotation errors below 1e-4 rad, the controller does
    # not, which shows on near-zero torques only; judged per loaded tick.
    ok &= rel[loaded].max() <= 0.01
    sent_vs_law = np.abs(log["tau_cmd"] - log["tau_law"]).max()
    print(f"  sent vs law torque: largest difference {sent_vs_law:.3f} Nm "
          "(joint walls, rate limit, clipping)")
    # The rate starts at 0 on the first tick of a motion.
    print(f"  control command success rate: lowest {log['control_command_success_rate'][1:].min():.3f}")
    return ok


def worker(hostname, out, start=True):
    panda = panda_py.Panda(hostname)
    print(f"  panda-py {panda_py.__version__}, server version "
          f"{panda.get_robot().server_version()}")
    log, meta = run(panda, out, start)
    ok = analyse(log, meta)
    print("\n  " + ("All checks passed." if ok else "SOME CHECKS FAILED."))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("hostname")
    parser.add_argument("username")
    parser.add_argument("password", nargs="?")
    parser.add_argument("--platform", default="panda")
    parser.add_argument("--stiffness", nargs=2, type=float, metavar=("K_T", "K_R"),
                        help="translational and rotational stiffness (default 400 30)")
    parser.add_argument("--out", default=None, help="npz path (default results/task_impedance_<time>.npz)")
    parser.add_argument("--analyse", metavar="NPZ", help="only analyse a saved run")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    # For the fake robot, which cannot move to the start pose.
    parser.add_argument("--no-start", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.analyse:
        log, meta = telemetry.load(args.analyse)
        return 0 if analyse(log, meta) else 1
    global STIFFNESS  # pylint: disable=global-statement
    if args.stiffness:
        STIFFNESS = [args.stiffness[0]] * 3 + [args.stiffness[1]] * 3
    tag = f"_K{STIFFNESS[0]:g}" if args.stiffness else ""
    out = args.out or f"results/task_impedance{tag}_{time.strftime('%Y%m%d-%H%M%S')}.npz"
    if args.worker:
        return worker(args.hostname, out, start=not args.no_start)
    pathlib.Path(out).parent.mkdir(parents=True, exist_ok=True)
    password = args.password or getpass.getpass("  Desk password: ")

    print(f"panda-py {panda_py.__version__} from {panda_py.__file__}")
    desk = panda_py.Desk(args.hostname, args.username, password, platform=args.platform)
    print("  desk: logged in, control token acquired")
    status = 1
    try:
        print("\n  Unlocking the brakes WILL make the robot move. The test moves it to the")
        print("  start pose, then steps the flange 10 mm and 0.05 rad and back, and moves")
        print("  it 20 mm in a slow sine. Keep the workspace clear and the user stop within")
        print("  reach.\n")
        if input("  Type YES to continue: ") != "YES":
            print("  aborted")
            return 1
        desk.unlock()
        print("  desk: brakes unlocked")
        desk.activate_fci()
        print("  desk: FCI activated")
        command = [sys.executable, __file__, args.hostname, args.username, "-",
                   "--worker", "--out", out]
        if args.stiffness:
            command += ["--stiffness", *map(str, args.stiffness)]
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
