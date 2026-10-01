"""Hardware check of online stiffness, the tank and the joint servo.

Deployment request items 9, 10 and 11, and the tank meter self-check of
item 12, in free space at panda-py's start pose:

  1. stiffness (item 9): flange control with a 5 mm reference offset; the
     stiffness switches between the variable-stiffness extremes, 50/5 and
     800/150, at 50 Hz for 2 s. Every switch may change the torque by the
     law's own Delta K e and Delta D v, nothing more, and the damping must
     follow D = 2 sqrt(K) on every tick.
  2. tank (item 10, item 12): a slow 20 mm sine in y, 4 s, three times:
     E0 = 1e9 J (the gate must stay 1: the plain controller), E0 = 0.05 J in
     power mode and 0.5 N s in impulse mode (the gate must ramp as
     E_T / (0.25 E0)). The meter is replayed from the logged wrench and
     velocity: E_drawn must match within 2 %.
  3. servo (item 11): the POS variant's gains, a 20 mrad step on joint 4 and
     back; the response is compared with the second-order prediction from
     the joint's inertia, and the gains are read back from the controller.

Small, slow moves only. Keep the workspace clear and the user stop within
reach. The robot is always restored: FCI off, brakes locked, control released.

    python tests/hardware/check_vic.py <robot-ip> <desk-user> [--platform panda]
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

RATE = 50
SPEED_FACTOR = 0.1
SOFT = np.array([50.0] * 3 + [5.0] * 3)
STIFF = np.array([800.0] * 3 + [150.0] * 3)
FIXED = np.array([400.0] * 3 + [30.0] * 3)
SERVO_K = np.array([600.0, 600, 600, 600, 250, 150, 50])
SERVO_D = np.array([30.0, 30, 30, 30, 10, 10, 5])
# The simulator's joint armature, kg m^2, added to the arm's inertia there.
ARMATURE = 0.1


def task_impedance(stiffness=FIXED):
    ctrl = controllers.TaskImpedance(stiffness, frame="flange", telemetry=6000)
    ctrl.set_leash(0.025, 0.5)
    return ctrl


def record(panda, ctrl, body):
    """Starts ctrl, runs body(ctrl, ctx) at 50 Hz under a recorder, stops."""
    recorder = telemetry.Recorder(ctrl)
    panda.start_controller(ctrl)
    try:
        recorder.start()
        with panda.create_context(frequency=RATE) as ctx:
            body(ctrl, ctx)
        time.sleep(0.05)
    finally:
        recorder.stop()
        panda.stop_controller()
    return recorder.result()


def wait(ctx, seconds):
    for _ in range(int(seconds * RATE)):
        if not ctx.ok():
            raise RuntimeError("the control loop stopped")


def sine(ctrl, ctx, seconds=4.0):
    for i in range(int(seconds * RATE)):
        t0, t1 = i / RATE, (i + 1) / RATE
        dy = 0.02 * (np.sin(2 * np.pi * t1 / 4) - np.sin(2 * np.pi * t0 / 4))
        if not ctx.ok():
            raise RuntimeError("the control loop stopped")
        ctrl.step_reference([0, dy, 0], [0, 0, 0])


def run(panda, out, fake=False):
    def start():
        if not fake:
            assert panda.move_to_start(speed_factor=SPEED_FACTOR), "did not reach the start pose"

    start()
    logs, meta = {}, {"panda_py": panda_py.__version__}

    print("    stiffness switching", flush=True)

    def switching(ctrl, ctx):
        wait(ctx, 0.5)
        ctrl.step_reference([0.005, 0, 0], [0, 0, 0])
        wait(ctx, 1.0)
        for i in range(2 * RATE):
            if not ctx.ok():
                raise RuntimeError("the control loop stopped")
            ctrl.step_reference([0, 0, 0], [0, 0, 0], SOFT if i % 2 else STIFF)
        ctrl.set_stiffness(FIXED)
        wait(ctx, 0.5)

    logs["stiffness"] = record(panda, task_impedance(), switching)

    for name, E0, mode in (("tank_off", 1e9, "power"), ("tank_power", 0.05, "power"),
                           ("tank_impulse", 0.5, "impulse")):
        print(f"    {name}: E0 {E0:g}, {mode}", flush=True)
        ctrl = task_impedance()
        ctrl.set_tank(E0, mode)

        def tank_run(ctrl, ctx):
            wait(ctx, 0.3)
            sine(ctrl, ctx)
            wait(ctx, 0.5)

        logs[name] = record(panda, ctrl, tank_run)
        meta[name] = {"E0": E0, "mode": mode}
        start()

    print("    servo: joint 4 +20 mrad and back", flush=True)
    state = panda.get_state()
    mass = np.array(panda.get_model().mass(state)).reshape(7, 7, order="F")
    servo = controllers.JointPosition(SERVO_K, SERVO_D, telemetry=6000)
    meta["servo"] = {"stiffness": servo.get_stiffness().tolist(),
                     "damping": servo.get_damping().tolist(),
                     "inertia": float(mass[3, 3])}

    def steps(ctrl, ctx):
        wait(ctx, 0.5)
        ctrl.set_control(np.array(panda.q) + np.r_[0, 0, 0, 0.02, 0, 0, 0])
        wait(ctx, 1.5)
        ctrl.set_control(np.array(panda.q) - np.r_[0, 0, 0, 0.02, 0, 0, 0])
        wait(ctx, 1.5)

    logs["servo"] = record(panda, servo, steps)

    for name, log in logs.items():
        telemetry.save(f"{out}_{name}.npz", log, {**meta, "part": name})
    print(f"  saved {out}_*.npz")
    return logs, meta


# -- analysis -------------------------------------------------------------------


def analyse_stiffness(log):
    ok = True
    damping_ok = np.allclose(log["damping"], 2 * np.sqrt(log["stiffness"]), rtol=1e-12)
    print(f"  damping = 2 sqrt(K) on every tick: {damping_ok}")
    ok &= damping_ok
    switches = np.flatnonzero(np.any(np.diff(log["stiffness"], axis=0) != 0, axis=1)) + 1
    excess, jump, predicted = [], [], []
    for k in switches:
        J = log["jacobian"][k].reshape(6, 7, order="F")
        dk = log["stiffness"][k] - log["stiffness"][k - 1]
        dd = log["damping"][k] - log["damping"][k - 1]
        error = log["wrench_active"][k - 1] / np.where(log["stiffness"][k - 1] > 0,
                                                       log["stiffness"][k - 1], 1)
        velocity = -log["wrench_passive"][k - 1] / np.where(log["damping"][k - 1] > 0,
                                                            log["damping"][k - 1], 1)
        law = J.T @ (dk * error - dd * velocity)
        change = log["tau_task"][k] - log["tau_task"][k - 1]
        jump.append(np.abs(change).max())
        predicted.append(np.abs(law).max())
        excess.append(np.abs(change - law).max())
    excess = np.array(excess)
    print(f"  {len(switches)} switches: largest torque change {max(jump):.2f} Nm, of which "
          f"the law's own Delta K e, Delta D v {max(predicted):.2f} Nm; largest change "
          f"beyond it {excess.max():.3f} Nm (from the state moving in 1 ms)")
    ok &= excess.max() < 0.2
    rate = np.abs(np.diff(log["tau_cmd"], axis=0)).max()
    print(f"  sent torque: largest change per tick {rate:.3f} Nm (rate limit 1 Nm)")
    return ok


def analyse_tank(name, log, meta):
    ok = True
    E0, mode = meta[name]["E0"], meta[name]["mode"]
    level, drawn = E0, 0.0
    worst_alpha = worst_level = 0.0
    for k in range(len(log["tick"])):
        J = log["jacobian"][k].reshape(6, 7, order="F")
        alpha, level, drawn = controllers.TaskImpedance.tank_step(
            level, drawn, log["wrench_active"][k], J @ log["dq"][k], log["duration"][k],
            E0, mode)
        worst_alpha = max(worst_alpha, abs(alpha - log["alpha"][k]))
        worst_level = max(worst_level, abs(level - log["tank"][k]))
    logged = log["tank_drawn"][-1]
    print(f"  {name}: gate min {log['alpha'].min():.3f}, final level {log['tank'][-1]:.4g}, "
          f"drawn {logged:.4g}; replayed meter: gate within {worst_alpha:.1e}, level within "
          f"{worst_level:.1e}, drawn {drawn:.4g}")
    ok &= worst_alpha < 1e-9 and abs(drawn - logged) <= 0.02 * max(logged, 1e-12)
    if E0 >= 1e9:
        plain = np.all(log["alpha"] == 1.0)
        print(f"  {name}: gate 1 on every tick, the plain controller: {plain}")
        ok &= plain
        # Item 12: the meter against the integral of the logged active power.
        jacobian = log["jacobian"].reshape(-1, 7, 6).transpose(0, 2, 1)  # column-major
        velocity = np.einsum("nrc,nc->nr", jacobian, log["dq"])
        power = np.einsum("nr,nr->n", log["wrench_active"], velocity)
        integral = np.sum(np.maximum(power, 0) * log["duration"])
        agree = abs(integral - logged) <= 0.02 * max(integral, 1e-12)
        print(f"  {name}: E_drawn {logged:.4g} J vs integral of the active power "
              f"{integral:.4g} J: within 2 %: {agree}")
        ok &= agree
    else:
        gated = log["alpha"] < 1.0
        ramp = np.allclose(log["alpha"][1:][gated[1:]],
                           np.clip(log["tank"][:-1][gated[1:]] / (0.25 * E0), 0, 1), atol=1e-9)
        print(f"  {name}: gate below 1 on {gated.sum()} ticks; ramps as E_T / (0.25 E0): {ramp}")
        ok &= ramp and gated.any()
    return ok


def analyse_servo(log, meta):
    servo = meta["servo"]
    K, D = np.array(servo["stiffness"])[3], np.array(servo["damping"])[3]
    print(f"  servo gains read back: K {servo['stiffness']}, D {servo['damping']}")
    ok = np.allclose(servo["stiffness"], SERVO_K) and np.allclose(servo["damping"], SERVO_D)
    updates = np.flatnonzero(log["reference_update"] > 0)
    for inertia, label in ((servo["inertia"], "arm"), (servo["inertia"] + ARMATURE,
                                                        "arm + simulator armature")):
        wn, zeta = np.sqrt(K / inertia), D / (2 * np.sqrt(K * inertia))
        over = np.exp(-np.pi * zeta / np.sqrt(1 - zeta**2)) if zeta < 1 else 0.0
        settle = 3.0 / (zeta * wn)
        print(f"  prediction ({label}, I = {inertia:.3f} kg m^2): wn {wn:.1f} rad/s, "
              f"zeta {zeta:.2f}, overshoot {over * 100:.0f} %, settles (5 %) in {settle:.3f} s")
    for n, k in enumerate(updates[:2]):
        end = updates[1] if n == 0 and len(updates) > 1 else len(log["tick"])
        q = log["q"][k:end, 3]
        target, start = log["q_d"][k, 3], log["q"][k, 3]
        progress = (q - start) / (target - start)
        over = max(0.0, progress.max() - 1.0)
        outside = np.flatnonzero(np.abs(progress - 1.0) > 0.05)
        settle = (log["time"][k + outside[-1]] - log["time"][k]) if len(outside) else 0.0
        residual = (q[-50:].mean() - target) * 1e3
        print(f"  step {'+' if n == 0 else '-'}20 mrad: overshoot {over * 100:.0f} %, settles "
              f"(5 %) in {settle:.3f} s, residual {residual:+.2f} mrad")
    return ok


def analyse(logs, meta):
    ok = True
    for name, log in logs.items():
        gaps = telemetry.check(log)
        print(f"  {name}: {gaps['samples']} rows, {gaps['missing']} missing, "
              f"{gaps['lost_cycles']} lost cycles (marked)")
        ok &= gaps["missing"] == 0
    print()
    ok &= analyse_stiffness(logs["stiffness"])
    for name in ("tank_off", "tank_power", "tank_impulse"):
        ok &= analyse_tank(name, logs[name], meta)
    ok &= analyse_servo(logs["servo"], meta)
    return ok


def load(prefix):
    logs, meta = {}, {}
    for name in ("stiffness", "tank_off", "tank_power", "tank_impulse", "servo"):
        logs[name], meta = telemetry.load(f"{prefix}_{name}.npz")
    return logs, meta


def worker(hostname, out, fake=False):
    panda = panda_py.Panda(hostname)
    print(f"  panda-py {panda_py.__version__}, server version {panda.get_robot().server_version()}")
    logs, meta = run(panda, out, fake)
    ok = analyse(logs, meta)
    print("\n  " + ("All checks passed." if ok else "SOME CHECKS FAILED."))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("hostname")
    parser.add_argument("username")
    parser.add_argument("password", nargs="?")
    parser.add_argument("--platform", default="panda")
    parser.add_argument("--out", default=None, help="prefix of the npz files")
    parser.add_argument("--analyse", metavar="PREFIX", help="only analyse a saved run")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--fake", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.analyse:
        return 0 if analyse(*load(args.analyse)) else 1
    out = args.out or f"results/vic_{time.strftime('%Y%m%d-%H%M%S')}"
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
        print("  start pose, offsets the flange 5 mm while switching its stiffness, moves")
        print("  it 20 mm in a slow sine three times, and steps joint 4 by 20 mrad. Keep the")
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
