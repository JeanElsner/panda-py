"""
``panda-check``: exercises panda-py on a robot and writes a report.

Connects to the robot, reads its state and model, runs every controller,
the motion generators, the inverse kinematics, the guards, error recovery
and teaching mode, each with small motions around the start pose, and the
gripper if asked to. It writes a Markdown report and a JSON file with every
measurement: the robot's protocol version and type, panda-py's and
libfranka's versions, the host, and per check what was done, what was
measured and whether it passed.

The robot must be unlocked with FCI active, or give ``--desk-user`` and
panda-check does that through Desk and restores the previous state at the
end. It moves the robot: keep the workspace around the start pose clear and
the user stop at hand. ``--no-motion`` runs only the checks that do not move.

    panda-check <robot-ip> [--desk-user USER] [--gripper] [--out DIR]
"""

import argparse
import dataclasses
import datetime
import getpass
import json
import logging
import pathlib
import platform
import socket
import sys
import time
import traceback
import typing

import numpy as np

import panda_py
from panda_py import constants, controllers, libfranka

__all__ = ["main"]

GUARD = {"force": 40.0, "force_time": 0.1, "speed": 0.4}
"""Guards every controller check runs with: no check needs more."""

TELEMETRY = 20000


@dataclasses.dataclass
class Check:
    name: str
    description: str
    status: str = "skipped"  # passed, failed, skipped
    seconds: float = 0.0
    measurements: dict = dataclasses.field(default_factory=dict)
    problems: list = dataclasses.field(default_factory=list)
    error: typing.Optional[str] = None


class Checker:
    """Runs the checks in order and keeps the report."""

    def __init__(self, args):
        self.args = args
        self.panda: typing.Optional[panda_py.Panda] = None
        self.checks: typing.List[Check] = []
        self.info: dict = {}

    # -- bookkeeping -----------------------------------------------------------

    def run(self, name, description, function, moves=False, needs_robot=True):
        check = Check(name, description)
        self.checks.append(check)
        if moves and self.args.no_motion:
            check.problems.append("skipped: --no-motion")
            print(f"  -    {name}: skipped (--no-motion)")
            return check
        if needs_robot and self.panda is None and name != "connect":
            check.problems.append("skipped: no connection")
            print(f"  -    {name}: skipped (no connection)")
            return check
        start = time.monotonic()
        try:
            function(check)
            check.status = "failed" if check.problems else "passed"
        except Exception as error:  # pylint: disable=broad-except
            check.status = "failed"
            check.error = f"{type(error).__name__}: {error}"
            check.measurements["traceback"] = traceback.format_exc()
            self._recover()
        check.seconds = time.monotonic() - start
        mark = {"passed": "ok  ", "failed": "FAIL"}[check.status]
        detail = check.error or "; ".join(check.problems)
        print(f"  {mark} {name}" + (f": {detail}" if detail else ""), flush=True)
        return check

    def _recover(self):
        if self.panda is None:
            return
        try:
            self.panda.stop_controller()
            self.panda.recover()
        except Exception:  # pylint: disable=broad-except
            pass

    @staticmethod
    def expect(check, condition, problem):
        if not condition:
            check.problems.append(problem)

    def controller_run(self, controller, actions, duration):
        """Runs a controller for `duration` s, calling each (t, action) at t s,
        and returns its telemetry joined over the run."""
        panda = self.panda
        controller.set_guard(**GUARD)
        parts = []
        panda.start_controller(controller)
        start = time.monotonic()
        pending = sorted(actions, key=lambda a: a[0])
        try:
            while time.monotonic() - start < duration:
                elapsed = time.monotonic() - start
                while pending and pending[0][0] <= elapsed:
                    pending.pop(0)[1](controller)
                parts.append(controller.read_telemetry())
                panda.raise_error()
                time.sleep(0.02)
        finally:
            panda.stop_controller()
        parts.append(controller.read_telemetry())
        panda.raise_error()
        return {k: np.concatenate([p[k] for p in parts if len(p["tick"])]) for k in parts[0]}

    def loop_health(self, check, telemetry, controller):
        ticks = telemetry["tick"]
        durations = telemetry["duration"][1:]
        check.measurements.update(
            ticks=int(len(ticks)),
            telemetry_dropped=int(controller.telemetry_dropped),
            longest_tick_gap_ms=float(durations.max() * 1e3) if len(durations) else None,
            ticks_above_1ms=int((durations > 0.0015).sum()),
            min_success_rate=float(telemetry["control_command_success_rate"].min()),
            guard=controller.guard_state,
        )
        self.expect(check, len(ticks) > 0, "no telemetry")
        self.expect(check, controller.telemetry_dropped == 0, "telemetry samples dropped")
        self.expect(check, np.all(np.diff(ticks) == 1), "ticks missing from the telemetry")
        self.expect(check, telemetry["control_command_success_rate"].min() > 0.9,
                    "command success rate below 0.9")
        self.expect(check, not controller.guard_state["tripped"],
                    f"guard tripped: {controller.guard_state['reason']}")

    # -- the checks --------------------------------------------------------------

    def environment(self, check):
        realtime, message = panda_py.realtime_priority_available()
        check.measurements.update(
            panda_py=panda_py.__version__,
            libfranka=libfranka.__version__,
            python=sys.version.split()[0],
            platform=platform.platform(),
            host=socket.gethostname(),
            realtime_kernel=bool(libfranka.has_realtime_kernel()),
            realtime_priority=bool(realtime),
            realtime_message=message,
        )
        self.info.update(check.measurements)
        if not check.measurements["realtime_kernel"]:
            check.measurements["note"] = (
                "Not a realtime kernel: control usually works, but latency spikes "
                "can abort motions.")

    def connect(self, check):
        self.panda = panda_py.Panda(self.args.hostname)
        robot = self.panda.get_robot()
        limits = self.panda.limits
        state = self.panda.get_state()
        check.measurements.update(
            protocol_version=int(robot.server_version()),
            robot=str(limits.type).split(".")[-1],
            envelope=limits.name,
            robot_mode=str(state.robot_mode),
            current_errors=str(state.current_errors),
            q=list(map(float, state.q)),
        )
        self.info.update(protocol_version=int(robot.server_version()),
                         robot=check.measurements["robot"], envelope=limits.name)
        q = np.asarray(state.q)
        self.expect(check, np.all(q >= limits.q_lower - 1e-3) and np.all(q <= limits.q_upper + 1e-3),
                    "joint positions outside the robot's envelope")
        for field in ("q", "dq", "tau_J", "O_T_EE", "O_F_ext_hat_K"):
            self.expect(check, np.all(np.isfinite(getattr(state, field))), f"{field} not finite")
        O_T_EE = np.asarray(state.O_T_EE).reshape(4, 4, order="F")
        R = O_T_EE[:3, :3]
        self.expect(check, np.allclose(R @ R.T, np.eye(3), atol=1e-4), "O_T_EE not a rotation")

    def model(self, check):
        state = self.panda.get_state()
        model = self.panda.get_model()
        mass = np.asarray(model.mass(state)).reshape(7, 7, order="F")
        gravity = np.asarray(model.gravity(state))
        coriolis = np.asarray(model.coriolis(state))
        eigenvalues = np.linalg.eigvalsh((mass + mass.T) / 2)
        check.measurements.update(
            mass_symmetry_error=float(np.abs(mass - mass.T).max()),
            mass_min_eigenvalue=float(eigenvalues.min()),
            gravity=gravity.tolist(), coriolis=coriolis.tolist())
        self.expect(check, np.abs(mass - mass.T).max() < 1e-6, "mass matrix not symmetric")
        self.expect(check, eigenvalues.min() > 0, "mass matrix not positive definite")
        self.expect(check, np.all(np.isfinite(gravity)) and np.all(np.isfinite(coriolis)),
                    "gravity or Coriolis not finite")
        # panda-py's kinematics against the robot's own pose, with its end effector.
        F_T_EE = np.asarray(state.F_T_EE).reshape(4, 4, order="F")
        O_T_EE = np.asarray(state.O_T_EE).reshape(4, 4, order="F")
        fk = panda_py.fk(state.q, F_T_EE=F_T_EE)
        position_error = float(np.linalg.norm(fk[:3, 3] - O_T_EE[:3, 3]))
        angle = float(np.arccos(np.clip((np.trace(fk[:3, :3].T @ O_T_EE[:3, :3]) - 1) / 2, -1, 1)))
        check.measurements.update(fk_position_error_mm=position_error * 1e3,
                                  fk_orientation_error_rad=angle)
        self.expect(check, position_error < 1e-3 and angle < 1e-3,
                    "panda_py.fk differs from the robot's O_T_EE")

    def settings(self, check):
        panda = self.panda
        panda.set_default_behavior()
        options = panda.get_control_options()
        panda.set_control_options(torque_rate_limit=True, limit_rate=False,
                                  cutoff_frequency=100.0)
        check.measurements["control_options"] = {k: (v if isinstance(v, (bool, float, int)) else str(v))
                                                 for k, v in options.items()}
        panda.set_joint_walls(False)
        off = panda.get_joint_walls()
        panda.set_joint_walls(True)
        self.expect(check, not off and panda.get_joint_walls(), "joint walls do not toggle")

    def recovery(self, check):
        self.panda.recover()
        self.panda.raise_error()

    def move_to_start(self, check):
        ok = self.panda.move_to_start(speed_factor=0.2)
        q = np.asarray(self.panda.q)
        check.measurements["error_rad"] = float(
            np.abs(q - np.asarray(constants.JOINT_POSITION_START)).max())
        self.expect(check, ok, "move_to_start reported failure")

    def joint_impedance(self, check):
        q0 = np.asarray(self.panda.q)
        target = q0 + np.r_[0, 0, 0, 0, 0, 0, 0.15]
        ctrl = controllers.JointImpedance(telemetry=TELEMETRY)
        t = self.controller_run(ctrl, [(0.5, lambda c: c.set_reference(target)),
                                       (2.5, lambda c: c.set_reference(q0))], 4.5)
        self.loop_health(check, t, ctrl)
        reached = t["q"][np.searchsorted(t["time"] - t["time"][0], 2.4), 6] - q0[6]
        back = float(np.abs(t["q"][-1] - q0).max())
        check.measurements.update(joint7_reached_rad=float(reached), returned_error_rad=back)
        self.expect(check, reached > 0.1, "joint 7 did not follow the reference")
        self.expect(check, back < 0.03, "did not return to the start")

    def joint_velocity(self, check):
        q0 = np.asarray(self.panda.q)
        ctrl = controllers.JointVelocity(telemetry=TELEMETRY)
        v = np.r_[0, 0, 0, 0, 0, 0, 0.1]
        t = self.controller_run(ctrl, [(0.5, lambda c: c.set_reference(v)),
                                       (1.5, lambda c: c.set_reference(-v)),
                                       (2.5, lambda c: c.set_reference(np.zeros(7)))], 3.5)
        self.loop_health(check, t, ctrl)
        peak = float((t["q"][:, 6] - q0[6]).max())
        check.measurements.update(joint7_peak_rad=peak,
                                  returned_error_rad=float(np.abs(t["q"][-1] - q0).max()))
        self.expect(check, 0.05 < peak < 0.15, "joint 7 did not move about 0.1 rad")

    def joint_torque(self, check):
        q0 = np.asarray(self.panda.q)
        ctrl = controllers.JointTorque(damping=np.full(7, 2.0), telemetry=TELEMETRY)
        t = self.controller_run(ctrl, [], 1.5)
        self.loop_health(check, t, ctrl)
        drift = float(np.abs(t["q"][-1] - q0).max())
        check.measurements["drift_rad"] = drift
        self.expect(check, drift < 0.05,
                    "drifted with zero torque: is the end effector's load set in Desk?")

    def task_impedance(self, check):
        ctrl = controllers.TaskImpedance(telemetry=TELEMETRY)
        position0 = self.panda.get_position()
        orientation0 = self.panda.get_orientation()
        up = position0 + np.r_[0, 0, 0.03]
        t = self.controller_run(ctrl, [(0.5, lambda c: c.set_reference(up, orientation0)),
                                       (2.5, lambda c: c.set_reference(position0, orientation0))],
                                4.5)
        self.loop_health(check, t, ctrl)
        i = np.searchsorted(t["time"] - t["time"][0], 2.4)
        reached = float(t["position"][i, 2] - position0[2])
        back = float(np.linalg.norm(t["position"][-1] - position0))
        check.measurements.update(reached_m=reached, returned_error_m=back)
        self.expect(check, reached > 0.02, "did not follow the reference up")
        self.expect(check, back < 0.008, "did not return to the start")

    def task_wrench(self, check):
        p0 = self.panda.get_position()
        ctrl = controllers.TaskWrench(damping=np.full(7, 2.0), telemetry=TELEMETRY)
        t = self.controller_run(ctrl, [], 1.5)
        self.loop_health(check, t, ctrl)
        drift = float(np.linalg.norm(self.panda.get_position() - p0))
        check.measurements["drift_m"] = drift
        self.expect(check, drift < 0.03, "drifted with zero wrench")

    def task_force(self, check):
        ctrl = controllers.TaskForce(telemetry=TELEMETRY)
        t = self.controller_run(ctrl, [], 1.5)
        self.loop_health(check, t, ctrl)
        check.measurements["max_displacement_m"] = float(t["displacement"].max())

    def guards(self, check):
        # A force threshold the external force estimate exceeds in free air
        # trips the force guard; then the manual trip and the rearm.
        ctrl = controllers.TaskImpedance(telemetry=TELEMETRY)
        ctrl.set_guard(force=0.01, force_time=0.0)
        self.panda.start_controller(ctrl)
        try:
            time.sleep(0.5)
            first = dict(ctrl.guard_state)
            ctrl.rearm()
            ctrl.set_guard(**GUARD)
            time.sleep(0.2)
            rearmed = dict(ctrl.guard_state)
            ctrl.trip()
            time.sleep(0.2)
            manual = dict(ctrl.guard_state)
            ctrl.rearm()
            time.sleep(0.2)
            final = dict(ctrl.guard_state)
        finally:
            self.panda.stop_controller()
        self.panda.raise_error()
        check.measurements.update(force_trip=first, after_rearm=rearmed, manual=manual,
                                  final=final)
        self.expect(check, first["reason"] == "force", "the force guard did not trip")
        self.expect(check, not rearmed["tripped"], "rearm did not clear the trip")
        self.expect(check, manual["reason"] == "manual", "trip() did not trip")
        self.expect(check, not final["tripped"], "the second rearm did not clear the trip")

    def motion(self, check):
        panda = self.panda
        q0 = np.asarray(panda.q)
        target = q0 + np.r_[0.1, -0.1, 0.1, 0.1, -0.1, 0.1, 0.1]
        ok_out = panda.move_to_joint_position(target, speed_factor=0.2)
        ok_back = panda.move_to_joint_position(q0, speed_factor=0.2)
        p0, o0 = panda.get_position(), panda.get_orientation()
        ok_pose = panda.move_to_pose(p0 + np.r_[0.03, -0.03, 0.03], o0, speed_factor=0.2)
        pose_error = float(np.linalg.norm(panda.get_position() - (p0 + np.r_[0.03, -0.03, 0.03])))
        ok_pose_back = panda.move_to_pose(p0, o0, speed_factor=0.2)
        check.measurements.update(joint_out=ok_out, joint_back=ok_back, pose_out=ok_pose,
                                  pose_error_m=pose_error, pose_back=ok_pose_back)
        for name, ok in (("move_to_joint_position", ok_out and ok_back),
                         ("move_to_pose", ok_pose and ok_pose_back)):
            self.expect(check, ok, f"{name} reported failure")

    def ik(self, check):
        panda = self.panda
        state = panda.get_state()
        F_T_EE = np.asarray(state.F_T_EE).reshape(4, 4, order="F")
        q0 = np.asarray(state.q)
        goal = panda_py.fk(q0, F_T_EE=F_T_EE)
        goal[:3, 3] += [0.0, 0.04, -0.03]
        q = panda_py.ik(goal, q_init=q0, limits=panda.limits, F_T_EE=F_T_EE)
        ok = panda.move_to_joint_position(q, speed_factor=0.2)
        reached = np.asarray(panda.get_state().O_T_EE).reshape(4, 4, order="F")
        error = float(np.linalg.norm(reached[:3, 3] - goal[:3, 3]))
        panda.move_to_joint_position(q0, speed_factor=0.2)
        check.measurements.update(solution=q.tolist(), reached_error_m=error, moved=ok)
        self.expect(check, ok, "the move to the IK solution reported failure")
        self.expect(check, error < 0.005, "the robot did not reach the IK pose")

    def teaching(self, check):
        self.panda.teaching_mode(True)
        time.sleep(1.0)
        self.panda.teaching_mode(False)
        self.panda.raise_error()

    def gripper(self, check):
        gripper = libfranka.Gripper(self.args.hostname)
        state = gripper.read_once()
        check.measurements.update(server_version=int(gripper.server_version()),
                                  max_width=float(state.max_width), width=float(state.width))
        self.expect(check, gripper.homing(), "homing failed")
        state = gripper.read_once()
        self.expect(check, gripper.move(state.max_width / 2, 0.05), "move failed")
        self.expect(check, gripper.move(state.max_width, 0.05), "move back failed")


# -- the report -------------------------------------------------------------------


def write_report(checker, out):
    info = checker.info
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    stem = f"panda-check-{info.get('robot', 'robot')}-v{info.get('protocol_version', 'x')}-{stamp}"
    out.mkdir(parents=True, exist_ok=True)
    data = {"date": datetime.datetime.now().isoformat(timespec="seconds"),
            "robot_address": checker.args.hostname, "info": info,
            "checks": [dataclasses.asdict(c) for c in checker.checks]}
    (out / f"{stem}.json").write_text(json.dumps(data, indent=1, default=str))
    passed = sum(c.status == "passed" for c in checker.checks)
    failed = [c for c in checker.checks if c.status == "failed"]
    lines = [
        f"# panda-check report, {data['date']}",
        "",
        f"- **Robot:** {info.get('robot', '?')}"
        + (f" ({info['envelope']})" if info.get("envelope") not in (None, info.get("robot")) else "")
        + ", "
        f"research interface protocol version {info.get('protocol_version', '?')}, at "
        f"{checker.args.hostname}",
        f"- **panda-py:** {info.get('panda_py')}, built with libfranka {info.get('libfranka')}",
        f"- **Host:** {info.get('host')}, {info.get('platform')}, Python {info.get('python')}, "
        f"realtime kernel: {'yes' if info.get('realtime_kernel') else 'no'}",
        f"- **Result:** {passed} passed, {len(failed)} failed, "
        f"{len(checker.checks) - passed - len(failed)} skipped",
        "",
        "| check | result | time | notes |",
        "|---|---|---|---|",
    ]
    for c in checker.checks:
        note = c.error or "; ".join(c.problems)
        lines.append(f"| {c.name} | {c.status} | {c.seconds:.1f} s | {note} |")
    lines += ["", "## Measurements", ""]
    for c in checker.checks:
        shown = {k: v for k, v in c.measurements.items() if k != "traceback"}
        if not shown:
            continue
        lines.append(f"### {c.name}")
        lines.append("")
        lines.append(c.description)
        lines.append("")
        for key, value in shown.items():
            if isinstance(value, float):
                value = f"{value:.6g}"
            lines.append(f"- {key}: {value}")
        lines.append("")
    if failed:
        lines += ["## Failures", ""]
        for c in failed:
            if "traceback" in c.measurements:
                lines += [f"### {c.name}", "", "```", c.measurements["traceback"].rstrip(), "```", ""]
    (out / f"{stem}.md").write_text("\n".join(lines))
    return out / f"{stem}.md", out / f"{stem}.json"


def main(argv=None):
    parser = argparse.ArgumentParser(prog="panda-check", description=__doc__.splitlines()[1])
    parser.add_argument("hostname", help="the robot's address")
    parser.add_argument("--desk-user", help="unlock and activate FCI through Desk, and restore after")
    parser.add_argument("--desk-password", help="otherwise asked for")
    parser.add_argument("--gripper", action="store_true", help="also check the Franka Hand (it moves)")
    parser.add_argument("--no-motion", action="store_true", help="only the checks that do not move")
    parser.add_argument("--yes", action="store_true", help="do not ask before moving")
    parser.add_argument("--out", default=".", help="directory for the report")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.WARNING)

    print(f"panda-check {panda_py.__version__}, libfranka {libfranka.__version__}")
    if not args.no_motion and not args.yes:
        print("\n  The robot will move to its start pose and make small motions around it"
              "\n  (a few centimetres, up to 0.15 rad per joint)" +
              (", and the gripper will home and open." if args.gripper else ".") +
              "\n  Keep the workspace clear and the user stop within reach.")
        if input("  Type YES to continue: ") != "YES":
            print("  aborted")
            return 1

    checker = Checker(args)
    desk = None
    try:
        checker.run("environment", "panda-py, libfranka and the host.", checker.environment,
                    needs_robot=False)
        if args.desk_user:
            password = args.desk_password or getpass.getpass("  Desk password: ")

            def take_over(check):
                nonlocal desk
                desk = panda_py.Desk(args.hostname, args.desk_user, password)
                desk.unlock()
                desk.activate_fci()

            checker.run("desk", "Logs in to Desk, unlocks the joints, activates FCI.",
                        take_over, needs_robot=False)
        checker.run("connect", "Connects, reads a state and checks it.", checker.connect)
        checker.run("model", "The dynamics model, and panda_py.fk against the robot's pose.",
                    checker.model)
        checker.run("settings", "Default behaviour, control options, joint walls.",
                    checker.settings)
        checker.run("recovery", "Automatic error recovery.", checker.recovery)
        checker.run("move_to_start", "move_to_start.", checker.move_to_start, moves=True)
        checker.run("JointImpedance", "Joint 7 by 0.15 rad and back.", checker.joint_impedance,
                    moves=True)
        checker.run("JointVelocity", "Joint 7 at +-0.1 rad/s for 1 s each.",
                    checker.joint_velocity, moves=True)
        checker.run("JointTorque", "Zero torque with damping for 1.5 s: the arm should hold.",
                    checker.joint_torque, moves=True)
        checker.run("TaskImpedance", "The end effector 3 cm up and back.", checker.task_impedance,
                    moves=True)
        checker.run("TaskWrench", "Zero wrench with damping for 1.5 s.", checker.task_wrench,
                    moves=True)
        checker.run("TaskForce", "Zero wrench in free air for 1.5 s.", checker.task_force,
                    moves=True)
        checker.run("guards", "The force guard, trip() and rearm() on TaskImpedance.",
                    checker.guards, moves=True)
        checker.run("motion", "move_to_joint_position and move_to_pose, out and back.",
                    checker.motion, moves=True)
        checker.run("ik", "panda_py.ik for a pose 5 cm away, moved to and back.", checker.ik,
                    moves=True)
        checker.run("teaching", "Teaching mode on for 1 s, then off.", checker.teaching,
                    moves=True)
        if args.gripper:
            checker.run("gripper", "Gripper state, homing, half open, fully open.",
                        checker.gripper, moves=True)
    finally:
        if checker.panda is not None:
            checker._recover()  # pylint: disable=protected-access
            del checker.panda
            checker.panda = None
        if desk is not None:
            for call in (desk.deactivate_fci, desk.lock, desk.release_control):
                try:
                    call()
                except Exception:  # pylint: disable=broad-except
                    pass
        markdown, data = write_report(checker, pathlib.Path(args.out))
        print(f"\n  report: {markdown}\n  data:   {data}")
    return 0 if all(c.status != "failed" for c in checker.checks) else 1


if __name__ == "__main__":
    sys.exit(main())
