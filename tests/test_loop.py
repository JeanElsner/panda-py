"""Every controller's 1 kHz loop, end to end against a fake robot.

tests/fake_robot is a fake control unit (from libfranka-universal) that
connects in a given research interface protocol version, streams a robot at
rest at about 1 kHz and takes commands. panda_py.Panda connects to it as to a
robot, so this runs the control loop, the controllers, their commands, guards
and telemetry, without hardware. It runs one protocol version of each robot:
5 (an FER) and 10 (an FR3).
"""

import pathlib
import socket
import subprocess
import sys
import time

import numpy as np
import pytest

import panda_py
from panda_py import controllers

FAKE = pathlib.Path(__file__).parent / "fake_robot" / "fake_robot.py"


def _port_free():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", 1337))
        except OSError:
            return False
    return True


@pytest.fixture(params=[5, 10], ids=["FER", "FR3"])
def robot(request):
    if not _port_free():
        pytest.skip("127.0.0.1:1337 is in use")
    process = subprocess.Popen(
        [sys.executable, str(FAKE), "--version", str(request.param), "--seconds", "60",
         "--still"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    assert process.stdout.readline().strip() == "ready"
    panda = panda_py.Panda("127.0.0.1")
    yield request.param, panda
    panda.stop_controller()
    del panda
    process.kill()
    process.wait()
    process.stdout.close()


def run(panda, controller, seconds=0.3, command=None):
    panda.start_controller(controller)
    time.sleep(0.1)
    if command:
        command(controller)
    time.sleep(seconds)
    telemetry = controller.read_telemetry()
    panda.stop_controller()
    return telemetry


def test_the_robot_type_follows_the_protocol(robot):
    version, panda = robot
    expected = panda_py.RobotType.FER if version <= 5 else panda_py.RobotType.FR3
    assert panda.limits.type == expected
    assert panda.get_robot().server_version() == version


CONTROLLERS = {
    "JointImpedance": lambda c: c.step_reference(np.zeros(7)),
    "JointVelocity": lambda c: c.set_reference(np.zeros(7)),
    "JointTorque": lambda c: c.set_reference(np.zeros(7)),
    "TaskImpedance": lambda c: c.step_reference(np.zeros(3), np.zeros(3)),
    "TaskWrench": lambda c: c.set_reference(np.zeros(6)),
    "TaskForce": lambda c: c.set_reference(np.zeros(6)),
}


@pytest.mark.parametrize("name", sorted(CONTROLLERS))
def test_every_controller_runs(robot, name):
    _, panda = robot
    controller = getattr(controllers, name)(telemetry=5000)
    telemetry = run(panda, controller, command=CONTROLLERS[name])
    ticks = telemetry["tick"]
    assert len(ticks) > 100
    np.testing.assert_array_equal(np.diff(ticks), 1)  # no tick lost
    assert telemetry["reference_update"].sum() == 1
    assert np.isfinite(telemetry["tau_cmd"]).all()
    assert not controller.guard_state["tripped"]


def test_a_trip_drops_the_active_term_until_rearmed(robot):
    _, panda = robot
    controller = controllers.JointTorque(telemetry=5000)
    panda.start_controller(controller)
    controller.set_reference(np.full(7, 0.5))
    time.sleep(0.1)
    controller.trip()
    time.sleep(0.1)
    assert controller.guard_state == {**controller.guard_state, "tripped": True,
                                      "reason": "manual"}
    controller.rearm()
    time.sleep(0.1)
    telemetry = controller.read_telemetry()
    panda.stop_controller()
    tripped = telemetry["guard"] != 0
    assert tripped.any() and not tripped[-10:].any()
    # The robot is at rest, so the law's torque is the feed-forward one or zero.
    np.testing.assert_allclose(telemetry["tau_law"][tripped], 0.0)
    assert np.allclose(telemetry["tau_law"][~tripped][:5], 0.5)
    # After the rearm the feed-forward torque is zero until set again.
    np.testing.assert_allclose(telemetry["tau_law"][-5:], 0.0)


def test_move_to_joint_position_runs_and_reports_the_robot_did_not_follow(robot):
    _, panda = robot
    q = panda.q
    # The fake robot does not move: the controller tracks, gives up after the
    # settle timeout and reports the goal as missed.
    assert panda.move_to_joint_position(q + 0.05, speed_factor=0.5) is False


def test_panda_check_runs_without_motion(tmp_path):
    """panda-check's non-moving checks against the fake robot, and its report."""
    from panda_py import check  # pylint: disable=import-outside-toplevel

    if not _port_free():
        pytest.skip("127.0.0.1:1337 is in use")
    process = subprocess.Popen(
        [sys.executable, str(FAKE), "--version", "10", "--seconds", "30", "--still"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        assert process.stdout.readline().strip() == "ready"
        check.main(["127.0.0.1", "--no-motion", "--out", str(tmp_path)])
    finally:
        process.kill()
        process.wait()
        process.stdout.close()
    report = next(tmp_path.glob("panda-check-FR3-v10-*.md")).read_text()
    assert "research interface protocol version 10" in report
    for name in ("environment", "connect", "model", "settings", "recovery"):
        assert f"| {name} |" in report
    assert "| JointImpedance | skipped |" in report
    assert next(tmp_path.glob("panda-check-FR3-v10-*.json")).stat().st_size > 0
