"""The controllers' Python API, without a robot: construction, setters and
their validation, the shared guard and telemetry interface."""

import math

import numpy as np
import pytest

from panda_py import controllers

COMMON_TELEMETRY = {
    "tick", "time", "duration", "reference_update", "control_command_success_rate",
    "tau_law", "tau_cmd", "q", "dq", "tau_J", "tau_J_d", "tau_ext_hat_filtered",
    "O_T_EE", "F_T_EE", "O_F_ext_hat_K", "K_F_ext_hat_K", "guard",
}

ALL = {
    "JointImpedance": ({"q_d", "dq_d", "stiffness", "damping", "tau_active", "tau_passive"},
                       lambda c: c.set_reference(np.zeros(7))),
    "JointVelocity": ({"q_d", "dq_d", "stiffness", "damping"},
                      lambda c: c.set_reference(np.full(7, 0.1))),
    "JointTorque": ({"tau_d", "damping"}, lambda c: c.set_reference(np.ones(7))),
    "TaskImpedance": ({"position", "orientation", "wrench_active", "tau_task"},
                      lambda c: c.set_reference(np.zeros(3), np.array([1.0, 0, 0, 0]))),
    "TaskWrench": ({"wrench_d", "damping"}, lambda c: c.set_reference(np.zeros(6))),
    "TaskForce": ({"wrench_d", "tau_ext", "tau_error_integral", "gains", "displacement"},
                  lambda c: c.set_reference(np.array([0, 0, -5.0, 0, 0, 0]))),
}


@pytest.mark.parametrize("name", sorted(ALL))
def test_every_controller_has_the_shared_interface(name):
    own_fields, set_reference = ALL[name]
    ctrl = getattr(controllers, name)(telemetry=100)
    assert isinstance(ctrl, controllers.TorqueController)
    set_reference(ctrl)
    telemetry = ctrl.read_telemetry()
    assert COMMON_TELEMETRY | own_fields <= set(telemetry)
    assert all(len(v) == 0 for v in telemetry.values())
    assert ctrl.telemetry_capacity == 100
    assert ctrl.telemetry_dropped == 0
    ctrl.set_guard(force=80.0, speed=0.5)
    assert ctrl.get_guard()["force"] == 80.0
    assert ctrl.guard_state == {"tripped": False, "reason": "none", "time": 0.0,
                                "value": 0.0, "joint": -1}
    ctrl.trip()
    ctrl.rearm()


@pytest.mark.parametrize("name", ["JointImpedance", "JointVelocity"])
def test_joint_gains(name):
    ctrl = getattr(controllers, name)()
    ctrl.set_stiffness(np.full(7, 100.0))
    ctrl.set_damping(np.full(7, 10.0))
    np.testing.assert_array_equal(ctrl.get_stiffness(), 100.0)
    np.testing.assert_array_equal(ctrl.get_damping(), 10.0)
    with pytest.raises(ValueError):
        ctrl.set_stiffness(np.full(7, -1.0))
    with pytest.raises(ValueError):
        ctrl.set_damping(np.full(7, np.nan))


def test_references_must_be_finite():
    with pytest.raises(ValueError):
        controllers.JointImpedance().set_reference(np.full(7, np.inf))
    with pytest.raises(ValueError):
        controllers.JointVelocity().set_reference(np.full(7, np.nan))
    with pytest.raises(ValueError):
        controllers.JointTorque().set_reference(np.full(7, np.nan))
    with pytest.raises(ValueError):
        controllers.TaskWrench().set_reference(np.full(6, np.inf))
    with pytest.raises(ValueError):
        controllers.TaskForce().set_reference(np.full(6, np.nan))


def test_joint_velocity_timeout():
    ctrl = controllers.JointVelocity(command_timeout=0.2)
    assert ctrl.get_command_timeout() == 0.2
    ctrl.set_command_timeout(1.0)
    assert ctrl.get_command_timeout() == 1.0
    assert math.isinf(controllers.JointVelocity().get_command_timeout())
    with pytest.raises(ValueError):
        ctrl.set_command_timeout(0.0)


def test_task_force_parameters():
    ctrl = controllers.TaskForce(k_p=0.5, k_i=1.0, max_displacement=0.05)
    assert ctrl.get_gains() == (0.5, 1.0)
    ctrl.set_gains(2.0, 3.0)
    assert ctrl.get_gains() == (2.0, 3.0)
    assert ctrl.get_max_displacement() == 0.05
    with pytest.raises(ValueError):
        ctrl.set_gains(-1.0, 0.0)
    with pytest.raises(ValueError):
        ctrl.set_max_displacement(0.0)


@pytest.mark.parametrize("name", ["JointTorque", "TaskWrench", "TaskForce"])
def test_damping(name):
    ctrl = getattr(controllers, name)()
    ctrl.set_damping(np.full(7, 2.0))
    np.testing.assert_array_equal(ctrl.get_damping(), 2.0)
    with pytest.raises(ValueError):
        ctrl.set_damping(np.full(7, -2.0))


def test_the_v1_names_are_gone():
    for name in ("JointPosition", "IntegratedVelocity", "AppliedTorque", "AppliedForce",
                 "Force", "CartesianImpedance"):
        assert not hasattr(controllers, name)
