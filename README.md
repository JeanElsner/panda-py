<div align="center"><img alt="panda-py Logo" src="https://raw.githubusercontent.com/JeanElsner/panda-py/main/logo.png" /></div>

<h1 align="center">panda-py</h1>

<p align="center">
  <a href="https://github.com/JeanElsner/panda-py/actions/workflows/main.yml"><img alt="Continuous Integration" src="https://img.shields.io/github/actions/workflow/status/JeanElsner/panda-py/main.yml" /></a>
  <a href="https://github.com/JeanElsner/panda-py/blob/main/LICENSE"><img alt="Apache-2.0 License" src="https://img.shields.io/github/license/JeanElsner/panda-py" /></a>
  <a href="https://pypi.org/project/panda-python/"><img alt="PyPI - Published Version" src="https://img.shields.io/pypi/v/panda-python"></a>
  <img alt="PyPI - Python Version" src="https://img.shields.io/pypi/pyversions/panda-python">
  <a href="https://jeanelsner.github.io/panda-py"><img alt="Documentation" src="https://shields.io/badge/-Documentation-informational" /><a/>
</p>

Finally, Python bindings for the Franka Emika Robot (Panda) and the Franka Research 3. These will increase your productivity by 1000%, guaranteed[^1]!

## Install

```
pip install panda-python
```

One install for every robot: panda-py 2 is built with
[libfranka-universal](https://github.com/JeanElsner/libfranka/tree/universal),
a libfranka that speaks every research interface protocol version, so it
connects to a Franka Emika Robot (FER, formerly Panda) or a Franka Research 3
(FR3) on any system version below and adapts to it: joint limits, motion
limits and the robot's dynamics model follow the robot connected.

| Robot | Robot system version | Protocol version |
| ---- | ---- | ---- |
| FR3 | >= 5.9.0 | 10 |
| FR3 | >= 5.7.2 | 9 |
| FR3 | >= 5.7.0 | 8 |
| FR3 | >= 5.5.0 | 7 |
| FR3 | >= 5.2.0 | 6 |
| FER | >= 4.2.1 | 5 |
| FER | >= 4.0.0 | 4 |
| FER | >= 3.0.0 | 3 |

Protocol versions 5 and 10 are confirmed on hardware; the others are tested
against a simulated control unit. If panda-py misbehaves with your robot,
please [open an issue](https://github.com/JeanElsner/panda-py/issues) with the
report of `panda-check` (below).

## Getting started

```python
import panda_py
from panda_py import controllers

panda = panda_py.Panda("172.16.0.2")    # the robot's address
panda.move_to_start()

# Move the end effector 10 cm down, then hold it there compliantly.
pose = panda.get_pose()
pose[2, 3] -= 0.1
panda.move_to_joint_position(panda_py.ik(pose, q_init=panda.q, limits=panda.limits))

ctrl = controllers.TaskImpedance()
panda.start_controller(ctrl)
ctrl.set_reference(panda.get_position(), panda.get_orientation())
```

The [tutorial paper](https://www.sciencedirect.com/science/article/pii/S2352711023002285),
the Jupyter [notebooks](https://github.com/JeanElsner/panda-py/tree/main/examples/notebooks)
and the [examples](https://github.com/JeanElsner/panda-py/tree/main/examples) run
directly on your robot. The [documentation](https://jeanelsner.github.io/panda-py/)
has a guide and the full API.

### Controllers

Every controller runs in panda-py's 1 kHz loop, takes its commands
(`set_reference`) on the next tick without ever making the loop wait, and has
the same guards (force, speed, workspace, joint velocity, torque saturation)
and 1 kHz telemetry.

| Joint space | Task space |
| ---- | ---- |
| `JointImpedance`: spring and damper per joint | `TaskImpedance`: Cartesian impedance with a posture term |
| `JointVelocity`: joint velocities | `TaskWrench`: a feed-forward wrench |
| `JointTorque`: feed-forward joint torques | `TaskForce`: wrench regulation |

`move_to_joint_position`, `move_to_pose` and `move_to_start` plan time-optimal
trajectories within the connected robot's limits. `panda_py.fk`, `jacobian` and
`ik` are the robots' kinematics with any end effector; `ik` is numerical,
respects the joint limits and stays near the configuration it starts from.

### Checking a robot

```
panda-check <robot-ip> --desk-user <user>
```

exercises everything above on the robot, with small motions around its start
pose, and writes a report: the robot, its protocol version, panda-py's and
libfranka's versions, and every measurement.

### Upgrading from panda-py 1

panda-py 2 renames the controllers into joint and task space, replaces
`CartesianImpedance` with `TaskImpedance` and the analytic IK with a numerical
one; see the [migration guide](https://jeanelsner.github.io/panda-py/migration.html)
and the [changelog](CHANGELOG.md).

## Extensions

* [franka_desk](https://github.com/geriatronics/franka_desk) Client for the Desk REST API, with a ROS 2 wrapper. Requires an FR3 with robot system version 5.8.0 or newer.

# Citation

If you use panda-py in published research, please consider citing the [original software paper](https://www.sciencedirect.com/science/article/pii/S2352711023002285).

```
@article{elsner2023taming,
title = {Taming the Panda with Python: A powerful duo for seamless robotics programming and integration},
journal = {SoftwareX},
volume = {24},
pages = {101532},
year = {2023},
issn = {2352-7110},
doi = {https://doi.org/10.1016/j.softx.2023.101532},
url = {https://www.sciencedirect.com/science/article/pii/S2352711023002285},
author = {Jean Elsner}
}
```

[^1]: Not actually guaranteed. Based on a sample size of one.
