# Changelog

All notable changes to panda-py are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
Releases before 1.0.0 are documented in the
[GitHub releases](https://github.com/JeanElsner/panda-py/releases).

## [Unreleased]

panda-py 2 is one build for every robot, its controllers share one 1 kHz loop
with guards and telemetry, and its kinematics are numerical. See the
[migration guide](https://jeanelsner.github.io/panda-py/migration.html).

### Changed

- **Breaking:** panda-py is built with
  [libfranka-universal](https://github.com/JeanElsner/libfranka/tree/universal),
  a libfranka 0.21.3 that speaks every research interface protocol version
  from 3 to 10 and connects in the one the robot speaks: one build for the
  Franka Emika Robot and the Franka Research 3 on any system version. The
  per-libfranka-version builds are gone. `IncompatibleVersionError` is only
  raised for robots outside protocols 3 to 10;
  `exceptions.LIBFRANKA_FOR_SERVER_VERSION` gives way to
  `SUPPORTED_PROTOCOL_VERSIONS`.
- **Breaking:** the controllers are named after the space they act in and
  share one loop (see Added): `JointPosition` is `JointImpedance`,
  `IntegratedVelocity` `JointVelocity`, `AppliedTorque` `JointTorque`,
  `AppliedForce` `TaskWrench` and `Force` `TaskForce`. Every controller takes
  its command with `set_reference` (and `step_reference` where it applies)
  instead of `set_control`, and the input filters (`filter_coeff`,
  `set_filter()`) are gone.
- **Breaking:** `controllers.CartesianImpedance` is replaced by
  `controllers.TaskImpedance`. Its law is
  `tau = J^T (alpha K e - D J dq) + N M u`, `u = k (q0 - q) - 2 sqrt(k) dq`:
  - The control frame is selectable: `frame="flange"` or `"end_effector"`
    (the default), plus a fixed `frame_transform` relative to it. Pose,
    velocity and Jacobian are all taken at that frame.
  - The orientation error is the axis-angle vector of `q_ref q^-1`, the full
    rotation angle. CartesianImpedance used the error quaternion's vector part,
    about half the angle, so the same rotational stiffness is now about twice
    as stiff.
  - The posture term uses the dynamically consistent projection,
    `N = I - J^T (J M^-1 J^T)^-1 J M^-1`, by default (`nullspace="dynamic"`),
    which cannot perturb the task. `"kinematic"` is the exact
    `I - J^T (J J^T)^-1 J`; CartesianImpedance used a damped pseudo-inverse.
    `"none"` drops the term. The default nullspace stiffness is 10.
  - Stiffness is a diagonal 6-vector, and the damping is `2 zeta sqrt(K)`,
    recomputed whenever the stiffness or damping ratio is set.
  - No Coriolis torque by default (`coriolis=True` adds it), and no input
    filter: the reference is applied as set.
  - `set_control(position, orientation, q_nullspace)` is now
    `set_reference(position, orientation)` and `set_nullspace_target(q)`.
  - The pose of the control frame comes from the robot state (`O_T_EE`, and
    `O_T_EE F_T_EE^-1` for the flange); only the Jacobian and mass matrix
    come from the model.
- **Breaking:** `panda_py.ik` is numerical (see Added); `ik_full` and the
  `q_7` argument are gone, and `ik` raises `IKError` instead of returning NaN.
  `panda_py.fk` places the Franka Hand 0.1034 m from the flange, as libfranka
  does (0.103 m before), and takes another end effector as `F_T_EE`.
- `TaskForce` takes a wrench (force and torque), trips its guard rather than
  raising when the end effector leaves `max_displacement` (`threshold`
  before), and resets its integral while tripped.
- `JointVelocity` adds the reference velocity as feed-forward and clamps its
  integrated reference to the connected robot's joint limits, not the FER's.
- The motion generators plan with the connected robot's limits: on an FR3 they
  used the FER's joint accelerations, above the FR3's 10 rad/s^2.
- `move_to_pose` runs on TaskImpedance (`TaskTrajectory`) at the end-effector
  frame, with the kinematic projection and Coriolis compensation as before.
  Its default rotational stiffness is 20 instead of 40, the same stiffness
  under the new orientation error. The `impedance` argument must be diagonal.

### Added

- One 1 kHz loop for every controller: setters that only take a mutex the
  loop try-locks, so a command takes effect on the next tick and the loop
  never waits; guards; telemetry; and `TorqueController.commanded()`, called
  with the torque actually sent.
- Guards in every controller, `set_guard()`: external force above a threshold
  for a time, a joint torque at its limit for a time, the controller frame's
  speed, joint speeds, and a workspace of up to eight oriented boxes. When one
  trips the loop drops the controller's active term on that tick and keeps the
  damping until `rearm()`; `trip()` raises it from Python and `guard_state`
  says why. `panda_py.safety` has `set_collision_thresholds()` and
  `box_along_axis()`.
- 1 kHz telemetry in every controller: `telemetry=<capacity>` records one
  sample per tick into a lock-free buffer, with the robot state's main fields,
  the controller's reference, gains and terms, the law's torque and the torque
  sent. `read_telemetry()` drains it; `panda_py.telemetry` has a background
  `Recorder`, `check()` for gaps and npz `save()`/`load()`.
- `TaskImpedance`: `step_reference()` for incremental references, `set_leash()`
  to keep the reference near the current pose, `get_snapshot()`, an energy (or
  impulse) tank that gates the active wrench (`set_tank()`), a joint-space
  spring and damper per joint (`set_joint_spring()`), Coulomb friction
  compensation (`set_friction_compensation()`), rotor inertia in the posture
  term (`set_nullspace_armature()`), and `compute()`, `tank_step()` and
  `step_reference_update()` to replay a law offline.
- `JointImpedance.step_reference()`; `JointVelocity(command_timeout=)`, which
  zeroes the velocity when commands stop arriving.
- `Panda.limits`, `panda_py.limits(server_version)` and
  `panda_py.conservative_limits()`: a robot's type (`RobotType.FER` or `FR3`),
  joint envelope and joint and Cartesian velocity and acceleration limits. The
  trajectory generators take `limits=`.
- Numerical inverse kinematics, `panda_py.ik(pose, q_init, limits=, F_T_EE=)`:
  damped least squares within the joint limits, the redundancy drawn toward
  `q_init`, deterministic restarts; `IKError` carries the best `IKResult`.
  `panda_py.jacobian(q, F_T_EE)`.
- `panda-check <robot-ip>`: exercises panda-py on a robot with small motions
  and writes a Markdown and JSON report with the robot, its protocol version,
  panda-py's and libfranka's versions and every measurement.
- `panda_py.libfranka.__version__`, the libfranka panda-py was built with.
- `Panda.set_joint_walls()` and `Panda.get_joint_walls()`: the virtual joint
  walls can be switched off, also while a controller runs; on by default.
- `Panda.set_control_options()` and `Panda.get_control_options()`: panda-py's
  torque rate limit on or off, and libfranka's `limit_rate` and low-pass
  cutoff for the next controller.
- Tests that run every controller, `move_to_joint_position` and `panda-check`
  against a fake control unit as an FER and an FR3.

### Fixed

- Destroying a `Panda` after the robot went away aborted the process: the
  destructor read the robot state, which threw inside a destructor.
- `move_to_joint_position` reported success short of the goal: it compared
  relative to the size of the joint vector instead of the largest joint
  error, as its warning said.
- A motion to where the robot already is, all waypoints the same, made the
  trajectory planner retry until its timeout (30 s) and fail.

## [1.1.1] - 2026-10-01

Fixes a deadlock in `stop_controller()` and control loop stalls caused by
`get_log()`, both in 1.1.0 and earlier. Upgrade if you log during control or
stop controllers.

### Fixed

- `stop_controller()` could deadlock. It waited for the control thread while
  holding the GIL, and the control thread takes the GIL to log, for instance
  when its loop ends with an error such as a reflex during the stop, so neither
  could proceed. It now releases the GIL while waiting, as does the `Panda`
  destructor, which takes the same path.
- `get_log()` stalled the 1 kHz control loop for as long as it took to copy the
  log, because both used the same lock; with a 60 s log that was long enough
  for the robot to abort the motion. The log now has its own lock, which the
  control loop only tries to take, so a sample arriving during a read is not
  logged instead of the loop waiting. Reading 17,500 samples during control
  now leaves a 3.6 ms gap between commands, against 23.8 ms before.

## [1.1.0] - 2026-09-30

Fixes the segfault in every `move_to_start`, `move_to_joint_position` and
`move_to_pose` call in 1.0.0 (#59), so anyone using motion on 1.0.0 should
upgrade. Also names the panda-py build to install when the robot speaks another
protocol version, and warns when realtime scheduling is unavailable.

### Added

- `panda_py.IncompatibleVersionError`, raised when the robot speaks a different
  research interface protocol version than the libfranka panda-py was built
  with. libfranka's `IncompatibleVersionException` used to reach Python as a
  plain `RuntimeError`, losing the robot's version. The new error carries
  `server_version` and `library_version`, and its message names the panda-py
  build to install. It derives from `RuntimeError`, so existing handlers still
  catch it. `panda_py.exceptions.LIBFRANKA_FOR_SERVER_VERSION` maps each protocol
  version to the libfranka version of its build.
- `Panda` warns on connect when the process cannot obtain realtime scheduling,
  or when the kernel is not a realtime kernel. libfranka always tries to put the
  control thread on `SCHED_FIFO` but only raises about it when the
  `RealtimeConfig` is `kEnforce`, and panda-py defaults to `kIgnore` so that
  gentle motions work on a stock kernel. Both conditions therefore failed
  silently, and the symptom the robot produces,
  `communication_constraints_violation`, points at the network rather than the
  scheduler.
- `panda_py.realtime_priority_available()`, returning libfranka's verdict and
  its message, to check without a robot attached. The kernel half of the same
  check was already available as `libfranka.has_realtime_kernel()`.

### Changed

- `move_to_pose` judges success by absolute tolerances: `success_threshold` is
  now a distance in metres, 0.02 by default, and the new `orientation_threshold`
  a rotation angle in radians, 0.1 by default. It used Eigen's relative
  `isApprox`, so the position tolerance was 1% of the goal's distance from the
  robot base, the orientation was compared as raw quaternion coefficients,
  about 1.1 degrees and sensitive to their sign, and the warning reported only
  the position although the orientation usually decided it. The Cartesian
  controller is an impedance controller without integral action and settles a
  few millimetres and degrees short wherever friction balances its spring; on an
  FER at the default impedance that was up to 7.7 mm and 3 degrees, which
  returned `False` on most moves. The warning now reports both errors and both
  thresholds. `move_to_joint_position` is unchanged.
- `Desk` now detects whether the robot serves the FER or the FR3 brake endpoints
  instead of relying on the `platform` argument, which is now optional. The two
  robots serve mutually exclusive endpoints and answer 404 for the other's
  without touching the brakes, so `lock()` and `unlock()` retry on the other
  endpoint and remember the result. Previously an FR3 user who did not pass
  `platform="fr3"` got an HTML 404 page as an error message. The argument is
  still honoured as a hint that avoids the extra request, and a mismatch between
  it and the robot is logged as a warning.

### Fixed

- `move_to_start`, `move_to_joint_position` and `move_to_pose` segfaulted in
  1.0.0, on every Python version and libfranka build. They build their
  trajectory with the GIL released, and the trajectory constructors then
  released it a second time, which crashes on a thread that does not hold it.
  The release is now nested inside the constructors' own acquire, so it is
  balanced whether or not the caller holds the GIL. `JointTrajectory` and
  `CartesianTrajectory` are also bound with the GIL released, so constructing
  one from Python no longer blocks other threads and the tests exercise the same
  path the robot does. Trajectories built directly from Python were unaffected,
  which is why the test suite passed.
- A `speed_factor` of zero made trajectory generation, and with it every
  `move_to_*` call, loop forever while allocating memory until the process was
  killed; `timeout` did not stop it and neither did Ctrl-C. Very small factors
  did the same in effect. `JointTrajectory`, `CartesianTrajectory` and the
  `move_to_*` methods now raise `ValueError` for a `speed_factor` below 0.001 or
  not finite. At 0.001 a move across the full joint range takes 43 minutes and
  computes in under half a second.

## [1.0.0] - 2026-08-12

Adds support for the libfranka versions used by current Franka Research 3 system
software, while keeping the Franka Emika Robot (Panda) line on libfranka 0.9.2.

### Added

- Support for libfranka 0.13.2, 0.13.6, 0.14.2, 0.17.0 and 0.21.3. Versions from
  0.14.0 onwards compute the robot's dynamic model with Pinocchio, which the
  build now provides.
- Wheels for Python 3.13 and 3.14.
- Prebuilt container images with libfranka and its dependencies installed,
  published to GHCR and used to build the wheels. See `Dockerfile`.
- All nine overloads of `libfranka.Robot.control`. Previously only the one
  taking a `Torques` callback was bound, so a callback returning
  `JointPositions`, `JointVelocities`, `CartesianPose` or `CartesianVelocities`
  failed with a conversion error (#44). The torque controllers remain the
  recommended interface.
- `Panda.is_moving()` and `Panda.refresh_state()`. The state getters now read the
  robot when no controller is running, instead of returning a cached state that
  could be arbitrarily stale.
- Joint limits for the FR3, both the original set and the wider one introduced
  with robot system 5.9.0, exported from `panda_py.constants` alongside the FER
  set, plus `Panda.get_joint_limits_lower()` and `get_joint_limits_upper()` for
  the set in use.

### Changed

- Built against the current pybind11, which adds support for numpy 2. This
  required dropping Python 3.7 and 3.8.
- Wheels are now `manylinux_2_28` rather than `manylinux2014`, raising the
  minimum glibc to 2.28. manylinux2014 is based on CentOS 7, end of life since
  June 2024, and numpy 2 does not publish wheels for it either.
- Wheels shrank from about 12 MB to about 2 MB. Poco was previously built
  without a `CMAKE_BUILD_TYPE` and left unstripped, which added roughly 35 MB of
  symbols to every wheel.
- `LIBFRANKA_VER` is derived from the libfranka that CMake finds instead of being
  set by hand in two places that could disagree.

### Removed

- Support for Python 3.7 and 3.8.
- `bin/before_install_ubuntu.sh`. It only installed Poco and Eigen, so it could
  not build libfranka 0.14.0 or later. Build against a system libfranka or use
  the container images described in `CONTRIBUTING.md`.

### Fixed

- Virtual walls had no damping in the PD zone. `kPDZoneDamping` was initialised
  from itself instead of from `kPDZoneDampingData`, so the value was never set.
- `motion.CartesianTrajectory` constructed from a list of poses left its
  trajectory unset, so using the resulting object crashed. The constructor
  created a temporary instead of initialising the object.
- `controllers.CartesianImpedance.set_control` ignored its `q_nullspace`
  argument. The nullspace target was never propagated out of the filter update,
  so it stayed at the configuration the controller started in.
- `controllers.CartesianImpedance.set_impedance` left the controller
  mis-damped. The damping was derived from the previous stiffness rather than
  the one being set, and the filter never corrected it.
- Cumulative path section lengths omitted half of every blend segment, so
  trajectories built with a non-zero `max_deviation` resolved positions to the
  wrong waypoint and interpolated the wrong orientation.
- `move_to_joint_position` and `move_to_pose` reported a motion as complete as
  soon as the robot stopped moving, which with impedance control can be well
  short of the goal (#49). They now also require the goal to be reached, within
  a one second settling window, and log a warning with the remaining error when
  it is not.
- The virtual walls used the FER joint envelope on every robot, so on an FR3
  they threw from inside the control loop for legal configurations, joint 6
  above 3.7525 rad in particular, and the robot stopped until recovery. The
  envelope is now chosen from the robot's server version.
- Several data races between the control thread and the caller: `_setState`
  unlocked a mutex its own guard still owned, `stopController` read the robot
  state without the mutex, and the pending exception was passed between threads
  unsynchronised.
- `Desk` raised `ConnectionError` on any response other than 200, so endpoints
  answering 204 No Content, such as releasing a control token, appeared to fail.
- The nullspace stiffness was read from the control thread without holding the
  mutex that guards it.
- Both trajectory generators released the GIL and then threw from inside that
  region, so a failed trajectory computation segfaulted the interpreter on
  Python 3.9 through 3.11 instead of raising. They also accepted non-finite
  waypoints, mismatched position and orientation lists, and single waypoints,
  each of which crashed further down; these now raise `ValueError`.
- `bin/build.sh` left `pyproject.toml` pinned to a nonexistent libfranka version
  after every run, because its reset path reused the package version as the
  libfranka version.
