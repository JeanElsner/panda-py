# Changelog

All notable changes to panda-py are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
Releases before 1.0.0 are documented in the
[GitHub releases](https://github.com/JeanElsner/panda-py/releases).

## [1.0.0] - Unreleased

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
