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

- Support for libfranka 0.12.1, 0.13.6, 0.14.2, 0.15.0 and 0.21.3. Versions from
  0.14.0 onwards compute the robot's dynamic model with Pinocchio, which the
  build now provides.
- Wheels for Python 3.13 and 3.14.
- Prebuilt container images with libfranka and its dependencies installed,
  published to GHCR and used to build the wheels. See `Dockerfile`.

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

### Fixed

- Virtual walls had no damping in the PD zone. `kPDZoneDamping` was initialised
  from itself instead of from `kPDZoneDampingData`, so the value was never set.
- `motion.CartesianTrajectory` constructed from a list of poses left its
  trajectory unset, so using the resulting object crashed. The constructor
  created a temporary instead of initialising the object.
- `controllers.CartesianImpedance.set_control` ignored its `q_nullspace`
  argument. The nullspace target was never propagated out of the filter update,
  so it stayed at the configuration the controller started in.
- The nullspace stiffness was read from the control thread without holding the
  mutex that guards it.
- `bin/build.sh` left `pyproject.toml` pinned to a nonexistent libfranka version
  after every run, because its reset path reused the package version as the
  libfranka version.
