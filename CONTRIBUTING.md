# Contributing

Thanks for considering a contribution. This document covers the parts of the
project that are not obvious from the source: how to build against a specific
libfranka, which libfranka versions belong in the matrix, and how a release is
put together.

## Getting set up

Formatting and hygiene checks run through [pre-commit](https://pre-commit.com):

```
pip install pre-commit
pre-commit install
```

Python is formatted with black and C++ with clang-format, both configured in the
repository. Two paths are excluded on purpose: `bin/patches`, where stripping
whitespace would stop the patches applying, and the stubs under `src/panda_py`,
which `bin/gen_stubs.sh` generates.

The pybind11 binding chains in `src/_core.cpp` and `src/libfranka.cpp` are
wrapped in `// clang-format off`/`on`. Their hand alignment is deliberate; please
keep new bindings inside the guarded region and matching the surrounding style.

## Building

panda-py needs libfranka, and the libfranka version determines which robots the
result can talk to. There are two ways to get one.

### Against a system libfranka

If libfranka and its dependencies are already installed, an editable install
works:

```
pip install -e .
```

The version guards in the C++ are derived by CMake from the libfranka that
`find_package` actually finds, so nothing has to be configured by hand. CMake
warns if the version found differs in the minor version from the `LIBFRANKA_VER`
in the environment.

Installing libfranka itself is left to you. From 0.14.0 on it pulls in
Pinocchio, and with it Boost, urdfdom, Assimp, TinyXML2 and console_bridge, so
the container images below are usually less work.

### Against a container image

The images used by CI carry libfranka and its dependencies preinstalled, which
avoids building Pinocchio and friends locally. Build one, then point
cibuildwheel at it:

```
docker build -t panda-py-build:0.9.2 --build-arg LIBFRANKA_VER=0.9.2 .

CIBW_MANYLINUX_X86_64_IMAGE=panda-py-build:0.9.2 \
CIBW_BEFORE_ALL= \
CIBW_BUILD="cp312-*" \
CIBW_ENVIRONMENT=LIBFRANKA_VER=0.9.2 \
  python -m cibuildwheel --platform linux
```

For libfranka 0.14.0 and later add `--build-arg DEPS=modern`, which adds the
Pinocchio chain. The published images live at
`ghcr.io/jeanelsner/panda-py-build:<libfranka version>`.

## Tests

The suite needs no robot. cibuildwheel runs it against each wheel it builds, so
the command above exercises it; to run it directly:

```
pytest tests
```

Please add coverage for anything that can be checked without hardware, which is
most of the kinematics, the trajectory generation and the `Desk` client. Two
things are worth knowing when touching the C++:

- Test on Python 3.9, not just a recent one. Some mistakes only show up there,
  most notably throwing while the GIL is released, which segfaults on 3.9
  through 3.11 and appears to work on 3.12 and later.
- Anything reachable from the control loop runs at 1 kHz on a real-time thread.
  State shared with the caller needs the mutex, and nothing in that path should
  allocate or block if it can be avoided.

## Type stubs

The stubs in `src/panda_py` are generated, not written. After changing a
binding, regenerate them against the libfranka version shipped on PyPI:

```
./bin/gen_stubs.sh
```

## Documentation

```
cd docs && make html
```

`bin/gen_docs_api.sh` regenerates the sphinx apidoc `rst` files, which is only
needed when modules are added or removed.

## Adding a libfranka version

Compatibility with a robot is decided by the research interface protocol
version, not by the libfranka release. Every libfranka release within a band
speaks the same protocol, so shipping more than one of them adds build time
without supporting a single additional robot.

The rule is therefore **one wheel per protocol version, built from the newest
libfranka release in that band**:

| Protocol | libfranka        | Robot system      |
| -------- | ---------------- | ----------------- |
| 3        | 0.7.1            | FER >= 3.0.0      |
| 4        | 0.8.0            | FER >= 4.0.0      |
| 5        | 0.9.1 to 0.9.2   | FER >= 4.2.1      |
| 6        | 0.10.0 to 0.13.2 | FR3 >= 5.2.0      |
| 7        | 0.13.3 to 0.14.0 | FR3 >= 5.5.0      |
| 8        | 0.14.1 to 0.14.2 | FR3 >= 5.7.0      |
| 9        | 0.15.0 to 0.17.0 | FR3 >= 5.7.2      |
| 10       | 0.18.0 and newer | FR3 >= 5.9.0      |

The protocol version is the `kVersion` constant in
`common/include/research_interface/robot/service_types.h` in the libfranka
sources. A new libfranka release only warrants a new matrix row if it bumps that
constant; otherwise the existing row should move to the newer release.

Adding or moving a row means editing the matrix in all four workflows and the
compatibility table in `README.md`. `.github/workflows/build_images.yml` also
needs the `deps` flavour, `legacy` below 0.14.0 and `modern` from 0.14.0 on.
Wheel builds fail until the image for a row exists, so let that workflow finish
first.

## Releasing

1. Update `version` in `pyproject.toml` and `__version__` in
   `src/panda_py/__init__.py`, and move the unreleased section of
   `CHANGELOG.md` under the new version.
2. Merge to `main`. If the build recipe or the patches changed,
   `build_images.yml` republishes the images.
3. Publish a GitHub release with the new tag. `release.yml` then builds every
   matrix row, uploads one zip of wheels per libfranka version as a release
   asset, and publishes the default row to PyPI.

The default published to PyPI is the `LIBFRANKA_VER` in `pyproject.toml`, which
is deliberately the newest release supporting the Franka Emika Robot. Everything
else is distributed as release assets, because PyPI has no way to express which
libfranka a wheel was built against.
