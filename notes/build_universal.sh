#!/bin/bash
# Builds panda-py against the universal libfranka fork, for trying it on a robot.
#
#   1. panda-py-build:0.21.3-universal: the libfranka 0.21.3 build image with the
#      fork installed over the stock libfranka, built with the same options as
#      bin/before_install_centos.sh.
#   2. A cp313 panda-py wheel built in that image, with the test suite run.
#   3. .venv-universal with that wheel installed.
#
# Needs docker, uv, and a checkout of the fork (branch universal of
# github.com/JeanElsner/libfranka). Run from the panda-py root:
#
#   notes/build_universal.sh [path/to/libfranka]
set -euo pipefail

LIBFRANKA="$(realpath "${1:-$HOME/dev/libfranka}")"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE=panda-py-build:0.21.3-universal
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

echo "== image $IMAGE from $LIBFRANKA ($(git -C "$LIBFRANKA" rev-parse --short HEAD))"
rsync -a --exclude 'build*' --exclude '.git' "$LIBFRANKA/" "$WORK/libfranka/"
cp "$ROOT/bin/cmake/franka_find_deps.cmake" "$WORK/"
cat > "$WORK/Dockerfile" <<'EOF'
FROM panda-py-build:0.21.3-ml228
COPY libfranka /tmp/libfranka
COPY franka_find_deps.cmake /tmp/franka_find_deps.cmake
RUN cmake -S /tmp/libfranka -B /tmp/libfranka/build -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_PROJECT_INCLUDE=/tmp/franka_find_deps.cmake \
 && cmake --build /tmp/libfranka/build -j"$(nproc)" \
 && cmake --install /tmp/libfranka/build --strip \
 && rm -rf /tmp/libfranka /tmp/franka_find_deps.cmake
EOF
docker build -q -t "$IMAGE" "$WORK"

echo "== wheel"
CIBW_MANYLINUX_X86_64_IMAGE="$IMAGE" CIBW_BEFORE_ALL= CIBW_ENVIRONMENT=LIBFRANKA_VER=0.21.3 \
  CIBW_BUILD=cp313-manylinux_x86_64 CIBW_TEST_REQUIRES="pytest pytest-timeout" \
  CIBW_TEST_COMMAND="pytest {project}/tests -q -p no:cacheprovider --timeout 60" \
  uvx cibuildwheel --platform linux --output-dir "$WORK/wheelhouse" "$ROOT"

echo "== .venv-universal"
uv venv -q --allow-existing "$ROOT/.venv-universal" --python 3.13
uv pip install -q --python "$ROOT/.venv-universal/bin/python" --reinstall-package panda-python \
  "$WORK"/wheelhouse/*.whl
"$ROOT/.venv-universal/bin/python" -c "import panda_py; print('panda-py', panda_py.__version__)"
