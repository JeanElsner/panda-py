#! /bin/bash
# Builds libfranka and its dependencies inside the manylinux container.
#
# Usage: before_install_centos.sh [all|system|pinocchio|poco|libfranka]
#
# The default, "all", is what cibuildwheel's before-all runs. The individual
# stages exist so Dockerfile can build each one as its own cached layer; keeping
# a single recipe here avoids the build instructions drifting between the two.
#
# Fail loudly: this recipe is up to ten stages deep, and without -e a failed
# stage is skipped silently, leaving libfranka to link against whatever happens
# to be installed.
set -euo pipefail

: "${LIBFRANKA_VER:?must be set, e.g. LIBFRANKA_VER=0.9.2}"

# Resolve patches relative to this script, not the caller's working directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH_DIR="$SCRIPT_DIR/patches"

# Build out of tree. cibuildwheel runs before-all with the project directory as
# the working directory, so cloning into "." would litter the source tree.
WORK_DIR="${WORK_DIR:-$(mktemp -d)}"

# Function to compare version numbers (returns true if first version >= second version)
version_ge() {
    test "$(printf '%s\n' "$@" | sort -rV | head -n 1)" == "$1";
}

install_system() {
    yum install -y eigen3-devel openssl-devel wget
}

# Dependencies of libfranka >= 0.14.0, which computes its dynamic model with
# Pinocchio. Everything is built static and position independent so it can be
# linked into the extension modules.
install_pinocchio_chain() {
    cd "$WORK_DIR"

    # 1. Boost 1.77.0
    #
    # The release tarball avoids cloning ~150 Boost submodules, and Pinocchio
    # only needs three compiled libraries (the rest of Boost is header-only as
    # far as this build is concerned):
    #   find_dependency(Boost REQUIRED COMPONENTS filesystem serialization system)
    wget -q https://archives.boost.io/release/1.77.0/source/boost_1_77_0.tar.bz2
    tar xf boost_1_77_0.tar.bz2
    cd boost_1_77_0
    ./bootstrap.sh --prefix=/usr/local \
        --with-libraries=filesystem,serialization,system
    ./b2 install -j$(nproc) \
        --with-filesystem --with-serialization --with-system
    cd .. && rm -rf boost_1_77_0 boost_1_77_0.tar.bz2

    # 2. TinyXML2

    git clone --depth 1 --branch 10.0.0 https://github.com/leethomason/tinyxml2.git
    cd tinyxml2
    mkdir build && cd build

    cmake .. \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
      -DBUILD_SHARED_LIBS=OFF \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_INSTALL_LIBDIR=/usr/lib

    make -j$(nproc)
    make install

    cd ../.. && rm -rf tinyxml2

    # 3. console_bridge
    #
    # Deliberately unpinned: the 1.0.2 release predates CMake 4 and its
    # cmake_minimum_required is too low to configure. Tracking master makes the
    # build non-reproducible, so pin this to a commit once one is known good.

    git clone --depth 1 --branch master https://github.com/ros/console_bridge.git
    cd console_bridge
    mkdir build && cd build

    cmake .. \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
      -DBUILD_SHARED_LIBS=OFF \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_INSTALL_LIBDIR=/usr/lib

    make -j$(nproc)
    make install

    cd ../..
    rm -rf console_bridge

    # 4. urdfdom_headers

    git clone --depth 1 --branch 1.1.2 https://github.com/ros/urdfdom_headers.git
    cd urdfdom_headers
    mkdir build && cd build

    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_INSTALL_LIBDIR=/usr/lib

    make -j$(nproc)
    make install

    cd ../..
    rm -rf urdfdom_headers

    # 5. urdfdom (with patch)

    git clone --depth 1 --branch 4.0.0 https://github.com/ros/urdfdom.git
    cd urdfdom

    git apply "$PATCH_DIR/urdfdom.patch"

    mkdir build && cd build
    cmake .. \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
      -DBUILD_SHARED_LIBS=OFF \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_INSTALL_LIBDIR=/usr/lib

    make -j$(nproc)
    make install
    cd ../..
    rm -rf urdfdom

    # 6. Assimp

    git clone --depth 1 --recurse-submodules --shallow-submodules \
        --branch v5.4.3 https://github.com/assimp/assimp.git
    cd assimp && mkdir build && cd build
    cmake .. -DBoost_USE_STATIC_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DBUILD_SHARED_LIBS=OFF -DASSIMP_BUILD_TESTS=OFF \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_INSTALL_LIBDIR=/usr/lib
    make -j$(nproc) && make install
    cd ../.. && rm -rf assimp

    # 7. Pinocchio (with patch)

    git clone --depth 1 --recurse-submodules --shallow-submodules \
        --branch v3.4.0 https://github.com/stack-of-tasks/pinocchio.git
    cd pinocchio && git apply "$PATCH_DIR/pinocchio.patch"
    mkdir build && cd build
    cmake .. -DBoost_USE_STATIC_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DBUILD_SHARED_LIBS=OFF -DBUILD_PYTHON_INTERFACE=OFF \
        -DBUILD_DOCUMENTATION=OFF -DBUILD_TESTING=OFF \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_INSTALL_LIBDIR=/usr/lib
    make -j$(nproc) && make install
    cd ../.. && rm -rf pinocchio

    # 8. fmt

    git clone --depth 1 --branch 11.2.0 https://github.com/fmtlib/fmt.git
    cd fmt && mkdir build && cd build
    cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE \
        -DCMAKE_BUILD_TYPE=Release -DFMT_TEST=OFF
    make -j$(nproc)
    make install
    cd ../.. && rm -rf fmt
}

install_poco() {
    cd "$WORK_DIR"
    git clone --depth 1 --branch poco-1.11.0-release https://github.com/pocoproject/poco.git
    cd poco
    mkdir cmake-build
    cd cmake-build
    # CMAKE_BUILD_TYPE must be set at configure time: --config is a no-op for the
    # Makefile generator, and without it Poco builds unoptimized and unstripped,
    # which previously added ~35 MB of debug symbols to every wheel.
    cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_ENCODINGS=OFF \
    -DENABLE_ENCODINGS_COMPILER=OFF \
    -DENABLE_XML=ON \
    -DENABLE_JSON=ON \
    -DENABLE_MONGODB=OFF \
    -DENABLE_DATA_SQLITE=OFF \
    -DENABLE_REDIS=OFF \
    -DENABLE_PDF=OFF \
    -DENABLE_UTIL=ON \
    -DENABLE_NET=ON \
    -DENABLE_SEVENZIP=OFF \
    -DENABLE_ZIP=OFF \
    -DENABLE_CPPPARSER=OFF \
    -DENABLE_POCODOC=OFF \
    -DENABLE_PAGECOMPILER=OFF \
    -DENABLE_PAGECOMPILER_FILE2PAGE=OFF \
    -DENABLE_ACTIVERECORD=OFF \
    -DENABLE_ACTIVERECORD_COMPILER=OFF ..
    cmake --build . -j$(nproc)
    cmake --install . --strip
    cd ../.. && rm -rf poco
}

install_libfranka() {
    cd "$WORK_DIR"
    # For legacy versions, use my patched repository
    repo="https://github.com/frankaemika/libfranka.git"
    if [[ "$LIBFRANKA_VER" == "0.7.1" || "$LIBFRANKA_VER" == "0.8.0" ]]; then
      repo="https://github.com/JeanElsner/libfranka.git"
    fi

    git clone --recursive $repo
    cd libfranka
    git checkout $LIBFRANKA_VER
    git submodule update
    mkdir build && cd build

    # libfranka 0.14.x-0.17.x reach Pinocchio's urdfdom export without finding
    # console_bridge, so its imported target is missing. The injected file only
    # makes QUIET lookups, so it is a no-op for versions that do not need it.
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_PROJECT_INCLUDE="$SCRIPT_DIR/cmake/franka_find_deps.cmake" ..
    cmake --build . -j$(nproc)
    cmake --install . --strip
    cd ../.. && rm -rf libfranka
}

case "${1:-all}" in
  system)    install_system ;;
  pinocchio) install_pinocchio_chain ;;
  poco)      install_poco ;;
  libfranka) install_libfranka ;;
  all)
    install_system
    # Only needed for libfranka versions >= 0.14.0
    if version_ge "$LIBFRANKA_VER" "0.14.0"; then
      install_pinocchio_chain
    fi
    install_poco
    install_libfranka
    ;;
  *)
    echo "usage: $0 [all|system|pinocchio|poco|libfranka]" >&2
    exit 2
    ;;
esac
