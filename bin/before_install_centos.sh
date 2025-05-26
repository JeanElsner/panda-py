#! /bin/bash

yum install -y eigen3-devel boost-devel openssl-devel

# Function to compare version numbers (returns true if first version >= second version)
version_ge() { 
    test "$(printf '%s\n' "$@" | sort -rV | head -n 1)" == "$1";
}

# Only install these dependencies for libfranka versions >= 0.14.0
if version_ge "$LIBFRANKA_VER" "0.14.0"; then
    git clone --branch 11.2.0 https://github.com/fmtlib/fmt.git
    cd fmt && mkdir fmt && cd fmt
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE ..
    make
    make install
    cd ../..

    git clone --branch 1.0.5 https://github.com/ros/urdfdom_headers.git
    cd urdfdom_headers && mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_LIBDIR=/usr/lib -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    make
    make install
    cd ../..

    git clone --branch 10.0.0 https://github.com/leethomason/tinyxml2.git
    cd tinyxml2 && mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_LIBDIR=/usr/lib -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    make
    make install
    cd ../..

    git clone --branch 1.0.2 https://github.com/ros/console_bridge.git
    cd console_bridge && mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_LIBDIR=/usr/lib -DCMAKE_POLICY_VERSION_MINIMUM=3.5 
    make
    make install
    cd ../..

    git clone --branch 4.0.0 https://github.com/ros/urdfdom.git
    cd urdfdom && mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_LIBDIR=/usr/lib
    make
    make install
    cd ../..

    git clone --recursive --branch v2.7.0 https://github.com/stack-of-tasks/pinocchio.git
    cd pinocchio && mkdir build && cd build
    cmake .. -DBUILD_TESTING=OFF -DBUILD_PYTHON_INTERFACE=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR=/usr/lib
    make -j4
    make install
    cd ../..
fi

git clone https://github.com/pocoproject/poco.git
cd poco
git checkout poco-1.11.0-release
mkdir cmake-build
cd cmake-build
cmake \
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
cmake --build . --config Release
cmake --build . --target install

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
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ..
cmake --build .
make install
