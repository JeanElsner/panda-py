#! /bin/bash

yum install -y openssl-devel
git clone https://github.com/pocoproject/poco.git
cd poco
git checkout poco-1.9.2-release # focal, jammy uses 1.11.0
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

yum install -y eigen3-devel boost-devel
git clone --recursive $repo
cd libfranka
git checkout $LIBFRANKA_VER
git submodule update
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF ..
cmake --build .
make install
