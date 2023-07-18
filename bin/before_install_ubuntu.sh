#!/bin/bash

sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

# For legacy versions, use my patched repository
repo="https://github.com/frankaemika/libfranka.git"
if [[ "$LIBFRANKA_VER" == "0.7.1" || "$LIBFRANKA_VER" == "0.8.0" ]]; then
  repo="https://github.com/JeanElsner/libfranka.git"
fi

sudo apt remove "*libfranka*"
git clone --recursive $repo
cd libfranka
git checkout $LIBFRANKA_VER
git submodule update
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF ..
cmake --build .
cpack -G DEB
sudo dpkg -i libfranka*.deb
