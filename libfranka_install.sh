#!/bin/bash

# Input the robot firmware version
echo "Please enter the robot firmware version (e.g., 5.7.x or other versions):"
read firmware_version


# Version comparison functions
version_ge() {  # greater than or equal (>=)
    echo "$1 $2" | awk -v ver1="$1" -v ver2="$2" '
    BEGIN {
        n1 = split(ver1, v1, ".");
        n2 = split(ver2, v2, ".");
        n = (n1 > n2) ? n1 : n2;
        for (i = 1; i <= n; i++) {
            if (v1[i] == "") v1[i] = 0;
            if (v2[i] == "") v2[i] = 0;
            if (v1[i] + 0 < v2[i] + 0) exit 1;
            if (v1[i] + 0 > v2[i] + 0) exit 0;
        }
        exit 0;
    }'
    return $?
}

version_lt() {  # less than (<)
    echo "$1 $2" | awk -v ver1="$1" -v ver2="$2" '
    BEGIN {
        n1 = split(ver1, v1, ".");
        n2 = split(ver2, v2, ".");
        n = (n1 > n2) ? n1 : n2;
        for (i = 1; i <= n; i++) {
            if (v1[i] == "") v1[i] = 0;
            if (v2[i] == "") v2[i] = 0;
            if (v1[i] + 0 < v2[i] + 0) exit 0;
            if (v1[i] + 0 > v2[i] + 0) exit 1;
        }
        exit 1;
    }'
    return $?
}

# Use version comparison functions to determine the corresponding libfranka version
if version_ge "$firmware_version" "5.7.2"; then
    libfranka_version="0.15.0"
elif version_ge "$firmware_version" "5.7.0" && version_lt "$firmware_version" "5.7.2"; then
    libfranka_version="0.14.1"
elif version_ge "$firmware_version" "5.5.0" && version_lt "$firmware_version" "5.7.0"; then
    libfranka_version="0.13.3"
elif version_ge "$firmware_version" "5.2.0" && version_lt "$firmware_version" "5.5.0"; then
    libfranka_version="0.12.1"
elif version_ge "$firmware_version" "5.2.0" && version_lt "$firmware_version" "5.3.0"; then
    libfranka_version="0.11.0"
elif version_ge "$firmware_version" "5.2.0" && version_lt "$firmware_version" "5.2.5"; then
    libfranka_version="0.10.0"
elif version_ge "$firmware_version" "4.2.1" && version_lt "$firmware_version" "5.2.0"; then
    libfranka_version="0.9.1"
elif version_ge "$firmware_version" "4.0.0" && version_lt "$firmware_version" "4.2.0"; then
    libfranka_version="0.8.0"
elif version_ge "$firmware_version" "3.0.0" && version_lt "$firmware_version" "4.0.0"; then
    libfranka_version="0.7.1"
elif version_ge "$firmware_version" "1.3.0" && version_lt "$firmware_version" "3.0.0"; then
    libfranka_version="0.5.0"
elif version_ge "$firmware_version" "1.2.0" && version_lt "$firmware_version" "1.3.0"; then
    libfranka_version="0.3.0"
elif version_ge "$firmware_version" "1.1.0" && version_lt "$firmware_version" "1.2.0"; then
    libfranka_version="0.2.0"
else
    echo "Unsupported firmware version!"
    exit 1
fi

echo "Selected libfranka version: $libfranka_version"

# Modify the pyproject.toml file
echo "Modifying pyproject.toml file..."
sed -i "58s/.*/environment = \"LIBFRANKA_VER=$libfranka_version\"/" /pyproject.toml


# Modify the pyproject.toml file in the current directory
echo "Modifying pyproject.toml file..."
sed -i "58s/.*/environment = \"LIBFRANKA_VER=$libfranka_version\"/" ./pyproject.toml

# Install system dependencies
echo "Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev lsb-release curl

# Install Pinocchio dependencies
echo "Installing Pinocchio dependencies..."
sudo mkdir -p /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
sudo apt-get update
sudo apt-get install -y robotpkg-pinocchio

# Set environment variables for OpenRobots
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH 
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

# Remove existing libfranka installations
echo "Removing existing libfranka installations..."
sudo apt-get remove "*libfranka*"

# Clone and checkout the required libfranka version
echo "Cloning the libfranka repository and checking out version $libfranka_version..."
git clone --recurse-submodules https://github.com/frankaemika/libfranka.git
cd libfranka
git checkout $libfranka_version
git submodule update

# Create and enter the build directory
mkdir build
cd build

# Configure and build libfranka
echo "Configuring and building libfranka..."
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..
make

# Install libfranka
echo "Installing libfranka..."
sudo make install

# Update the shared library cache
sudo ldconfig

# Modify the pyproject.toml file to update LIBFRANKA_VER
cd ..
cd ..

# Create Python 3.11 virtual environment
echo "Creating Python 3.11 virtual environment..."
python3.10 -m venv mypandapy

# Activate the virtual environment
echo "Activating the virtual environment..."
source mypandapy/bin/activate

# Install dependencies
echo "Installing dependencies..."
pip install scikit-build-core pybind11
pip install -r requirements.txt
pip install --upgrade pip

# Install Python project
echo "Installing Python project..."
pip install -e .

echo "Installation complete!"
