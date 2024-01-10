#!/bin/bash

# Install Python dependencies
python -m pip install packaging toml cibuildwheel

# Set pyproject.toml path
root=$(dirname $0)/..
toml="$root/pyproject.toml"

# Store current version
version=$(python <<END
from packaging.version import parse
import toml
with open('$toml', 'r') as f:
    data = toml.load(f)
    version = parse(data['project']['version'])
print(f'{version.major}.{version.minor}.{version.micro}')
END
)

# Print the version
echo "Current version is $version"

change_version() {
  local pandapy_version="$version+libfranka-$1"
  local libfranka_version="LIBFRANKA_VER=$1"
  local vacuum_gripper="ON"

  if [[ "$1" == "0.7.1" ]]; then
    vacuum_gripper="OFF"
  fi
  echo "VACUUM_GRIPPER: $vacuum_gripper"
  python <<END
import toml

with open('$toml', 'r') as f:
    data = toml.load(f)

data['project']['version'] = '$pandapy_version'
data['tool']['cibuildwheel']['environment'] = '$libfranka_version'
data['tool']['scikit-build']['cmake']['define']['VACUUM_GRIPPER'] = '$vacuum_gripper'

with open('$toml', 'w') as f:
    toml.dump(data, f)
END
}

mkdir $root/archive

# Build wheels for common libfranka versions
libfranka=("0.7.1" "0.8.0" "0.9.2" "0.10.0" "0.11.0" "0.12.1" "0.13.2")
for value in "${libfranka[@]}"; do
  echo "Changing libfranka version in pyproject.toml to: $value"
  change_version "$value"
  export LIBFRANKA_VER=$value
  archive=panda_py_${version}_libfranka_${value}
  python -m cibuildwheel --platform linux --output-dir $root/archive/$archive $root
  zip -j $root/archive/$archive.zip $root/archive/$archive/*.whl
done

# Change back to default version
change_version "LIBFRANKA_VER=0.9.2"
