#!/bin/bash

# Check if the libfranka version is provided as an argument
if [ -z "$1" ]; then
    echo "Usage: $0 <libfranka_version>"
    exit 1
fi

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
  local version_num=$(echo "$1" | awk -F. '{ printf "0x%02x%02x%02x", $1, $2, $3 }')
  python <<END
import toml

with open('$toml', 'r') as f:
    data = toml.load(f)

data['project']['version'] = '$pandapy_version'
data['tool']['cibuildwheel']['environment'] = 'LIBFRANKA_VER=$1'
data['tool']['scikit-build']['cmake']['define']['LIBFRANKA_VER'] = '$version_num'

with open('$toml', 'w') as f:
    toml.dump(data, f)
END
}

mkdir $root/archive

# Call the change_version function with the provided libfranka version
libfranka_version="$1"
echo "Changing libfranka version in pyproject.toml to: $libfranka_version"
change_version "$libfranka_version"
archive=panda_py_${version}_libfranka_${libfranka_version}
python -m cibuildwheel --output-dir $root/archive/$archive $root
zip -j $root/archive/$archive.zip $root/archive/$archive/*.whl

# Change back to default version
change_version $version
