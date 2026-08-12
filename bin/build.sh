#!/bin/bash
set -euo pipefail

# Check if the libfranka version is provided as an argument
if [ -z "${1:-}" ]; then
    echo "Usage: $0 <libfranka_version>"
    exit 1
fi

# Install Python dependencies
python -m pip install packaging toml cibuildwheel

# Set pyproject.toml path
root=$(dirname $0)/..
toml="$root/pyproject.toml"

# pyproject.toml is rewritten in place below. Always restore the original,
# including when cibuildwheel fails part way through.
backup=$(mktemp)
cp "$toml" "$backup"
trap 'cp "$backup" "$toml"; rm -f "$backup"' EXIT

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

# Pin the package version and the libfranka version that
# bin/before_install_centos.sh builds. The LIBFRANKA_VER preprocessor guard is
# derived from the installed libfranka by CMake, so it is not set here.
change_version() {
  python <<END
import toml

with open('$toml', 'r') as f:
    data = toml.load(f)

data['project']['version'] = '$version+libfranka-$1'
data['tool']['cibuildwheel']['environment'] = 'LIBFRANKA_VER=$1'

with open('$toml', 'w') as f:
    toml.dump(data, f)
END
}

mkdir -p $root/archive

# Call the change_version function with the provided libfranka version
libfranka_version="$1"
echo "Changing libfranka version in pyproject.toml to: $libfranka_version"
change_version "$libfranka_version"
archive=panda_py_${version}_libfranka_${libfranka_version}
python -m cibuildwheel --output-dir $root/archive/$archive $root
zip -j $root/archive/$archive.zip $root/archive/$archive/*.whl
