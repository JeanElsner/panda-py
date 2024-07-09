#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Define the output directory relative to the script directory
OUTPUT_DIR="$SCRIPT_DIR/../src"

# Run pybind11-stubgen with the modified output directory
pybind11-stubgen -o "$OUTPUT_DIR" --enum-class-locations RealtimeConfig:panda_py.libfranka --enum-class-locations VacuumGripperProductionSetupProfile:panda_py.libfranka --numpy-array-use-type-var panda_py
