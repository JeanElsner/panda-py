#!/bin/bash
# Checks whether every research interface generation panda-py supports can be
# compiled into one translation unit, which decides whether a single build can
# talk to every robot firmware. See notes/universal-libfranka.md.
#
# Fetches the protocol headers from the libfranka-common commits pinned by each
# libfranka release, wraps each generation in its own namespace, and prints the
# wire layouts. Needs network access, g++ and curl. Writes to a temp directory.
set -euo pipefail

WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

# Protocol version -> the libfranka-common commit that libfranka pins for it.
# v3 0.7.1, v4 0.8.0, v5 0.9.2, v6 0.13.2, v7 0.13.6, v8 0.14.2, v9 0.17.0,
# v10 0.21.3.
COMMONS="v3:277a0fc6ce3d v4:af64d3e64087 v5:e6aa0fc210d9 v6:5adeec6566d6 \
v7:dd768c882855 v8:22b083750c45 v9:cd38d0ec300b v10:2e090a65e51c"
BASE="https://raw.githubusercontent.com/frankaemika/libfranka-common"

echo "Fetching protocol headers..."
for pair in $COMMONS; do
  gen="${pair%%:*}"
  sha="${pair##*:}"
  for header in rbk_types service_types; do
    curl -fsSL "$BASE/$sha/include/research_interface/robot/$header.h" \
      -o "$WORK_DIR/${gen}_${header}.h"
  done
done

# Generations whose rbk_types.h is byte identical to an earlier one cannot each
# get their own namespace: GCC deduplicates #pragma once by file content, so the
# second include is silently skipped and the namespace comes up empty. Work out
# which are duplicates rather than hardcoding it, so this keeps working as
# libfranka adds generations.
echo
echo "Distinct 1 kHz header files:"
declare -A FIRST_SEEN
STATE_OWNER=""
ALIASES=""
for pair in $COMMONS; do
  gen="${pair%%:*}"
  sum="$(md5sum "$WORK_DIR/${gen}_rbk_types.h" | cut -d' ' -f1)"
  if [[ -n "${FIRST_SEEN[$sum]:-}" ]]; then
    echo "  $gen: identical to ${FIRST_SEEN[$sum]}, aliased"
    ALIASES+="namespace wire_$gen = wire_${FIRST_SEEN[$sum]};"$'\n'
  else
    FIRST_SEEN[$sum]="$gen"
    STATE_OWNER+="$gen "
    echo "  $gen: distinct"
  fi
done

{
  # Hoisted so the nested includes below become no-ops and do not drag the
  # standard library into a wrapper namespace.
  printf '#include <%s>\n' algorithm array cstdint cstring optional stdexcept \
    string type_traits vector
  for gen in $STATE_OWNER; do
    printf 'namespace wire_%s {\n#include "%s_rbk_types.h"\n}\n' "$gen" "$gen"
  done
  printf '%s' "$ALIASES"
  for pair in $COMMONS; do
    gen="${pair%%:*}"
    printf 'namespace cmd_%s {\n#include "%s_service_types.h"\n}\n' "$gen" "$gen"
  done
  echo '#include <cstdio>'
  echo 'int main() {'
  echo '  printf("  %-4s %8s %11s %13s %17s\n", "gen", "kVersion",'
  echo '         "RobotState", "RobotCommand", "Connect::Request");'
  for pair in $COMMONS; do
    gen="${pair%%:*}"
    cat <<EOF
  printf("  %-4s %8d %11zu %13zu %17zu\n", "$gen",
         (int)cmd_$gen::research_interface::robot::kVersion,
         sizeof(wire_$gen::research_interface::robot::RobotState),
         sizeof(wire_$gen::research_interface::robot::RobotCommand),
         sizeof(cmd_$gen::research_interface::robot::Connect::Request));
EOF
  done
  echo '  return 0;'
  echo '}'
} >"$WORK_DIR/coexist.cpp"

echo
echo "Compiling all generations into one translation unit..."
g++ -std=c++17 -I"$WORK_DIR" -o "$WORK_DIR/coexist" "$WORK_DIR/coexist.cpp"
echo
"$WORK_DIR/coexist"
