# Spike: one libfranka build for every protocol generation

Investigating whether a single binary, and therefore a single wheel, can talk to
every robot firmware panda-py supports. Findings only; nothing here is built.

## The compatibility axis is the protocol version, not the release

libfranka checks the research interface version on connect and refuses a
mismatch. That version lives in a **submodule**, `libfranka-common`
(`https://github.com/frankaemika/libfranka-common.git`), pinned per libfranka
release, in `include/research_interface/robot/service_types.h` as `kVersion`.

| libfranka | pinned `common` | `kVersion` |
|---|---|---|
| 0.7.1 | `277a0fc6ce3d` | 3 |
| 0.8.0 | `af64d3e64087` | 4 |
| 0.9.2 | `e6aa0fc210d9` | 5 |
| 0.10.0 | `ea26b89aa302` | 6 |
| 0.12.1, 0.13.2 | `5adeec6566d6` | 6 |
| 0.13.6 | `dd768c882855` | 7 |
| 0.14.2 | `22b083750c45` | 8 |
| 0.15.0, 0.17.0 | `cd38d0ec300b` | 9 |
| 0.21.3 | `2e090a65e51c` | 10 |

panda-py's eight matrix rows map one-to-one onto protocol versions 3 through 10,
so the matrix is already exactly one row per generation. A universal build has
to speak all eight.

## The wire layouts are far more stable than the version numbers suggest

Measured by compiling each generation's `rbk_types.h` and printing `sizeof`:

| `kVersion` | `RobotState` | `MotionGeneratorCommand` | `RobotCommand` |
|---|---|---|---|
| 3 | 2085 | 306 | 370 |
| 4 | 2341 | 306 | 370 |
| 5 | 2373 | 306 | 370 |
| 6 | 2373 | 306 | 370 |
| 7 | 2373 | 306 | 371 |
| 8 | 2373 | 306 | 371 |
| 9 | 2373 | 306 | 371 |
| 10 | **1377** | 306 | 371 |

So across all eight generations there are only:

- **four** `RobotState` layouts — v3, v4, v5–v9, v10
- **two** `RobotCommand` layouts — v3–v6 and v7–v10
- **one** `MotionGeneratorCommand` layout, unchanged throughout

Five consecutive generations, v5 through v9, share one state layout. The entire
diff of `rbk_types.h` from v5 to v9 is a copyright line, a `kNone` added to
`MotionGeneratorMode`, and a `bool torque_command_finished` added to
`ControllerCommand` — which is the single byte in `RobotCommand` at v7.

### What v10 did

Protocol 10 converted the pose and inertia fields of `RobotState` from `double`
to `float` on the wire, using a `floatarray<N>` adapter class that converts back
to `std::array<double, N>` implicitly. With `#pragma pack(1)` this changes every
field offset and shrinks the struct from 2373 to 1377 bytes. This is the only
hard break in the 1 kHz path.

## The conversion is already a single choke point

The wire struct does not leak into the public API. `robot_impl.h:27` declares

```cpp
RobotState convertRobotState(const research_interface::robot::RobotState&) noexcept;
```

implemented at `robot_impl.cpp:502`, and `Robot::Impl::updateState` is the only
other place taking the wire type. So the natural design is one wire-struct
definition and one `convertRobotState` per layout, dispatched at runtime after
the version handshake, with the public `franka::RobotState` unchanged.

## Assessment

The 1 kHz path looks tractable: four state parsers, two command serialisers, and
a runtime switch behind an already-isolated conversion function.

The **TCP command path is the larger job**. `service_types.h` churns much more,
because it defines every command message as a template specialisation used
throughout `robot_impl.cpp`:

| transition | changed lines in `service_types.h` |
|---|---|
| v3 → v4 | 12 |
| v4 → v5 | 2 |
| v5 → v6 | 67 |
| v6 → v7 | 5 |
| v7 → v8 | **178** |
| v8 → v9 | 2 |
| v9 → v10 | 53 |

v8 is the big one: it adds `GetRobotModel` and a `DynamicSizedCommandMessage`
wrapper, which is how libfranka 0.14 and later fetch the robot model instead of
computing it locally. That is the same boundary where panda-py switched to
Pinocchio.

## The handshake is version-agnostic, and already tells you the answer

There is no negotiation: the client offers exactly one version. `Connect`'s
request hardcodes it at compile time.

```cpp
struct Connect : CommandBase<Connect, Command::kConnect> {
  enum class Status : uint8_t { kSuccess, kIncompatibleLibraryVersion };
  struct Request : public RequestBase<Connect> {
    Request(uint16_t udp_port) : version(kVersion), udp_port(udp_port) {}
    const Version version;
    const uint16_t udp_port;
  };
  struct Response : public ResponseBase<Connect> {
    Response(Status status) : ResponseBase(status), version(kVersion) {}
    const Version version;
  };
};
```

But the **response carries the robot's version**, and no brute-force retry loop
is needed, because every piece of the handshake is byte-identical across all
eight generations:

- the `Connect` struct itself — identical (same md5 of the declaration, v3 to v10)
- `CommandHeader` and `ResponseBase` — identical
- `Command` is an `enum class : uint32_t` with `kConnect` first, so it is `0`
  everywhere

So a client speaking *any* generation can send `Connect`, parse the response,
and learn what the robot speaks. Note this holds for the handshake only: v6
removed `kGetCartesianLimit`, which shifted the numeric value of every command
after it, so nothing else about the command set is version-stable.

libfranka already does exactly this, in `src/network.h`:

```cpp
switch (connect_response.status) {
  case (T::Status::kIncompatibleLibraryVersion):
    throw IncompatibleVersionException(connect_response.version, kLibraryVersion);
  case (T::Status::kSuccess):
    *ri_version = connect_response.version;
```

and `franka::IncompatibleVersionException` exposes it as a **public
`server_version` member**. Both the exception member and this `connect()`
implementation are present and identical in every version panda-py supports,
0.7.1 through 0.21.3.

### Consequence for a universal build

Version discovery is one connect attempt, not a retry loop, and needs no
protocol changes at all. A universal client would connect with any generation's
`Connect`, read the robot's version from the response, then select the right
`RobotState` parser and command set before proceeding. This removes the open
question about observable retry behaviour, so it needs no robot to establish.

### Consequence for panda-py today, independent of this spike

panda-py registers no exception translators, and `franka::Exception` derives
from `std::runtime_error`, so pybind11 maps an `IncompatibleVersionException` to
a plain Python `RuntimeError` and the structured `server_version` is thrown
away. The user is left parsing a message string.

Binding the exception, or just a helper that attempts a connect and reports the
robot's protocol version, would let panda-py tell a user exactly which wheel
they need instead of making them guess from the compatibility table. That is
shippable now and does not depend on any of the above.

## All eight generations do compile into one translation unit

Verified, not assumed: `notes/protocol_coexistence_check.sh` fetches the headers
from the pinned `libfranka-common` commits, wraps each generation in a
namespace, compiles them together and prints the layouts. Zero errors, with
correct distinct sizes and `kVersion` values for all eight, on both GCC 15.2
locally and GCC 14.2.1 inside the manylinux build image that produces the
wheels.

Included naively, the two headers collide with 23 errors, so two things are
needed.

**Hoist the standard headers to global scope first.** The protocol headers
include only standard library headers, so pulling them in beforehand makes the
nested includes no-ops. Without that, `#include <array>` inside a wrapper
namespace puts the standard library in the wrapper namespace.

**Alias the generations whose headers are byte identical, do not duplicate
them.** Only six of the eight ship a distinct `rbk_types.h`: v6 is identical to
v5 and v9 to v8. GCC deduplicates `#pragma once` **by file content, not by
path**, so wrapping an identical file in a second namespace silently skips the
include and the namespace comes up empty. That surfaces as
`'RobotState' is not a member of 'gen_v6::research_interface::robot'`, which is
at least a hard error rather than a silent alias, but it is not an obvious one.

This is not a workaround so much as the design asserting itself: there are fewer
layouts than versions, so the namespaces should be per layout with a version to
layout mapping, which is what the aliases express. All eight `service_types.h`
are distinct, if only in `kVersion`, so those do get one namespace each.

Empirically, `Connect::Request` is 4 bytes in every generation, which confirms
the handshake compatibility argued above by inspection.

## Open questions

- Gripper protocol: this spike only looked at `research_interface/robot`. There
  is a separate `research_interface/gripper` with its own `kVersion`.
- Licensing: libfranka is Apache-2.0, so a patched fork is fine, but the
  distribution story needs deciding — vendored patches like the existing
  `bin/patches`, or a real fork.

## Validation constraint

Cannot be merged on inspection. Cross-generation compatibility is the entire
point, so it needs testing on at least one FER and one FR3.
