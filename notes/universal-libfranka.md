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

## Open questions

- Can the eight `service_types.h` variants coexist in one translation unit, or
  do the template specialisations collide? They share type names in one
  namespace, so each generation probably needs its own inline namespace.
- Does the connect handshake let a client offer a version range, or does it
  offer exactly one? If exactly one, a universal client has to retry the
  connect per version, which is observable behaviour worth checking against a
  real robot.
- Gripper protocol: this spike only looked at `research_interface/robot`. There
  is a separate `research_interface/gripper` with its own `kVersion`.
- Licensing: libfranka is Apache-2.0, so a patched fork is fine, but the
  distribution story needs deciding — vendored patches like the existing
  `bin/patches`, or a real fork.

## Validation constraint

Cannot be merged on inspection. Cross-generation compatibility is the entire
point, so it needs testing on at least one FER and one FR3.
