# panda-py 2.0 release plan

Decisions (Jean, 2026-10-09):

- The release builds against the libfranka-universal fork only
  (github.com/JeanElsner/libfranka, branch `universal`), which speaks every
  research interface protocol 3 to 10. The per-libfranka-version wheel matrix
  and panda-py's `LIBFRANKA_VER` branches go. Protocols that cannot be tested
  on hardware are supported on the strength of the fake robot; users report
  problems.
- Dissertation-only features are dropped; these stay, made generic:
  per-joint spring/damper (was the joint-7 spring), energy tank, Coulomb
  friction compensation, leash and `step_reference`.
- IK becomes numerical only: damped least squares, the connected robot's
  joint limits, seeded from `q_init`, nearest solution or a clear error. The
  analytic He solver and `ik_full` go.
- Every controller is kept and modernised onto one loop (TaskImpedance's):
  setters that never block the 1 kHz loop, guards, 1 kHz telemetry.
- Examples and docs updated; the roboticstoolbox `mmc.py` example goes.
- A hardware check (`panda-check`) exercises everything with small motions and
  writes a report (firmware, system, protocol, robot type, serial, versions).
  Jean runs it on an FR3 for confirmation.

## Controller names

Joint space `Joint*`, task (Cartesian) space `Task*`:

| v1 | v2 |
|---|---|
| JointPosition | JointImpedance |
| IntegratedVelocity | JointVelocity |
| AppliedTorque | JointTorque |
| JointTrajectory | JointTrajectory |
| CartesianImpedance | TaskImpedance |
| AppliedForce | TaskWrench |
| Force | TaskForce |
| CartesianTrajectory | TaskTrajectory |

## Phases

1. libfranka-universal speaks protocols 3 to 10. DONE, fork 132400c: all 805
   upstream tests, and `test/universal/run_versions.sh` passes in every version.
2. panda-py knows the connected robot (FER or FR3, protocol) and takes joint
   position, velocity and acceleration limits from it, for the motion
   generators, joint walls and guard defaults.
3. Controllers on a shared loop base, renamed, research wording removed.
4. Numerical IK.
5. `panda-check` hardware check and report.
6. Docs, examples, migration guide.
7. Release pipeline: GHCR image from a pinned fork tag, CI and release on it,
   NOTICE for the modified libfranka, PR v2 -> main (ask Jean first).
