# Running the protocol probe on a robot

Confirms the one claim in `universal-libfranka.md` that inspection cannot settle:
that a control unit rejects a `Connect` carrying an unknown version by replying
with **its own** version. All version discovery in a universal build rests on
that.

## What you need

Python 3.8 or newer. That is all.

**No libfranka. No build. No container. No pip install.** Both scripts use only
the standard library, which is why they run on a machine that has never seen
libfranka.

## Get it

```bash
git clone https://github.com/JeanElsner/panda-py.git
cd panda-py
git checkout jean/spike/universal-libfranka
```

Or, without cloning, just take the two files:

```bash
curl -O https://raw.githubusercontent.com/JeanElsner/panda-py/jean/spike/universal-libfranka/notes/probe_protocol_version.py
curl -O https://raw.githubusercontent.com/JeanElsner/panda-py/jean/spike/universal-libfranka/notes/prepare_and_probe.py
```

## Run it

A freshly booted robot has its brakes locked and the FCI off, so port 1337 is
not answering yet. This logs into the Desk, takes a control token, unlocks the
brakes, activates the FCI, then probes:

```bash
python3 notes/prepare_and_probe.py <robot-ip> <desk-user>
```

The password is prompted for, so it stays out of your shell history. To pass it
inline instead, append it as a third argument.

> **Unlocking the brakes makes the robot move.** The joints settle as the brakes
> release and the arm becomes back-drivable, so anything resting against it can
> shift. Stand clear and keep the emergency stop within reach. The script asks
> you to type `YES` before it unlocks. Nothing here commands a trajectory: there
> is no `Move`, no torque and no motion generator in either file. The only
> motion is the brake release itself.

Afterwards it **puts the robot back as it found it**: FCI off, brakes locked,
control token released. That happens even if the probe fails or you interrupt it.
Releasing the token matters most, because one left held locks out the next user
until someone forces it from the Pilot.

Useful flags:

| flag | effect |
|---|---|
| `--yes` | skip the confirmation prompt |
| `--leave-prepared` | leave it unlocked, FCI on and control held |
| `--platform panda\|fr3` | skip brake endpoint detection |

### Decoding the live state stream

The stronger test. Add `--read-state N` and it will also decode `N` seconds of
the 1 kHz stream, choosing the `RobotState` layout from the version the robot
just reported:

```bash
python3 notes/prepare_and_probe.py <robot-ip> <desk-user> --read-state 3
```

This still does not move the robot. It receives state and sends nothing back;
there is no `Move`, no `MotionGeneratorCommand` and no torque in any of these
files, and a robot only moves in response to a `Move` that is never sent. It is
what libfranka's `readOnce` does. Restoration afterwards is unchanged.

It prints joint positions, velocities, the end effector translation and torques,
then checks them: joints within range, `O_T_EE` a valid homogeneous transform,
success rate in [0, 1]. A wrong layout gives plausible-looking garbage, so those
checks are the point.

### If the robot is already unlocked with the FCI on

Then no Desk interaction is needed at all, and this touches nothing:

```bash
python3 notes/probe_protocol_version.py <robot-ip>   # versions only
python3 notes/read_state.py <robot-ip> --seconds 3   # and decode the stream
```

## Done: FR3. Still wanted: FER

An FR3 at protocol 10 was probed on 2026-08-31 and behaved exactly as predicted,
rejecting with `kIncompatibleLibraryVersion` and reporting version 10, gripper 3.

What is still missing is the **same 16 bytes against an FER**. One robot confirms
the mechanism; two confirm it across firmware generations, which is the property
a universal build depends on. An FER should report robot version 3, 4 or 5, and
gripper version 3.

## What to send back

The whole output, or just the summary block:

```
Summary
  host                     172.16.0.2
  desk platform            fr3  (legacy=False)
  robot protocol version   10
  spoken by libfranka      0.21.x
  panda-py wheel to use    the libfranka 0.21.3 build
  gripper protocol version 3   (expected 3)
```

The lines that matter are `robot protocol version`, the `status=` in the probe
output, which should read `kIncompatibleLibraryVersion`, and the raw received
bytes, which let the decode be checked independently.

## What has already been verified without a robot

So that a failure on hardware points at the robot rather than at these scripts:

- the wire layouts were measured by compiling the real `libfranka-common`
  headers, not read off by eye: robot 12 byte header plus 4 byte body with a
  3 byte response, gripper 10 byte header plus 4 byte body with a 4 byte
  response
- the probe was run against mock endpoints answering as an FER (version 5) and
  an FR3 (version 10), and decoded both correctly
- the Desk flow was run end to end against a mock Desk over TLS, covering login,
  control token, the FR3 to FER brake endpoint fallback, and FCI activation
- the multipart body for the brake request is byte for byte what `requests`
  emits for `files={"force": True}`, including the odd `filename="force"`, since
  that is the form the Desk is known to accept
- the retry loop was tested against a robot that starts answering only on the
  third attempt, since the Desk can return before port 1337 is listening
- both scripts fail cleanly against a closed port

## Expected results

| robot | expected robot version | expected gripper version |
|---|---|---|
| FER on system 4.x | 3, 4 or 5 depending on firmware | 3 |
| FR3 | 6 or higher, 10 on current firmware | 3 |

A gripper version other than 3 would be the surprise: it has been 3 in every
`libfranka-common` commit pinned by any libfranka release from 0.7.1 to 0.21.3.
