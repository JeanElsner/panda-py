"""A fake control unit that speaks any research interface protocol version, 3 to 10.

A copy of libfranka-universal's test/universal/fake_robot.py, with its
state_layouts.py and the FER and FR3 descriptions next to it.

It behaves like a robot of that version (3 to 5: a Franka Emika Robot; 6 to
10: a Franka Research 3), as far as connecting, streaming state and taking
commands go:

  - A Connect offering any other version is rejected with status
    kIncompatibleLibraryVersion and the robot's version, and the connection
    closed. That is what a real FER and a real FR3 were observed to do.
  - A Connect offering the robot's version is accepted, and the robot state is
    then streamed over UDP at 1 kHz to the port the client named, in that
    version's layout (state_layouts.py).
  - TCP commands are answered with kSuccess, and logged with their name in that
    version's command list and their size. A Move switches the streamed modes
    to the requested ones and answers kMotionStarted; when a 1 kHz command
    reports the motion finished, the modes return to idle and the Move is
    answered kSuccess. GetRobotModel (version 8 on) is answered with the
    robot's description.
  - 1 kHz commands are checked to be the version's size (370 bytes up to
    version 6, 371 from 7, which added torque_command_finished) and logged.

Every field of the state carries a distinct value with a marker in its tenth
decimal, so a client that routes the state through the float-based current
protocol types, or reads a field at the wrong offset, gets caught. The joint
positions are those of a recorded FER configuration, which lets the client
check its model against Franka's for the same q.

    python fake_robot.py [--port 1337] [--version 5] [--seconds 30]

It runs a real control loop's worth of checks without a robot; it is no
substitute for one.
"""

import argparse
import pathlib
import socket
import struct
import threading
import time

from state_layouts import LAYOUTS

# Set by serve() for the robot's version.
LAYOUT = None
STATE_FORMAT = None

HEADER = struct.Struct("<III")  # command, command id, size
CONNECT_REQUEST = struct.Struct("<HH")  # version, udp port
CONNECT_RESPONSE = struct.Struct("<BH")  # status, version
K_CONNECT, K_SUCCESS, K_INCOMPATIBLE = 0, 0, 1
K_MOTION_STARTED = 1
# Each version's Command enum, in order: the command numbers.
_FER = ["Connect", "Move", "StopMove", "GetCartesianLimit", "SetCollisionBehavior",
        "SetJointImpedance", "SetCartesianImpedance", "SetGuidingMode", "SetEEToK", "SetNEToEE",
        "SetLoad", "SetFilters", "AutomaticErrorRecovery", "LoadModelLibrary"]
_FR3 = ["Connect", "Move", "StopMove", "SetCollisionBehavior", "SetJointImpedance",
        "SetCartesianImpedance", "SetGuidingMode", "SetEEToK", "SetNEToEE", "SetLoad",
        "AutomaticErrorRecovery", "LoadModelLibrary"]
COMMAND_LISTS = {
    3: [c if c != "SetNEToEE" else "SetFToEE" for c in _FER], 4: _FER, 5: _FER,
    6: _FR3, 7: _FR3, 8: _FR3 + ["GetRobotModel"], 9: _FR3 + ["GetRobotModel"],
    10: [c for c in _FR3 if c != "LoadModelLibrary"] + ["GetRobotModel"],
}
STATUS_RESPONSE = struct.Struct("<B")
# RobotCommand: message_id, MotionGeneratorCommand (q_c, dq_c, O_T_EE_c, O_dP_EE_c, elbow_c,
# valid_elbow, motion_generation_finished), ControllerCommand (tau_J_d, from version 7
# torque_command_finished).
ROBOT_COMMAND_OLD = struct.Struct("<Q7d7d16d6d2d??7d")
ROBOT_COMMAND_NEW = struct.Struct("<Q7d7d16d6d2d??7d?")
assert (ROBOT_COMMAND_OLD.size, ROBOT_COMMAND_NEW.size) == (370, 371)
# Move: controller mode, motion generator mode, two deviations; version 10 adds the
# asynchronous motion generator's flag and maximum velocities.
MOVE_REQUEST = struct.Struct("<II3d3d")
# Next to this file in panda-py's tests; in libfranka-universal's tree, its
# src/protocol/descriptions.
DESCRIPTIONS = pathlib.Path(__file__).resolve().parent
if not (DESCRIPTIONS / "fer.urdf").exists():
    DESCRIPTIONS = pathlib.Path(__file__).resolve().parents[2] / "src" / "protocol" / "descriptions"

VERSION = None  # set by serve()


def commands():
    return COMMAND_LISTS[VERSION]


def robot_command_struct():
    return ROBOT_COMMAND_NEW if VERSION >= 7 else ROBOT_COMMAND_OLD


def reflex_aborted():
    """Move::Status kReflexAborted: version 6 inserted two safety statuses before it."""
    return 6 if VERSION <= 5 else 8

# A recorded FER configuration, and Franka's gravity for it with no payload.
Q = [0.7936381933528991, -0.8116399619540988, -2.6598748181993264, -3.022184038143356,
     1.8152757280698988, 3.423588526337011, 0.6179116662605004]
IDENTITY = [1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0]


def marker(field_index, element):
    """A value unique to the field and element, with a 1e-10 precision marker."""
    return field_index + element / 100 + (field_index + 1) * 1e-10


class Robot:
    """What the fake reports, shared between the TCP and UDP threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.motion_generator_mode = 0  # kIdle
        self.controller_mode = 3  # kOther
        self.move = None  # (connection, command id) of the running Move
        self.commands = 0
        self.wrong_sizes = 0
        self.nonzero_dq = 0
        self.last_tau = None
        self.dq_history = []
        self.move_started = None
        self.mode_override = None
        self.last_command_time = None
        self.longest_gap = 0.0
        # What the robot reports back as desired, which libfranka's rate limiting and filter
        # start from: the last command, and zero while idle.
        self.q_d = list(Q)
        self.dq_d = [0.0] * 7
        self.ddq_d = [0.0] * 7
        self.tau_J_d = [0.0] * 7


STILL = False  # --still: a robot at rest, for clients that check physical plausibility
STOP_STREAMING_AFTER = None  # --stop-streaming-after: go silent this long into a Move
REFLEX_AFTER = None  # --reflex-after: abort the Move with a reflex this long into it
IGNORE_FINISH = False  # --ignore-finish: keep a Move running when the client finishes it


def state_values(message_id, robot):
    values = {}
    for index, (name, code, count) in enumerate(LAYOUT):
        if code in "df":
            values[name] = [marker(index, e) for e in range(count)]
        elif code == "?":
            values[name] = [False] * count
    values.update(
        message_id=message_id,
        q=list(Q),
        # Frames the model uses must be valid transforms; no payload.
        F_T_EE=list(IDENTITY), EE_T_K=list(IDENTITY), F_T_NE=list(IDENTITY),
        NE_T_EE=list(IDENTITY),
        O_ddP_O=[0.0, 0.0, -9.81],  # the gravity vector, as robots report it
        q_d=list(robot.q_d), dq_d=list(robot.dq_d), ddq_d=list(robot.ddq_d), tau_J_d=list(robot.tau_J_d),
        m_ee=0.0, m_load=0.0, F_x_Cee=[0.0] * 3, F_x_Cload=[0.0] * 3, I_ee=[0.0] * 9,
        I_load=[0.0] * 9,
        motion_generator_mode=robot.motion_generator_mode,
        controller_mode=robot.controller_mode,
        robot_mode=robot.mode_override or (2 if robot.move is not None else 1),  # kMove, kIdle
        control_command_success_rate=0.75 + 1e-10,
    )
    values["errors"][7] = True  # one error set, to check the error mapping
    n_errors = len(values["errors"])
    if STILL:
        for name in ("dq", "dtheta", "dtau_J", "O_dP_EE_d", "O_dP_EE_c", "O_ddP_EE_c",
                     "tau_ext_hat_filtered", "O_F_ext_hat_K", "K_F_ext_hat_K", "joint_contact",
                     "cartesian_contact", "joint_collision", "cartesian_collision", "elbow_c",
                     "delbow_c", "ddelbow_c"):
            values[name] = [0.0] * len(values[name])
        values.update(theta=list(Q), O_T_EE=list(IDENTITY), O_T_EE_d=list(IDENTITY),
                      O_T_EE_c=list(IDENTITY), errors=[False] * n_errors)
    flat = []
    for name, code, count in LAYOUT:
        value = values[name]
        flat += value if count > 1 else [value]
    return struct.pack(STATE_FORMAT, *flat)


def reply(conn, command, command_id, status, body=b""):
    payload = STATUS_RESPONSE.pack(status) + body
    conn.sendall(HEADER.pack(command, command_id, HEADER.size + len(payload)) + payload)


def stream(address, stop, robot):
    """Send state at 1 kHz and take the 1 kHz commands that come back on the same socket."""
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setblocking(False)
    message_id = 1
    while not stop.is_set():
        with robot.lock:
            if (REFLEX_AFTER is not None and robot.move is not None
                    and time.monotonic() - robot.move_started > REFLEX_AFTER):
                conn, command_id = robot.move
                robot.motion_generator_mode, robot.controller_mode = 0, 3
                robot.move, robot.mode_override = None, 4  # kReflex
                reply(conn, 1, command_id, reflex_aborted())
                print(f"move {command_id} aborted by a reflex", flush=True)
            silent = (STOP_STREAMING_AFTER is not None and robot.move_started is not None
                      and time.monotonic() - robot.move_started > STOP_STREAMING_AFTER)
            if not silent:
                udp.sendto(state_values(message_id, robot), address)
        message_id += 1
        time.sleep(0.001)
        while True:
            try:
                data = udp.recv(4096)
            except BlockingIOError:
                break
            with robot.lock:
                robot.commands += 1
                now = time.monotonic()
                if robot.last_command_time is not None:
                    robot.longest_gap = max(robot.longest_gap, now - robot.last_command_time)
                robot.last_command_time = now
                layout = robot_command_struct()
                if len(data) != layout.size:
                    robot.wrong_sizes += 1
                    continue
                fields = layout.unpack(data)
                dq_c, tau = fields[8:15], fields[41:48]
                # From version 7 a pure torque Move finishes on torque_command_finished.
                finished = fields[40] or (VERSION >= 7 and fields[48])
                robot.nonzero_dq += any(v != 0 for v in dq_c)
                robot.last_tau = tau
                robot.ddq_d = [(new - old) / 0.001 for new, old in zip(dq_c, robot.dq_d)]
                robot.dq_d, robot.tau_J_d = list(dq_c), list(tau)
                robot.dq_history.append(dq_c)
                if finished and robot.move is not None and not IGNORE_FINISH:
                    conn, command_id = robot.move
                    robot.motion_generator_mode, robot.controller_mode = 0, 3
                    robot.move = None
                    robot.dq_d, robot.ddq_d = [0.0] * 7, [0.0] * 7
                    reply(conn, 1, command_id, K_SUCCESS)
                    print(f"  longest gap between 1 kHz commands {robot.longest_gap * 1000:.1f} ms",
                          flush=True)
                    print(f"move {command_id} finished after {robot.commands} 1 kHz commands, "
                          f"{robot.wrong_sizes} of the wrong size, {robot.nonzero_dq} with nonzero"
                          f" dq_c, last tau_J_d {[round(t, 3) for t in tau]}", flush=True)
                    history = robot.dq_history
                    if robot.nonzero_dq and len(history) > 2:
                        # Peak commanded joint acceleration, which the rate limiter bounds.
                        peak = [max(abs(b[j] - a[j]) / 0.001 for a, b in zip(history, history[1:]))
                                for j in range(7)]
                        print(f"  peak commanded acceleration {[round(p, 3) for p in peak]} rad/s^2",
                              flush=True)


def handle_command(conn, command, command_id, body, robot):
    names = commands()
    name = names[command] if command < len(names) else f"#{command}"
    print(f"command {command} {name}, {HEADER.size + len(body)} bytes", flush=True)
    if name == "Move":
        controller, generator, *_ = MOVE_REQUEST.unpack_from(body)
        with robot.lock:
            # Move numbers its modes without kIdle; the state's MotionGeneratorMode starts with it.
            robot.motion_generator_mode, robot.controller_mode = generator + 1, controller
            robot.move = (conn, command_id)
            robot.commands = robot.wrong_sizes = robot.nonzero_dq = 0
            robot.dq_history = []
            robot.move_started = time.monotonic()
            robot.last_command_time, robot.longest_gap = None, 0.0
        print(f"  controller mode {controller}, motion generator mode {generator}", flush=True)
        reply(conn, command, command_id, K_MOTION_STARTED)
    elif name == "StopMove":
        with robot.lock:
            robot.motion_generator_mode, robot.controller_mode = 0, 3
            move, robot.move = robot.move, None
        reply(conn, command, command_id, K_SUCCESS)
        if move is not None:
            reply(move[0], 1, move[1], 2)  # kPreempted
    elif name == "GetRobotModel":
        urdf = (DESCRIPTIONS / ("fer.urdf" if VERSION <= 5 else "fr3.urdf")).read_bytes()
        reply(conn, command, command_id, K_SUCCESS, urdf)
    else:
        reply(conn, command, command_id, K_SUCCESS)


def recv_exactly(conn, size):
    data = b""
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk:
            raise ConnectionError("client closed the connection")
        data += chunk
    return data


def serve(port, version, seconds):
    global VERSION, LAYOUT, STATE_FORMAT
    VERSION, LAYOUT = version, LAYOUTS[version]
    STATE_FORMAT = "<" + "".join(f"{count}{code}" for _, code, count in LAYOUT)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", port))
    server.listen()
    server.settimeout(seconds)
    print("ready", flush=True)
    stop = threading.Event()
    robot = Robot()
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            try:
                conn, peer = server.accept()
            except socket.timeout:
                break
            command, command_id, size = HEADER.unpack(recv_exactly(conn, HEADER.size))
            body = recv_exactly(conn, size - HEADER.size)
            offered, udp_port = CONNECT_REQUEST.unpack(body[: CONNECT_REQUEST.size])
            accepted = command == K_CONNECT and offered == version
            status = K_SUCCESS if accepted else K_INCOMPATIBLE
            conn.sendall(
                HEADER.pack(command, command_id, HEADER.size + CONNECT_RESPONSE.size)
                + CONNECT_RESPONSE.pack(status, version)
            )
            print(f"connect offering {offered}: {'accepted' if accepted else 'rejected'}",
                  flush=True)
            if not accepted:
                conn.close()
                continue
            threading.Thread(
                target=stream, args=((peer[0], udp_port), stop, robot), daemon=True
            ).start()
            # Serve commands until the client leaves or time runs out.
            conn.settimeout(max(0.1, end - time.monotonic()))
            try:
                while True:
                    command, command_id, size = HEADER.unpack(recv_exactly(conn, HEADER.size))
                    handle_command(conn, command, command_id,
                                   recv_exactly(conn, size - HEADER.size), robot)
            except (socket.timeout, OSError, ConnectionError):
                pass
            stop.set()
            conn.close()
            break
    finally:
        stop.set()
        server.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=1337)
    parser.add_argument("--version", type=int, default=5)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--still", action="store_true", help="report a robot at rest")
    parser.add_argument("--stop-streaming-after", type=float,
                        help="stop sending state this many seconds into a Move")
    parser.add_argument("--reflex-after", type=float,
                        help="abort a Move with a reflex this many seconds into it")
    parser.add_argument("--ignore-finish", action="store_true",
                        help="keep a Move running when the client finishes it")
    args = parser.parse_args()
    STILL = args.still
    REFLEX_AFTER = args.reflex_after
    IGNORE_FINISH = args.ignore_finish
    STOP_STREAMING_AFTER = args.stop_streaming_after
    serve(args.port, args.version, args.seconds)
