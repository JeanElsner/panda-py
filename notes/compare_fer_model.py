"""Compare Franka's FER model with Pinocchio on the published FER parameters.

A universal libfranka built on 0.21.3 computes the robot model with Pinocchio
from a URDF. Robots from protocol version 8 send that URDF; an FER cannot, so
the idea is to ship one built from franka_description's published parameters.
This measures how close that comes to the model Franka's own libfranka 0.9.2
downloads from the robot (libfcimodels), which is what panda-py uses today.

Two steps, only the first needs the robot:

  record   Download the model from the robot and evaluate it on sampled
           joint configurations. Only FCI is activated; the brakes stay
           locked and nothing moves. Needs panda-py built with libfranka 0.9.2.

      python notes/compare_fer_model.py record <robot-ip> <desk-user> [password]

  compare  Build the FER URDF from franka_description and evaluate the same
           quantities with Pinocchio. Offline.

      python notes/compare_fer_model.py compare fer_model_<time>.npz

Compared: the pose and zero Jacobian of every libfranka frame, gravity, the
mass matrix and the Coriolis vector, each without a payload and with one.
Kinematics should agree to rounding; the dynamics are what is in question.
"""

import argparse
import datetime
import faulthandler
import getpass
import pathlib
import signal
import subprocess
import sys
import urllib.request

import numpy as np

# franka_description commit the parameters are read from, Apache-2.0.
DESCRIPTION_COMMIT = "7aeeddc"
DESCRIPTION_URL = (
    "https://raw.githubusercontent.com/frankarobotics/franka_description/"
    f"{DESCRIPTION_COMMIT}/robots/fer/"
)

SAMPLES = 2000
SEED = 0
FRAMES = ["kJoint1", "kJoint2", "kJoint3", "kJoint4", "kJoint5", "kJoint6", "kJoint7"]
FRAMES += ["kFlange", "kEndEffector", "kStiffness"]
# A non-trivial end effector and stiffness frame, so the chain past the flange
# is exercised too: roughly a Franka Hand, and a 5 cm stiffness offset.
F_T_EE = np.array(
    [
        [np.cos(-np.pi / 4), -np.sin(-np.pi / 4), 0, 0],
        [np.sin(-np.pi / 4), np.cos(-np.pi / 4), 0, 0],
        [0, 0, 1, 0.1034],
        [0, 0, 0, 1],
    ]
)
EE_T_K = np.eye(4)
EE_T_K[2, 3] = 0.05
# (name, mass kg, centre of mass in the flange frame m, inertia about it kg m^2)
PAYLOADS = [
    ("none", 0.0, np.zeros(3), np.zeros((3, 3))),
    ("0.5 kg", 0.5, np.array([0.01, -0.02, 0.05]), np.diag([1e-3, 2e-3, 1.5e-3])),
]


def sample_inputs():
    from panda_py import constants

    rng = np.random.default_rng(SEED)
    lower = np.asarray(constants.JOINT_LIMITS_LOWER)
    upper = np.asarray(constants.JOINT_LIMITS_UPPER)
    q = rng.uniform(lower, upper, size=(SAMPLES, 7))
    dq = rng.uniform(-1.5, 1.5, size=(SAMPLES, 7))
    return q, dq


# ---------------------------------------------------------------- recording


def record(hostname, output):
    """Evaluate Franka's model. Runs in a child process with FCI active."""
    faulthandler.enable()
    from panda_py import libfranka

    robot = libfranka.Robot(hostname)
    model = robot.load_model()
    save_model_library(output)
    q, dq = sample_inputs()
    f_t_ee, ee_t_k = F_T_EE.flatten(order="F"), EE_T_K.flatten(order="F")
    results = {"q": q, "dq": dq, "F_T_EE": F_T_EE, "EE_T_K": EE_T_K}
    for frame in FRAMES:
        f = getattr(libfranka.Frame, frame)
        results[f"pose_{frame}"] = np.array([model.pose(f, x, f_t_ee, ee_t_k) for x in q])
        results[f"jacobian_{frame}"] = np.array(
            [model.zero_jacobian(f, x, f_t_ee, ee_t_k) for x in q]
        )
    for index, (_, mass, com, inertia) in enumerate(PAYLOADS):
        i_total = inertia.flatten(order="F")
        results[f"gravity_{index}"] = np.array([model.gravity(x, mass, com) for x in q])
        results[f"mass_{index}"] = np.array([model.mass(x, i_total, mass, com) for x in q])
        results[f"coriolis_{index}"] = np.array(
            [model.coriolis(x, v, i_total, mass, com) for x, v in zip(q, dq)]
        )
    np.savez_compressed(output, **results)
    print(f"  recorded {SAMPLES} configurations to {output}")
    return 0


def save_model_library(output):
    """Keep a copy of the model library libfranka downloaded from the robot.

    libfranka 0.9.2 writes it to a temporary file and dlopens it, so it is
    mapped into this process, possibly already unlinked. With the copy the
    reference model can be evaluated later without the robot.
    """
    target = pathlib.Path(output).with_suffix(".so")
    for line in pathlib.Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) < 6 or "/" not in fields[5]:
            continue
        path = fields[5].replace(" (deleted)", "")
        # Poco::TemporaryFile names it tmp<id> in TMPDIR, plus the .so suffix.
        name = pathlib.Path(path).name
        if not (name.startswith("tmp") and name.endswith(".so")):
            continue
        source = pathlib.Path(path)
        try:
            data = source.read_bytes() if source.exists() else None
            if data is None:
                # Unlinked: read it back through the mapping's file entry.
                data = pathlib.Path(f"/proc/self/map_files/{fields[0]}").read_bytes()
            target.write_bytes(data)
            print(f"  saved the robot's model library ({len(data)} bytes) to {target}")
            return target
        except OSError as error:
            print(f"  ! could not copy the model library from {path}: {error}")
    print("  ! model library not found among this process's mappings; samples only")
    return None


def record_with_desk(args):
    import panda_py

    password = args.password or getpass.getpass("  Desk password: ")
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    output = pathlib.Path(args.output) / f"fer_model_{stamp}.npz"
    output.parent.mkdir(parents=True, exist_ok=True)
    print(f"panda-py {panda_py.__version__} from {panda_py.__file__}")
    desk = panda_py.Desk(args.hostname, args.username, password, platform="panda")
    print("  desk: logged in, control token acquired")
    status = 1
    try:
        if args.unlock:
            print("\n  Unlocking the brakes WILL make the robot move. Stand clear.")
            if input("  Type YES to continue: ") != "YES":
                return 1
            desk.unlock()
            print("  desk: brakes unlocked")
        try:
            desk.activate_fci()
        except Exception as error:  # pylint: disable=broad-except
            print(f"\n  ! FCI activation failed: {error}")
            if not args.unlock:
                print("    If the Desk needs the brakes open for FCI, rerun with --unlock.")
            return 1
        print("  desk: FCI activated, brakes " + ("unlocked" if args.unlock else "locked"))
        command = [sys.executable, __file__, "_record", args.hostname, str(output)]
        status = subprocess.run(command, check=False).returncode
        if status < 0:
            print(f"\n  ! crashed: {signal.Signals(-status).name}")
    finally:
        print("\n  restoring the previous state")
        steps = [("FCI deactivated", desk.deactivate_fci)]
        steps += [("brakes locked", desk.lock)] if args.unlock else []
        steps += [("control released", desk.release_control)]
        for name, call in steps:
            try:
                call()
                print(f"  desk: {name}")
            except Exception as error:  # pylint: disable=broad-except
                print(f"  desk: FAILED before '{name}': {error}")
    return status


# ---------------------------------------------------------------- the URDF


def fetch_description(cache):
    import yaml

    cache.mkdir(parents=True, exist_ok=True)
    data = {}
    for name in ("kinematics", "inertials", "joint_limits"):
        path = cache / f"{name}.yaml"
        if not path.exists():
            with urllib.request.urlopen(f"{DESCRIPTION_URL}{name}.yaml", timeout=30) as reply:
                path.write_bytes(reply.read())
        data[name] = yaml.safe_load(path.read_text())
    return data


def build_urdf(description):
    """The FER arm as franka_description's fer.urdf.xacro builds it, minus meshes.

    Links link0 to link8 and joints joint1 to joint8, named as libfranka's
    RobotModel expects: it looks up link8 as the flange.
    """
    kinematics, inertials = description["kinematics"], description["inertials"]
    limits = description["joint_limits"]

    def origin(k):
        k = k["kinematic"]
        return (
            f'<origin xyz="{k["x"]} {k["y"]} {k["z"]}" rpy="{k["roll"]} {k["pitch"]} {k["yaw"]}"/>'
        )

    def link(name):
        if name not in inertials:
            return f'  <link name="{name}"/>'
        i = inertials[name]
        m = i["inertia"]
        return (
            f'  <link name="{name}">\n    <inertial>\n'
            f'      <origin xyz="{i["origin"]["xyz"]}" rpy="{i["origin"]["rpy"]}"/>\n'
            f'      <mass value="{i["mass"]}"/>\n'
            f'      <inertia ixx="{m["xx"]}" ixy="{m["xy"]}" ixz="{m["xz"]}"'
            f' iyy="{m["yy"]}" iyz="{m["yz"]}" izz="{m["zz"]}"/>\n'
            f"    </inertial>\n  </link>"
        )

    parts = ['<?xml version="1.0"?>', '<robot name="fer">', link("link0")]
    for j in range(1, 8):
        lim = limits[f"joint{j}"]["limit"]
        parts += [
            link(f"link{j}"),
            f'  <joint name="joint{j}" type="revolute">',
            f"    {origin(kinematics[f'joint{j}'])}",
            f'    <parent link="link{j - 1}"/>\n    <child link="link{j}"/>',
            '    <axis xyz="0 0 1"/>',
            f'    <limit lower="{lim["lower"]}" upper="{lim["upper"]}"'
            f' velocity="{lim["velocity"]}" effort="{lim["effort"]}"/>',
            "  </joint>",
        ]
    parts += [
        link("link8"),
        '  <joint name="joint8" type="fixed">',
        f"    {origin(kinematics['joint8'])}",
        '    <parent link="link7"/>\n    <child link="link8"/>',
        "  </joint>",
        "</robot>",
    ]
    return "\n".join(parts) + "\n"


# ---------------------------------------------------------------- comparison


class PinocchioFer:
    """Evaluates the quantities libfranka's Model offers, the way 0.21.3 does."""

    def __init__(self, urdf, f_t_ee, ee_t_k):
        import pinocchio as pin

        self.pin = pin
        self.model = pin.buildModelFromXML(urdf)
        self.flange = self.model.getFrameId("link8")
        self.joint7 = self.model.frames[self.flange].parentJoint
        flange_placement = self.model.frames[self.flange].placement
        self.base_inertia = self.model.inertias[self.joint7].copy()
        ee = pin.SE3(f_t_ee[:3, :3], f_t_ee[:3, 3])
        k = pin.SE3(ee_t_k[:3, :3], ee_t_k[:3, 3])
        for name, placement in (("ee", flange_placement * ee), ("k", flange_placement * ee * k)):
            self.model.addFrame(
                pin.Frame(name, self.joint7, self.flange, placement, pin.FrameType.OP_FRAME)
            )
        self.frame_ids = {f"kJoint{j}": self.model.getFrameId(f"link{j}") for j in range(1, 8)}
        self.frame_ids.update(
            kFlange=self.flange,
            kEndEffector=self.model.getFrameId("ee"),
            kStiffness=self.model.getFrameId("k"),
        )
        self.flange_placement = flange_placement
        self.data = self.model.createData()

    def set_payload(self, mass, com, inertia):
        """Add the payload to the last link, as libfranka's RobotModel does."""
        pin = self.pin
        self.model.inertias[self.joint7] = self.base_inertia.copy()
        if mass > 0:
            payload = pin.Inertia(mass, com, inertia)
            self.model.inertias[self.joint7] += self.flange_placement.act(payload)
        self.data = self.model.createData()

    def pose(self, q, frame):
        self.pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.frame_ids[frame]].homogeneous

    def jacobian(self, q, frame):
        pin = self.pin
        return pin.computeFrameJacobian(
            self.model, self.data, q, self.frame_ids[frame], pin.LOCAL_WORLD_ALIGNED
        )

    def gravity(self, q):
        return self.pin.computeGeneralizedGravity(self.model, self.data, q)

    def mass(self, q):
        m = self.pin.crba(self.model, self.data, q)
        return np.triu(m) + np.triu(m, 1).T

    def coriolis(self, q, dq):
        return self.pin.rnea(self.model, self.data, q, dq, np.zeros(7)) - self.gravity(q)


def relative(error, reference):
    return np.linalg.norm(error, axis=-1) / np.maximum(np.linalg.norm(reference, axis=-1), 1e-9)


def compare(path, cache):
    ref = np.load(path)
    urdf = build_urdf(fetch_description(cache))
    urdf_path = pathlib.Path(path).with_suffix(".urdf")
    urdf_path.write_text(urdf)
    fer = PinocchioFer(urdf, ref["F_T_EE"], ref["EE_T_K"])
    q, dq = ref["q"], ref["dq"]
    print(f"Pinocchio {fer.pin.__version__} on franka_description {DESCRIPTION_COMMIT}")
    print(f"  URDF written to {urdf_path}, {len(q)} configurations\n")

    print("Kinematics, worst case over all configurations")
    print("  frame          position mm   rotation deg   jacobian max abs")
    for frame in FRAMES:
        franka = ref[f"pose_{frame}"].reshape(-1, 4, 4).transpose(0, 2, 1)
        ours = np.array([fer.pose(x, frame) for x in q])
        position = np.linalg.norm(franka[:, :3, 3] - ours[:, :3, 3], axis=1).max() * 1000
        relative_rotation = np.einsum("nji,njk->nik", franka[:, :3, :3], ours[:, :3, :3])
        cos = np.clip((np.trace(relative_rotation, axis1=1, axis2=2) - 1) / 2, -1, 1)
        rotation = np.degrees(np.arccos(cos)).max()
        franka_j = ref[f"jacobian_{frame}"].reshape(-1, 7, 6).transpose(0, 2, 1)
        ours_j = np.array([fer.jacobian(x, frame) for x in q])
        print(
            f"  {frame:<13} {position:11.6f}   {rotation:12.6f}   {np.abs(franka_j - ours_j).max():.2e}"
        )

    for index, (name, mass, com, inertia) in enumerate(PAYLOADS):
        fer.set_payload(mass, com, inertia)
        franka_g = ref[f"gravity_{index}"]
        franka_m = ref[f"mass_{index}"].reshape(-1, 7, 7).transpose(0, 2, 1)
        franka_c = ref[f"coriolis_{index}"]
        ours_g = np.array([fer.gravity(x) for x in q])
        ours_m = np.array([fer.mass(x) for x in q])
        ours_c = np.array([fer.coriolis(x, v) for x, v in zip(q, dq)])
        print(f"\nDynamics, payload {name}")
        print("  quantity        median rel   95th pct rel   max abs            per joint max abs")
        for label, franka, ours, unit in (
            ("gravity", franka_g, ours_g, "Nm"),
            ("coriolis", franka_c, ours_c, "Nm"),
        ):
            rel = relative(ours - franka, franka)
            per_joint = np.abs(ours - franka).max(0)
            print(
                f"  {label:<14} {np.median(rel):10.2%}   {np.percentile(rel, 95):11.2%}"
                f"   {np.abs(ours - franka).max():6.3f} {unit}   "
                + " ".join(f"{v:.3f}" for v in per_joint)
            )
        mass_rel = np.linalg.norm(ours_m - franka_m, axis=(1, 2)) / np.linalg.norm(
            franka_m, axis=(1, 2)
        )
        diagonal = np.abs(np.diagonal(ours_m - franka_m, axis1=1, axis2=2)).max(0)
        print(
            f"  {'mass matrix':<14} {np.median(mass_rel):10.2%}   {np.percentile(mass_rel, 95):11.2%}"
            f"   {np.abs(ours_m - franka_m).max():6.3f} kg m2  diagonal "
            + " ".join(f"{v:.3f}" for v in diagonal)
        )
        mean_diag = np.diagonal(franka_m - ours_m, axis1=1, axis2=2).mean(0)
        print(
            "  mass diagonal, Franka minus Pinocchio, mean: "
            + " ".join(f"{v:+.4f}" for v in mean_diag)
        )
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="step", required=True)
    rec = sub.add_parser("record", help="download the model from the robot and sample it")
    rec.add_argument("hostname")
    rec.add_argument("username")
    rec.add_argument("password", nargs="?")
    rec.add_argument("--output", default=".", help="directory for the recording")
    rec.add_argument("--unlock", action="store_true", help="unlock the brakes first")
    cmp = sub.add_parser("compare", help="compare a recording with Pinocchio, offline")
    cmp.add_argument("recording")
    cmp.add_argument(
        "--cache", default=pathlib.Path.home() / ".cache" / "panda-py" / "fer-description"
    )
    worker = sub.add_parser("_record")
    worker.add_argument("hostname")
    worker.add_argument("output")
    args = parser.parse_args()
    if args.step == "record":
        return record_with_desk(args)
    if args.step == "_record":
        return record(args.hostname, args.output)
    return compare(args.recording, pathlib.Path(args.cache))


if __name__ == "__main__":
    sys.exit(main())
