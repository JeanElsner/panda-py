"""Build the FER robot description a universal libfranka loads for protocols 3 to 5.

From protocol version 8 a robot sends libfranka 0.21.3 its URDF, from which the
model is computed with Pinocchio and the joint velocity limits are read. An FER
never speaks version 8, so panda-py has to ship this file for it instead.

Sources:

  kinematics      franka_description (Apache-2.0), robots/fer, commit 7aeeddc
  joint limits    the same
  inertials       fitted to Franka's FER model library, as libfranka 0.9.2
                  downloads it from the robot, on a recording made with
                  notes/compare_fer_model.py. The fit starts from the
                  parameters identified by Gaz et al. (below) and moves them
                  as little as reproducing Franka's model exactly requires.

Gaz, Cognetti, Oliva, Robuffo Giordano, De Luca, "Dynamic Identification of the
Franka Emika Panda Robot With Retrieval of Feasible Parameters Using
Penalty-Based Optimization", IEEE Robotics and Automation Letters 4(4), 2019.

The fitted inertials reproduce Franka's gravity, mass matrix and Coriolis terms
to rounding, but are not physically consistent: no physically consistent set
can reproduce that model, so Franka's own parameters are not either. The
libfranka computations do not care; simulators may.

Velocity limits: libfranka 0.21.3 reads a position-based limit per joint,
min(v_max, -offset + sqrt(2 * deceleration * distance to the limit)). The FER
never had one; libfranka 0.9.2 limits to a constant v_max. By default the
deceleration is set so large that the limit is v_max up to within about a
microradian of the joint limit, which is what 0.9.2 does. --decelerate uses the
FER's joint acceleration limits instead, as the FR3 description does.

    python notes/make_fer_description.py notes/data/fer_model_20260930-193643.npz -o notes/fer.urdf
"""

import argparse
import datetime
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import compare_fer_model  # noqa: E402  pylint: disable=wrong-import-position

# Gaz et al. 2019, as listed in their supplementary material and in
# github.com/marcocognetti/FrankaEmikaPandaDynModel: centre of mass in the link
# frame, mass, and inertia about the centre of mass in the order ixx iyy izz ixy
# ixz iyz. Link 1's centre of mass along its axis cannot be identified and is 0
# there; -0.04762 is franka_description's value, and has no effect on the model.
GAZ_2019 = {
    "link1": (
        (3.875e-03, 2.081e-03, -4.762e-02),
        4.970684,
        (7.0337e-01, 7.0661e-01, 9.1170e-03, -1.3900e-04, 6.7720e-03, 1.9169e-02),
    ),
    "link2": (
        (-3.141e-03, -2.872e-02, 3.495e-03),
        0.646926,
        (7.9620e-03, 2.8110e-02, 2.5995e-02, -3.9250e-03, 1.0254e-02, 7.0400e-04),
    ),
    "link3": (
        (2.7518e-02, 3.9252e-02, -6.6502e-02),
        3.228604,
        (3.7242e-02, 3.6155e-02, 1.0830e-02, -4.7610e-03, -1.1396e-02, -1.2805e-02),
    ),
    "link4": (
        (-5.317e-02, 1.04419e-01, 2.7454e-02),
        3.587895,
        (2.5853e-02, 1.9552e-02, 2.8323e-02, 7.7960e-03, -1.3320e-03, 8.6410e-03),
    ),
    "link5": (
        (-1.1953e-02, 4.1065e-02, -3.8437e-02),
        1.225946,
        (3.5549e-02, 2.9474e-02, 8.6270e-03, -2.1170e-03, -4.0370e-03, 2.2900e-04),
    ),
    "link6": (
        (6.0149e-02, -1.4117e-02, -1.0517e-02),
        1.666555,
        (1.9640e-03, 4.3540e-03, 5.4330e-03, 1.0900e-04, -1.1580e-03, 3.4100e-04),
    ),
    "link7": (
        (1.0517e-02, -4.252e-03, 6.1597e-02),
        7.35522e-01,
        (1.2516e-02, 1.0027e-02, 4.8150e-03, -4.2800e-04, -1.1960e-03, -7.4100e-04),
    ),
}
# FER joint acceleration limits, rad/s^2, as in libfranka 0.9.2's rate limiter.
FER_MAX_ACCELERATION = [15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]
# Large enough that the position-based limit reaches v_max within ~1e-6 rad.
CONSTANT_DECELERATION = 1e7
FIT_SAMPLES = 1000


def number(value):
    return repr(float(value))


def link_xml(name, mass, com, inertia):
    c, i = com, inertia
    return (
        f'  <link name="{name}">\n'
        "    <inertial>\n"
        f'      <origin xyz="{number(c[0])} {number(c[1])} {number(c[2])}" rpy="0 0 0"/>\n'
        f'      <mass value="{number(mass)}"/>\n'
        f'      <inertia ixx="{number(i[0, 0])}" ixy="{number(i[0, 1])}" ixz="{number(i[0, 2])}"'
        f' iyy="{number(i[1, 1])}" iyz="{number(i[1, 2])}" izz="{number(i[2, 2])}"/>\n'
        "    </inertial>\n"
        "  </link>"
    )


def seed_urdf(description):
    """franka_description's FER with the Gaz et al. 2019 inertials."""
    urdf = compare_fer_model.build_urdf(description)
    for name, (com, mass, (ixx, iyy, izz, ixy, ixz, iyz)) in GAZ_2019.items():
        inertia = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        start = urdf.index(f'<link name="{name}">')
        end = urdf.index("</link>", start) + len("</link>")
        urdf = urdf[: start - 2] + link_xml(name, mass, com, inertia) + urdf[end:]
    return urdf


def fit(recording, seed):
    """Smallest change to the seed parameters that reproduces Franka's model."""
    import pinocchio as pin

    model = pin.buildModelFromXML(seed)
    data = model.createData()
    zero = np.zeros(7)

    def regressor(q, a, gravity):
        model.gravity.linear = np.array([0, 0, -9.81]) if gravity else np.zeros(3)
        return pin.computeJointTorqueRegressor(model, data, q, zero, a)

    theta0 = np.concatenate([model.inertias[j].toDynamicParameters() for j in range(1, 8)])
    rows, rhs = [], []
    for i in range(FIT_SAMPLES):
        q = recording["q"][i]
        rows.append(regressor(q, zero, True))
        rhs.append(recording["gravity_0"][i])
        mass = recording["mass_0"][i].reshape(7, 7).T
        for k in range(7):
            rows.append(regressor(q, np.eye(7)[k], False))
            rhs.append(mass[:, k])
    a, b = np.vstack(rows), np.concatenate(rhs)
    theta = theta0 + np.linalg.lstsq(a, b - a @ theta0, rcond=None)[0]
    return [pin.Inertia.FromDynamicParameters(theta[10 * j : 10 * j + 10]) for j in range(7)]


def build(recording_path, decelerate):
    recording = np.load(recording_path)
    description = compare_fer_model.fetch_description(
        pathlib.Path.home() / ".cache" / "panda-py" / "fer-description"
    )
    inertias = fit(recording, seed_urdf(description))
    kinematics, limits = description["kinematics"], description["joint_limits"]

    def origin(name):
        k = kinematics[name]["kinematic"]
        return (
            f'<origin xyz="{k["x"]} {k["y"]} {k["z"]}"'
            f' rpy="{k["roll"]} {k["pitch"]} {k["yaw"]}"/>'
        )

    lines = [
        '<?xml version="1.0"?>',
        "<!--",
        "  Franka Emika Robot (FER, Panda) for libfranka's Pinocchio model, generated by",
        f"  panda-py notes/make_fer_description.py on {datetime.date.today()}.",
        "  Kinematics and joint limits: franka_description robots/fer, commit"
        f" {compare_fer_model.DESCRIPTION_COMMIT}.",
        "  Inertials: fitted to Franka's FER model library (libfcimodels via libfranka",
        "  0.9.2), seeded with the parameters identified by Gaz et al., IEEE RA-L 4(4),",
        "  2019. They reproduce that model's gravity, mass matrix and Coriolis terms,",
        "  but are not physically consistent, as no physically consistent set can.",
        "  Velocity limits: "
        + (
            "position based, from the FER joint acceleration limits."
            if decelerate
            else "constant, as in libfranka 0.9.2."
        ),
        "-->",
        '<robot name="fer">',
        '  <link name="base"/>',
        '  <joint name="base_joint" type="fixed">',
        '    <origin xyz="0 0 0" rpy="0 0 0"/>',
        '    <parent link="base"/>\n    <child link="link0"/>',
        "  </joint>",
        '  <link name="link0"/>',
    ]
    for j in range(1, 8):
        inertia = inertias[j - 1]
        lim = limits[f"joint{j}"]["limit"]
        deceleration = FER_MAX_ACCELERATION[j - 1] if decelerate else CONSTANT_DECELERATION
        lines += [
            link_xml(f"link{j}", inertia.mass, inertia.lever, inertia.inertia),
            f'  <joint name="joint{j}" type="revolute">',
            f"    {origin(f'joint{j}')}",
            f'    <parent link="link{j - 1}"/>\n    <child link="link{j}"/>',
            '    <axis xyz="0 0 1"/>',
            f'    <limit effort="{lim["effort"]}" lower="{lim["lower"]}"'
            f' upper="{lim["upper"]}" velocity="{lim["velocity"]}"/>',
            f'    <position_based_velocity_limits deceleration_limit="{deceleration}"'
            ' velocity_offset="0.0"/>',
            "  </joint>",
        ]
    lines += [
        '  <link name="link8"/>',
        '  <joint name="joint8" type="fixed">',
        f"    {origin('joint8')}",
        '    <parent link="link7"/>\n    <child link="link8"/>',
        "  </joint>",
        "</robot>",
    ]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("recording", help="fer_model_<time>.npz from compare_fer_model.py")
    parser.add_argument("-o", "--output", default="fer.urdf")
    parser.add_argument("--decelerate", action="store_true", help="position-based velocity limits")
    args = parser.parse_args()
    pathlib.Path(args.output).write_text(build(args.recording, args.decelerate))
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
