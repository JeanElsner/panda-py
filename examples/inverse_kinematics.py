"""
Moves the end effector through the corners of a square in front of the
robot: each corner's joint positions come from panda_py.ik, solved from the
previous ones so the arm stays in the same configuration, within the
connected robot's joint limits.

@warning The robot moves up to 10 cm from its start pose.
"""

import sys

import numpy as np

import panda_py

if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise RuntimeError(f"Usage: python {sys.argv[0]} <robot-hostname>")

    panda = panda_py.Panda(sys.argv[1])
    panda.move_to_start()
    state = panda.get_state()
    # The end effector configured in Desk, so ik solves for the same frame
    # the robot reports.
    F_T_EE = np.asarray(state.F_T_EE).reshape(4, 4, order="F")
    start = panda.get_pose()

    q = panda.q
    waypoints = []
    for dy, dz in ((0.1, 0.0), (0.1, -0.1), (-0.1, -0.1), (-0.1, 0.0), (0.0, 0.0)):
        goal = start.copy()
        goal[1, 3] += dy
        goal[2, 3] += dz
        q = panda_py.ik(goal, q_init=q, limits=panda.limits, F_T_EE=F_T_EE)
        waypoints.append(q)

    panda.move_to_joint_position(waypoints, speed_factor=0.2)
    print(f"Back at the start, {np.linalg.norm(panda.get_position() - start[:3, 3]) * 1e3:.1f} mm away.")
