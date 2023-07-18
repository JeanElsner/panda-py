"""
This is an implementation of the controller from:
J. Haviland and P. Corke, “A purely-reactive manipulability-maximising
motion controller,” arXiv preprint arXiv:2002.11901,2020.
"""
import sys

import numpy as np
import qpsolvers as qp
import roboticstoolbox as rtb
import spatialmath as sm

import panda_py
from panda_py import controllers

if __name__ == '__main__':
  if len(sys.argv) < 2:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')

  # Initialize robot hardware and controller
  ctrl = controllers.IntegratedVelocity()
  panda = panda_py.Panda(sys.argv[1])
  panda.move_to_start()
  panda.start_controller(ctrl)

  # Initialize roboticstoolbox model
  panda_rtb = rtb.models.Panda()

  # Set the desired end-effector pose
  Tep = panda_rtb.fkine(panda.q) * sm.SE3(0.3, 0.2, 0.3)

  # Number of joint in the panda which we are controlling
  n = 7

  arrived = False

  with panda.create_context(frequency=20) as ctx:
    while ctx.ok() and not arrived:

      # The pose of the Panda's end-effector
      Te = panda_rtb.fkine(panda.q)

      # Transform from the end-effector to desired pose
      eTep = Te.inv() * Tep

      # Spatial error
      e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))

      # Calulate the required end-effector spatial velocity for the robot
      # to approach the goal. Gain is set to 1.0
      v, arrived = rtb.p_servo(Te, Tep, 1.0)

      # Gain term (lambda) for control minimisation
      Y = 0.01

      # Quadratic component of objective function
      Q = np.eye(n + 6)

      # Joint velocity component of Q
      Q[:n, :n] *= Y

      # Slack component of Q
      Q[n:, n:] = (1 / e) * np.eye(6)

      # The equality contraints
      Aeq = np.c_[panda_rtb.jacobe(panda.q), np.eye(6)]
      beq = v.reshape((6,))

      # Linear component of objective function: the manipulability Jacobian
      c = np.r_[-panda_rtb.jacobm().reshape((n,)), np.zeros(6)]

      # The lower and upper bounds on the joint velocity and slack variable
      lb = -np.r_[panda_rtb.qdlim[:n], 10 * np.ones(6)]
      ub = np.r_[panda_rtb.qdlim[:n], 10 * np.ones(6)]

      # Solve for the joint velocities dq
      qd = qp.solve_qp(Q, c, None, None, Aeq, beq, lb=lb, ub=ub, solver='daqp')

      # Apply the joint velocities to the Panda
      ctrl.set_control(qd[:n])
