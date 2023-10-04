"""
Simple teaching demonstration. Teaches three joint positions
and connects them using motion generators. Also teaches a joint
trajectory that is then replayed.
"""
import sys
import time

import panda_py
from panda_py import controllers

if __name__ == '__main__':
  if len(sys.argv) < 2:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')

  panda = panda_py.Panda(sys.argv[1])

  print('Please teach three poses to the robot.')
  positions = []
  panda.teaching_mode(True)
  for i in range(3):
    print(f'Move the robot into pose {i+1} and press enter to continue.')
    input()
    positions.append(panda.q)

  panda.teaching_mode(False)
  input('Press enter to move through the three poses.')
  panda.move_to_joint_position(positions)

  LEN = 10
  input(
      f'Next, teach a trajectory for {LEN} seconds. Press enter to begin.')
  panda.teaching_mode(True)
  panda.enable_logging(LEN * 1000)
  time.sleep(LEN)
  panda.teaching_mode(False)

  q = panda.get_log()['q']
  dq = panda.get_log()['dq']

  input('Press enter to replay trajectory')
  panda.move_to_joint_position(q[0])
  i = 0
  ctrl = controllers.JointPosition()
  panda.start_controller(ctrl)
  with panda.create_context(frequency=1000, max_runtime=LEN) as ctx:
    while ctx.ok():
      ctrl.set_control(q[i], dq[i])
      i += 1
