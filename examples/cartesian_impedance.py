import sys

import numpy as np

import panda_py
from panda_py import controllers

if __name__ == '__main__':
  if len(sys.argv) < 2:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')

  panda = panda_py.Panda(sys.argv[1])
  panda.move_to_start()
  ctrl = controllers.CartesianImpedance(filter_coeff=1.0)
  x0 = panda.get_position()
  q0 = panda.get_orientation()
  runtime = np.pi * 4.0
  panda.start_controller(ctrl)

  with panda.create_context(frequency=1e3, max_runtime=runtime) as ctx:
    while ctx.ok():
      x_d = x0.copy()
      x_d[1] += 0.1 * np.sin(ctrl.get_time())
      ctrl.set_control(x_d, q0)
