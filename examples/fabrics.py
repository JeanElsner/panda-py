import sys
import typing

import numpy as np

import panda_py
from panda_py import controllers

_Shape = typing.Tuple[int, ...]

if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')

    stiffness = np.array([600., 600., 600., 600., 250., 150., 50.]) / 10

    ctrl = controllers.IntegratedVelocity(stiffness=stiffness)
    panda = panda_py.Panda(sys.argv[1])
    q_0 = panda.get_state().q
    panda.move_to_start()
    panda.start_controller(ctrl)
    runtime = 5
    qdot_des = np.zeros(7)

    joint_positions = []
    joint_velocities = []



    with panda.create_context(frequency=100, max_runtime=runtime) as ctx:
        while ctx.ok():
            #qdot_des[0] += 0.01 * np.cos(ctrl.get_time())
            qdot_des = 0.3 * np.cos(ctrl.get_time()) * np.ones(7)
            ctrl.set_control(qdot_des)
            state = panda.get_state()
            joint_positions.append(state.q)
            joint_velocities.append(state.dq)

    np.save('q', np.array(joint_positions))
    np.save('q_d', np.array(joint_velocities))