"""
Lowers the end effector until it touches something, then stops: the force
guard of TaskImpedance drops the spring on the tick the external force
exceeds 8 N, and only damping remains. The telemetry shows when and where.

@warning Clear the space below the end effector down to the surface it is
to touch, at most 20 cm below the start pose; keep the user stop at hand.
"""

import sys
import time

import numpy as np

import panda_py
from panda_py import controllers

if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise RuntimeError(f"Usage: python {sys.argv[0]} <robot-hostname>")

    panda = panda_py.Panda(sys.argv[1])
    panda.move_to_start()

    controller = controllers.TaskImpedance(telemetry=30000)
    # The force estimate has a bias; measure it in free space and subtract it.
    bias = np.mean([panda.get_state().O_F_ext_hat_K[:3] for _ in range(100)], axis=0)
    controller.set_guard(force=8.0, force_time=0.02, speed=0.2, force_bias=bias)
    panda.start_controller(controller)

    position, orientation = panda.get_position(), panda.get_orientation()
    lowest = position[2] - 0.2
    with panda.create_context(frequency=100, max_runtime=12) as ctx:
        while ctx.ok() and not controller.guard_state["tripped"]:
            # 2 cm/s down, to at most 20 cm below the start.
            position[2] = max(position[2] - 0.0002, lowest)
            controller.set_reference(position, orientation)
    time.sleep(0.5)
    telemetry = controller.read_telemetry()
    panda.stop_controller()

    state = controller.guard_state
    if state["tripped"]:
        tick = np.flatnonzero(telemetry["guard"])[0]
        print(f"Stopped by the {state['reason']} guard at {state['value']:.1f}, "
              f"{telemetry['position'][tick, 2] * 1e3:.1f} mm above the base.")
    else:
        print("Nothing touched within 20 cm.")
    panda.move_to_start()
