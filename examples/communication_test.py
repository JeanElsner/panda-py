"""
Measures the quality of the connection to the robot, as libfranka's
`communication_test` does, from the controller's 1 kHz telemetry.

Runs JointTorque with zero torque for 10 seconds: with the end effector
configured correctly in Desk, the robot holds still. Every control tick is
recorded, so the numbers are exact: ticks the robot ran without a command
from this computer show up as durations above 1 ms.

@warning Before executing this example, make sure there is enough space in front of the robot.
"""

import sys

import numpy as np

import panda_py
from panda_py import controllers, telemetry

if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise RuntimeError(f"Usage: python {sys.argv[0]} <robot-hostname>")

    panda = panda_py.Panda(sys.argv[1])
    controller = controllers.JointTorque(telemetry=20000)
    recorder = telemetry.Recorder(controller)
    panda.start_controller(controller)
    recorder.start()
    with panda.create_context(frequency=10, max_runtime=10) as ctx:
        while ctx.ok():
            pass
    recorder.stop()
    panda.stop_controller()
    log = recorder.result()

    success = log["control_command_success_rate"]
    durations = np.diff(log["time"])
    missed = int(np.round(durations / 1e-3).sum() - len(durations))
    print(f"{len(log['tick'])} control ticks in {log['time'][-1] - log['time'][0]:.1f} s")
    print(f"control command success rate: min {success.min():.3f}, "
          f"mean {success.mean():.3f}, max {success.max():.3f}")
    print(f"robot cycles without a command from this computer: {missed}")
    print(f"longest gap between commands: {durations.max() * 1e3:.1f} ms")
    if success.mean() < 0.9:
        print("\nWARNING: this setup is probably not sufficient for FCI; try another PC or NIC.")
    elif success.mean() < 0.95:
        print("\nWARNING: many packets got lost; see "
              "https://frankarobotics.github.io/docs/troubleshooting.html")
