"""
Recreation of `communication_test.cpp` from libfranka using `panda_py`.

An example indicating the network performance.

@warning Before executing this example, make sure there is enough space in front of the robot.
"""
import sys

import panda_py
from panda_py import constants, controllers

if __name__ == '__main__':
  if len(sys.argv) < 2:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')

  COUNTER = 0
  AVG_SUCCESS_RATE = 0.0
  MIN_SUCCESS_RATE = 1.0
  MAX_SUCCESS_RATE = 0.0
  TIME = 0

  panda = panda_py.Panda(sys.argv[1])
  controller = controllers.AppliedTorque()
  # Sending zero torques - if EE is configured correctly, robot should not move
  panda.start_controller(controller)

  # Run a loop at 1kHz for 10 seconds.
  # This thread is decoupled from the control loop,
  # as such the number of lost packages is an estimate.
  with panda.create_context(frequency=1e3, max_runtime=10) as ctx:
    while ctx.ok():
      success_rate = panda.get_state().control_command_success_rate
      if ctx.num_ticks % 100 == 0:
        print(f'#{ctx.num_ticks} Current success rate: {success_rate:.3f}')
      AVG_SUCCESS_RATE += success_rate
      if success_rate > MAX_SUCCESS_RATE:
        MAX_SUCCESS_RATE = success_rate
      if success_rate < MIN_SUCCESS_RATE and success_rate > 0:
        MIN_SUCCESS_RATE = success_rate
    panda.stop_controller()
    TIME = controller.get_time()
    COUNTER = ctx.num_ticks
    print('\nFinished test, shutting down example')

  AVG_SUCCESS_RATE /= COUNTER
  print('\n')
  print('#######################################################')
  lost_robot_states = TIME * 1000 * (1 - AVG_SUCCESS_RATE)
  if lost_robot_states > 0:
    print(
        f'The control loop did not get executed {lost_robot_states:.0f} times in the'
    )
    print(
        f'last {round(TIME*1000)} milliseconds (lost {lost_robot_states:.0f}) robot states'
    )
    print('\n')
  print(f'Control command success rate of {COUNTER} samples:')
  print(f'Max: {MAX_SUCCESS_RATE:.3f}')
  print(f'Avg: {AVG_SUCCESS_RATE:.3f}')
  print(f'Min: {MIN_SUCCESS_RATE:.3f}')
  if AVG_SUCCESS_RATE < 0.9:
    print('\nWARNING: THIS SETUP IS PROBABLY NOT SUFFICIENT FOR FCI!')
    print('PLEASE TRY OUT A DIFFERENT PC / NIC')
  if AVG_SUCCESS_RATE < 0.95:
    print('\nWARNING: MANY PACKETS GOT LOST!')
    print('PLEASE INSPECT YOUR SETUP AND FOLLOW ADVICE ON')
    print('https://frankaemika.github.io/docs/troubleshooting.html')
  print('#######################################################')
