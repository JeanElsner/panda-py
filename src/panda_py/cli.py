"""
This submodule contains convenience functions that are installed
as executables. These executables can be integrated into bash
scripts, alias commands etc. The table below lists the available
executables and the corresponding entry-points.

+--------------+-------------------+
| Executable   | Entry-point       |
+==============+===================+
| panda-unlock | :py:func:`unlock` |
+--------------+-------------------+
| panda-lock   | :py:func:`lock`   |
+--------------+-------------------+
| panda-reboot | :py:func:`reboot` |
+--------------+-------------------+

"""
import argparse
from . import Desk


def unlock():
  """
  Unlocks the robot's brakes and activates the FCI.

  Args:
    host: IP or hostname of the control unit running the Desk.
    user: Username used to log into the Desk.
    password: Password of the given username.
  """
  parser = argparse.ArgumentParser()
  parser.add_argument('host',  type=str, help='Robot Desk IP or hostname.')
  parser.add_argument('user', type=str, help='Desk username.')
  parser.add_argument('password', type=str, help='Desk password.')
  args = parser.parse_args()

  desk = Desk(args.host, args.user, args.password)
  desk.unlock()
  desk.activate_fci()


def lock():
  """
  Locks the robot's brakes and deactivates the FCI.

  Args:
    host: IP or hostname of the control unit running the Desk.
    user: Username used to log into the Desk.
    password: Password of the given username.
  """
  parser = argparse.ArgumentParser()
  parser.add_argument('host',  type=str, help='Robot Desk IP or hostname.')
  parser.add_argument('user', type=str, help='Desk username.')
  parser.add_argument('password', type=str, help='Desk password.')
  args = parser.parse_args()

  desk = Desk(args.host, args.user, args.password)
  desk.lock()
  desk.deactivate_fci()


def reboot():
  """
  Reboots the robot. Current versions of the robot's Desk software
  will eventually hang up when running the FCI continuously for
  several days. This function can be conveniently  integrated
  into cronjobs or similar to regularly reboot the robot.

  Args:
    host: IP or hostname of the control unit running the Desk.
    user: Username used to log into the Desk.
    password: Password of the given username.
  """
  parser = argparse.ArgumentParser()
  parser.add_argument('host',  type=str, help='Robot Desk IP or hostname.')
  parser.add_argument('user', type=str, help='Desk username.')
  parser.add_argument('password', type=str, help='Desk password.')
  args = parser.parse_args()

  desk = Desk(args.host, args.user, args.password)
  desk.reboot()
