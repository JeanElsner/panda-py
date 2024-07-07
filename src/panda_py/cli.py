"""
This submodule contains convenience functions that are installed
as executables. These executables can be integrated into bash
scripts, alias commands etc. The table below lists the available
executables and the corresponding entry-points.

+-----------------------+----------------------------+
| Executable            | Entry-point                |
+=======================+============================+
| panda-unlock          | :py:func:`unlock`          |
+-----------------------+----------------------------+
| panda-lock            | :py:func:`lock`            |
+-----------------------+----------------------------+
| panda-reboot          | :py:func:`reboot`          |
+-----------------------+----------------------------+
| panda-take-control    | :py:func:`take_control`    |
+-----------------------+----------------------------+
| panda-release-control | :py:func:`release_control` |
+-----------------------+----------------------------+

"""
import argparse

from . import Desk


def _create_argument_parser(needs_platform: bool = True) -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser()
  parser.add_argument('host',  type=str, help='Robot Desk IP or hostname.')
  parser.add_argument('user', type=str, help='Desk username.')
  parser.add_argument('password', type=str, help='Desk password.')
  if needs_platform:
    parser.add_argument('--platform', type=str, default='panda', help='Platform of robot, i.e. panda (default) or fr3.')
  return parser


def unlock():
  """
  Unlocks the robot's brakes and activates the FCI.

  Args:
    host: IP or hostname of the control unit running the Desk.
    user: Username used to log into the Desk.
    password: Password of the given username.
    platform: The targeted robot platform, i.e. panda or fr3.
  """
  parser = _create_argument_parser()
  args = parser.parse_args()

  desk = Desk(args.host, args.user, args.password, platform=args.platform)
  desk.unlock()
  desk.activate_fci()


def lock():
  """
  Locks the robot's brakes and deactivates the FCI.

  Args:
    host: IP or hostname of the control unit running the Desk.
    user: Username used to log into the Desk.
    password: Password of the given username.
    platform: The targeted robot platform, i.e. panda or fr3.
  """
  parser = _create_argument_parser()
  args = parser.parse_args()

  desk = Desk(args.host, args.user, args.password, platform=args.platform)
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
  parser = _create_argument_parser(needs_platform=False)
  args = parser.parse_args()

  desk = Desk(args.host, args.user, args.password)
  desk.reboot()


def take_control():
  """
  Take control of the Desk with the given user credentials.

  Args:
    host: IP or hostname of the control unit running the Desk.
    user: Username used to log into the Desk.
    password: Password of the given username.
    force: Forcibly take control from another active user
      by pressing circle button on physical robot.
  """
  parser = _create_argument_parser(needs_platform=False)
  parser.add_argument('--force', action='store_true', help='Force takeover if another user is in control.')
  args = parser.parse_args()

  desk = Desk(args.host, args.user, args.password)
  desk.take_control(force=args.force)


def release_control():
  """
  Release control of the desk. This will allow another user to
  take control without needing physical access to the robot.

  Args:
    host: IP or hostname of the control unit running the Desk.
    user: Username used to log into the Desk.
    password: Password of the given username.
    force: Forcibly take control from another active user
      by pressing circle button on physical robot.
  """
  parser = _create_argument_parser(needs_platform=False)
  args = parser.parse_args()

  desk = Desk(args.host, args.user, args.password)
  desk.release_control()
