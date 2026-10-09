"""
Introduction
------------

panda-py is a Python library for the Franka Emika Robot System
that allows you to program and control the robot in real-time.


"""
from __future__ import annotations
import base64 as base64
import configparser as configparser
import dataclasses as dataclasses
import hashlib as hashlib
import json as json_module
import logging as logging
import os as os
from panda_py._core import IKResult
from panda_py._core import Panda
from panda_py._core import PandaContext
from panda_py._core import RobotLimits
from panda_py._core import RobotType
from panda_py._core import _ik
from panda_py._core import conservative_limits
from panda_py._core import fk
from panda_py._core import jacobian
from panda_py._core import limits
from panda_py._core import realtime_priority_available
from panda_py.exceptions import IKError
from panda_py.exceptions import IncompatibleVersionError
import requests as requests
import ssl as ssl
import threading as threading
import typing as typing
from urllib import parse
import urllib3 as urllib3
from websockets.sync.client import connect
from . import libfranka
from . import _core
from . import exceptions
__all__: list = ['Panda', 'PandaContext', 'RobotLimits', 'RobotType', 'conservative_limits', 'limits', 'constants', 'controllers', 'libfranka', 'motion', 'fk', 'ik', 'jacobian', 'IKResult', 'IKError', 'realtime_priority_available', 'IncompatibleVersionError', 'Desk', 'TOKEN_PATH']
class Token:
    """
    Represents a Desk token owned by a user.
    """
    __dataclass_fields__: typing.ClassVar[dict]
    __dataclass_params__: typing.ClassVar[dataclasses._DataclassParams]
    __hash__: typing.ClassVar[None] = None
    __match_args__: typing.ClassVar[tuple] = ('id', 'owned_by', 'token')
    id: typing.ClassVar[str] = ''
    owned_by: typing.ClassVar[str] = ''
    token: typing.ClassVar[str] = ''
    def __eq__(self, other):
        ...
    def __init__(self, id: str = '', owned_by: str = '', token: str = '') -> None:
        ...
    def __replace__(self, **changes):
        ...
    def __repr__(self):
        ...
class Desk:
    """
    Connects to the control unit running the web-based Desk interface
    to manage the robot. Use this class to interact with the Desk
    from Python, e.g. if you use a headless setup. This interface
    supports common tasks such as unlocking the brakes, activating
    the FCI etc.
    
    Newer versions of the system software use role-based access
    management to allow only one user to be in control of the Desk
    at a time. The controlling user is authenticated using a token.
    The :py:class:`Desk` class saves those token in :py:obj:`TOKEN_PATH`
    and will use them when reconnecting to the Desk, retaking control.
    Without a token, control of a Desk can only be taken, if there is
    no active claim or the controlling user explicitly relinquishes control.
    If the controlling user's token is lost, a user can take control
    forcefully (cf. :py:func:`Desk.take_control`) but needs to confirm
    physical access to the robot by pressing the circle button on the
    robot's Pilot interface.
    
    The FER and the FR3 serve mutually exclusive endpoints for the brakes.
    Which one to use is detected on the first call to :py:func:`Desk.lock`
    or :py:func:`Desk.unlock`, so the ``platform`` argument is optional.
    """
    _BRAKE_ENDPOINTS: typing.ClassVar[dict] = {'panda': {'lock': '/desk/api/robot/close-brakes', 'unlock': '/desk/api/robot/open-brakes'}, 'fr3': {'lock': '/desk/api/joints/lock', 'unlock': '/desk/api/joints/unlock'}}
    @staticmethod
    def _is_missing_endpoint(response: requests.models.Response) -> bool:
        """
        Whether a response means the endpoint does not exist on this robot.
        
        The FR3 answers an unknown path with ``No handler accepted "<path>"``
        and the FER with ``File not found``. Both come back as 404, but the
        body is matched as well so that a firmware using another status code
        for the same condition is still recognised.
        """
    @staticmethod
    def encode_password(username: str, password: str) -> bytes:
        """
        Encodes the password into the form needed to log into the Desk interface.
        """
    def __init__(self, hostname: str, username: str, password: str, platform: str | None = None) -> None:
        ...
    def _brakes(self, action: typing.Literal['lock', 'unlock'], force: bool, headers: dict[str, str] = None) -> None:
        """
        Operates the brakes, discovering which endpoint this robot serves.
        
        The FER and the FR3 each serve only their own pair of brake endpoints
        and answer 404 for the other's, without touching the brakes. So the
        platform can be detected by trying and retrying, which costs nothing
        when the hint is right and needs no separate probe request.
        """
    def _detected_platform(self, platform: str) -> None:
        """
        Records the platform a brake call succeeded on.
        """
    def _get_active_token(self) -> Token:
        ...
    def _listen(self, cb, timeout):
        ...
    def _load_token(self) -> Token:
        ...
    def _request(self, method: typing.Literal['post', 'get', 'delete'], url: str, json: dict[str, str] = None, headers: dict[str, str] = None, files: dict[str, str] = None, check: bool = True) -> requests.models.Response:
        ...
    def _save_token(self, token: Token) -> None:
        ...
    def activate_fci(self) -> None:
        """
        Activates the Franka Research Interface (FCI). Note that the
        brakes must be unlocked first. For older Desk versions, this
        function does nothing.
        """
    def deactivate_fci(self) -> None:
        """
        Deactivates the Franka Research Interface (FCI). For older
        Desk versions, this function does nothing.
        """
    def has_control(self) -> bool:
        """
        Returns:
          bool: True if this instance is in control of the Desk.
        """
    def listen(self, cb: typing.Callable[[dict], NoneType]) -> None:
        """
        Starts a thread listening to Pilot button events. All the Pilot buttons,
        except for the `Pilot Mode` button can be captured. Make sure Pilot Mode is
        set to Desk instead of End-Effector to receive direction key events. You can
        change the Pilot mode by pressing the `Pilot Mode` button or changing the mode
        in the Desk. Events will be triggered while buttons are pressed down or released.
        
        Args:
          cb: Callback fucntion that is called whenever a button event is received from the
            Desk. The callback receives a dict argument that contains the triggered buttons
            as keys. The values of those keys will depend on the kind of event, either True
            for a button pressed down or False when released.
            The possible buttons are: `circle`, `cross`, `check`, `left`, `right`, `down`,
            and `up`.
        """
    def lock(self, force: bool = True) -> None:
        """
        Locks the brakes. API call blocks until the brakes are locked.
        """
    def login(self) -> None:
        """
        Uses the object's instance parameters to log into the Desk.
        The :py:class`Desk` class's constructor will try to connect
        and login automatically.
        """
    def logout(self) -> None:
        """
        Logs the current user out of the Desk. API calls will no longer
        be possible.
        """
    def reboot(self) -> None:
        """
        Reboots the robot hardware (this will close open connections).
        """
    def release_control(self) -> None:
        """
        Explicitly relinquish control of the Desk. This will allow
        other users to take control or transfer control to the next
        user if there is an active queue of control requests.
        """
    def stop_listen(self) -> None:
        """
        Stop listener thread (cf. :py:func:`panda_py.Desk.listen`).
        """
    def take_control(self, force: bool = False) -> bool:
        """
        Takes control of the Desk, generating a new control token and saving it.
        If `force` is set to True, control can be taken forcefully even if another
        user is already in control. However, the user will have to press the circle
        button on the robot's Pilot within an alotted amount of time to confirm
        physical access.
        
        For legacy versions of the Desk, this function does nothing.
        """
    def unlock(self, force: bool = True) -> None:
        """
        Unlocks the brakes. API call blocks until the brakes are unlocked.
        """
def ik(pose, q_init = None, *, limits: ForwardRef('RobotLimits') | None = None, F_T_EE = None, position_tolerance: float = 1e-05, orientation_tolerance: float = 0.0001, max_iterations: int = 200, restarts: int = 20):
    """
    Inverse kinematics: joint positions that put the end effector at a pose.
    
    Numerical (damped least squares), within the joint limits, starting from
    ``q_init`` and drawing the arm's redundancy toward it, so the solution is
    the one near ``q_init``: pass the current joint positions to stay in the
    same configuration. If that start does not converge, ``restarts`` more
    are tried from random joint positions within the limits (always the same
    ones, so a call is deterministic).
    
    Args:
      pose: The end effector's pose in the base frame, a 4x4 transform, or a
        ``(position, orientation)`` pair with a scalar-last quaternion.
      q_init: Joint positions to start from and stay near; by default the
        start pose (:py:data:`panda_py.constants.JOINT_POSITION_START`).
      limits: The joint limits to respect, e.g. :py:attr:`Panda.limits`; by
        default :py:func:`conservative_limits`, valid on the FER and the FR3.
      F_T_EE: The end effector relative to the flange; by default the Franka
        Hand's, as :py:func:`fk`.
      position_tolerance: Accepted position error, m.
      orientation_tolerance: Accepted orientation error, rad.
      max_iterations: Iterations per start.
      restarts: Further starts if the first does not converge.
    
    Returns:
      The joint positions, shape (7,).
    
    Raises:
      IKError: No solution within the limits and tolerances; its ``result``
        holds the best found.
    """
TOKEN_PATH: str = '~/.panda_py/token.conf'
__version__: str = '2.0.0.dev0'
_logger: logging.Logger
