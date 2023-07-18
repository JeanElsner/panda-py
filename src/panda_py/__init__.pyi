"""
Introduction
------------

panda-py is a Python library for the Franka Emika Robot System
that allows you to program and control the robot in Python.


"""
from __future__ import annotations
import panda_py
import typing
from panda_py._core import Panda
from panda_py._core import PandaContext
import base64
import configparser
import dataclasses
import hashlib
import json
import logging
import numpy
import os
import requests
import urllib.parse
import urllib3
_Shape = typing.Tuple[int, ...]

__all__ = [
    "Desk",
    "Panda",
    "PandaContext",
    "TOKEN_PATH",
    "constants",
    "controllers",
    "fk",
    "ik",
    "ik_full",
    "libfranka",
    "motion"
]


class Desk():
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
    """
    pass
class Token():
    """
    Represents a Desk token owned by a user.
    """
    __dataclass_fields__: dict # value = {'id': Field(name='id',type=<class 'str'>,default='',default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD), 'owned_by': Field(name='owned_by',type=<class 'str'>,default='',default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD), 'token': Field(name='token',type=<class 'str'>,default='',default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD)}
    __dataclass_params__: dataclasses._DataclassParams # value = _DataclassParams(init=True,repr=True,eq=True,order=False,unsafe_hash=False,frozen=False)
    __hash__ = None
    __match_args__ = ('id', 'owned_by', 'token')
    id = ''
    owned_by = ''
    token = ''
    pass
def fk(q: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> numpy.ndarray[numpy.float64, _Shape[4, 4]]:
    """
    Computes end-effector pose in base frame from joint positions.
    """
@typing.overload
def ik(O_T_EE: numpy.ndarray[numpy.float64, _Shape[4, 4]], q_init: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0.        , -0.78539816,  0.        , -2.35619449,  0.        , 1.57079633,  0.78539816]), q_7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]:
    """
    Compute analytical inverse kinematics. 
    Solution is case consistent with configuration  given in `q_init`.

    Args:
      O_T_EE: Homogeneous transform :math:`\mathbb{R}^{4\times 4}` describing
        the end-effector pose.
      q_init: Reference joint positions, the result will be consistent
        with this configuration.
      q_7: Joint 7 is considered the redundant joint, use `q_7` to set the
        desired joint position (default: :math:`\frac{\pi}{4}`).

    Returns:
      Vector of shape (7,) containing joint positions.



    Same as :py:func:`ik` above, but takes position and orientation arguments.
    """
@typing.overload
def ik(position: numpy.ndarray[numpy.float64, _Shape[3, 1]], orientation: numpy.ndarray[numpy.float64, _Shape[4, 1]], q_init: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0.        , -0.78539816,  0.        , -2.35619449,  0.        , 1.57079633,  0.78539816]), q_7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[7, 1]]:
    pass
@typing.overload
def ik_full(O_T_EE: numpy.ndarray[numpy.float64, _Shape[4, 4]], q_init: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0.        , -0.78539816,  0.        , -2.35619449,  0.        , 1.57079633,  0.78539816]), q_7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[4, 7]]:
    pass
@typing.overload
def ik_full(position: numpy.ndarray[numpy.float64, _Shape[3, 1]], orientation: numpy.ndarray[numpy.float64, _Shape[4, 1]], q_init: numpy.ndarray[numpy.float64, _Shape[7, 1]] = array([ 0.        , -0.78539816,  0.        , -2.35619449,  0.        , 1.57079633,  0.78539816]), q_7: float = 0.7853981633974483) -> numpy.ndarray[numpy.float64, _Shape[4, 7]]:
    pass
TOKEN_PATH = '~/.panda_py/token.conf'
__all__ = ['Panda', 'PandaContext', 'constants', 'controllers', 'libfranka', 'motion', 'fk', 'ik', 'ik_full', 'Desk', 'TOKEN_PATH']
_logger: logging.Logger # value = <Logger desk (INFO)>
