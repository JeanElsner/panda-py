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
import ssl as ssl
import threading as threading
import typing as typing
from urllib import parse

import requests as requests
import urllib3 as urllib3
from websockets.sync.client import connect

from panda_py._core import Panda, PandaContext, fk, ik, ik_full

from . import _core, libfranka

__all__: list = [
    'Panda', 'PandaContext', 'constants', 'controllers', 'libfranka', 'motion',
    'fk', 'ik', 'ik_full', 'Desk', 'TOKEN_PATH'
]


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
      
    """

  @staticmethod
  def encode_password(username: str, password: str) -> bytes:
    """
        
            Encodes the password into the form needed to log into the Desk interface.
            
        """

  def __init__(self,
               hostname: str,
               username: str,
               password: str,
               platform: str = 'panda') -> None:
    ...

  def _get_active_token(self) -> Token:
    ...

  def _listen(self, cb, timeout):
    ...

  def _load_token(self) -> Token:
    ...

  def _request(self,
               method: typing.Literal['post', 'get', 'delete'],
               url: str,
               json: typing.Dict[str, str] = None,
               headers: typing.Dict[str, str] = None,
               files: typing.Dict[str, str] = None) -> requests.models.Response:
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

  def listen(self, cb: typing.Callable[[typing.Dict], NoneType]) -> None:
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


class Token:
  """
    
      Represents a Desk token owned by a user.
      
    """
  __dataclass_fields__: typing.ClassVar[
      dict]  # value = {'id': Field(name='id',type=<class 'str'>,default='',default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD), 'owned_by': Field(name='owned_by',type=<class 'str'>,default='',default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD), 'token': Field(name='token',type=<class 'str'>,default='',default_factory=<dataclasses._MISSING_TYPE object>,init=True,repr=True,hash=None,compare=True,metadata=mappingproxy({}),kw_only=False,_field_type=_FIELD)}
  __dataclass_params__: typing.ClassVar[
      dataclasses.
      _DataclassParams]  # value = _DataclassParams(init=True,repr=True,eq=True,order=False,unsafe_hash=False,frozen=False,match_args=True,kw_only=False,slots=False,weakref_slot=False)
  __hash__: typing.ClassVar[None] = None
  __match_args__: typing.ClassVar[tuple] = ('id', 'owned_by', 'token')
  id: typing.ClassVar[str] = ''
  owned_by: typing.ClassVar[str] = ''
  token: typing.ClassVar[str] = ''

  def __eq__(self, other):
    ...

  def __init__(self, id: str = '', owned_by: str = '', token: str = '') -> None:
    ...

  def __repr__(self):
    ...


TOKEN_PATH: str = '~/.panda_py/token.conf'
__version__: str = '0.8.1'
_logger: logging.Logger  # value = <Logger desk (INFO)>
