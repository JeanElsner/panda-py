"""
Introduction
------------

panda-py is a Python library for the Franka Emika Robot System
that allows you to program and control the robot in real-time.


"""

import base64
import configparser
import dataclasses
import hashlib
import json as json_module
import logging
import os
import ssl
import threading
import typing
from urllib import parse

import requests
from requests.packages import urllib3
from websockets.sync.client import connect

# pylint: disable=no-name-in-module
from ._core import Panda, PandaContext, fk, ik, ik_full

__all__ = [
    'Panda', 'PandaContext', 'constants', 'controllers', 'libfranka', 'motion',
    'fk', 'ik', 'ik_full', 'Desk', 'TOKEN_PATH'
]

__version__ = '0.7.2'

_logger = logging.getLogger('desk')

TOKEN_PATH = '~/.panda_py/token.conf'
"""
Path to the configuration file holding known control tokens.
If :py:class:`Desk` is used to connect to a control unit's
web interface and takes control, the generated token is stored
in this file under the unit's IP address or hostname.
"""


@dataclasses.dataclass
class Token:
  """
  Represents a Desk token owned by a user.
  """
  id: str = ''
  owned_by: str = ''
  token: str = ''


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

  def __init__(self, hostname: str, username: str, password: str, platform: str = 'panda') -> None:
    urllib3.disable_warnings()
    self._session = requests.Session()
    self._session.verify = False
    self._hostname = hostname
    self._username = username
    self._password = password
    self._logged_in = False
    self._token = self._load_token()
    self._listening = False
    self._listen_thread = None
    self.login()
    self._legacy = False

    if platform.lower() in ['panda', 'fer', 'franka_emika_robot', 'frankaemikarobot']:
      self._platform = 'panda'
    elif platform.lower() in ['fr3', 'frankaresearch3', 'franka_research_3']:
      self._platform = 'fr3'
    else:
      raise ValueError("Unknown platform! Must be either 'panda' or 'fr3'!")

    try:
      self.take_control()
    except ConnectionError as error:
      if 'File not found' in str(error):
        _logger.info('Legacy desk detected.')
        self._legacy = True
      else:
        raise error

  def lock(self, force: bool = True) -> None:
    """
    Locks the brakes. API call blocks until the brakes are locked.
    """
    if self._platform == 'panda':
      url = '/desk/api/robot/close-brakes'
    elif self._platform == 'fr3':
      url = '/desk/api/joint/lock'

    self._request('post',
                  url,
                  files={'force': force})

  def unlock(self, force: bool = True) -> None:
    """
    Unlocks the brakes. API call blocks until the brakes are unlocked.
    """
    if self._platform == 'panda':
      url = '/desk/api/robot/open-brakes'
    elif self._platform == 'fr3':
      url = '/desk/api/joint/unlock'

    self._request('post',
                  url,
                  files={'force': force},
                  headers={'X-Control-Token': self._token.token})

  def reboot(self) -> None:
    """
    Reboots the robot hardware (this will close open connections).
    """
    self._request('post',
                  '/admin/api/reboot',
                  headers={'X-Control-Token': self._token.token})

  def activate_fci(self) -> None:
    """
    Activates the Franka Research Interface (FCI). Note that the
    brakes must be unlocked first. For older Desk versions, this
    function does nothing.
    """
    if not self._legacy:
      self._request('post',
                    '/admin/api/control-token/fci',
                    json={'token': self._token.token})

  def deactivate_fci(self) -> None:
    """
    Deactivates the Franka Research Interface (FCI). For older
    Desk versions, this function does nothing.
    """
    if not self._legacy:
      self._request('delete',
                    '/admin/api/control-token/fci',
                    json={'token': self._token.token})

  def _load_token(self) -> Token:
    config_path = os.path.expanduser(TOKEN_PATH)
    config = configparser.ConfigParser()
    token = Token()
    if os.path.exists(config_path):
      config.read(config_path)
      if config.has_section(self._hostname):
        token.id = config.get(self._hostname, 'id')
        token.owned_by = config.get(self._hostname, 'owned_by')
        token.token = config.get(self._hostname, 'token')
    return token

  def _save_token(self, token: Token) -> None:
    config_path = os.path.expanduser(TOKEN_PATH)
    config = configparser.ConfigParser()
    if os.path.exists(config_path):
      config.read(config_path)
    config[self._hostname] = {
        'id': token.id,
        'owned_by': token.owned_by,
        'token': token.token
    }
    os.makedirs(os.path.dirname(config_path), exist_ok=True)
    with open(config_path, 'w') as config_file:
      config.write(config_file)
    self._token = token

  def take_control(self, force: bool = False) -> bool:
    """
    Takes control of the Desk, generating a new control token and saving it.
    If `force` is set to True, control can be taken forcefully even if another
    user is already in control. However, the user will have to press the circle
    button on the robot's Pilot within an alotted amount of time to confirm
    physical access.

    For legacy versions of the Desk, this function does nothing.
    """
    if self._legacy:
      return True
    active = self._get_active_token()
    if active.id != '' and self._token.id == active.id:
      _logger.info('Retaken control.')
      return True
    if active.id != '' and not force:
      _logger.warning('Cannot take control. User %s is in control.',
                      active.owned_by)
      return False
    response = self._request(
        'post',
        f'/admin/api/control-token/request{"?force" if force else ""}',
        json={
            'requestedBy': self._username
        }).json()
    if force:
      timeout = self._request('get',
                              '/admin/api/safety').json()['tokenForceTimeout']
      _logger.warning(
          'You have %d seconds to confirm control by pressing circle button on robot.',
          timeout)
      with connect(f'wss://{self._hostname}/desk/api/navigation/events',
                   server_hostname='robot.franka.de',
                   additional_headers={
                       'authorization':
                           self._session.cookies.get('authorization')
                   }) as websocket:
        while True:
          event: typing.Dict = json_module.loads(websocket.recv(timeout))
          if 'circle' in event.keys():
            if event['circle']:
              break
    self._save_token(
        Token(str(response['id']), self._username, response['token']))
    _logger.info('Taken control.')
    return True

  def release_control(self) -> None:
    """
    Explicitly relinquish control of the Desk. This will allow
    other users to take control or transfer control to the next
    user if there is an active queue of control requests.
    """
    if self._legacy:
      return
    _logger.info('Releasing control.')
    try:
      self._request('delete',
                    '/admin/api/control-token',
                    json={'token': self._token.token})
    except ConnectionError as err:
      if 'ControlTokenUnknown' in str(err):
        _logger.warning('Control release failed. Not in control.')
      else:
        raise err
    self._token = Token()

  @staticmethod
  def encode_password(username: str, password: str) -> bytes:
    """
    Encodes the password into the form needed to log into the Desk interface.
    """
    bytes_str = ','.join([
        str(b) for b in hashlib.sha256((
            f'{password}#{username}@franka').encode('utf-8')).digest()
    ])
    return base64.encodebytes(bytes_str.encode('utf-8')).decode('utf-8')

  def login(self) -> None:
    """
    Uses the object's instance parameters to log into the Desk.
    The :py:class`Desk` class's constructor will try to connect
    and login automatically.
    """
    login = self._request(
        'post',
        '/admin/api/login',
        json={
            'login': self._username,
            'password': self.encode_password(self._username, self._password)
        })
    self._session.cookies.set('authorization', login.text)
    self._logged_in = True
    _logger.info('Login succesful.')

  def logout(self) -> None:
    """
    Logs the current user out of the Desk. API calls will no longer
    be possible.
    """
    self._request('post', '/admin/api/logout')
    self._session.cookies.clear()
    self._logged_in = False
    _logger.info('Logout successful.')

  def _get_active_token(self) -> Token:
    token = Token()
    if self._legacy:
      return Token()
    response = self._request('get', '/admin/api/control-token').json()
    if response['activeToken'] is not None:
      token.id = str(response['activeToken']['id'])
      token.owned_by = response['activeToken']['ownedBy']
    return token

  def has_control(self) -> bool:
    """
    Returns:
      bool: True if this instance is in control of the Desk.
    """
    if self._legacy:
      return True
    return self._token.id == self._get_active_token().id

  def _request(self,
               method: typing.Literal['post', 'get', 'delete'],
               url: str,
               json: typing.Dict[str, str] = None,
               headers: typing.Dict[str, str] = None,
               files: typing.Dict[str, str] = None) -> requests.Response:
    fun = getattr(self._session, method)
    response: requests.Response = fun(parse.urljoin(f'https://{self._hostname}',
                                                    url),
                                      json=json,
                                      headers=headers,
                                      files=files)
    if response.status_code != 200:
      raise ConnectionError(response.text)
    return response

  def _listen(self, cb, timeout):
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    with connect(
        f'wss://{self._hostname}/desk/api/navigation/events',
        # server_hostname='robot.franka.de',
        ssl_context=ctx,
        additional_headers={
            'authorization': self._session.cookies.get('authorization')
        }) as websocket:
      self._listening = True
      while self._listening:
        try:
          event: typing.Dict = json_module.loads(websocket.recv(timeout))
          cb(event)
        except TimeoutError:
          pass

  def listen(self, cb: typing.Callable[[typing.Dict], None]) -> None:
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
    self._listen_thread = threading.Thread(target=self._listen, args=(cb, 1.0))
    self._listen_thread.start()

  def stop_listen(self) -> None:
    """
    Stop listener thread (cf. :py:func:`panda_py.Desk.listen`).
    """
    self._listening = False
    if self._listen_thread is not None:
      self._listen_thread.join()
