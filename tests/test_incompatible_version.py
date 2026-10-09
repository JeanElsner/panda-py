"""The error raised when the robot speaks another protocol version.

No robot required. A fake control unit on localhost answers libfranka's
``Connect`` the way a real one does when the versions differ, which was
confirmed on an FER (version 5) and an FR3 (version 10): status
``kIncompatibleLibraryVersion`` and its own version. libfranka then throws
``IncompatibleVersionException`` from C++, so this covers the translator end
to end rather than only the Python class.
"""

import pathlib
import pickle
import re
import subprocess
import sys

import pytest

import panda_py
from panda_py import libfranka
from panda_py.exceptions import SUPPORTED_PROTOCOL_VERSIONS, IncompatibleVersionError

# Runs in a subprocess rather than a thread: the libfranka.Robot binding holds
# the GIL while it connects, so a fake in this interpreter would never answer.
# The wire format is byte-identical in every protocol version from 3 to 10.
FAKE_CONTROL_UNIT = r"""
import socket, struct, sys

header = struct.Struct("<III")  # command, command id, size
request = struct.Struct("<HH")  # version, udp port
response = struct.Struct("<BH")  # status, version
k_incompatible_library_version = 1

def recv_exactly(conn, size):
    data = b""
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk:
            raise ConnectionError("client closed the connection")
        data += chunk
    return data

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    sock.bind(("127.0.0.1", 1337))
except OSError as error:
    print("unavailable", error, flush=True)
    sys.exit()
sock.listen()
sock.settimeout(10)
print("ready", flush=True)
conn, _ = sock.accept()
with conn:
    command, command_id, size = header.unpack(recv_exactly(conn, header.size))
    offered, _ = request.unpack(recv_exactly(conn, size - header.size)[: request.size])
    # A version newer than any panda-py speaks: older ones it would reconnect in.
    replied = 11
    conn.sendall(
        header.pack(command, command_id, header.size + response.size)
        + response.pack(k_incompatible_library_version, replied)
    )
    print(offered, replied, flush=True)
"""


class FakeControlUnit:
    """Rejects one Connect with a protocol version panda-py does not speak."""

    def __init__(self):
        self._process = subprocess.Popen(
            [sys.executable, "-c", FAKE_CONTROL_UNIT], stdout=subprocess.PIPE, text=True
        )
        status = self._process.stdout.readline()
        if not status.startswith("ready"):
            self._process.wait()
            pytest.skip(f"cannot listen on 127.0.0.1:1337: {status.strip()}")
        self._versions = None

    def _read_versions(self):
        if self._versions is None:
            offered, replied = self._process.stdout.readline().split()
            self._versions = int(offered), int(replied)
            self._process.wait(timeout=10)
        return self._versions

    @property
    def offered(self):
        return self._read_versions()[0]

    @property
    def replied(self):
        return self._read_versions()[1]

    def close(self):
        if self._process.poll() is None:
            self._process.kill()
        self._process.wait()
        self._process.stdout.close()


@pytest.fixture
def control_unit():
    unit = FakeControlUnit()
    yield unit
    unit.close()


@pytest.mark.parametrize(
    "connect",
    [
        pytest.param(lambda: libfranka.Robot("127.0.0.1"), id="libfranka.Robot"),
        pytest.param(lambda: panda_py.Panda("127.0.0.1"), id="Panda"),
    ],
)
def test_connecting_raises_with_both_versions(control_unit, connect):
    with pytest.raises(IncompatibleVersionError) as info:
        connect()
    assert info.value.server_version == control_unit.replied
    assert info.value.library_version == control_unit.offered


def test_existing_runtime_error_handlers_still_catch_it(control_unit):
    with pytest.raises(RuntimeError):
        libfranka.Robot("127.0.0.1")


def test_supports_every_protocol_version_from_3_to_10():
    assert list(SUPPORTED_PROTOCOL_VERSIONS) == list(range(3, 11))


def test_a_newer_robot_says_to_upgrade():
    error = IncompatibleVersionError(11, 10)
    assert "speaks version 11" in str(error)
    assert "pip install -U panda-python" in str(error)


def test_an_older_robot_says_it_is_unsupported():
    error = IncompatibleVersionError(2, 10)
    assert "not supported" in str(error)


def test_pickles():
    error = pickle.loads(pickle.dumps(IncompatibleVersionError(5, 10)))
    assert (error.server_version, error.library_version) == (5, 10)
    assert str(error) == str(IncompatibleVersionError(5, 10))
