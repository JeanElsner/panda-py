"""The Desk client, exercised against a fake session rather than a robot."""

import pytest

from panda_py import Desk, Token


class _Response:
    def __init__(self, status_code, text="", payload=None):
        self.status_code = status_code
        self.text = text
        self._payload = payload

    def json(self):
        return self._payload


class _Session:
    """Records the last request and returns a canned response."""

    def __init__(self, response):
        self._response = response
        self.calls = []

    def _record(self, method):
        def call(url, json=None, headers=None, files=None):
            self.calls.append((method, url, json, headers, files))
            return self._response

        return call

    def __getattr__(self, name):
        if name in ("post", "get", "delete"):
            return self._record(name)
        raise AttributeError(name)


def _desk(response):
    """A Desk with the network replaced, bypassing the connecting constructor."""
    desk = Desk.__new__(Desk)
    desk._session = _Session(response)
    desk._hostname = "robot.local"
    desk._legacy = False
    desk._platform = "panda"
    desk._token = Token()
    return desk


@pytest.mark.parametrize("status", [200, 201, 204])
def test_successful_status_codes_are_accepted(status):
    """204 No Content is a success.

    Only 200 used to be accepted, so releasing a control token, which is a
    DELETE and answers 204, looked like a failure.
    """
    desk = _desk(_Response(status))
    assert desk._request("post", "/admin/api/login").status_code == status


@pytest.mark.parametrize("status", [400, 401, 404, 500])
def test_error_status_codes_raise(status):
    desk = _desk(_Response(status, text="nope"))
    with pytest.raises(ConnectionError):
        desk._request("post", "/admin/api/login")


def test_request_builds_an_absolute_https_url():
    desk = _desk(_Response(200))
    desk._request("get", "/admin/api/safety")
    _, url, _, _, _ = desk._session.calls[0]
    assert url == "https://robot.local/admin/api/safety"


def test_encode_password_is_deterministic_and_text():
    first = Desk.encode_password("admin", "secret")
    second = Desk.encode_password("admin", "secret")
    assert first == second
    assert isinstance(first, str)
    assert Desk.encode_password("admin", "other") != first
    assert Desk.encode_password("other", "secret") != first


def test_platform_selects_the_brake_endpoints():
    """The FER and the FR3 expose different endpoints for the brakes."""
    desk = _desk(_Response(200))
    desk._platform = "panda"
    desk.unlock()
    assert desk._session.calls[-1][1].endswith("/desk/api/robot/open-brakes")

    desk = _desk(_Response(200))
    desk._platform = "fr3"
    desk.unlock()
    assert desk._session.calls[-1][1].endswith("/desk/api/joints/unlock")


def test_legacy_desk_skips_control_tokens():
    desk = _desk(_Response(200))
    desk._legacy = True
    assert desk.take_control() is True
    assert desk.has_control() is True
    # Nothing should have gone over the wire.
    assert desk._session.calls == []
