"""The Desk client, exercised against a fake session rather than a robot."""

import logging

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
    """Records the last request and returns a canned response.

    A dict of url suffix to response makes a robot that serves only some of the
    endpoints, which is how the FER and the FR3 differ.
    """

    def __init__(self, response, by_url=None):
        self._response = response
        self._by_url = by_url or {}
        self.calls = []

    def _record(self, method):
        def call(url, json=None, headers=None, files=None):
            self.calls.append((method, url, json, headers, files))
            for suffix, response in self._by_url.items():
                if url.endswith(suffix):
                    return response
            return self._response

        return call

    def __getattr__(self, name):
        if name in ("post", "get", "delete"):
            return self._record(name)
        raise AttributeError(name)


def _desk(response, by_url=None, platform="panda", platform_given=True):
    """A Desk with the network replaced, bypassing the connecting constructor."""
    desk = Desk.__new__(Desk)
    desk._session = _Session(response, by_url)
    desk._hostname = "robot.local"
    desk._legacy = False
    desk._platform = platform
    desk._platform_given = platform_given
    desk._token = Token()
    return desk


#: A robot that serves only the FR3 brake endpoints, i.e. answers 404 for the
#: FER's, which is what an FR3 does to a client that assumes an FER.
FR3_ONLY = {
    "/desk/api/robot/open-brakes": _Response(404, text='No handler accepted "..."'),
    "/desk/api/robot/close-brakes": _Response(404, text='No handler accepted "..."'),
}

#: A robot that serves only the FER brake endpoints.
FER_ONLY = {
    "/desk/api/joints/unlock": _Response(404, text="File not found"),
    "/desk/api/joints/lock": _Response(404, text="File not found"),
}


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


@pytest.mark.parametrize(
    "platform,action,expected",
    [
        ("panda", "unlock", "/desk/api/robot/open-brakes"),
        ("panda", "lock", "/desk/api/robot/close-brakes"),
        ("fr3", "unlock", "/desk/api/joints/unlock"),
        ("fr3", "lock", "/desk/api/joints/lock"),
    ],
)
def test_platform_hint_is_tried_first(platform, action, expected):
    """The FER and the FR3 expose different endpoints for the brakes."""
    desk = _desk(_Response(200), platform=platform)
    getattr(desk, action)()
    assert len(desk._session.calls) == 1, "no fallback needed when the hint is right"
    assert desk._session.calls[0][1].endswith(expected)


@pytest.mark.parametrize("action", ["lock", "unlock"])
@pytest.mark.parametrize(
    "hint,serves,detected",
    [("panda", FR3_ONLY, "fr3"), ("fr3", FER_ONLY, "panda")],
)
def test_wrong_platform_falls_back_to_the_other_endpoint(
    action, hint, serves, detected
):
    """A 404 from the brake endpoint means the other generation's is the right one.

    The wrong endpoint does not touch the brakes, so retrying is safe. Before
    this, an FR3 user who did not pass platform='fr3' got a raw HTML 404.
    """
    desk = _desk(_Response(200), by_url=serves, platform=hint)
    getattr(desk, action)()
    urls = [call[1] for call in desk._session.calls]
    assert len(urls) == 2, urls
    assert urls[-1].endswith(Desk._BRAKE_ENDPOINTS[detected][action])
    # The detection is remembered, so the next call goes straight there.
    assert desk._platform == detected


def test_detection_is_not_repeated():
    desk = _desk(_Response(200), by_url=FR3_ONLY, platform="panda")
    desk.unlock()
    desk.unlock()
    assert len(desk._session.calls) == 3, "one fallback, then straight to the FR3"


def test_a_non_404_failure_is_raised_rather_than_retried():
    """Only a missing endpoint justifies a retry; an auth failure must surface."""
    desk = _desk(_Response(401, text="not in control"))
    with pytest.raises(ConnectionError, match="not in control"):
        desk.unlock()
    assert len(desk._session.calls) == 1


def test_brakes_raise_when_neither_endpoint_exists():
    desk = _desk(_Response(404, text="File not found"))
    with pytest.raises(ConnectionError, match="neither the FER nor the FR3"):
        desk.unlock()


def test_explicit_wrong_platform_warns(caplog):
    """Silently correcting an explicit argument would hide a bad configuration."""
    desk = _desk(_Response(200), by_url=FR3_ONLY, platform="panda")
    with caplog.at_level(logging.INFO):
        desk.unlock()
    assert any(record.levelno == logging.WARNING for record in caplog.records)


def test_detection_without_a_hint_only_logs_info(caplog):
    desk = _desk(
        _Response(200), by_url=FR3_ONLY, platform="panda", platform_given=False
    )
    with caplog.at_level(logging.INFO):
        desk.unlock()
    assert desk._platform == "fr3"
    assert not any(record.levelno >= logging.WARNING for record in caplog.records)


def test_legacy_desk_skips_control_tokens():
    desk = _desk(_Response(200))
    desk._legacy = True
    assert desk.take_control() is True
    assert desk.has_control() is True
    # Nothing should have gone over the wire.
    assert desk._session.calls == []
