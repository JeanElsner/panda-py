"""
Exceptions raised by panda-py.
"""

import typing

__all__ = ["IncompatibleVersionError", "LIBFRANKA_FOR_SERVER_VERSION"]

LIBFRANKA_FOR_SERVER_VERSION = {
    3: "0.7.1",
    4: "0.8.0",
    5: "0.9.2",
    6: "0.13.2",
    7: "0.13.6",
    8: "0.14.2",
    9: "0.17.0",
    10: "0.21.3",
}
"""
The libfranka version panda-py is built against for each research interface
protocol version, i.e. the server version a robot reports. There is one
panda-py build per protocol version, listed in the README.
"""

_BUILDS_URL = "https://github.com/JeanElsner/panda-py#libfranka-version"


class IncompatibleVersionError(RuntimeError):
    """
    The robot speaks a different research interface protocol version than
    the libfranka this panda-py was built with, so it refused the connection.

    Raised in place of libfranka's ``IncompatibleVersionException``, which
    would otherwise reach Python as a plain :py:class:`RuntimeError` and lose
    the robot's version. It still derives from :py:class:`RuntimeError`, so
    existing handlers keep working. The message names the panda-py build to
    install instead.
    """

    def __init__(self, server_version: int, library_version: int) -> None:
        self.server_version = server_version
        """Protocol version the robot speaks."""
        self.library_version = library_version
        """Protocol version this panda-py's libfranka speaks."""
        super().__init__(self._describe())

    def __reduce__(self):
        return (type(self), (self.server_version, self.library_version))

    @property
    def libfranka_version(self) -> typing.Optional[str]:
        """
        The libfranka version of the panda-py build that supports this robot,
        or None if no build does.
        """
        return LIBFRANKA_FOR_SERVER_VERSION.get(self.server_version)

    def _describe(self) -> str:
        mismatch = (
            f"The robot speaks protocol version {self.server_version}, but this "
            f"panda-py was built with a libfranka that speaks version "
            f"{self.library_version}."
        )
        if self.libfranka_version is None:
            return (
                f"{mismatch} No panda-py build supports protocol version "
                f"{self.server_version} yet. Available builds: {_BUILDS_URL}"
            )
        return (
            f"{mismatch} Install the panda-py build for libfranka "
            f"{self.libfranka_version} instead: {_BUILDS_URL}"
        )
