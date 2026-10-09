"""
Exceptions raised by panda-py.
"""

__all__ = ["IncompatibleVersionError", "SUPPORTED_PROTOCOL_VERSIONS"]

SUPPORTED_PROTOCOL_VERSIONS = range(3, 11)
"""
The research interface protocol versions (the server version a robot reports)
panda-py speaks: 3 to 5 are the Franka Emika Robot's, 6 to 10 the Franka
Research 3's. panda-py connects in whichever of them the robot speaks.
"""

_ISSUES_URL = "https://github.com/JeanElsner/panda-py/issues"


class IncompatibleVersionError(RuntimeError):
    """
    The robot speaks a research interface protocol version panda-py does not,
    so it refused the connection: a robot newer than this panda-py, or one
    older than any it supports.

    Raised in place of libfranka's ``IncompatibleVersionException``, which
    would otherwise reach Python as a plain :py:class:`RuntimeError` and lose
    the robot's version. It still derives from :py:class:`RuntimeError`, so
    existing handlers keep working.
    """

    def __init__(self, server_version: int, library_version: int) -> None:
        self.server_version = server_version
        """Protocol version the robot speaks."""
        self.library_version = library_version
        """The newest protocol version this panda-py speaks."""
        super().__init__(self._describe())

    def __reduce__(self):
        return (type(self), (self.server_version, self.library_version))

    def _describe(self) -> str:
        supported = (
            f"panda-py speaks protocol versions {SUPPORTED_PROTOCOL_VERSIONS[0]} to "
            f"{SUPPORTED_PROTOCOL_VERSIONS[-1]}, but the robot speaks version "
            f"{self.server_version}."
        )
        if self.server_version > SUPPORTED_PROTOCOL_VERSIONS[-1]:
            return (
                f"{supported} The robot is newer than this panda-py: upgrade with "
                f"`pip install -U panda-python`, and if that does not help, report "
                f"it at {_ISSUES_URL}"
            )
        return f"{supported} Robots this old are not supported."
