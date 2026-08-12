# Security policy

## Reporting a vulnerability

Please report security issues privately through
[GitHub's advisory form](https://github.com/JeanElsner/panda-py/security/advisories/new)
rather than opening a public issue.

## Scope

panda-py commands a robot arm in real time. Anything that can make the robot
move unexpectedly, or that lets code influence the control loop, is in scope,
as is anything affecting the `Desk` client, which handles credentials and
control tokens for the robot's web interface.

Note that the Franka Control Interface has no authentication of its own: any
host that can reach the robot on the network can control it. Keeping the robot
on an isolated network is part of a normal installation, not a vulnerability in
panda-py.

## Supported versions

Fixes go onto the latest release. Because a wheel is tied to the libfranka
version it was built against, a fix generally means rebuilding the affected
rows of the compatibility matrix rather than patching a single wheel.
