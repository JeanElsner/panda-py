"""The realtime scheduling diagnostic.

No robot required. libfranka always tries to put the control thread on
SCHED_FIFO but only complains when the RealtimeConfig is kEnforce, and panda-py
defaults to kIgnore, so this is the only thing standing between a user and a
silent 1 kHz control loop running at normal priority.

The value itself depends on the machine, so these pin the properties that hold
everywhere rather than the verdict.
"""

import os

import panda_py
from panda_py import libfranka


def test_returns_a_verdict_and_a_reason():
    available, reason = panda_py.realtime_priority_available()
    assert isinstance(available, bool)
    assert isinstance(reason, str)
    # libfranka only fills the message in when it could not set the priority.
    assert (reason == "") == available


def test_querying_does_not_change_the_callers_priority():
    """The probe has to run on a thread of its own.

    libfranka's function raises the priority of whichever thread calls it, so
    asking the question on the main thread would change the interpreter's own
    scheduling as a side effect. This holds whether or not the attempt
    succeeds, which is the case that would otherwise go unnoticed.
    """
    before = (os.sched_getscheduler(0), os.sched_getparam(0).sched_priority)
    panda_py.realtime_priority_available()
    after = (os.sched_getscheduler(0), os.sched_getparam(0).sched_priority)
    assert after == before


def test_repeated_queries_agree():
    """Nothing is cached, so a second call must not answer differently."""
    first = panda_py.realtime_priority_available()
    assert panda_py.realtime_priority_available() == first


def test_the_kernel_half_of_the_check_is_reachable():
    """The other condition kEnforce checks, exposed but never used until now."""
    assert isinstance(libfranka.has_realtime_kernel(), bool)
