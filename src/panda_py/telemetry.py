"""
Recording a controller's 1 kHz telemetry.

A controller created with ``telemetry=<capacity>`` records one sample per
control tick into a buffer of that size. :py:class:`Recorder` drains it from a
background thread and assembles the samples, so the buffer only has to hold
what arrives between two drains::

    ctrl = controllers.TaskImpedance(frame="flange", telemetry=10_000)
    panda.start_controller(ctrl)
    with telemetry.Recorder(ctrl) as recorder:
        ...  # run the trial
    log = recorder.result()
    telemetry.save("trial.npz", log, metadata={"seed": 42})
"""

import json
import threading
import time

import numpy as np

__all__ = ["Recorder", "check", "save", "load"]


class Recorder:
    """
    Drains a controller's telemetry every ``interval`` seconds while active.

    Args:
      controller: A controller with ``read_telemetry()``, such as
        :py:class:`panda_py.controllers.TaskImpedance` created with a
        telemetry capacity.
      interval: Seconds between drains. The buffer must hold at least this
        many milliseconds of samples.
    """

    def __init__(self, controller, interval=0.1):
        if not controller.telemetry_capacity:
            raise ValueError("The controller records no telemetry: create it with telemetry=<capacity>.")
        if controller.telemetry_capacity < interval * 1000 * 2:
            raise ValueError(
                f"A telemetry capacity of {controller.telemetry_capacity} samples "
                f"cannot hold two drain intervals of {interval} s."
            )
        self._controller = controller
        self._interval = interval
        self._chunks = []
        self._stop = threading.Event()
        self._thread = None
        self._dropped_at_start = 0

    def start(self):
        """Discards what is buffered and starts recording."""
        self._controller.read_telemetry()
        self._chunks = []
        self._dropped_at_start = self._controller.telemetry_dropped
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        """Stops recording, after a last drain."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        self._drain()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    def _run(self):
        while not self._stop.wait(self._interval):
            self._drain()

    def _drain(self):
        chunk = self._controller.read_telemetry()
        if len(chunk["tick"]):
            self._chunks.append(chunk)

    def result(self):
        """
        The recording as a dict of arrays, one row per tick, plus ``dropped``,
        the number of samples the buffer lost while recording.
        """
        if self._chunks:
            log = {key: np.concatenate([c[key] for c in self._chunks]) for key in self._chunks[0]}
        else:
            log = {key: value for key, value in self._controller.read_telemetry().items()}
        log["dropped"] = np.array(self._controller.telemetry_dropped - self._dropped_at_start)
        return log


def check(log, period=1e-3):
    """
    The gaps in a recording.

    Returns a dict with ``samples``, ``missing`` (ticks the buffer lost, from
    gaps in ``tick``), ``lost_cycles`` (robot cycles without a command, from
    ``duration`` above one period) and ``ok``, true when both are zero.
    """
    tick = np.asarray(log["tick"])
    missing = int(np.sum(np.diff(tick) - 1)) if len(tick) > 1 else 0
    duration = np.asarray(log["duration"])[1:]
    lost = int(np.sum(np.maximum(np.round(duration / period) - 1, 0)))
    return {
        "samples": len(tick),
        "missing": missing,
        "lost_cycles": lost,
        "ok": missing == 0 and lost == 0,
    }


def save(path, log, metadata=None):
    """Writes a recording, with optional JSON-serialisable metadata, to npz."""
    arrays = dict(log)
    arrays["metadata"] = np.array(json.dumps(metadata or {}, default=str))
    arrays["saved"] = np.array(time.strftime("%Y-%m-%dT%H:%M:%S%z"))
    np.savez_compressed(path, **arrays)


def load(path):
    """Reads a recording written by :py:func:`save`; returns (log, metadata)."""
    with np.load(path) as data:
        log = {key: data[key] for key in data.files}
    metadata = json.loads(str(log.pop("metadata", "{}")))
    return log, metadata
