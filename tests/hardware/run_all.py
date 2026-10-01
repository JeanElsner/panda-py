"""Runs the hardware checks one after another in a single Desk session.

Logs in to Desk once, unlocks the brakes and activates FCI once, then runs
each check's robot part in its own process, so a crash in one cannot take the
session down. Before each check it says what the check does and waits: Enter
runs it, s skips it, q ends the session. Every check's output is shown and
also written to results/run_<time>/<check>.log; a summary closes the run. The
robot is always restored at the end: FCI off, brakes locked, control
released.

    python tests/hardware/run_all.py <robot-ip> <desk-user> [--platform panda]
        [--only name,name] [--extra path/to/checks.py]

--extra adds the checks a Python file lists in a CHECKS variable, in the
format below, after these, or in the order its ORDER variable names.
"""

import argparse
import getpass
import importlib.util
import pathlib
import signal
import subprocess
import sys
import time

import panda_py

HERE = pathlib.Path(__file__).resolve().parent
POSE_PATH = pathlib.Path.home() / "dev" / "panda-py" / "notes" / "check_pose_path.py"

# name, what it does (shown before it runs), whether you have to do something,
# and the command line for its robot part: a function of (hostname, username,
# output prefix).
CHECKS = [
    (
        "task_impedance",
        "TaskImpedance at the flange: 10 mm and 0.05 rad steps, a 20 mm sine; "
        "telemetry and parity with vic_reference.",
        False,
        lambda host, user, out: [str(HERE / "check_task_impedance.py"), host, user, "-",
                                 "--worker", "--out", f"{out}.npz"],
    ),
    (
        "guards",
        "The guards. YOU PUSH the flange sideways until the force guard trips; then "
        "a retreat, workspace, speed and manual trips.",
        True,
        lambda host, user, out: [str(HERE / "check_guards.py"), host, user, "-",
                                 "--worker", "--out", f"{out}.npz"],
    ),
    (
        "vic",
        "Stiffness switching 50/800 at 50 Hz, the tank in three settings, the joint "
        "servo's 20 mrad step.",
        False,
        lambda host, user, out: [str(HERE / "check_vic.py"), host, user, "-",
                                 "--worker", "--out", out],
    ),
    (
        "pose_path",
        "move_to_pose, now on TaskImpedance: 5 cm moves at several stiffnesses "
        "(notes/check_pose_path.py of the spike checkout).",
        False,
        lambda host, user, out: [str(POSE_PATH), host, user, "-", "--worker", out],
    ),
]


def load_extra(path):
    """A file's CHECKS, and its ORDER of all checks by name, if it has one."""
    spec = importlib.util.spec_from_file_location("extra_checks", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return list(module.CHECKS), getattr(module, "ORDER", None)


def run_check(command, log_path):
    """Runs a check, showing and logging its output; returns its exit status."""
    # Forwarded as it arrives, not by line, so that a check's input() prompt
    # shows before you answer it; stdin is the terminal's.
    with open(log_path, "wb") as log, subprocess.Popen(
        [sys.executable, "-u", *command], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    ) as process:
        while True:
            chunk = process.stdout.read1(4096)
            if not chunk:
                break
            sys.stdout.buffer.write(chunk)
            sys.stdout.flush()
            log.write(chunk)
        return process.wait()


def describe(status):
    if status is None:
        return "skipped"
    if status == 0:
        return "passed"
    if status < 0:
        return f"crashed ({signal.Signals(-status).name})"
    return f"failed (exit status {status})"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("hostname")
    parser.add_argument("username")
    parser.add_argument("password", nargs="?")
    parser.add_argument("--platform", default="panda")
    parser.add_argument("--only", help="comma-separated names of the checks to run")
    parser.add_argument("--extra", action="append", default=[],
                        help="a Python file with more CHECKS")
    args = parser.parse_args()

    checks, order = list(CHECKS), None
    for path in args.extra:
        extra, extra_order = load_extra(path)
        checks += extra
        order = extra_order or order
    if order:
        rank = {name: i for i, name in enumerate(order)}
        checks.sort(key=lambda c: rank.get(c[0], len(rank)))
    if args.only:
        wanted = args.only.split(",")
        unknown = set(wanted) - {c[0] for c in checks}
        if unknown:
            sys.exit(f"unknown checks: {', '.join(sorted(unknown))}")
        checks = [c for c in checks if c[0] in wanted]
    if not POSE_PATH.exists():
        checks = [c for c in checks if c[0] != "pose_path"]

    run_dir = pathlib.Path("results") / f"run_{time.strftime('%Y%m%d-%H%M%S')}"
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"panda-py {panda_py.__version__} from {panda_py.__file__}")
    print(f"results in {run_dir}\n")
    print("  Checks in this session:")
    for i, (name, what, human, _) in enumerate(checks, 1):
        print(f"   {i}. {name}{'  [needs you]' if human else ''}: {what}")
    password = args.password or getpass.getpass("\n  Desk password: ")
    desk = panda_py.Desk(args.hostname, args.username, password, platform=args.platform)
    print("  desk: logged in, control token acquired")
    results = []
    try:
        print("\n  Unlocking the brakes WILL make the robot move, and every check moves it.")
        print("  Keep the workspace clear and the user stop within reach throughout.\n")
        if input("  Type YES to continue: ") != "YES":
            print("  aborted")
            return 1
        desk.unlock()
        print("  desk: brakes unlocked")
        desk.activate_fci()
        print("  desk: FCI activated")
        for i, (name, what, human, command) in enumerate(checks, 1):
            print(f"\n=== {i}/{len(checks)} {name}{'  [needs you]' if human else ''}")
            print(f"    {what}")
            answer = input("    Enter to run, s to skip, q to quit: ").strip().lower()
            if answer == "q":
                break
            if answer == "s":
                results.append((name, None, None))
                continue
            out = str(run_dir / name)
            started = time.monotonic()
            status = run_check(command(args.hostname, args.username, out), f"{out}.log")
            results.append((name, status, time.monotonic() - started))
            print(f"\n    {name}: {describe(status)}")
    finally:
        print("\n  restoring the previous state")
        for label, call in (
            ("FCI deactivated", desk.deactivate_fci),
            ("brakes locked", desk.lock),
            ("control released", desk.release_control),
        ):
            try:
                call()
                print(f"  desk: {label}")
            except Exception as error:  # pylint: disable=broad-except
                print(f"  desk: FAILED before '{label}': {error}")
        print("\n  Summary")
        for name, status, seconds in results:
            took = f"  ({seconds:.0f} s)" if seconds is not None else ""
            print(f"    {name:<20} {describe(status)}{took}")
        summary = "\n".join(f"{name}\t{describe(status)}" for name, status, _ in results)
        (run_dir / "summary.txt").write_text(summary + "\n")
        print(f"\n  logs and data in {run_dir}")
    return 0 if all(status in (0, None) for _, status, _ in results) else 1


if __name__ == "__main__":
    sys.exit(main())
