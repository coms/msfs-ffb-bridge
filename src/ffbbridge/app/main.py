"""Command line entry point.

``ffbbridge`` on its own opens the tuning interface. The subcommands exist for
the situations where the interface is not what you need: proving the hardware,
working out why nothing is happening, or examining the force model offline.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import logging
import sys
import time
from pathlib import Path

from ..core.config import ProfileSet
from ..core.engine import BridgeEngine
from ..core.synthetic import SyntheticFlight
from ..core.telemetry import WheelState
from . import paths
from .bench import BENCH_TESTS, find_test
from .doctor import format_report, run_checks
from .loop import BridgeRuntime

LOGGER = logging.getLogger(__name__)


def load_profiles(path: Path | None = None) -> ProfileSet:
    """Load the user's profiles, falling back to the shipped defaults."""
    target = path or paths.profiles_path()
    if target.is_file():
        return ProfileSet.load_or_default(target)
    bundled = paths.bundled_profile()
    if bundled is not None:
        return ProfileSet.load_or_default(bundled)
    return ProfileSet()


def configure_logging(verbose: bool = False) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler(sys.stdout)]
    with contextlib.suppress(OSError):
        handlers.append(logging.FileHandler(paths.log_path(), encoding="utf-8"))
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        handlers=handlers,
        force=True,
    )


def command_devices(args: argparse.Namespace) -> int:
    from ..io.ffb_sdl import init_sdl, list_devices

    init_sdl()
    devices = list_devices()
    if not devices:
        print("No game controllers found.")
        return 1
    for device in devices:
        feedback = "force feedback" if device.is_haptic else "no force feedback"
        print(f"  [{device.index}] {device.name}")
        print(f"        {device.num_axes} axes, {device.num_buttons} buttons, {feedback}")
    return 0


def command_doctor(args: argparse.Namespace) -> int:
    profiles = load_profiles(args.config)
    findings = run_checks(profiles.default)
    print(format_report(findings))
    return 0 if all(f.level.value != "fail" for f in findings) else 1


def command_bench(args: argparse.Namespace) -> int:
    """Play one effect at a time so the hardware can be judged on its own."""
    if args.test in (None, "list"):
        print("Available bench tests:\n")
        for test in BENCH_TESTS:
            print(f"  {test.id:12} {test.name}")
            print(f"               {test.description}")
        print("\nRun one with: ffbbridge bench <id>")
        return 0

    test = find_test(args.test)
    if test is None:
        print(f"No such bench test: {args.test}")
        return 1

    runtime = BridgeRuntime(load_profiles(args.config))
    runtime.start()
    print(f"\n{test.name}\n{test.description}\n")

    started = time.perf_counter()
    config = runtime.engine.config
    runtime.set_bench(lambda now, wheel: test.build(now - started, wheel, config))
    try:
        deadline = started + (args.seconds or test.duration)
        while time.perf_counter() < deadline:
            time.sleep(0.1)
            snapshot = runtime.snapshot()
            if not snapshot.device_connected and snapshot.device_error:
                print(f"Wheel not available: {snapshot.device_error}")
                return 1
    except KeyboardInterrupt:
        pass
    finally:
        runtime.set_bench(None)
        runtime.stop()
    print("Done.")
    return 0


def command_simulate(args: argparse.Namespace) -> int:
    """Fly the scripted sortie through the force model and report what it produced.

    No simulator and no hardware involved, which makes it the quickest way to
    see what a configuration change actually does.
    """
    profiles = load_profiles(args.config)
    engine = BridgeEngine(profiles)
    flight = SyntheticFlight()
    wheel = WheelState(position=args.wheel, connected=True)

    rows = [
        (telemetry, engine.tick(telemetry, wheel, telemetry.t)) for telemetry in flight.stream()
    ]
    return _report_run(rows, "scripted flight", args.csv)


def command_replay(args: argparse.Namespace) -> int:
    """Run a recorded flight back through the force model.

    Record once, then replay as often as you like with different settings. It is
    also how a flight can be handed to someone else to look at.
    """
    from ..core.recording import read_recording, recording_header

    path = Path(args.recording)
    if not path.is_file():
        print(f"No such recording: {path}")
        return 1

    header = recording_header(path)
    if header.get("note"):
        print(f"Recording note: {header['note']}")

    profiles = load_profiles(args.config)
    engine = BridgeEngine(profiles)
    wheel = WheelState(position=args.wheel, connected=True)
    rows = [
        (telemetry, engine.tick(telemetry, wheel, telemetry.t))
        for telemetry in read_recording(path)
    ]
    if not rows:
        print("The recording contained no samples.")
        return 1
    return _report_run(rows, str(path), args.csv)


def _report_run(rows, source: str, csv_path: str | None) -> int:
    """Summarise a run of the force model, and optionally write the full trace."""
    if csv_path:
        _write_trace(Path(csv_path), rows)
        print(f"Wrote {len(rows)} samples to {csv_path}")

    duration = rows[-1][0].t - rows[0][0].t
    print(f"\nRan {duration:.0f} s of {source}, {len(rows)} ticks.\n")
    print(f"{'time':>7} {'phase':>10} {'gs':>6} {'force':>7} {'spring':>7}  effects")

    step = max(duration / 16.0, 1.0)
    marker = rows[0][0].t
    while marker <= rows[-1][0].t:
        telemetry, result = min(rows, key=lambda pair: abs(pair[0].t - marker))
        spring = result.force.spring.coefficient if result.force.spring else 0.0
        effects = ",".join(p.label for p in result.force.periodics)
        print(
            f"{telemetry.t:7.1f} {result.context.mode.value:>10} {telemetry.gs_kt:6.1f} "
            f"{result.force.constant:+7.3f} {spring:7.3f}  {effects}"
        )
        marker += step

    peak = max(rows, key=lambda pair: abs(pair[1].force.constant))
    print(f"\nPeak steady force {peak[1].force.constant:+.3f} at t={peak[0].t:.1f}s")
    print(f"Ticks at the force ceiling: {sum(1 for _, r in rows if r.force.clipped)}")
    return 0


def _write_trace(path: Path, rows) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "t",
                "mode",
                "ground_weight",
                "ias_kt",
                "gs_kt",
                "agl_ft",
                "on_ground",
                "constant",
                "spring_k",
                "spring_center",
                "damper_k",
                "aileron",
                "rudder",
                "effects",
            ]
        )
        for telemetry, result in rows:
            force = result.force
            writer.writerow(
                [
                    f"{telemetry.t:.4f}",
                    result.context.mode.value,
                    f"{result.context.ground_weight:.3f}",
                    f"{telemetry.ias_kt:.2f}",
                    f"{telemetry.gs_kt:.2f}",
                    f"{telemetry.agl_ft:.1f}",
                    int(telemetry.weight_on_wheels),
                    f"{force.constant:.4f}",
                    f"{force.spring.coefficient:.4f}" if force.spring else "0",
                    f"{force.spring.center:.4f}" if force.spring else "0",
                    f"{force.damper.coefficient:.4f}" if force.damper else "0",
                    f"{result.axis.aileron:.4f}",
                    f"{result.axis.rudder:.4f}",
                    "|".join(
                        f"{p.label}@{p.frequency_hz:.1f}:{p.magnitude:.3f}" for p in force.periodics
                    ),
                ]
            )


def command_run(args: argparse.Namespace) -> int:
    profiles = load_profiles(args.config)
    runtime = BridgeRuntime(profiles, config_path=args.config or paths.profiles_path())

    if args.console:
        return _run_console(runtime)

    try:
        from .gui import run_gui
    except ImportError as exc:
        print(f"The interface needs Dear PyGui, which is not installed ({exc}).")
        print("Install it with 'pip install dearpygui', or run with --console.")
        return 1
    return run_gui(runtime)


def _run_console(runtime: BridgeRuntime) -> int:
    """A one-line status display, for when a window is not wanted."""
    runtime.start()
    print("Bridge running. Press Ctrl+C to stop.\n")
    try:
        while True:
            time.sleep(0.25)
            snapshot = runtime.snapshot()
            sim = "sim" if snapshot.sim_connected else "no sim"
            wheel = snapshot.device_name if snapshot.device_connected else "no wheel"
            line = (
                f"\r[{sim:>6} | {wheel:<22}] {snapshot.mode.value:<9} "
                f"force {snapshot.force.constant:+.3f} "
                f"ias {snapshot.telemetry.ias_kt:5.1f} "
                f"loop {snapshot.loop_hz:5.1f} Hz "
                f"{'PANIC' if snapshot.panic else '     '}"
            )
            sys.stdout.write(line[:120])
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        runtime.stop()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ffbbridge",
        description="Force feedback bridge from Microsoft Flight Simulator to a MOZA wheelbase.",
    )
    parser.add_argument("--config", type=Path, help="profile file to use")
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    parser.set_defaults(func=command_run, console=False)

    subparsers = parser.add_subparsers(dest="command")

    run_parser = subparsers.add_parser("run", help="run the bridge (the default)")
    run_parser.add_argument("--console", action="store_true", help="no window, just a status line")
    run_parser.set_defaults(func=command_run)

    subparsers.add_parser("devices", help="list game controllers").set_defaults(
        func=command_devices
    )
    subparsers.add_parser("doctor", help="check the setup and explain any problems").set_defaults(
        func=command_doctor
    )

    bench_parser = subparsers.add_parser("bench", help="feel one effect at a time, no simulator")
    bench_parser.add_argument("test", nargs="?", help="which test to run, or 'list'")
    bench_parser.add_argument("--seconds", type=float, help="override the duration")
    bench_parser.set_defaults(func=command_bench)

    simulate_parser = subparsers.add_parser(
        "simulate", help="fly a scripted sortie through the force model offline"
    )
    simulate_parser.add_argument("--csv", help="write the full force trace here")
    simulate_parser.add_argument(
        "--wheel", type=float, default=0.0, help="hold the wheel at this position"
    )
    simulate_parser.set_defaults(func=command_simulate)

    replay_parser = subparsers.add_parser(
        "replay", help="run a recorded flight back through the force model"
    )
    replay_parser.add_argument("recording", help="a .jsonl recording")
    replay_parser.add_argument("--csv", help="write the full force trace here")
    replay_parser.add_argument(
        "--wheel", type=float, default=0.0, help="hold the wheel at this position"
    )
    replay_parser.set_defaults(func=command_replay)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    configure_logging(args.verbose)
    try:
        return args.func(args)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
