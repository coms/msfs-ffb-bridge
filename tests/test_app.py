"""Tests for the runtime, the bench tests and the command line.

The loop cannot be given a wheel here, but running it without one is itself
worth testing: the bridge has to survive a missing device and a missing
simulator, keep publishing state, and let go of everything on the way out.
"""

from __future__ import annotations

import time

import pytest

from ffbbridge.app import paths
from ffbbridge.app.bench import BENCH_TESTS, find_test
from ffbbridge.app.doctor import Level, format_report, run_checks
from ffbbridge.app.loop import BridgeRuntime, RuntimeSnapshot
from ffbbridge.app.main import build_parser, load_profiles, main
from ffbbridge.core.config import BridgeConfig, ProfileSet
from ffbbridge.core.telemetry import WheelState

pytest.importorskip("sdl2")


# --- Bench tests ---------------------------------------------------------

BENCH_CONFIG = BridgeConfig()
BENCH_CONFIG.wheel.rotation_deg = 1080.0
BENCH_CONFIG.wheel.soft_lock_deg = 180.0


def play(test_id: str, t: float, position: float = 0.0):
    """One frame of a bench test, at a moment and a wheel position."""
    return find_test(test_id).build(t, WheelState(position=position), BENCH_CONFIG)


def test_every_bench_test_produces_bounded_forces():
    """These drive the wheel directly, with no mixer in between to clamp them."""
    for test in BENCH_TESTS:
        for step in range(200):
            wheel = WheelState(position=(step % 41) / 20.0 - 1.0)
            force = test.build(step * 0.05, wheel, BENCH_CONFIG)
            assert -1.0 <= force.constant <= 1.0, test.id
            for periodic in force.periodics:
                assert 0.0 <= periodic.magnitude <= 1.0, test.id
                assert periodic.frequency_hz > 0.0, test.id
            if force.spring:
                assert 0.0 <= force.spring.coefficient <= 1.0, test.id
                assert -1.0 <= force.spring.center <= 1.0, test.id


def test_bench_directions_are_opposite():
    left = play("left", 0.0)
    right = play("right", 0.0)
    assert left.constant < 0 < right.constant


def test_bench_sweep_crosses_zero():
    """Direction and smoothness are what this test is for, so it has to reverse."""
    values = [play("sweep", t * 0.1).constant for t in range(200)]
    assert min(values) < -0.2
    assert max(values) > 0.2


def test_bench_rumble_sweeps_upward_then_restarts():
    frequencies = [play("rumble", t * 0.5).periodics[0].frequency_hz for t in range(16)]
    assert frequencies[0] < frequencies[7]


def test_bench_touchdown_is_a_repeating_transient_not_a_drone():
    magnitudes = [
        (play("touchdown", t * 0.05).periodics or [None])[0] for t in range(80)
    ]
    quiet = sum(1 for m in magnitudes if m is None)
    assert quiet > len(magnitudes) // 2


def test_bench_everything_exercises_all_channels():
    force = play("everything", 1.0)
    assert force.spring is not None
    assert force.damper is not None
    assert len(force.periodics) == 3


def test_bench_soft_lock_is_the_real_stop_at_the_profile_s_travel():
    """It runs the module, so the bench cannot drift from what you will fly.

    180 degrees of lock on a 1080 degree wheel is 90 either side, which is a
    sixth of the travel: silent inside it, pushing back outside it, and pushing
    back the other way on the other side.
    """
    assert play("softlock", 0.0, position=0.10).constant == 0.0
    assert play("softlock", 0.0, position=0.16).constant == 0.0

    right = play("softlock", 0.0, position=0.30)
    left = play("softlock", 0.0, position=-0.30)
    assert right.constant < 0 < left.constant
    assert right.damper is not None


def test_bench_soft_lock_carries_its_ratchet_state_across_a_run():
    """A module rebuilt every tick can never remember a peak to release from.

    The bench must keep one module alive across a run's rising ``t``, or the
    release the module implements is invisible on the bench even though it
    works correctly when driven by a persistent instance.
    """
    test = find_test("softlock")
    dt = 1 / 100
    t = 0.0
    for _ in range(30):
        held = test.build(t, WheelState(position=0.30), BENCH_CONFIG)
        t += dt
    # Back off well past the release hysteresis, still within one run.
    eased = test.build(t, WheelState(position=0.25), BENCH_CONFIG)
    assert 0.0 < -eased.constant < -held.constant


def test_bench_soft_lock_starts_fresh_when_a_new_run_begins():
    """The clock going backwards is what a fresh 'Play' click looks like."""
    test = find_test("softlock")
    t = 0.0
    for _ in range(30):
        test.build(t, WheelState(position=0.30), BENCH_CONFIG)
        t += 1 / 100
    test.build(t, WheelState(position=0.25), BENCH_CONFIG)  # released

    restarted = test.build(0.0, WheelState(position=0.25), BENCH_CONFIG)
    fresh = play("softlock", 0.0, position=0.25)
    assert restarted.constant == pytest.approx(fresh.constant)


def test_bench_soft_lock_follows_the_profile_rather_than_a_fixed_number():
    narrow = BridgeConfig()
    narrow.wheel.rotation_deg = 1080.0
    narrow.wheel.soft_lock_deg = 90.0
    wide = BridgeConfig()
    wide.wheel.rotation_deg = 1080.0
    wide.wheel.soft_lock_deg = 540.0

    at = WheelState(position=0.2)
    assert find_test("softlock").build(0.0, at, narrow).constant < 0.0
    assert find_test("softlock").build(0.0, at, wide).constant == 0.0


def test_bench_ids_are_unique():
    ids = [test.id for test in BENCH_TESTS]
    assert len(ids) == len(set(ids))


def test_unknown_bench_test_returns_nothing():
    assert find_test("does-not-exist") is None


# --- Doctor --------------------------------------------------------------


def test_doctor_always_mentions_the_pit_house_trap():
    """The wheelbase's own spring overrides ours, and it is on by default."""
    findings = run_checks(BridgeConfig())
    assert any("Pit House" in finding.title for finding in findings)


def test_doctor_reports_a_missing_wheel_as_a_failure():
    findings = run_checks(BridgeConfig())
    controllers = [f for f in findings if f.title == "Controllers"]
    assert controllers
    assert controllers[0].level in (Level.FAIL, Level.INFO)


def test_doctor_findings_explain_how_to_fix_failures():
    for finding in run_checks(BridgeConfig()):
        if finding.level is Level.FAIL:
            assert finding.fix, f"{finding.title} says what is wrong but not what to do"


def test_doctor_report_is_printable():
    report = format_report(run_checks(BridgeConfig()))
    assert "diagnostics" in report
    assert isinstance(report, str)


# --- Runtime -------------------------------------------------------------


def test_runtime_survives_having_no_hardware_and_no_simulator():
    """The common first-run case: the bridge started before anything else."""
    runtime = BridgeRuntime(ProfileSet())
    runtime.start()
    try:
        deadline = time.monotonic() + 2.0
        snapshot = runtime.snapshot()
        while time.monotonic() < deadline and not snapshot.running:
            time.sleep(0.05)
            snapshot = runtime.snapshot()
        assert runtime.running
        assert snapshot.running
        assert snapshot.sim_connected is False
        assert snapshot.device_connected is False
        assert snapshot.force.constant == 0.0
    finally:
        runtime.stop()
    assert not runtime.running


def test_runtime_keeps_ticking_and_reports_its_rate():
    runtime = BridgeRuntime(ProfileSet())
    runtime.start()
    try:
        time.sleep(1.0)
        snapshot = runtime.snapshot()
        assert snapshot.loop_hz > 20.0, f"loop only reached {snapshot.loop_hz:.1f} Hz"
    finally:
        runtime.stop()


def test_queued_commands_run_on_the_loop_thread():
    runtime = BridgeRuntime(ProfileSet())
    runtime.start()
    try:
        seen = []
        runtime.post(lambda: seen.append(True))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not seen:
            time.sleep(0.02)
        assert seen
    finally:
        runtime.stop()


def test_panic_is_reflected_in_the_snapshot():
    runtime = BridgeRuntime(ProfileSet())
    runtime.start()
    try:
        runtime.set_panic(True)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not runtime.snapshot().panic:
            time.sleep(0.02)
        assert runtime.snapshot().panic is True
        assert runtime.snapshot().force.constant == 0.0
    finally:
        runtime.stop()


def test_stopping_twice_is_harmless():
    runtime = BridgeRuntime(ProfileSet())
    runtime.start()
    runtime.stop()
    runtime.stop()
    assert not runtime.running


def test_snapshot_is_available_before_the_loop_starts():
    runtime = BridgeRuntime(ProfileSet())
    snapshot = runtime.snapshot()
    assert isinstance(snapshot, RuntimeSnapshot)
    assert snapshot.running is False
    assert snapshot.force.constant == 0.0


# --- Command line --------------------------------------------------------


def test_simulate_runs_the_whole_flight_offline(tmp_path, capsys):
    output = tmp_path / "trace.csv"
    assert main(["simulate", "--csv", str(output)]) == 0
    assert output.is_file()

    lines = output.read_text(encoding="utf-8").splitlines()
    assert len(lines) > 10000  # a full sortie at 60 Hz
    assert lines[0].startswith("t,mode,ground_weight")
    printed = capsys.readouterr().out
    assert "Peak steady force" in printed


def test_simulate_trace_covers_both_axis_modes(tmp_path):
    output = tmp_path / "trace.csv"
    main(["simulate", "--csv", str(output)])
    modes = {line.split(",")[1] for line in output.read_text().splitlines()[1:]}
    assert {"ground", "air", "to_air", "to_ground"} <= modes


def test_bench_list_needs_no_hardware(capsys):
    assert main(["bench", "list"]) == 0
    assert "Available bench tests" in capsys.readouterr().out


def test_doctor_command_reports_rather_than_crashing(capsys):
    exit_code = main(["doctor"])
    assert exit_code in (0, 1)
    assert "diagnostics" in capsys.readouterr().out


def test_parser_defaults_to_running_the_bridge():
    args = build_parser().parse_args([])
    assert args.func.__name__ == "command_run"
    assert args.console is False


def test_parser_accepts_console_mode():
    args = build_parser().parse_args(["run", "--console"])
    assert args.console is True


def test_profiles_fall_back_when_the_file_is_missing(tmp_path):
    profiles = load_profiles(tmp_path / "nothing.json")
    assert profiles.default.name


def test_config_paths_are_creatable():
    assert paths.config_dir().is_dir()
    assert paths.recordings_dir().is_dir()
    assert paths.profiles_path().parent.is_dir()
