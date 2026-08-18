"""Tests for the signal-processing helpers."""

from __future__ import annotations

import math

import pytest

from ffbbridge.core.filters import (
    BandNoise,
    DwellTimer,
    EdgeDetector,
    HighPass,
    Hysteresis,
    LowPass,
    OneShot,
    PhaseAccumulator,
    RateOfChange,
    SlewLimiter,
    clamp,
    deadband,
    expo,
    inverse_gamma,
    lerp,
    map_range,
    smoothstep,
)


def test_clamp_bounds():
    assert clamp(2.0) == 1.0
    assert clamp(-2.0) == -1.0
    assert clamp(0.3) == 0.3
    assert clamp(5.0, 0.0, 10.0) == 5.0


def test_lerp_clamps_parameter():
    assert lerp(0.0, 10.0, 0.5) == 5.0
    assert lerp(0.0, 10.0, -1.0) == 0.0
    assert lerp(0.0, 10.0, 2.0) == 10.0


def test_map_range_clips_by_default():
    assert map_range(5.0, 0.0, 10.0, 0.0, 100.0) == 50.0
    assert map_range(-5.0, 0.0, 10.0, 0.0, 100.0) == 0.0
    assert map_range(-5.0, 0.0, 10.0, 0.0, 100.0, clip=False) == -50.0


def test_map_range_handles_zero_width_input():
    assert map_range(3.0, 2.0, 2.0, 7.0, 9.0) == 7.0


def test_smoothstep_is_monotonic_and_bounded():
    values = [smoothstep(x / 20.0, 0.2, 0.8) for x in range(21)]
    assert values[0] == 0.0
    assert values[-1] == 1.0
    assert all(b >= a for a, b in zip(values, values[1:], strict=False))


def test_deadband_is_continuous_at_the_edge():
    assert deadband(0.05, 0.1) == 0.0
    # Just outside the deadband the output must emerge from zero, not jump.
    assert deadband(0.1001, 0.1) == pytest.approx(0.0, abs=1e-3)
    assert deadband(1.0, 0.1) == pytest.approx(1.0)
    assert deadband(-1.0, 0.1) == pytest.approx(-1.0)


def test_expo_preserves_endpoints():
    assert expo(0.0, 0.5) == 0.0
    assert expo(1.0, 0.5) == pytest.approx(1.0)
    assert expo(-1.0, 0.5) == pytest.approx(-1.0)
    # Mid travel is softened.
    assert expo(0.5, 1.0) < 0.5


def test_inverse_gamma_preserves_endpoints():
    assert inverse_gamma(0.0, 0.5) == 0.0
    assert inverse_gamma(1.0, 0.5) == pytest.approx(1.0)
    assert inverse_gamma(-1.0, 0.5) == pytest.approx(-1.0)
    # Mid travel is sharpened -- the opposite of expo.
    assert inverse_gamma(0.5, 1.0) > 0.5


def test_inverse_gamma_is_linear_at_zero_amount():
    assert inverse_gamma(0.3, 0.0) == 0.3
    assert inverse_gamma(-0.7, 0.0) == -0.7


def test_inverse_gamma_is_the_opposite_curve_from_expo():
    value = 0.4
    assert inverse_gamma(value, 0.6) > value > expo(value, 0.6)


def test_lowpass_converges_and_primes_on_first_sample():
    lp = LowPass(tau=0.1)
    assert lp.update(5.0, 0.01) == 5.0  # primes rather than ramping from zero
    for _ in range(500):
        lp.update(1.0, 0.01)
    assert lp.value == pytest.approx(1.0, abs=1e-3)


def test_lowpass_is_framerate_independent():
    fast, slow = LowPass(tau=0.2), LowPass(tau=0.2)
    fast.update(0.0, 0.001)
    slow.update(0.0, 0.001)
    for _ in range(1000):
        fast.update(1.0, 0.001)
    for _ in range(100):
        slow.update(1.0, 0.01)
    assert fast.value == pytest.approx(slow.value, abs=1e-3)


def test_highpass_removes_steady_state():
    hp = HighPass(tau=0.1)
    for _ in range(1000):
        value = hp.update(1.0, 0.01)
    assert value == pytest.approx(0.0, abs=1e-3)


def test_highpass_passes_a_step():
    hp = HighPass(tau=0.5)
    hp.update(0.0, 0.01)
    assert hp.update(1.0, 0.01) > 0.9


def test_slew_limiter_bounds_rate():
    slew = SlewLimiter(max_rate=1.0)
    assert slew.update(10.0, 0.1) == pytest.approx(0.1)
    assert slew.update(10.0, 0.1) == pytest.approx(0.2)
    slew.reset(0.0)
    assert slew.update(-10.0, 0.5) == pytest.approx(-0.5)


def test_slew_limiter_passes_through_with_zero_rate():
    slew = SlewLimiter(max_rate=0.0)
    assert slew.update(3.0, 0.1) == 3.0


def test_rate_of_change_reports_zero_on_first_call():
    rate = RateOfChange(smoothing=0.0)
    assert rate.update(5.0, 0.1) == 0.0
    assert rate.update(6.0, 0.1) == pytest.approx(10.0)


def test_hysteresis_needs_separate_thresholds():
    h = Hysteresis(on_threshold=0.6, off_threshold=0.4)
    assert h.update(0.5) is False
    assert h.update(0.7) is True
    assert h.update(0.5) is True  # stays on between the thresholds
    assert h.update(0.3) is False


def test_edge_detector_does_not_fire_on_first_sample():
    """Starting the bridge on the runway must not look like a touchdown."""
    edge = EdgeDetector()
    assert edge.update(True) == 0
    assert edge.update(True) == 0
    assert edge.update(False) == -1
    assert edge.update(True) == 1


def test_dwell_timer_requires_continuous_condition():
    dwell = DwellTimer(0.5)
    for _ in range(29):
        assert dwell.update(True, 1 / 60) is False
    # Accumulating 1/60 thirty times lands a hair under 0.5, so allow the extra
    # frame rather than pretending float addition is exact.
    dwell.update(True, 1 / 60)
    assert dwell.update(True, 1 / 60) is True
    assert dwell.update(False, 1 / 60) is False
    assert dwell.update(True, 1 / 60) is False  # has to start over


def test_oneshot_rises_then_decays():
    shot = OneShot(attack=0.01, decay=0.1)
    shot.fire(1.0)
    peak = shot.update(0.01)
    assert peak == pytest.approx(1.0, abs=0.05)
    for _ in range(20):
        shot.update(0.01)
    assert shot.active is False
    assert shot.update(0.01) == 0.0


def test_oneshot_keeps_the_stronger_of_overlapping_hits():
    """A weak hit in the same tick must not displace a strong one.

    Regression: the guard compared against the envelope's current value, which
    is zero while it is still rising, so a nosewheel arriving in the same frame
    as the mains quietly replaced the main gear thump with a smaller one.
    """
    shot = OneShot(attack=0.01, decay=0.2)
    shot.fire(0.9)
    shot.fire(0.3)
    assert shot.update(0.01) == pytest.approx(0.9, abs=0.05)


def test_oneshot_stronger_hit_replaces_a_fading_one():
    shot = OneShot(attack=0.01, decay=0.2)
    shot.fire(0.3)
    shot.update(0.05)
    shot.fire(0.9)
    assert shot.update(0.01) == pytest.approx(0.9, abs=0.1)


def test_band_noise_is_deterministic_and_bounded():
    a, b = BandNoise(seed=7), BandNoise(seed=7)
    for _ in range(200):
        x, y = a.update(0.01), b.update(0.01)
        assert x == y
        assert -1.0 <= x <= 1.0


def test_phase_accumulator_stays_continuous_across_frequency_changes():
    """Ground rumble must glide with speed rather than click on every update."""
    phase = PhaseAccumulator()
    dt = 1 / 200
    previous = phase.phase
    # Sweep 5 Hz to 40 Hz, the range ground rumble covers as speed builds. The
    # property that matters is that phase only ever advances by exactly one
    # step's worth: evaluating sin(2*pi*f*t) directly would jump every time the
    # frequency changed, which is audible as a click.
    for step in range(200):
        frequency = 5.0 + step * 0.175
        phase.update(frequency, dt)
        advance = (phase.phase - previous) % 1.0
        assert advance == pytest.approx(frequency * dt, abs=1e-9)
        previous = phase.phase
    assert 0.0 <= phase.phase < 1.0


def test_phase_accumulator_tracks_frequency():
    phase = PhaseAccumulator()
    for _ in range(100):
        phase.update(1.0, 0.01)  # one full cycle
    assert phase.phase == pytest.approx(0.0, abs=1e-9) or phase.phase == pytest.approx(
        1.0, abs=1e-9
    )
    assert phase.sine() == pytest.approx(0.0, abs=1e-6)
    assert math.isfinite(phase.sine())
