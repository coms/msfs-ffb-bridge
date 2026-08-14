"""Tests for the force vocabulary and how contributions combine."""

from __future__ import annotations

import pytest

from ffbbridge.core.forces import (
    Contribution,
    Damper,
    ForceOutput,
    Periodic,
    Spring,
    Waveform,
    combine_dampers,
    combine_springs,
)


@pytest.mark.parametrize(
    ("waveform", "phase", "expected"),
    [
        (Waveform.SINE, 0.0, 0.0),
        (Waveform.SINE, 0.25, 1.0),
        (Waveform.SINE, 0.75, -1.0),
        (Waveform.SQUARE, 0.1, 1.0),
        (Waveform.SQUARE, 0.6, -1.0),
        (Waveform.TRIANGLE, 0.0, 1.0),
        (Waveform.TRIANGLE, 0.5, -1.0),
        (Waveform.SAWTOOTH_UP, 0.0, -1.0),
        (Waveform.SAWTOOTH_UP, 1.0, -1.0),
        (Waveform.SAWTOOTH_DOWN, 0.0, 1.0),
    ],
)
def test_waveform_sampling(waveform, phase, expected):
    assert waveform.sample(phase) == pytest.approx(expected, abs=1e-9)


def test_waveforms_stay_in_range_and_wrap():
    for waveform in Waveform:
        for i in range(200):
            phase = i / 50.0  # deliberately exceeds 1.0 to exercise wrapping
            assert -1.0 <= waveform.sample(phase) <= 1.0


def test_spring_pulls_toward_its_centre():
    spring = Spring(coefficient=0.5, center=0.0, saturation=1.0)
    assert spring.force_at(0.4) < 0  # displaced right, pushed left
    assert spring.force_at(-0.4) > 0
    assert spring.force_at(0.0) == 0.0


def test_spring_centre_is_where_trim_puts_it():
    """An out-of-trim aircraft rests at a displaced neutral, not at centre."""
    spring = Spring(coefficient=0.5, center=0.3, saturation=1.0)
    assert spring.force_at(0.3) == 0.0
    assert spring.force_at(0.0) > 0  # pulled back toward the trimmed position


def test_spring_saturates():
    spring = Spring(coefficient=1.0, center=0.0, saturation=0.2)
    assert spring.force_at(1.0) == pytest.approx(-0.2)


def test_spring_deadband_is_continuous():
    spring = Spring(coefficient=1.0, deadband=0.1)
    assert spring.force_at(0.05) == 0.0
    assert spring.force_at(0.11) == pytest.approx(-0.01, abs=1e-6)


def test_damper_opposes_motion():
    damper = Damper(coefficient=0.5, saturation=1.0)
    assert damper.force_at(1.0) == pytest.approx(-0.5)
    assert damper.force_at(-1.0) == pytest.approx(0.5)
    assert damper.force_at(0.0) == 0.0


def test_combine_springs_adds_stiffness_and_averages_centres():
    combined = combine_springs(
        [Spring(coefficient=0.2, center=0.0), Spring(coefficient=0.6, center=0.4)]
    )
    assert combined is not None
    assert combined.coefficient == pytest.approx(0.8)
    # Stiffness-weighted: the stronger spring dominates where it settles.
    assert combined.center == pytest.approx((0.0 * 0.2 + 0.4 * 0.6) / 0.8)


def test_combine_springs_ignores_negligible_entries():
    assert combine_springs([]) is None
    assert combine_springs([Spring(coefficient=0.0)]) is None


def test_combine_springs_clamps_total_stiffness():
    combined = combine_springs([Spring(coefficient=0.8), Spring(coefficient=0.9)])
    assert combined is not None
    assert combined.coefficient == 1.0


def test_combine_dampers_sums():
    combined = combine_dampers([Damper(coefficient=0.3), Damper(coefficient=0.2)])
    assert combined is not None
    assert combined.coefficient == pytest.approx(0.5)
    assert combine_dampers([Damper(coefficient=0.0)]) is None


def test_contribution_add_periodic_drops_inaudible_requests():
    contribution = Contribution()
    contribution.add_periodic("quiet", frequency_hz=10.0, magnitude=0.0)
    contribution.add_periodic("static", frequency_hz=0.0, magnitude=0.5)
    contribution.add_periodic("real", frequency_hz=10.0, magnitude=0.5)
    assert [p.label for p in contribution.periodics] == ["real"]


def test_contribution_is_empty_detects_nothing_requested():
    assert Contribution().is_empty
    assert not Contribution(constant=0.1).is_empty
    assert not Contribution(spring=Spring(coefficient=0.1)).is_empty


def test_periodic_scaling_clamps():
    periodic = Periodic(label="x", magnitude=0.8, offset=0.5)
    assert periodic.scaled(2.0).magnitude == 1.0
    assert periodic.scaled(0.5).magnitude == pytest.approx(0.4)


def test_periodic_audibility():
    assert Periodic(label="x", magnitude=0.5, frequency_hz=10.0).is_audible
    assert not Periodic(label="x", magnitude=0.0, frequency_hz=10.0).is_audible
    assert not Periodic(label="x", magnitude=0.5, frequency_hz=0.0).is_audible


def test_force_output_totals_all_channels():
    output = ForceOutput(
        constant=0.1,
        spring=Spring(coefficient=0.5, center=0.0, saturation=1.0),
        damper=Damper(coefficient=0.2, saturation=1.0),
    )
    # 0.1 constant, minus 0.5*0.4 spring, minus 0.2*1.0 damping.
    assert output.total_at(0.4, 1.0) == pytest.approx(0.1 - 0.2 - 0.2)
