"""Tests for the mixer: slot budget, gains and the safety envelope."""

from __future__ import annotations

import pytest

from ffbbridge.core.config import ModuleSettings, SafetyConfig
from ffbbridge.core.context import TickContext
from ffbbridge.core.forces import Contribution, Damper, Spring, Waveform
from ffbbridge.core.mixer import EffectMixer
from ffbbridge.core.modules.base import EffectModule
from ffbbridge.core.telemetry import FlightTelemetry, WheelState


class FakeModule(EffectModule):
    """A module that returns whatever the test tells it to."""

    def __init__(self, module_id: str, contribution: Contribution, *, gain: float = 1.0) -> None:
        self.id = module_id
        self.name = module_id
        super().__init__(ModuleSettings(enabled=True, gain=gain))
        self._contribution = contribution
        self.reset_count = 0

    def update(self, tel, wheel, ctx, dt):
        return self._contribution

    def reset(self):
        self.reset_count += 1


class ExplodingModule(EffectModule):
    id = "exploding"
    name = "exploding"

    def __init__(self):
        super().__init__()
        self.reset_count = 0

    def update(self, tel, wheel, ctx, dt):
        raise ValueError("bad maths")

    def reset(self):
        self.reset_count += 1


def connected() -> FlightTelemetry:
    return FlightTelemetry(connected=True)


def make_mixer(modules, *, slots=3, safety=None):
    safety = safety or SafetyConfig(master_gain=1.0, max_slew_per_s=1000.0)
    return EffectMixer(modules, safety, periodic_slots=slots)


def run(mixer, *, ticks=1, dt=1 / 100, tel=None, ctx=None, wheel=None):
    tel = tel if tel is not None else connected()
    ctx = ctx if ctx is not None else TickContext()
    wheel = wheel if wheel is not None else WheelState()
    output = None
    for _ in range(ticks):
        output = mixer.update(tel, wheel, ctx, dt)
    return output


def test_constant_forces_sum_and_are_attributed():
    mixer = make_mixer(
        [
            FakeModule("a", Contribution(constant=0.2)),
            FakeModule("b", Contribution(constant=-0.05)),
        ]
    )
    output = run(mixer)
    assert output.constant == pytest.approx(0.15)
    assert output.breakdown == {"a": pytest.approx(0.2), "b": pytest.approx(-0.05)}


def test_module_gain_scales_its_contribution():
    mixer = make_mixer([FakeModule("a", Contribution(constant=0.4), gain=0.5)])
    assert run(mixer).constant == pytest.approx(0.2)


def test_disabled_modules_are_skipped():
    module = FakeModule("a", Contribution(constant=0.4))
    module.settings.enabled = False
    output = run(make_mixer([module]))
    assert output.constant == 0.0
    assert output.breakdown == {}


def test_master_gain_scales_everything():
    mixer = make_mixer(
        [
            FakeModule(
                "a",
                Contribution(
                    constant=0.4,
                    spring=Spring(coefficient=0.8, saturation=1.0),
                    damper=Damper(coefficient=0.4),
                ),
            )
        ],
        safety=SafetyConfig(master_gain=0.5, max_slew_per_s=1000.0),
    )
    output = run(mixer, ticks=200)
    assert output.constant == pytest.approx(0.2)
    assert output.spring.coefficient == pytest.approx(0.4, abs=0.01)
    assert output.damper.coefficient == pytest.approx(0.2)


def test_a_control_stop_is_not_scaled_by_the_master_strength():
    """Turning the force model down must not quietly widen the control travel.

    Master strength is taste; a stop is not. Softened to a third it stops being
    a stop, which is exactly how it was reported broken.
    """
    stop = FakeModule("stop", Contribution(constant=0.9))
    stop.ignores_master_gain = True
    ordinary = FakeModule("ordinary", Contribution(constant=0.9))

    safety = SafetyConfig(master_gain=0.32, max_force=1.0, max_slew_per_s=1000.0)
    assert run(make_mixer([stop], safety=safety), ticks=50).constant == pytest.approx(0.9)
    assert run(make_mixer([ordinary], safety=safety), ticks=50).constant == pytest.approx(0.288)


def test_a_control_stop_still_obeys_the_ceiling_and_the_envelope():
    """Opting out of taste is not opting out of safety."""
    stop = FakeModule("stop", Contribution(constant=0.9))
    stop.ignores_master_gain = True

    safety = SafetyConfig(master_gain=1.0, max_force=0.5, max_slew_per_s=1000.0)
    assert run(make_mixer([stop], safety=safety), ticks=50).constant == pytest.approx(0.5)

    stale = TickContext(telemetry_stale=True)
    faded = run(make_mixer([stop]), ticks=400, ctx=stale)
    assert faded.constant == pytest.approx(0.0, abs=1e-3)


def test_total_force_is_clamped_and_reported():
    mixer = make_mixer(
        [FakeModule("a", Contribution(constant=5.0))],
        safety=SafetyConfig(master_gain=1.0, max_force=0.6, max_slew_per_s=1000.0),
    )
    output = run(mixer)
    assert output.constant == pytest.approx(0.6)


def test_slew_limiting_bounds_the_steady_channel():
    """A telemetry glitch must not arrive at the rim as a step."""
    mixer = make_mixer(
        [FakeModule("a", Contribution(constant=1.0))],
        safety=SafetyConfig(master_gain=1.0, max_force=1.0, max_slew_per_s=2.0),
    )
    output = run(mixer, ticks=1, dt=0.01)
    assert output.constant == pytest.approx(0.02)


def test_vibration_bypasses_the_slew_limiter():
    """Rate limiting a 30 Hz rumble would erase it, so it is added afterwards."""
    contribution = Contribution()
    contribution.add_periodic("rumble", frequency_hz=30.0, magnitude=0.5)
    mixer = make_mixer(
        [FakeModule("a", contribution)],
        slots=0,  # force software synthesis
        safety=SafetyConfig(master_gain=1.0, max_force=1.0, max_slew_per_s=0.5),
    )
    peak = max(abs(run(mixer, ticks=1, dt=1 / 200).constant) for _ in range(60))
    assert peak > 0.1


def test_periodics_within_the_budget_go_to_hardware():
    contribution = Contribution()
    contribution.add_periodic("one", frequency_hz=10.0, magnitude=0.5)
    contribution.add_periodic("two", frequency_hz=20.0, magnitude=0.5)
    output = run(make_mixer([FakeModule("a", contribution)], slots=3))
    assert {p.label for p in output.periodics} == {"one", "two"}


def test_overflow_periodics_are_synthesised_not_dropped():
    contribution = Contribution()
    for i in range(5):
        contribution.add_periodic(f"p{i}", frequency_hz=10.0 + i, magnitude=0.3, priority=i)
    mixer = make_mixer([FakeModule("a", contribution)], slots=2)
    output = run(mixer)
    assert len(output.periodics) == 2
    assert set(mixer.diagnostics.software_periodics) == {"p0", "p1", "p2"}
    assert mixer.diagnostics.dropped_periodics == ()


def test_hardware_slots_go_to_the_highest_priority():
    contribution = Contribution()
    contribution.add_periodic("quiet_important", frequency_hz=10.0, magnitude=0.3, priority=100)
    contribution.add_periodic("loud_background", frequency_hz=20.0, magnitude=0.9, priority=1)
    output = run(make_mixer([FakeModule("a", contribution)], slots=1))
    assert [p.label for p in output.periodics] == ["quiet_important"]


def test_a_faint_effect_yields_its_slot_to_a_stronger_one():
    """A barely perceptible high-priority effect should not displace real feel."""
    contribution = Contribution()
    contribution.add_periodic("barely_there", frequency_hz=10.0, magnitude=0.005, priority=100)
    contribution.add_periodic("rumble", frequency_hz=20.0, magnitude=0.4, priority=10)
    output = run(make_mixer([FakeModule("a", contribution)], slots=1))
    assert [p.label for p in output.periodics] == ["rumble"]


def test_springs_from_several_modules_combine():
    mixer = make_mixer(
        [
            FakeModule("a", Contribution(spring=Spring(coefficient=0.3, center=0.0))),
            FakeModule("b", Contribution(spring=Spring(coefficient=0.3, center=0.4))),
        ]
    )
    output = run(mixer, ticks=200)
    assert output.spring is not None
    assert output.spring.coefficient == pytest.approx(0.6, abs=0.01)
    assert output.spring.center == pytest.approx(0.2)


def test_forces_fade_to_zero_when_telemetry_goes_stale():
    mixer = make_mixer(
        [FakeModule("a", Contribution(constant=0.5))],
        safety=SafetyConfig(master_gain=1.0, max_slew_per_s=1000.0, decay_ms=100.0),
    )
    run(mixer, ticks=50)
    stale = TickContext(telemetry_stale=True)
    for _ in range(20):
        output = mixer.update(connected(), WheelState(), stale, 0.01)
    assert output.constant == pytest.approx(0.0, abs=1e-6)


def test_the_fade_is_gradual_rather_than_a_step():
    """Cutting force instantly would feel like the wheel had been dropped."""
    mixer = make_mixer(
        [FakeModule("a", Contribution(constant=0.5))],
        safety=SafetyConfig(master_gain=1.0, max_slew_per_s=1000.0, decay_ms=500.0),
    )
    run(mixer, ticks=50)
    stale = TickContext(telemetry_stale=True)
    first = mixer.update(connected(), WheelState(), stale, 0.01)
    assert 0.0 < abs(first.constant) < 0.5


def test_forces_are_cut_when_the_sim_is_paused():
    mixer = make_mixer([FakeModule("a", Contribution(constant=0.5))])
    paused = FlightTelemetry(connected=True, paused=True)
    for _ in range(200):
        output = mixer.update(paused, WheelState(), TickContext(), 0.01)
    assert output.constant == pytest.approx(0.0, abs=1e-6)


def test_forces_are_cut_outside_the_cockpit():
    mixer = make_mixer([FakeModule("a", Contribution(constant=0.5))])
    menu = FlightTelemetry(connected=True, in_cockpit=False)
    for _ in range(200):
        output = mixer.update(menu, WheelState(), TickContext(), 0.01)
    assert output.constant == pytest.approx(0.0, abs=1e-6)


def test_a_failing_module_is_isolated_not_fatal():
    """One bad module must not take the wheel with it."""
    exploding = ExplodingModule()
    mixer = make_mixer([exploding, FakeModule("good", Contribution(constant=0.3))])
    output = run(mixer)
    assert output.constant == pytest.approx(0.3)
    assert "exploding" in mixer.diagnostics.module_errors
    assert exploding.reset_count == 1


def test_reset_clears_module_state():
    module = FakeModule("a", Contribution(constant=0.1))
    mixer = make_mixer([module])
    run(mixer, ticks=5)
    mixer.reset()
    assert module.reset_count == 1


def test_phase_state_is_retired_for_inactive_effects():
    contribution = Contribution()
    contribution.add_periodic("temporary", frequency_hz=10.0, magnitude=0.5)
    module = FakeModule("a", contribution)
    mixer = make_mixer([module], slots=0)
    run(mixer, ticks=5)
    assert mixer._phases  # noqa: SLF001 - asserting internal bookkeeping
    module._contribution = Contribution()
    run(mixer, ticks=2)
    assert not mixer._phases  # noqa: SLF001


def test_waveform_choice_survives_to_the_output():
    contribution = Contribution()
    contribution.add_periodic("square", frequency_hz=10.0, magnitude=0.5, waveform=Waveform.SQUARE)
    output = run(make_mixer([FakeModule("a", contribution)]))
    assert output.periodics[0].waveform is Waveform.SQUARE
