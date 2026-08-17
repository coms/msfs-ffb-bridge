"""Tests for the soft lock: the control stop at the end of usable travel."""

from __future__ import annotations

import pytest

from ffbbridge.core.config import ModuleSettings, RoutingConfig, SafetyConfig, WheelConfig
from ffbbridge.core.context import AxisMode, TickContext
from ffbbridge.core.forces import Contribution, Spring, combine_springs
from ffbbridge.core.mixer import EffectMixer
from ffbbridge.core.modules import SoftLock
from ffbbridge.core.modules.base import EffectModule
from ffbbridge.core.routing import AxisRouter, inverse_shape, shape_displacement
from ffbbridge.core.telemetry import FlightTelemetry, WheelState

DT = 1 / 60


def ground_state() -> FlightTelemetry:
    return FlightTelemetry(connected=True, on_ground=True, contact_compression=(0.4, 0.6, 0.6))


def air_state() -> FlightTelemetry:
    return FlightTelemetry(
        connected=True,
        on_ground=False,
        agl_ft=200.0,
        ias_kt=80.0,
        contact_compression=(0.0, 0.0, 0.0),
    )


def drive(router, telemetry, seconds):
    for _ in range(int(seconds / DT)):
        router.update(telemetry, WheelState(), DT)


# --- Finding where the control runs out ---------------------------------


@pytest.mark.parametrize("target", [0.1, 0.35, 0.5, 0.7, 0.95])
def test_inverse_shaping_round_trips(target):
    """The stop has to sit exactly where the axis saturates, not near it."""
    cfg = WheelConfig()
    displacement = inverse_shape(target, cfg)
    assert shape_displacement(displacement, cfg) == pytest.approx(target, abs=1e-4)


def test_inverse_shaping_handles_the_ends():
    cfg = WheelConfig()
    assert inverse_shape(1.0, cfg) == 1.0
    assert inverse_shape(0.0, cfg) == 0.0
    assert inverse_shape(2.0, cfg) == 1.0


def test_expo_pushes_the_stop_further_out():
    """Softening the centre means more travel is needed to reach full deflection."""
    linear = WheelConfig(expo=0.0, deadzone=0.0)
    curved = WheelConfig(expo=0.6, deadzone=0.0)
    assert inverse_shape(0.35, curved) > inverse_shape(0.35, linear)


def test_a_deadzone_pushes_the_stop_further_out():
    plain = WheelConfig(expo=0.0, deadzone=0.0)
    with_deadzone = WheelConfig(expo=0.0, deadzone=0.1)
    assert inverse_shape(0.35, with_deadzone) > inverse_shape(0.35, plain)


# --- The stop moves with the axis ----------------------------------------


def test_the_stop_is_further_out_for_steering_than_for_ailerons():
    """The two axes use different amounts of travel, so the stop has to move."""
    router = AxisRouter(RoutingConfig(), WheelConfig())
    drive(router, ground_state(), 1.0)
    on_ground = router.lock_displacement()

    drive(router, air_state(), 10.0)
    assert router.mode() is AxisMode.AIR
    in_air = router.lock_displacement()

    assert in_air < on_ground
    assert in_air == pytest.approx(inverse_shape(WheelConfig().air_range, WheelConfig()), abs=1e-4)


def test_the_stop_moves_smoothly_through_a_handoff():
    router = AxisRouter(RoutingConfig(transition_ms=1000.0, air_dwell_s=0.5), WheelConfig())
    drive(router, ground_state(), 1.0)

    previous = router.lock_displacement()
    seen = [previous]
    for _ in range(180):
        router.update(air_state(), WheelState(), DT)
        current = router.lock_displacement()
        assert abs(current - previous) < 0.05, "the stop jumped mid-handoff"
        previous = current
        seen.append(current)

    assert min(seen) < max(seen), "the stop never moved"


def test_the_context_carries_the_stop_to_the_modules():
    router = AxisRouter(RoutingConfig(), WheelConfig())
    drive(router, ground_state(), 1.0)
    context = router.context(telemetry_stale=False, seconds=0.0)
    assert 0.0 < context.lock_displacement < 1.0
    assert context.center == 0.0


# --- The module ----------------------------------------------------------


def make_context(lock: float, center: float = 0.0) -> TickContext:
    return TickContext(mode=AxisMode.AIR, ground_weight=0.0, lock_displacement=lock, center=center)


def test_no_force_at_all_within_the_usable_range():
    """The stop must be invisible while flying, or it would corrupt every effect."""
    spring = SoftLock().update(air_state(), WheelState(), make_context(0.45), DT).end_stop
    assert spring is not None
    for position in (0.0, 0.1, 0.25, 0.4, 0.45):
        assert spring.force_at(position) == 0.0


def test_resistance_builds_past_the_stop():
    spring = SoftLock().update(air_state(), WheelState(), make_context(0.45), DT).end_stop
    forces = [abs(spring.force_at(p)) for p in (0.55, 0.7, 0.85, 1.0)]
    assert forces == sorted(forces)
    assert forces[0] > 0.0
    assert forces[-1] > 0.3


def test_the_stop_pushes_back_toward_centre():
    spring = SoftLock().update(air_state(), WheelState(), make_context(0.45), DT).end_stop
    assert spring.force_at(0.9) < 0  # turned right, pushed left
    assert spring.force_at(-0.9) > 0


def test_the_stop_is_symmetric():
    spring = SoftLock().update(air_state(), WheelState(), make_context(0.45), DT).end_stop
    assert spring.force_at(0.8) == pytest.approx(-spring.force_at(-0.8))


def test_the_margin_leaves_full_deflection_reachable():
    """The stop must not eat the last of the travel the axis actually needs."""
    module = SoftLock()
    spring = module.update(air_state(), WheelState(), make_context(0.45), DT).end_stop
    assert spring.deadband > 0.45


def test_nothing_is_produced_when_the_whole_rim_is_mapped():
    """No wasted travel means no stop to add; the base's own limit is the stop."""
    contribution = SoftLock().update(air_state(), WheelState(), make_context(1.0), DT)
    assert contribution.end_stop is None
    assert contribution.is_empty


def test_zero_strength_disables_it():
    module = SoftLock(ModuleSettings(enabled=True, gain=1.0, params={"strength": 0.0}))
    assert module.update(air_state(), WheelState(), make_context(0.45), DT).is_empty


def test_the_stop_follows_a_calibrated_centre():
    spring = (
        SoftLock().update(air_state(), WheelState(), make_context(0.45, center=0.1), DT).end_stop
    )
    assert spring.center == pytest.approx(0.1)
    # The usable range travels with the centre rather than staying put.
    assert spring.force_at(0.5) == 0.0


# --- Keeping it out of the ordinary spring pool --------------------------


def test_merging_a_stop_into_the_spring_pool_would_be_dangerous():
    """Documents why the end stop is a separate channel.

    Springs combine by summing stiffness and taking the smallest deadband, so a
    wall plus a gentle centring spring collapses into a maximally stiff spring
    with no dead area: a wheel that snaps to centre instead of a stop to lean on.
    """
    merged = combine_springs(
        [
            Spring(coefficient=0.4, center=0.0, deadband=0.0),  # control loading
            Spring(coefficient=1.0, center=0.0, deadband=0.45),  # the stop
        ]
    )
    assert merged is not None
    assert merged.deadband == 0.0
    assert merged.coefficient == 1.0
    assert abs(merged.force_at(0.1)) > 0.0  # force where there should be none


class _Stub(EffectModule):
    def __init__(self, module_id: str, contribution: Contribution) -> None:
        self.id = module_id
        self.name = module_id
        super().__init__(ModuleSettings(enabled=True, gain=1.0))
        self._contribution = contribution

    def update(self, tel, wheel, ctx, dt):
        return self._contribution


def run_mixer(modules, *, ticks=200, safety=None):
    mixer = EffectMixer(modules, safety or SafetyConfig(master_gain=0.5), periodic_slots=3)
    output = None
    for _ in range(ticks):
        output = mixer.update(FlightTelemetry(connected=True), WheelState(), TickContext(), 0.01)
    return output


def test_the_mixer_keeps_the_stop_on_its_own_channel():
    output = run_mixer(
        [
            _Stub("loading", Contribution(spring=Spring(coefficient=0.4, deadband=0.0))),
            _Stub("stop", Contribution(end_stop=Spring(coefficient=1.0, deadband=0.45))),
        ]
    )
    assert output.spring is not None
    assert output.spring.deadband == 0.0
    assert output.end_stop is not None
    assert output.end_stop.deadband == pytest.approx(0.45)
    # The centring spring is untouched by the stop's stiffness.
    assert output.spring.coefficient < 0.5


def test_the_master_gain_does_not_soften_the_stop():
    """A stop scaled to a third is one you push straight through."""
    quiet = run_mixer(
        [_Stub("stop", Contribution(end_stop=Spring(coefficient=1.0, deadband=0.45)))],
        safety=SafetyConfig(master_gain=0.2),
    )
    loud = run_mixer(
        [_Stub("stop", Contribution(end_stop=Spring(coefficient=1.0, deadband=0.45)))],
        safety=SafetyConfig(master_gain=1.0),
    )
    assert quiet.end_stop.coefficient == pytest.approx(loud.end_stop.coefficient)


def test_the_stop_still_disappears_when_forces_are_cut():
    """Everything goes quiet when the sim is paused, the stop included."""
    mixer = EffectMixer(
        [_Stub("stop", Contribution(end_stop=Spring(coefficient=1.0, deadband=0.45)))],
        SafetyConfig(master_gain=1.0, decay_ms=100.0),
        periodic_slots=3,
    )
    paused = FlightTelemetry(connected=True, paused=True)
    for _ in range(100):
        output = mixer.update(paused, WheelState(), TickContext(), 0.01)
    assert output.end_stop is None or output.end_stop.coefficient == pytest.approx(0.0, abs=1e-6)


def test_the_tightest_stop_wins_if_two_are_offered():
    output = run_mixer(
        [
            _Stub("wide", Contribution(end_stop=Spring(coefficient=1.0, deadband=0.8))),
            _Stub("tight", Contribution(end_stop=Spring(coefficient=1.0, deadband=0.4))),
        ]
    )
    assert output.end_stop.deadband == pytest.approx(0.4)


# --- End to end -----------------------------------------------------------


def test_a_flown_sortie_produces_a_stop_that_tracks_the_axis():
    from ffbbridge.core.engine import BridgeEngine
    from ffbbridge.core.synthetic import SyntheticFlight

    engine = BridgeEngine()
    wheel = WheelState(position=0.0, connected=True)
    taxi, cruise = None, None
    for telemetry in SyntheticFlight().stream():
        result = engine.tick(telemetry, wheel, telemetry.t)
        if abs(telemetry.t - 30.0) < 0.01:
            taxi = result.force.end_stop
        if abs(telemetry.t - 180.0) < 0.01:
            cruise = result.force.end_stop

    assert taxi is not None and cruise is not None
    assert cruise.deadband < taxi.deadband, "the stop should tighten in the air"
    assert 0.0 < cruise.deadband < 1.0
