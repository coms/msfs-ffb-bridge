"""Tests for the ground/air axis handoff.

This is the part of the bridge that can bite: one physical axis has to change
meaning mid-flight, and it must never do so abruptly or at the wrong moment.
"""

from __future__ import annotations

import pytest

from ffbbridge.core.config import RoutingConfig, WheelConfig
from ffbbridge.core.context import AxisMode
from ffbbridge.core.routing import AxisRouter, OverrideState
from ffbbridge.core.telemetry import FlightTelemetry, WheelState

DT = 1 / 60


def rolling(**overrides) -> FlightTelemetry:
    base = {"connected": True, "on_ground": True, "contact_compression": (0.4, 0.6, 0.6)}
    base.update(overrides)
    return FlightTelemetry(**base)


def airborne(**overrides) -> FlightTelemetry:
    base = {
        "connected": True,
        "on_ground": False,
        "contact_compression": (0.0, 0.0, 0.0),
        "agl_ft": 200.0,
        "ias_kt": 80.0,
    }
    base.update(overrides)
    return FlightTelemetry(**base)


def make_router(**routing_overrides) -> AxisRouter:
    routing = RoutingConfig(**routing_overrides)
    return AxisRouter(routing, WheelConfig(deadzone=0.0, expo=0.0))


def drive(router, telemetry, seconds, *, wheel=None):
    """Run the router for a period and return the last command."""
    wheel = wheel if wheel is not None else WheelState()
    command = None
    for _ in range(int(seconds / DT)):
        command = router.update(telemetry, wheel, DT)
    return command


def test_starts_on_the_ground_steering():
    router = make_router()
    command = router.update(rolling(), WheelState(position=0.3), DT)
    assert command.mode is AxisMode.GROUND
    assert command.rudder > 0
    assert command.aileron == 0.0


def test_inactive_axis_is_held_at_zero_not_abandoned():
    """Leaving an axis unsent would freeze the sim's last value there."""
    router = make_router()
    command = drive(router, airborne(), 6.0, wheel=WheelState(position=0.5))
    assert command.mode is AxisMode.AIR
    assert command.rudder == 0.0
    assert command.aileron > 0


def test_handover_to_ailerons_after_liftoff():
    router = make_router(air_dwell_s=1.0, transition_ms=1000.0)
    drive(router, rolling(), 1.0)
    assert router.mode() is AxisMode.GROUND

    # The dwell has to elapse before anything moves.
    drive(router, airborne(), 0.5)
    assert router.mode() is AxisMode.GROUND

    drive(router, airborne(), 0.8)
    assert router.mode() is AxisMode.TO_AIR

    drive(router, airborne(), 1.2)
    assert router.mode() is AxisMode.AIR


def test_handover_back_to_steering_after_touchdown():
    router = make_router(ground_dwell_s=0.7, transition_ms=1000.0)
    drive(router, airborne(), 5.0)
    assert router.mode() is AxisMode.AIR

    drive(router, rolling(), 0.4)
    assert router.mode() is AxisMode.AIR  # dwell not yet satisfied

    drive(router, rolling(), 0.5)
    assert router.mode() is AxisMode.TO_GROUND

    drive(router, rolling(), 1.2)
    assert router.mode() is AxisMode.GROUND


def test_a_bounce_does_not_flip_the_axis():
    """A wheel touching briefly must not hand the axis over and back."""
    router = make_router(ground_dwell_s=0.7)
    drive(router, airborne(), 5.0)
    drive(router, rolling(), 0.3)  # brief contact
    drive(router, airborne(), 0.3)
    assert router.mode() is AxisMode.AIR
    assert router.ground_weight == 0.0


def test_a_bump_on_the_runway_does_not_hand_over():
    router = make_router(air_dwell_s=1.0)
    drive(router, rolling(), 2.0)
    # Wheels unload momentarily but the aircraft is nowhere near flying.
    drive(router, rolling(contact_compression=(0.0, 0.0, 0.0), agl_ft=0.5), 0.5)
    assert router.mode() is AxisMode.GROUND


def test_altitude_gate_stops_an_early_handover():
    """Unloaded wheels alone are not enough; the aircraft has to be climbing."""
    router = make_router(air_agl_ft=15.0, air_dwell_s=0.5)
    drive(router, rolling(contact_compression=(0.0, 0.0, 0.0), agl_ft=3.0, ias_kt=55.0), 3.0)
    assert router.mode() is AxisMode.GROUND


def test_transition_reverses_cleanly_from_wherever_it_is():
    """An aborted takeoff turns the handoff around without a discontinuity."""
    router = make_router(air_dwell_s=0.5, ground_dwell_s=0.5, transition_ms=2000.0)
    drive(router, rolling(), 1.0)
    drive(router, airborne(), 1.2)
    assert router.mode() is AxisMode.TO_AIR
    partway = router.ground_weight
    assert 0.0 < partway < 1.0

    # Reversing still waits out the ground dwell, so the blend keeps travelling
    # toward air for a moment before turning around from wherever it reached.
    drive(router, rolling(), 0.6)
    assert router.mode() is AxisMode.TO_GROUND
    turning_point = router.ground_weight
    assert 0.0 < turning_point < partway
    drive(router, rolling(), 0.3)
    assert router.ground_weight > turning_point  # now heading back
    drive(router, rolling(), 2.5)
    assert router.mode() is AxisMode.GROUND


def test_axis_commands_never_jump():
    """Whatever the sim does, the surfaces must not be commanded to snap."""
    router = make_router(air_dwell_s=0.5, transition_ms=800.0)
    wheel = WheelState(position=0.9)
    previous = router.update(rolling(), wheel, DT)
    states = [rolling()] * 60 + [airborne()] * 180 + [rolling()] * 120
    for telemetry in states:
        command = router.update(telemetry, wheel, DT)
        assert abs(command.aileron - previous.aileron) <= AxisRouter.MAX_AXIS_RATE * DT + 1e-9
        assert abs(command.rudder - previous.rudder) <= AxisRouter.MAX_AXIS_RATE * DT + 1e-9
        previous = command


def test_held_deflection_is_not_dumped_onto_the_incoming_axis():
    """The crosswind landing case: aileron held into wind as the axis changes.

    The incoming channel is referenced to the wheel position at handoff, so the
    first moments of the transition command near zero rather than translating a
    held aileron input straight into rudder.
    """
    router = make_router(ground_dwell_s=0.5, transition_ms=1500.0)
    held = WheelState(position=0.8)
    drive(router, airborne(), 5.0, wheel=held)

    drive(router, rolling(), 0.55, wheel=held)  # dwell satisfied, handoff begins
    assert router.mode() is AxisMode.TO_GROUND
    early = router.update(rolling(), held, DT)
    assert abs(early.rudder) < 0.2


def test_incoming_axis_reaches_full_authority_by_the_end():
    router = make_router(ground_dwell_s=0.5, transition_ms=800.0)
    held = WheelState(position=0.5)
    drive(router, airborne(), 5.0, wheel=held)
    command = drive(router, rolling(), 4.0, wheel=held)
    assert router.mode() is AxisMode.GROUND
    expected = 0.5 / WheelConfig().ground_range
    assert command.rudder == pytest.approx(min(1.0, expected), abs=0.02)


def test_wheel_range_scales_the_command():
    """Less wheel travel for full deflection means a more sensitive axis."""
    routing = RoutingConfig()
    sensitive = AxisRouter(routing, WheelConfig(deadzone=0.0, expo=0.0, ground_range=0.25))
    coarse = AxisRouter(routing, WheelConfig(deadzone=0.0, expo=0.0, ground_range=1.0))
    wheel = WheelState(position=0.2)
    # Drive past the rate limiter, which caps both on the opening tick.
    assert (
        drive(sensitive, rolling(), 1.0, wheel=wheel).rudder
        > drive(coarse, rolling(), 1.0, wheel=wheel).rudder
    )


def test_calibration_applies_deadzone_and_inversion():
    routing = RoutingConfig()
    router = AxisRouter(routing, WheelConfig(deadzone=0.2, expo=0.0, ground_range=1.0))
    assert router.update(rolling(), WheelState(position=0.1), DT).rudder == 0.0

    inverted = AxisRouter(routing, WheelConfig(deadzone=0.0, expo=0.0, invert=True))
    assert inverted.update(rolling(), WheelState(position=0.3), DT).rudder < 0


def test_aileron_only_mode_never_steers():
    router = make_router(mode="aileron_only")
    command = drive(router, rolling(), 4.0, wheel=WheelState(position=0.4))
    assert command.rudder == 0.0
    assert command.aileron > 0
    assert router.mode() is AxisMode.AIR


def test_rudder_only_mode_never_rolls():
    router = make_router(mode="rudder_only")
    command = drive(router, airborne(), 4.0, wheel=WheelState(position=0.4))
    assert command.aileron == 0.0
    assert command.rudder > 0


def test_override_button_cycles_modes():
    router = make_router(override_button=2)
    pressed = WheelState(position=0.0, buttons=(False, False, True))
    released = WheelState(position=0.0, buttons=(False, False, False))

    router.update(rolling(), pressed, DT)
    assert router.override == OverrideState.FORCE_GROUND
    router.update(rolling(), pressed, DT)  # held, not a new press
    assert router.override == OverrideState.FORCE_GROUND
    router.update(rolling(), released, DT)
    router.update(rolling(), pressed, DT)
    assert router.override == OverrideState.FORCE_AIR
    router.update(rolling(), released, DT)
    router.update(rolling(), pressed, DT)
    assert router.override == OverrideState.AUTO


def test_override_forces_the_axis_regardless_of_state():
    router = make_router(override_button=0)
    router.set_override(OverrideState.FORCE_AIR)
    command = drive(router, rolling(), 4.0, wheel=WheelState(position=0.4))
    assert command.aileron > 0
    assert command.rudder == 0.0


def test_missing_override_button_is_ignored():
    router = make_router(override_button=7)
    router.update(rolling(), WheelState(buttons=(True,)), DT)
    assert router.override == OverrideState.AUTO


def test_tiller_output_mirrors_the_rudder_when_enabled():
    assert make_router(use_tiller=False).update(rolling(), WheelState(), DT).steering is None
    command = make_router(use_tiller=True).update(rolling(), WheelState(position=0.3), DT)
    assert command.steering == command.rudder


def test_reset_assumes_the_state_it_is_told():
    router = make_router()
    router.reset(on_ground=False)
    assert router.mode() is AxisMode.AIR
    router.reset(on_ground=True)
    assert router.mode() is AxisMode.GROUND


def test_context_reports_transition_progress():
    router = make_router(air_dwell_s=0.5, transition_ms=1000.0)
    drive(router, rolling(), 1.0)
    context = router.context(telemetry_stale=False, seconds=0.0)
    assert context.transition_progress == 0.0
    assert context.ground_weight == 1.0

    drive(router, airborne(), 1.0)
    context = router.context(telemetry_stale=False, seconds=1.0)
    assert context.mode is AxisMode.TO_AIR
    assert 0.0 < context.transition_progress < 1.0
    assert context.air_weight == pytest.approx(1.0 - context.ground_weight)
