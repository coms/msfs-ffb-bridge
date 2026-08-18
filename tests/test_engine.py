"""End-to-end tests: the whole bridge flown through a complete sortie."""

from __future__ import annotations

import math

import pytest

from ffbbridge.core.config import BridgeConfig, ModuleSettings, ProfileSet, SafetyConfig
from ffbbridge.core.context import AxisMode
from ffbbridge.core.engine import BridgeEngine
from ffbbridge.core.synthetic import SCRIPT_EVENTS, SyntheticFlight
from ffbbridge.core.telemetry import FlightTelemetry, WheelState


def fly(
    engine: BridgeEngine | None = None,
    flight: SyntheticFlight | None = None,
    *,
    wheel: WheelState | None = None,
    **kwargs,
):
    """Fly the scripted sortie and return every (telemetry, result) pair."""
    engine = engine or BridgeEngine()
    flight = flight or SyntheticFlight(**kwargs)
    wheel = wheel if wheel is not None else WheelState(position=0.0, connected=True)
    return [(tel, engine.tick(tel, wheel, tel.t)) for tel in flight.stream()]


@pytest.fixture(scope="module")
def sortie():
    return fly()


def at(sortie, seconds):
    """The result nearest a given moment in the flight."""
    return min(sortie, key=lambda pair: abs(pair[0].t - seconds))[1]


def test_a_complete_flight_produces_no_invalid_forces(sortie):
    for telemetry, result in sortie:
        assert math.isfinite(result.force.constant), f"non-finite force at t={telemetry.t}"
        assert -1.0 <= result.force.constant <= 1.0
        if result.force.spring:
            assert 0.0 <= result.force.spring.coefficient <= 1.0
            assert -1.0 <= result.force.spring.center <= 1.0
        for periodic in result.force.periodics:
            assert 0.0 <= periodic.magnitude <= 1.0
            assert periodic.frequency_hz > 0.0


def test_forces_respect_the_configured_ceiling(sortie):
    ceiling = SafetyConfig().max_force
    assert all(abs(result.force.constant) <= ceiling + 1e-9 for _, result in sortie)


def test_no_module_raised_during_the_flight(sortie):
    assert all(not result.force.breakdown.get("__error__") for _, result in sortie)
    _, last = sortie[-1]
    assert last.profile_name == "Default GA"


def test_the_axis_hands_over_once_in_each_direction(sortie):
    transitions = []
    previous = None
    for telemetry, result in sortie:
        if result.context.mode is not previous:
            transitions.append((telemetry.t, result.context.mode))
            previous = result.context.mode

    modes = [mode for _, mode in transitions]
    assert modes == [
        AxisMode.GROUND,
        AxisMode.TO_AIR,
        AxisMode.AIR,
        AxisMode.TO_GROUND,
        AxisMode.GROUND,
    ]

    liftoff = next(t for t, mode in transitions if mode is AxisMode.TO_AIR)
    landing = next(t for t, mode in transitions if mode is AxisMode.TO_GROUND)
    assert SCRIPT_EVENTS["liftoff"] < liftoff < SCRIPT_EVENTS["liftoff"] + 3.0
    assert SCRIPT_EVENTS["touchdown"] < landing < SCRIPT_EVENTS["touchdown"] + 2.0


def test_the_wheel_steers_on_the_ground_and_rolls_in_the_air():
    wheel = WheelState(position=0.4, connected=True)
    engine = BridgeEngine()
    flight = SyntheticFlight()
    results = [(tel, engine.tick(tel, wheel, tel.t)) for tel in flight.stream()]

    taxi = min(results, key=lambda pair: abs(pair[0].t - 30.0))[1]
    assert taxi.axis.rudder > 0.1
    assert taxi.axis.aileron == 0.0

    cruise = min(results, key=lambda pair: abs(pair[0].t - 180.0))[1]
    assert cruise.axis.aileron > 0.1
    assert cruise.axis.rudder == 0.0


def test_taking_the_rudder_off_the_wheel_keeps_the_road_under_it():
    """Steering on the pedals should not cost the ground feel.

    Runway rumble, touchdown and brakes key off weight on wheels rather than
    off the axis blend, so pinning the wheel to ailerons silences the steering
    forces and leaves what comes up through the airframe untouched.
    """
    profiles = ProfileSet()
    profiles.default.routing.mode = "aileron_only"
    wheel = WheelState(position=0.25, connected=True)
    pinned = fly(BridgeEngine(profiles), wheel=wheel)
    automatic = fly(BridgeEngine(), wheel=wheel)

    def rumble(sortie, seconds):
        result = at(sortie, seconds)
        return next((p.magnitude for p in result.force.periodics if p.label == "ground_roll"), 0.0)

    assert rumble(pinned, 70.0) > 0.0
    assert rumble(pinned, 70.0) == pytest.approx(rumble(automatic, 70.0))
    assert all(result.axis.rudder == 0.0 for _, result in pinned)


def test_runway_rumble_tracks_the_takeoff_roll(sortie):
    """The tone rises with groundspeed as the roll builds.

    Sampled before rotation: by 62 knots the wings have taken most of the weight
    and the rumble is deliberately fading, which is a different property.
    """

    def rumble(seconds):
        result = at(sortie, seconds)
        for periodic in result.force.periodics:
            if periodic.label == "ground_roll":
                return periodic.frequency_hz
        return 0.0

    assert 0.0 < rumble(70.0) < rumble(74.0) < rumble(78.0)


def test_runway_rumble_fades_as_the_wings_take_the_weight(sortie):
    def strength(seconds):
        result = at(sortie, seconds)
        return next((p.magnitude for p in result.force.periodics if p.label == "ground_roll"), 0.0)

    assert strength(82.0) < strength(70.0)


def test_the_touchdown_is_felt(sortie):
    window = [
        r
        for t, r in sortie
        if SCRIPT_EVENTS["touchdown"] <= t.t <= SCRIPT_EVENTS["touchdown"] + 1.0
    ]
    burst = max(
        (p.magnitude for r in window for p in r.force.periodics if p.label == "touchdown"),
        default=0.0,
    )
    assert burst > 0.1


def test_braking_is_felt_during_the_rollout(sortie):
    result = at(sortie, SCRIPT_EVENTS["rollout_braking"])
    assert any(p.label == "brake_judder" for p in result.force.periodics)


def test_control_loading_builds_with_airspeed(sortie):
    climb = at(sortie, 100.0).force.spring
    cruise = at(sortie, 180.0).force.spring
    assert climb is not None and cruise is not None
    assert cruise.coefficient > climb.coefficient


def test_steering_feel_replaces_control_loading_on_the_ground(sortie):
    taxi = at(sortie, 30.0)
    assert taxi.force.breakdown.get("control_loading", 0.0) == 0.0
    assert taxi.force.spring is not None  # steering feel is providing it


def test_forces_fade_when_telemetry_stops():
    engine = BridgeEngine()
    flight = SyntheticFlight()
    wheel = WheelState(connected=True)
    for telemetry in flight.stream(start=0.0, end=40.0):
        engine.tick(telemetry, wheel, telemetry.t)

    frozen = flight.sample_at(40.0)
    now = 40.0
    for _ in range(200):
        now += 0.01
        result = engine.tick(frozen, wheel, now)
    assert result.stale is True
    assert result.force.constant == pytest.approx(0.0, abs=1e-6)


def test_a_disconnected_sim_produces_no_force():
    engine = BridgeEngine()
    wheel = WheelState(connected=True)
    for step in range(200):
        result = engine.tick(FlightTelemetry(connected=False), wheel, step * 0.01)
    assert result.force.constant == pytest.approx(0.0, abs=1e-6)


def test_a_stalled_loop_cannot_produce_one_enormous_step():
    engine = BridgeEngine()
    flight = SyntheticFlight()
    telemetry = flight.sample_at(70.0)
    engine.tick(telemetry, WheelState(connected=True), 0.0)
    result = engine.tick(flight.sample_at(70.1), WheelState(connected=True), 30.0)
    assert result.dt <= BridgeEngine.MAX_DT


def test_changing_aircraft_switches_profile_and_resets():
    profiles = ProfileSet(
        default=BridgeConfig(name="Default GA"),
        profiles=[BridgeConfig(name="Airliner", match=["*a320*"])],
    )
    engine = BridgeEngine(profiles)
    wheel = WheelState(connected=True)

    engine.tick(FlightTelemetry(connected=True, title="Cessna C172"), wheel, 0.0)
    assert engine.config.name == "Default GA"

    result = engine.tick(FlightTelemetry(connected=True, title="Airbus A320neo"), wheel, 0.1)
    assert result.profile_name == "Airliner"
    assert engine.status.aircraft == "Airbus A320neo"


def test_profile_gains_reach_the_modules():
    quiet = ProfileSet(
        default=BridgeConfig(
            name="Quiet",
            modules={"ground_roll": ModuleSettings(enabled=False)},
        )
    )
    results = fly(BridgeEngine(quiet))
    assert not any(
        p.label == "ground_roll" for _, result in results for p in result.force.periodics
    )


def test_live_gain_changes_apply_without_a_rebuild():
    engine = BridgeEngine()
    wheel = WheelState(connected=True)
    flight = SyntheticFlight()
    telemetry = flight.sample_at(74.0)  # mid takeoff roll
    for step in range(60):
        engine.tick(telemetry, wheel, step / 60.0)

    engine.config.module("ground_roll").gain = 0.0
    result = engine.tick(telemetry, wheel, 2.0)
    assert not any(p.label == "ground_roll" for p in result.force.periodics)


def test_device_slot_count_is_honoured():
    engine = BridgeEngine(periodic_slots=1)
    results = fly(engine)
    assert all(len(result.force.periodics) <= 1 for _, result in results)


def test_reset_returns_the_engine_to_a_cold_state():
    engine = BridgeEngine()
    fly(engine)
    engine.reset(on_ground=True)
    assert engine.router is not None
    assert engine.router.mode() is AxisMode.GROUND


def test_a_second_flight_behaves_like_the_first():
    """State from one flight must not leak into the next."""
    engine = BridgeEngine()
    first = fly(engine)
    engine.reset(on_ground=True)
    second = fly(engine)
    assert len(first) == len(second)
    for (_, a), (_, b) in zip(first[-600:], second[-600:], strict=True):
        assert a.force.constant == pytest.approx(b.force.constant, abs=0.05)
