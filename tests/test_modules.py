"""Tests for individual effect modules.

Each module is driven through the conditions it is supposed to respond to, and
just as importantly through conditions where it should stay silent.
"""

from __future__ import annotations

import pytest
from conftest import flying, periodic_named, run_module, taxiing

from ffbbridge.core.context import AxisMode, TickContext
from ffbbridge.core.forces import Waveform
from ffbbridge.core.modules import (
    BrakeShudder,
    Buffet,
    ControlLoading,
    Crosswind,
    EngineVibration,
    GearTransit,
    GroundRoll,
    HandoffAssist,
    LockedWheel,
    NosewheelShimmy,
    PropWash,
    SoftLock,
    SteeringFeel,
    Touchdown,
    Turbulence,
)
from ffbbridge.core.modules.engine_vibration import fold_to_band
from ffbbridge.core.telemetry import FlightTelemetry, SurfaceType, WheelState

GROUND = TickContext(ground_weight=1.0)
AIR = TickContext(mode=AxisMode.AIR, ground_weight=0.0)
DT = 1 / 60


# --- Ground roll ---------------------------------------------------------


def test_ground_roll_is_silent_in_the_air():
    contribution = run_module(GroundRoll(), flying(), ctx=AIR)
    assert contribution.is_empty


def test_ground_roll_is_silent_when_stopped():
    contribution = run_module(GroundRoll(), taxiing(gs_kt=0.0), ctx=GROUND)
    assert periodic_named(contribution, "ground_roll") is None


def test_ground_roll_frequency_rises_with_speed():
    frequencies = []
    for speed in (10.0, 30.0, 60.0, 90.0):
        contribution = run_module(GroundRoll(), taxiing(gs_kt=speed, ias_kt=speed), ctx=GROUND)
        frequencies.append(periodic_named(contribution, "ground_roll").frequency_hz)
    assert frequencies == sorted(frequencies)
    assert frequencies[0] < frequencies[-1]


def test_rough_surfaces_are_stronger_than_smooth_ones():
    def magnitude(surface):
        contribution = run_module(
            GroundRoll(), taxiing(gs_kt=30.0, ias_kt=30.0, surface_type=surface), ctx=GROUND
        )
        return periodic_named(contribution, "ground_roll").magnitude

    assert magnitude(SurfaceType.ICE) < magnitude(SurfaceType.CONCRETE)
    assert magnitude(SurfaceType.CONCRETE) < magnitude(SurfaceType.GRASS)
    assert magnitude(SurfaceType.GRASS) < magnitude(SurfaceType.GRASS_BUMPY)


def test_loose_surfaces_chatter_faster_than_slabs_pass():
    """Grass has no slab structure, so it is felt as a finer, faster texture."""

    def frequency(surface):
        contribution = run_module(
            GroundRoll(), taxiing(gs_kt=30.0, ias_kt=30.0, surface_type=surface), ctx=GROUND
        )
        return periodic_named(contribution, "ground_roll").frequency_hz

    assert frequency(SurfaceType.GRASS) > frequency(SurfaceType.CONCRETE) * 2


def test_jointed_surfaces_use_a_sharper_waveform():
    concrete = run_module(
        GroundRoll(), taxiing(gs_kt=30.0, surface_type=SurfaceType.CONCRETE), ctx=GROUND
    )
    grass = run_module(
        GroundRoll(), taxiing(gs_kt=30.0, surface_type=SurfaceType.GRASS), ctx=GROUND
    )
    assert periodic_named(concrete, "ground_roll").waveform is Waveform.TRIANGLE
    assert periodic_named(grass, "ground_roll").waveform is Waveform.SINE


def test_ground_roll_fades_as_the_wings_take_the_weight():
    loaded = run_module(
        GroundRoll(),
        taxiing(gs_kt=55.0, ias_kt=55.0, contact_compression=(0.4, 0.6, 0.6)),
        ctx=GROUND,
    )
    light = run_module(
        GroundRoll(),
        taxiing(gs_kt=55.0, ias_kt=55.0, contact_compression=(0.03, 0.05, 0.05)),
        ctx=GROUND,
    )
    assert (
        periodic_named(light, "ground_roll").magnitude
        < periodic_named(loaded, "ground_roll").magnitude
    )


# --- Touchdown -----------------------------------------------------------


def land(module, *, descent_fpm, approach_ticks=120):
    """Fly an approach at a given descent rate, then touch down."""
    approach = FlightTelemetry(
        connected=True,
        on_ground=False,
        contact_compression=(0.0, 0.0, 0.0),
        vs_fpm=-descent_fpm,
        agl_ft=50.0,
        ias_kt=65.0,
    )
    for _ in range(approach_ticks):
        module.update(approach, WheelState(), GROUND, DT)
    contact = FlightTelemetry(
        connected=True, on_ground=True, contact_compression=(0.1, 0.3, 0.3), ias_kt=60.0
    )
    peak = 0.0
    for _ in range(60):
        contribution = module.update(contact, WheelState(), GROUND, DT)
        burst = periodic_named(contribution, "touchdown")
        if burst:
            peak = max(peak, burst.magnitude)
    return peak


def test_touchdown_scales_with_descent_rate():
    greaser = land(Touchdown(), descent_fpm=60.0)
    normal = land(Touchdown(), descent_fpm=220.0)
    firm = land(Touchdown(), descent_fpm=500.0)
    assert 0.0 < greaser < normal < firm
    assert firm > 0.4


def test_touchdown_does_not_fire_below_the_threshold():
    assert land(Touchdown(), descent_fpm=5.0) == 0.0


def test_touchdown_does_not_fire_when_starting_on_the_runway():
    """A flight that begins parked must not open with a landing."""
    module = Touchdown()
    parked = taxiing(gs_kt=0.0)
    for _ in range(120):
        contribution = module.update(parked, WheelState(), GROUND, DT)
        assert periodic_named(contribution, "touchdown") is None


def test_touchdown_fires_once_per_arrival():
    module = Touchdown()
    approach = FlightTelemetry(
        connected=True,
        on_ground=False,
        contact_compression=(0.0, 0.0, 0.0),
        vs_fpm=-300.0,
        agl_ft=40.0,
    )
    for _ in range(120):
        module.update(approach, WheelState(), GROUND, DT)
    contact = FlightTelemetry(connected=True, on_ground=True, contact_compression=(0.1, 0.3, 0.3))

    firings, was_quiet = 0, True
    for _ in range(300):
        contribution = module.update(contact, WheelState(), GROUND, DT)
        burst = periodic_named(contribution, "touchdown")
        if burst and was_quiet:
            firings += 1
        was_quiet = burst is None
    assert firings == 1


def test_touchdown_is_felt_as_a_jolt_even_when_dead_straight():
    module = Touchdown()
    approach = FlightTelemetry(
        connected=True,
        on_ground=False,
        contact_compression=(0.0, 0.0, 0.0),
        vs_fpm=-350.0,
        agl_ft=40.0,
    )
    for _ in range(120):
        module.update(approach, WheelState(), GROUND, DT)
    contact = FlightTelemetry(connected=True, on_ground=True, contact_compression=(0.1, 0.3, 0.3))
    peak = max(abs(module.update(contact, WheelState(), GROUND, DT).constant) for _ in range(40))
    assert peak > 0.05


# --- Brakes --------------------------------------------------------------


def test_brakes_need_both_pressure_and_speed():
    rolling = taxiing(gs_kt=40.0, brake_left=0.6, brake_right=0.6)
    assert periodic_named(run_module(BrakeShudder(), rolling, ctx=GROUND), "brake_judder")

    stopped = taxiing(gs_kt=0.2, brake_left=0.6, brake_right=0.6)
    assert periodic_named(run_module(BrakeShudder(), stopped, ctx=GROUND), "brake_judder") is None

    coasting = taxiing(gs_kt=40.0)
    assert periodic_named(run_module(BrakeShudder(), coasting, ctx=GROUND), "brake_judder") is None


def test_hard_braking_adds_a_stutter():
    light = taxiing(gs_kt=40.0, brake_left=0.4, brake_right=0.4)
    hard = taxiing(gs_kt=40.0, brake_left=1.0, brake_right=1.0)
    assert periodic_named(run_module(BrakeShudder(), light, ctx=GROUND), "brake_skid") is None
    assert periodic_named(run_module(BrakeShudder(), hard, ctx=GROUND), "brake_skid")


def test_asymmetric_braking_pulls_toward_the_braked_side():
    right = run_module(
        BrakeShudder(), taxiing(gs_kt=40.0, brake_left=0.0, brake_right=0.8), ctx=GROUND
    )
    left = run_module(
        BrakeShudder(), taxiing(gs_kt=40.0, brake_left=0.8, brake_right=0.0), ctx=GROUND
    )
    assert right.constant > 0 > left.constant


def test_brakes_are_silent_in_the_air():
    assert run_module(BrakeShudder(), flying(brake_left=1.0), ctx=AIR).is_empty


def test_the_braking_pull_goes_with_the_rudder_but_the_judder_stays():
    """Uneven braking is a force in the steering; the judder is not.

    With the wheel pinned to ailerons the pedals are doing the steering, so the
    pull belongs to them - but the rollout should still feel like a rollout.
    """
    uneven = taxiing(gs_kt=40.0, brake_left=0.0, brake_right=0.8)
    no_rudder = TickContext(mode=AxisMode.AIR, ground_weight=0.0)

    on_the_wheel = run_module(BrakeShudder(), uneven, ctx=GROUND)
    on_the_pedals = run_module(BrakeShudder(), uneven, ctx=no_rudder)

    assert on_the_wheel.constant > 0.0
    assert on_the_pedals.constant == 0.0
    assert periodic_named(on_the_pedals, "brake_judder")


# --- Locked wheel --------------------------------------------------------


def rolling_then(module, *, learn_kt=40.0, ticks=240, **overrides):
    """Teach the module what a rolling wheel looks like, then change something.

    Nothing reports tyre radius, so the module learns RPM per knot with the
    brakes off before it can call anything a skid.
    """
    free = taxiing(gs_kt=learn_kt, wheel_rpm_left=learn_kt * 8.0, wheel_rpm_right=learn_kt * 8.0)
    for _ in range(ticks):
        module.update(free, WheelState(), GROUND, DT)
    return run_module(module, taxiing(**overrides), ctx=GROUND)


def test_a_rolling_wheel_is_not_a_skid():
    module = LockedWheel()
    contribution = rolling_then(
        module, gs_kt=40.0, wheel_rpm_left=320.0, wheel_rpm_right=320.0, brake_left=0.9
    )
    assert periodic_named(contribution, "skid_skip") is None


def test_a_locked_wheel_skips():
    module = LockedWheel()
    contribution = rolling_then(
        module, gs_kt=40.0, wheel_rpm_left=0.0, wheel_rpm_right=0.0, brake_left=1.0
    )
    skip = periodic_named(contribution, "skid_skip")
    assert skip is not None
    assert skip.waveform is Waveform.SQUARE


def test_one_locked_main_pulls_toward_the_locked_side():
    left = rolling_then(
        LockedWheel(), gs_kt=40.0, wheel_rpm_left=0.0, wheel_rpm_right=320.0, brake_left=1.0
    )
    right = rolling_then(
        LockedWheel(), gs_kt=40.0, wheel_rpm_left=320.0, wheel_rpm_right=0.0, brake_right=1.0
    )
    assert right.constant > 0 > left.constant


def test_both_wheels_locked_pull_nowhere_in_particular():
    contribution = rolling_then(
        LockedWheel(), gs_kt=40.0, wheel_rpm_left=0.0, wheel_rpm_right=0.0, brake_left=1.0
    )
    assert contribution.constant == pytest.approx(0.0)


def test_an_aircraft_that_reports_no_wheel_rpm_stays_silent():
    """The failure that matters: not every aeroplane fills these in.

    Rolling at forty knots reporting zero RPM teaches a ratio of zero, which
    expects nothing and so can never find a wheel short of it. Silence is the
    right answer here, not a permanent skid.
    """
    module = LockedWheel()
    never_reports = taxiing(gs_kt=40.0, wheel_rpm_left=0.0, wheel_rpm_right=0.0)
    for _ in range(240):
        module.update(never_reports, WheelState(), GROUND, DT)

    braking = taxiing(gs_kt=40.0, wheel_rpm_left=0.0, wheel_rpm_right=0.0, brake_left=1.0)
    assert run_module(module, braking, ctx=GROUND).is_empty


def test_the_skid_pull_goes_with_the_rudder():
    module = LockedWheel()
    free = taxiing(gs_kt=40.0, wheel_rpm_left=320.0, wheel_rpm_right=320.0)
    for _ in range(240):
        module.update(free, WheelState(), GROUND, DT)
    skidding = taxiing(gs_kt=40.0, wheel_rpm_left=0.0, wheel_rpm_right=320.0, brake_left=1.0)
    on_the_pedals = run_module(
        module, skidding, ctx=TickContext(mode=AxisMode.AIR, ground_weight=0.0)
    )
    assert on_the_pedals.constant == 0.0
    assert periodic_named(on_the_pedals, "skid_skip")


def test_the_skid_is_silent_in_the_air():
    assert run_module(LockedWheel(), flying(brake_left=1.0), ctx=AIR).is_empty


# --- Shimmy --------------------------------------------------------------


def test_shimmy_only_appears_in_its_speed_band():
    def magnitude(speed):
        contribution = run_module(
            NosewheelShimmy(), taxiing(gs_kt=speed), ctx=GROUND, wheel=WheelState(position=0.4)
        )
        periodic = periodic_named(contribution, "shimmy")
        return periodic.magnitude if periodic else 0.0

    assert magnitude(2.0) == 0.0
    assert magnitude(28.0) > 0.0
    assert magnitude(28.0) > magnitude(45.0)
    assert magnitude(90.0) == 0.0


def test_shimmy_needs_a_steering_input():
    straight = run_module(
        NosewheelShimmy(), taxiing(gs_kt=28.0), ctx=GROUND, wheel=WheelState(position=0.0)
    )
    turning = run_module(
        NosewheelShimmy(), taxiing(gs_kt=28.0), ctx=GROUND, wheel=WheelState(position=0.4)
    )
    assert periodic_named(straight, "shimmy") is None
    assert periodic_named(turning, "shimmy")


def test_shimmy_needs_weight_on_the_nose():
    airborne_nose = taxiing(gs_kt=28.0, contact_compression=(0.0, 0.5, 0.5))
    contribution = run_module(
        NosewheelShimmy(), airborne_nose, ctx=GROUND, wheel=WheelState(position=0.4)
    )
    assert periodic_named(contribution, "shimmy") is None


# --- Steering feel -------------------------------------------------------


def test_steering_is_heaviest_when_stopped_and_lightens_as_you_roll():
    stopped = run_module(SteeringFeel(), taxiing(gs_kt=0.0), ctx=GROUND)
    rolling = run_module(SteeringFeel(), taxiing(gs_kt=8.0), ctx=GROUND)
    assert stopped.spring.coefficient > rolling.spring.coefficient


def test_caster_centring_builds_with_speed():
    slow = run_module(SteeringFeel(), taxiing(gs_kt=15.0), ctx=GROUND)
    fast = run_module(SteeringFeel(), taxiing(gs_kt=50.0), ctx=GROUND)
    assert fast.spring.coefficient > slow.spring.coefficient


def test_rudder_trim_moves_the_neutral_point():
    contribution = run_module(SteeringFeel(), taxiing(gs_kt=30.0, rudder_trim_pct=0.5), ctx=GROUND)
    assert contribution.spring.center > 0.1


def test_steering_feel_fades_with_the_axis():
    assert run_module(SteeringFeel(), taxiing(gs_kt=30.0), ctx=AIR).is_empty


# --- Gear ----------------------------------------------------------------


def test_gear_rumbles_in_transit_and_clunks_at_the_lock():
    module = GearTransit()
    transiting = False
    for step in range(60):
        telemetry = flying(gear_pct=step / 60.0)
        contribution = module.update(telemetry, WheelState(), AIR, DT)
        if periodic_named(contribution, "gear_transit"):
            transiting = True
    assert transiting

    locked = flying(gear_pct=1.0)
    clunked = False
    for _ in range(30):
        contribution = module.update(locked, WheelState(), AIR, DT)
        if periodic_named(contribution, "gear_clunk"):
            clunked = True
    assert clunked


def test_gear_is_quiet_when_nothing_moves():
    module = GearTransit()
    assert run_module(module, flying(gear_pct=1.0), ctx=AIR, ticks=120).is_empty


# --- Engine --------------------------------------------------------------


@pytest.mark.parametrize(
    ("raw", "expected"),
    [(80.0, 40.0), (160.0, 40.0), (30.0, 30.0), (6.0, 24.0), (0.0, 0.0)],
)
def test_fold_to_band_lands_in_the_felt_range(raw, expected):
    assert fold_to_band(raw, 13.0, 42.0) == pytest.approx(expected)


def test_fold_to_band_rejects_a_nonsensical_band():
    assert fold_to_band(50.0, 40.0, 20.0) == 0.0


def test_engine_is_silent_when_stopped():
    assert run_module(
        EngineVibration(), taxiing(prop_rpm=(0.0,), eng_rpm=(0.0,)), ctx=GROUND
    ).is_empty


def test_engine_tone_tracks_rpm():
    idle = run_module(EngineVibration(), taxiing(prop_rpm=(700.0,), eng_rpm=(700.0,)), ctx=GROUND)
    cruise = run_module(EngineVibration(), flying(prop_rpm=(2400.0,), eng_rpm=(2400.0,)), ctx=AIR)
    assert (
        periodic_named(idle, "engine").frequency_hz != periodic_named(cruise, "engine").frequency_hz
    )


def test_engine_shakes_at_idle():
    idle = run_module(EngineVibration(), taxiing(prop_rpm=(600.0,), eng_rpm=(600.0,)), ctx=GROUND)
    cruise = run_module(EngineVibration(), flying(prop_rpm=(2400.0,), eng_rpm=(2400.0,)), ctx=AIR)
    assert periodic_named(idle, "engine_shake")
    assert periodic_named(cruise, "engine_shake") is None


def test_a_dead_engine_runs_rough():
    healthy = run_module(
        EngineVibration(), flying(prop_rpm=(2400.0,), eng_combustion=(True,)), ctx=AIR
    )
    windmilling = run_module(
        EngineVibration(), flying(prop_rpm=(2400.0,), eng_combustion=(False,)), ctx=AIR
    )
    assert (
        periodic_named(windmilling, "engine").magnitude
        > periodic_named(healthy, "engine").magnitude
    )


# --- Turbulence ----------------------------------------------------------


def test_smooth_air_produces_nothing():
    """Steady 1 G flight must high-pass away to silence."""
    module = Turbulence()
    contribution = run_module(module, flying(wind_velocity_kt=0.0), ctx=AIR, ticks=600)
    assert abs(contribution.constant) < 0.02


def test_a_gust_is_felt():
    module = Turbulence()
    calm = flying(wind_velocity_kt=0.0)
    for _ in range(600):
        module.update(calm, WheelState(), AIR, DT)
    gust = flying(wind_velocity_kt=0.0, accel_body=(12.0, 32.174, 0.0))
    assert abs(module.update(gust, WheelState(), AIR, DT).constant) > 0.05


# --- Slipstream and crosswind -------------------------------------------


def test_power_at_low_speed_swings_the_nose():
    contribution = run_module(
        PropWash(),
        taxiing(gs_kt=20.0, ias_kt=20.0, throttle_pct=(1.0,), prop_rpm=(2600.0,)),
        ctx=GROUND,
    )
    assert contribution.constant < -0.05


def test_the_swing_fades_with_airspeed():
    slow = run_module(
        PropWash(), taxiing(ias_kt=20.0, throttle_pct=(1.0,), prop_rpm=(2600.0,)), ctx=GROUND
    )
    fast = run_module(
        PropWash(), flying(ias_kt=120.0, throttle_pct=(1.0,), prop_rpm=(2600.0,)), ctx=AIR
    )
    assert abs(fast.constant) < abs(slow.constant)


def test_crosswind_weathervanes_toward_the_wind():
    from math import radians

    from_right = run_module(
        Crosswind(),
        taxiing(
            gs_kt=20.0,
            wind_velocity_kt=20.0,
            wind_direction_rad=radians(90.0),
            heading_true_rad=0.0,
        ),
        ctx=GROUND,
    )
    from_left = run_module(
        Crosswind(),
        taxiing(
            gs_kt=20.0,
            wind_velocity_kt=20.0,
            wind_direction_rad=radians(270.0),
            heading_true_rad=0.0,
        ),
        ctx=GROUND,
    )
    assert from_right.constant > 0 > from_left.constant


def test_sideslip_rolls_the_aircraft_in_flight():
    contribution = run_module(Crosswind(), flying(beta_rad=0.15), ctx=AIR)
    assert abs(contribution.constant) > 0.01


# --- Control loading -----------------------------------------------------


def test_control_force_grows_with_the_square_of_airspeed():
    def stiffness(speed):
        return run_module(ControlLoading(), flying(ias_kt=speed), ctx=AIR).spring.coefficient

    slow, medium, fast = stiffness(40.0), stiffness(80.0), stiffness(120.0)
    assert slow < medium < fast
    # Doubling the speed should more than double the increment, not less.
    assert (fast - medium) > (medium - slow)


def test_aileron_trim_moves_the_neutral_point():
    contribution = run_module(ControlLoading(), flying(aileron_trim_pct=0.5), ctx=AIR)
    assert contribution.spring.center > 0.1


def test_controls_go_light_at_the_stall():
    normal = run_module(ControlLoading(), flying(ias_kt=60.0), ctx=AIR)
    stalling = run_module(ControlLoading(), flying(ias_kt=60.0, stall_warning=True), ctx=AIR)
    assert stalling.spring.coefficient < normal.spring.coefficient


def test_control_loading_is_absent_on_the_ground_axis():
    assert run_module(ControlLoading(), flying(), ctx=GROUND).is_empty


# --- Buffet and handoff --------------------------------------------------


def test_buffet_ships_switched_off():
    assert Buffet.default_enabled is False


def test_buffet_shakes_near_the_stall_when_enabled():
    module = Buffet()
    contribution = run_module(
        module, flying(ias_kt=45.0, alpha_rad=0.27, stall_warning=True), ctx=AIR
    )
    assert periodic_named(contribution, "stall_buffet")


def test_buffet_is_quiet_on_the_ground():
    assert run_module(Buffet(), taxiing(alpha_rad=0.3), ctx=GROUND).is_empty


def test_handoff_assist_only_acts_during_a_transition():
    assert run_module(HandoffAssist(), flying(), ctx=AIR).is_empty
    transitioning = TickContext(mode=AxisMode.TO_AIR, ground_weight=0.5, transition_progress=0.2)
    contribution = run_module(HandoffAssist(), flying(), ctx=transitioning)
    assert contribution.spring is not None
    assert contribution.spring.center == 0.0


def test_handoff_assist_releases_as_the_transition_completes():
    early = run_module(
        HandoffAssist(),
        flying(),
        ctx=TickContext(mode=AxisMode.TO_AIR, ground_weight=0.9, transition_progress=0.1),
    )
    late = run_module(
        HandoffAssist(),
        flying(),
        ctx=TickContext(mode=AxisMode.TO_AIR, ground_weight=0.1, transition_progress=0.9),
    )
    assert late.spring.coefficient < early.spring.coefficient


# --- Soft lock -----------------------------------------------------------

SOFT_LOCK_180 = TickContext(ground_weight=1.0, wheel_rotation_deg=540.0, soft_lock_deg=180.0)


def test_soft_lock_is_silent_inside_the_limit():
    # 180 of 540 degrees is a third of the travel each way.
    wheel = WheelState(position=0.32)
    assert run_module(SoftLock(), taxiing(), ctx=SOFT_LOCK_180, wheel=wheel).is_empty


def test_soft_lock_pushes_back_past_the_limit():
    right = run_module(SoftLock(), taxiing(), ctx=SOFT_LOCK_180, wheel=WheelState(position=0.5))
    left = run_module(SoftLock(), taxiing(), ctx=SOFT_LOCK_180, wheel=WheelState(position=-0.5))
    assert right.constant < 0.0
    assert left.constant == pytest.approx(-right.constant)


def test_soft_lock_builds_over_the_ramp_rather_than_stepping():
    forces = []
    for position in (0.34, 0.35, 0.36, 0.40):
        contribution = run_module(
            SoftLock(), taxiing(), ctx=SOFT_LOCK_180, wheel=WheelState(position=position)
        )
        forces.append(-contribution.constant)
    assert forces == sorted(forces)
    assert forces[0] < forces[-1]


def test_soft_lock_damps_only_once_past_the_stop():
    inside = run_module(SoftLock(), taxiing(), ctx=SOFT_LOCK_180, wheel=WheelState(position=0.2))
    outside = run_module(SoftLock(), taxiing(), ctx=SOFT_LOCK_180, wheel=WheelState(position=0.6))
    assert inside.damper is None
    assert outside.damper.coefficient > 0.0


def test_soft_lock_does_nothing_when_not_configured():
    wheel = WheelState(position=0.95)
    assert run_module(SoftLock(), taxiing(), ctx=GROUND, wheel=wheel).is_empty
