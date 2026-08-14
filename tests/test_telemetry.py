"""Tests for the telemetry snapshot and its derived quantities."""

from __future__ import annotations

import math

import pytest

from ffbbridge.core.telemetry import G_FT_S2, EngineType, FlightTelemetry, SurfaceType


def test_defaults_are_safe_for_an_aircraft_that_reports_nothing():
    """A SimVar the aircraft does not implement reads as zero, not as a crash."""
    telemetry = FlightTelemetry()
    assert telemetry.max_prop_rpm == 0.0
    assert telemetry.max_throttle == 0.0
    assert telemetry.nose_compression == 0.0
    assert telemetry.main_compression == 0.0
    assert telemetry.any_engine_running is False


def test_surface_type_falls_back_for_unknown_values():
    assert SurfaceType.from_raw(1) is SurfaceType.GRASS
    assert SurfaceType.from_raw(999) is SurfaceType.CONCRETE
    assert EngineType.from_raw(5) is EngineType.TURBOPROP
    assert EngineType.from_raw(-3) is EngineType.PISTON


def test_weight_on_wheels_prefers_compression_over_the_flag():
    airborne = FlightTelemetry(on_ground=True, contact_compression=(0.0, 0.0, 0.0))
    assert airborne.weight_on_wheels is False
    loaded = FlightTelemetry(on_ground=False, contact_compression=(0.0, 0.4, 0.4))
    assert loaded.weight_on_wheels is True


def test_weight_on_wheels_falls_back_to_the_flag():
    """Aircraft that do not report contact points still have to work."""
    assert FlightTelemetry(on_ground=True).weight_on_wheels is True
    assert FlightTelemetry(on_ground=False).weight_on_wheels is False


def test_compression_helpers_pick_the_right_contact_points():
    telemetry = FlightTelemetry(contact_compression=(0.2, 0.5, 0.7))
    assert telemetry.nose_compression == 0.2
    assert telemetry.main_compression == 0.7


def test_brake_input_takes_the_greater_pedal():
    assert FlightTelemetry(brake_left=0.3, brake_right=0.8).brake_input == 0.8


def test_acceleration_is_reported_in_g():
    telemetry = FlightTelemetry(accel_body=(G_FT_S2, 2 * G_FT_S2, 0.0))
    assert telemetry.lateral_accel_g == pytest.approx(1.0)
    assert telemetry.vertical_accel_g == pytest.approx(2.0)


@pytest.mark.parametrize(
    ("wind_from_deg", "heading_deg", "expected_cross", "expected_head"),
    [
        (90.0, 0.0, 20.0, 0.0),  # wind from the right
        (270.0, 0.0, -20.0, 0.0),  # wind from the left
        (0.0, 0.0, 0.0, 20.0),  # straight down the runway
        (180.0, 0.0, 0.0, -20.0),  # tailwind
    ],
)
def test_wind_components_follow_the_aviation_convention(
    wind_from_deg, heading_deg, expected_cross, expected_head
):
    """A right crosswind is positive, and a headwind is positive.

    Regression: the crosswind property originally returned the direction the air
    pushes, which is the opposite sign to how the effect modules read it.
    """
    telemetry = FlightTelemetry(
        wind_velocity_kt=20.0,
        wind_direction_rad=math.radians(wind_from_deg),
        heading_true_rad=math.radians(heading_deg),
    )
    assert telemetry.crosswind_kt == pytest.approx(expected_cross, abs=1e-9)
    assert telemetry.headwind_kt == pytest.approx(expected_head, abs=1e-9)


def test_reference_speed_prefers_the_design_cruise_speed():
    assert FlightTelemetry(design_speed_vc_kt=150.0).reference_speed_kt() == 150.0


def test_reference_speed_falls_back_to_the_stall_speed():
    telemetry = FlightTelemetry(design_speed_vc_kt=0.0, design_speed_vs1_kt=50.0)
    assert telemetry.reference_speed_kt() == pytest.approx(125.0)


def test_reference_speed_has_a_last_resort():
    """An aircraft reporting no design speeds must still produce sane forces."""
    assert FlightTelemetry().reference_speed_kt() == 120.0


def test_q_ratio_follows_the_square_of_speed():
    telemetry = FlightTelemetry(design_speed_vc_kt=100.0, ias_kt=50.0)
    assert telemetry.q_ratio() == pytest.approx(0.25)
    doubled = FlightTelemetry(design_speed_vc_kt=100.0, ias_kt=100.0)
    assert doubled.q_ratio() == pytest.approx(1.0)


def test_max_prop_rpm_falls_back_to_engine_rpm_for_jets():
    jet = FlightTelemetry(prop_rpm=(0.0,), eng_rpm=(8000.0,))
    assert jet.max_prop_rpm == 8000.0


def test_telemetry_is_immutable():
    telemetry = FlightTelemetry()
    with pytest.raises((AttributeError, TypeError)):
        telemetry.ias_kt = 100.0  # type: ignore[misc]
