"""Shared fixtures and builders for the test suite."""

from __future__ import annotations

import pytest

from ffbbridge.core.context import TickContext
from ffbbridge.core.telemetry import FlightTelemetry, WheelState


@pytest.fixture
def ground_context() -> TickContext:
    return TickContext(ground_weight=1.0)


@pytest.fixture
def air_context() -> TickContext:
    from ffbbridge.core.context import AxisMode

    return TickContext(mode=AxisMode.AIR, ground_weight=0.0)


def taxiing(**overrides) -> FlightTelemetry:
    """A light aircraft rolling on a hard runway."""
    base = {
        "connected": True,
        "on_ground": True,
        "gs_kt": 20.0,
        "ias_kt": 18.0,
        "contact_compression": (0.45, 0.6, 0.6),
        "eng_rpm": (1000.0,),
        "prop_rpm": (1000.0,),
        "eng_combustion": (True,),
        "throttle_pct": (0.2,),
        "design_speed_vc_kt": 120.0,
    }
    base.update(overrides)
    return FlightTelemetry(**base)


def flying(**overrides) -> FlightTelemetry:
    """A light aircraft in the cruise."""
    base = {
        "connected": True,
        "on_ground": False,
        "agl_ft": 3000.0,
        "gs_kt": 115.0,
        "ias_kt": 112.0,
        "contact_compression": (0.0, 0.0, 0.0),
        "eng_rpm": (2350.0,),
        "prop_rpm": (2350.0,),
        "eng_combustion": (True,),
        "throttle_pct": (0.7,),
        "design_speed_vc_kt": 120.0,
    }
    base.update(overrides)
    return FlightTelemetry(**base)


def run_module(module, telemetry, *, ctx=None, wheel=None, ticks=30, dt=1 / 60):
    """Settle a module's filters and return its final contribution."""
    ctx = ctx if ctx is not None else TickContext()
    wheel = wheel if wheel is not None else WheelState()
    contribution = None
    for _ in range(ticks):
        contribution = module.update(telemetry, wheel, ctx, dt)
    return contribution


def periodic_named(contribution, label: str):
    """Find one periodic by label, or None."""
    for periodic in contribution.periodics:
        if periodic.label == label:
            return periodic
    return None
