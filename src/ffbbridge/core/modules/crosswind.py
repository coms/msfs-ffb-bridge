"""Crosswind: weathervaning on the ground, dihedral effect in the air."""

from __future__ import annotations

from ..context import TickContext
from ..filters import clamp, map_range
from ..forces import Contribution
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class Crosswind(EffectModule):
    """Steady pressure from the wind, on whichever axis the wheel currently is.

    On the ground the fin acts as a weathervane and the nose tries to swing into
    wind, so the wheel pulls that way and you hold off against it. Airborne, the
    sideslip that comes with a crosswind approach shows up as a roll tendency
    instead, which is the same control input arriving through a different route.
    """

    id = "crosswind"
    name = "Crosswind"
    description = "Weathervaning on the ground and sideslip-driven roll in the air."
    priority = 5
    params = (
        ParamSpec("ground_gain", 0.35, 0.0, 1.0, "Weathervane strength", ""),
        ParamSpec("air_gain", 0.20, 0.0, 1.0, "Sideslip roll strength", ""),
        ParamSpec("ref_wind_kt", 22.0, 5.0, 60.0, "Crosswind for full force", "kt"),
        ParamSpec("ref_slip_rad", 0.18, 0.02, 0.6, "Sideslip for full force", "rad"),
    )

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()

        if ctx.ground_weight > 0.01 and tel.weight_on_wheels:
            crosswind = tel.crosswind_kt
            strength = map_range(
                abs(crosswind), 2.0, self.p("ref_wind_kt"), 0.0, self.p("ground_gain")
            )
            # The fin needs some airflow over it before it can weathervane, and
            # the effect eases once the rudder becomes properly effective.
            authority = map_range(abs(tel.gs_kt), 0.0, 25.0, 0.35, 1.0) * map_range(
                abs(tel.gs_kt), 60.0, 110.0, 1.0, 0.5
            )
            force = strength * authority * (1.0 if crosswind >= 0.0 else -1.0)
            contribution.constant += clamp(force * ctx.ground_weight)

        if ctx.air_weight > 0.01:
            slip = tel.beta_rad
            roll = map_range(abs(slip), 0.01, self.p("ref_slip_rad"), 0.0, self.p("air_gain"))
            force = roll * (1.0 if slip >= 0.0 else -1.0)
            contribution.constant += clamp(force * ctx.air_weight)

        return contribution
