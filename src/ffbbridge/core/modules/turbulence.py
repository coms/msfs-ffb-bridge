"""Gusts, chop and the general business of air that will not sit still."""

from __future__ import annotations

from ..context import TickContext
from ..filters import BandNoise, HighPass, clamp, map_range
from ..forces import Contribution
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class Turbulence(EffectModule):
    """Jolts derived from what the airframe is actually doing.

    Rather than inventing shake, this high-passes the body accelerations and
    roll rate the sim reports. Steady flight has a constant 1 G on the vertical
    axis and no roll rate, so those drop out and only the disturbances survive.
    That means the effect is automatically correct for wake turbulence, thermals,
    gusts on approach and a hamfisted control input, without special cases.
    """

    id = "turbulence"
    name = "Turbulence"
    description = "Gust jolts high-passed from body accelerations, plus wind-scaled chop."
    priority = 30
    params = (
        ParamSpec("lateral_gain", 0.55, 0.0, 2.0, "Response to sideways jolts", ""),
        ParamSpec("vertical_gain", 0.30, 0.0, 2.0, "Response to vertical jolts", ""),
        ParamSpec("roll_gain", 0.40, 0.0, 2.0, "Response to being rolled by the air", ""),
        ParamSpec("chop", 0.22, 0.0, 1.0, "Background chop in strong wind", ""),
        ParamSpec("wind_ref_kt", 28.0, 5.0, 80.0, "Wind speed for full chop", "kt"),
        ParamSpec("tau", 0.7, 0.1, 3.0, "How slow a change still counts as a gust", "s"),
    )

    def __init__(self, settings=None) -> None:
        super().__init__(settings)
        tau = 0.7
        self._lateral = HighPass(tau)
        self._vertical = HighPass(tau)
        self._roll = HighPass(tau)
        self._noise = BandNoise(tau=0.12, seed=90210)

    def reset(self) -> None:
        tau = self.p("tau")
        self._lateral = HighPass(tau)
        self._vertical = HighPass(tau)
        self._roll = HighPass(tau)
        self._noise.reset()

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()

        lateral = self._lateral.update(tel.lateral_accel_g, dt)
        vertical = self._vertical.update(tel.vertical_accel_g, dt)
        roll = self._roll.update(tel.roll_rate, dt)
        noise = self._noise.update(dt)

        # On the ground the gear transmits bumps; ground_roll owns that story,
        # so fade this out rather than doubling up.
        airborne = 1.0 - ctx.ground_weight * 0.75
        if tel.weight_on_wheels:
            airborne *= 0.35

        jolt = (
            lateral * self.p("lateral_gain")
            + abs(vertical) * self.p("vertical_gain") * _sign(lateral)
            + roll * self.p("roll_gain")
        )

        chop = self.p("chop") * map_range(tel.wind_velocity_kt, 4.0, self.p("wind_ref_kt"), 0.0, 1.0)
        # Chop needs airflow to exist: parked in a gale the wheel stays quiet.
        chop *= map_range(tel.ias_kt, 20.0, 80.0, 0.0, 1.0)

        contribution.constant = clamp((jolt + noise * chop) * airborne)
        return contribution


def _sign(value: float) -> float:
    """Give vertical jolts a side to push toward, so they are not felt as nothing."""
    return 1.0 if value >= 0.0 else -1.0
