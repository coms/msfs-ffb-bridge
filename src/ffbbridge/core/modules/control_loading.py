"""Aerodynamic control loading: the wheel gets heavy as the air gets faster."""

from __future__ import annotations

from ..context import TickContext
from ..filters import clamp
from ..forces import Contribution, Damper, Spring
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class ControlLoading(EffectModule):
    """Airspeed-scaled centring for the aileron axis, including trim.

    Hinge moments go with dynamic pressure, so force rises with the square of
    speed: at half the reference speed the wheel is a quarter as heavy, and it
    goes genuinely slack near the stall. Trim moves the neutral point rather
    than adding force, which is what trim physically does.

    This is the baseline that makes the wheel flyable at all, so it stays
    deliberately simple; the character of the aircraft comes from the effects
    layered on top.
    """

    id = "control_loading"
    name = "Control loading"
    description = "Centring force that grows with the square of airspeed, offset by trim."
    priority = 20
    params = (
        ParamSpec("stiffness", 0.50, 0.0, 1.0, "Centring force at reference speed", ""),
        ParamSpec("floor", 0.06, 0.0, 0.5, "Centring force at a standstill", ""),
        ParamSpec("max_q", 2.0, 0.5, 6.0, "Dynamic pressure ratio where force tops out", ""),
        ParamSpec("saturation", 0.80, 0.1, 1.0, "Ceiling on the centring force", ""),
        ParamSpec("damping", 0.12, 0.0, 1.0, "Aerodynamic damping", ""),
        ParamSpec("trim_authority", 0.60, 0.0, 1.0, "How far full trim moves neutral", ""),
        ParamSpec("stall_relief", 0.55, 0.0, 1.0, "How slack the controls go at the stall", ""),
    )

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()
        weight = ctx.air_weight
        if weight <= 0.01:
            return contribution

        q = min(tel.q_ratio(), self.p("max_q"))
        stiffness = self.p("floor") + (self.p("stiffness") - self.p("floor")) * q
        stiffness = max(stiffness, self.p("floor"))

        # A stalled wing stops feeding the ailerons clean air and the controls
        # go noticeably light and vague just before the break.
        if tel.stall_warning:
            stiffness *= 1.0 - self.p("stall_relief")

        contribution.spring = Spring(
            coefficient=clamp(stiffness * weight, 0.0, 1.0),
            center=clamp(tel.aileron_trim_pct * self.p("trim_authority")),
            saturation=self.p("saturation"),
            deadband=0.0,
        )

        damping = self.p("damping") * (0.4 + 0.6 * min(q, 1.5)) * weight
        if damping > 1e-4:
            contribution.damper = Damper(coefficient=clamp(damping, 0.0, 1.0), saturation=0.6)

        return contribution
