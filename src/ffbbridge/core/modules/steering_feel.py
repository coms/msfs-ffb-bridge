"""How the nosewheel loads up the wheel while you are on the ground."""

from __future__ import annotations

from ..context import TickContext
from ..filters import clamp, map_range, smoothstep
from ..forces import Contribution, Damper, Spring
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class SteeringFeel(EffectModule):
    """Centring and resistance for the ground steering axis.

    Ground steering is heavy in two different ways and they behave oppositely
    with speed. Standing still, the tyre has to scrub sideways against the
    surface, which is pure resistance and fades as you roll. Moving, the
    castering geometry actively pushes the nosewheel straight, and that grows
    with speed. Modelling both is what makes a taxi feel like an aeroplane
    rather than a spring.
    """

    id = "steering_feel"
    name = "Steering feel"
    description = "Nosewheel scrub, caster centring and rudder trim on the ground axis."
    priority = 20
    params = (
        ParamSpec("scrub", 0.30, 0.0, 1.0, "Stationary scrub resistance", ""),
        ParamSpec("caster", 0.45, 0.0, 1.0, "Self-centring at speed", ""),
        ParamSpec("caster_speed_kt", 45.0, 5.0, 140.0, "Speed for full caster force", "kt"),
        ParamSpec("damping", 0.18, 0.0, 1.0, "Resistance to wheel movement", ""),
        ParamSpec("saturation", 0.75, 0.1, 1.0, "Ceiling on the centring force", ""),
        ParamSpec("trim_authority", 0.5, 0.0, 1.0, "How far rudder trim moves neutral", ""),
        ParamSpec("airborne_hold", 0.12, 0.0, 1.0, "Residual centring once airborne", ""),
    )

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()
        weight = ctx.ground_weight
        if weight <= 0.01:
            return contribution

        speed = abs(tel.gs_kt)
        on_wheels = tel.weight_on_wheels

        # Scrub is strongest stopped and gone by taxi speed.
        scrub = self.p("scrub") * (1.0 - smoothstep(speed, 0.0, 12.0)) if on_wheels else 0.0
        # Caster builds with speed, and only exists with weight on the nose.
        caster = self.p("caster") * smoothstep(speed, 3.0, self.p("caster_speed_kt"))
        if on_wheels and tel.contact_compression:
            caster *= clamp(tel.nose_compression * 2.5, 0.0, 1.0)
        elif not on_wheels:
            # Airborne with the axis still in ground mode: keep a light centring
            # so the rudder does not float free, but nothing that fights you.
            scrub = 0.0
            caster = self.p("airborne_hold")

        stiffness = clamp((scrub + caster) * weight, 0.0, 1.0)
        if stiffness > 1e-4:
            contribution.spring = Spring(
                coefficient=stiffness,
                center=clamp(tel.rudder_trim_pct * self.p("trim_authority")),
                saturation=self.p("saturation"),
                deadband=0.0,
            )

        damping = self.p("damping") * weight
        # A little more damping when stopped, matching the tyre's stiction.
        damping *= map_range(speed, 0.0, 20.0, 1.4, 1.0)
        if damping > 1e-4:
            contribution.damper = Damper(coefficient=clamp(damping, 0.0, 1.0), saturation=0.5)

        return contribution
