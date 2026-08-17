"""The end stop a direct-drive wheelbase does not have."""

from __future__ import annotations

import math

from ..context import TickContext
from ..filters import clamp, smoothstep
from ..forces import Contribution, Damper
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class SoftLock(EffectModule):
    """A rising wall at the edge of the aircraft's control travel.

    Aeroplane controls stop somewhere; a wheelbase keeps turning until the belt
    or your wrists give up. The soft lock puts the stop back, in software, at the
    rotation the profile asks for.

    It is a wall rather than a spring: nothing happens inside the limit, so the
    centring the aircraft actually has is left alone, and past it the force ramps
    up over a few degrees instead of arriving as a step. A little damping goes
    with it, because a hard stop with no damping is a bounce.

    The wall is still only as strong as the safety limits allow, which is why it
    is soft: shove hard enough and it yields rather than fighting you.
    """

    id = "soft_lock"
    name = "Soft lock"
    description = "A progressive end stop at the wheel travel the profile allows."
    priority = 90
    params = (
        ParamSpec("strength", 0.9, 0.0, 1.0, "Force at the stop", ""),
        ParamSpec("ramp_deg", 8.0, 1.0, 90.0, "Travel the stop builds over", "deg"),
        ParamSpec("damping", 0.45, 0.0, 1.0, "Resistance past the stop", ""),
    )

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()
        limit = ctx.soft_lock_limit
        if limit <= 0.0 or limit >= 1.0:
            # No soft lock configured, or one no narrower than the wheel itself.
            return contribution

        overshoot = abs(wheel.position) - limit
        if overshoot <= 0.0:
            return contribution

        ramp = max(ctx.degrees_to_axis(self.p("ramp_deg")), 1e-4)
        depth = smoothstep(overshoot, 0.0, ramp)

        contribution.constant = -math.copysign(self.p("strength") * depth, wheel.position)

        damping = self.p("damping") * depth
        if damping > 1e-4:
            contribution.damper = Damper(coefficient=clamp(damping, 0.0, 1.0), saturation=0.6)

        return contribution
