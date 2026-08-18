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
    up over a few degrees instead of arriving as a step, then keeps building the
    further you push. A little damping goes with it, because a hard stop with no
    damping is a bounce.

    The master strength does not scale it, unlike every other steady force. A
    stop softened to a third of itself is not a soft stop, it is a shove you
    push straight through, and someone who turns the whole force model down to
    something comfortable has not asked for their control travel to grow. The
    safety ceiling still applies, which is what makes it soft: lean hard enough
    and it yields rather than fighting you.
    """

    id = "soft_lock"
    name = "Soft lock"
    description = "A progressive end stop at the wheel travel the profile allows."
    priority = 90
    ignores_master_gain = True
    params = (
        ParamSpec("strength", 0.9, 0.0, 1.0, "Force at the stop", ""),
        ParamSpec("ramp_deg", 8.0, 1.0, 90.0, "Travel the stop builds over", "deg"),
        ParamSpec("damping", 0.45, 0.0, 1.0, "Resistance past the stop", ""),
    )

    #: How far past the ramp the wall goes on stiffening, in ramps.
    LEAN_RAMPS = 3.0

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

        # Past the ramp the wall keeps stiffening rather than sitting at one
        # value. A force that stops growing is one you learn to push through;
        # a stop should feel firmer the harder you lean on it.
        strength = self.p("strength")
        lean = clamp(overshoot / (ramp * self.LEAN_RAMPS), 0.0, 1.0)
        level = strength + (1.0 - strength) * lean

        contribution.constant = -math.copysign(level * depth, wheel.position)

        damping = self.p("damping") * depth
        if damping > 1e-4:
            contribution.damper = Damper(coefficient=clamp(damping, 0.0, 1.0), saturation=0.6)

        return contribution
