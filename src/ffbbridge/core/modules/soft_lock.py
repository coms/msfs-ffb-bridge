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
    further you push. Damping goes with it and stiffens the same way, because a
    hard stop with no damping is a bounce, and a fast one needs more resistance
    than a slow one to avoid becoming one. The damping is worth leaning on: it is
    a condition the device itself computes from its own velocity sensor, so it
    reacts at the wheel's own rate rather than at however often the bridge gets
    to send an update, and is the thing actually doing the work of not letting
    a hard hit pick up return speed in the first place.

    The wall only pushes at full weight while it is still being leaned into. It
    tracks the deepest overshoot reached on each visit to the stop; once the
    wheel has backed off that peak by a real margin, the wall lets go almost
    entirely and leaves the damping to manage the way back. A stop stops the
    wheel -- it does not throw it back -- and this is a peak-and-release rather
    than a reaction to instantaneous speed, because speed at the rim jitters
    through the collision itself. Watching velocity made the wall flicker
    open and shut while still being pressed, and by the time it consistently
    agreed the wheel was leaving, the wheel had already been shoved most of the
    way back through centre.

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

    #: What is left of the wall once the wheel has backed off its peak lean.
    #: Not quite zero: a wall that lets go completely leaves nothing pulling
    #: the wheel back out of the illegal zone if it stalls out there.
    RELEASE_FACTOR = 0.05
    #: How far back off the peak, in ramps, counts as actually leaving rather
    #: than the ordinary give-and-take of a firm hold against the stop.
    RELEASE_HYSTERESIS_RAMPS = 0.3

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        #: Deepest signed overshoot reached since the wheel last entered the
        #: stop from this side. Reset on the way past the limit or a change
        #: of side, so each visit to the wall ratchets independently.
        self._peak_overshoot = 0.0

    def reset(self) -> None:
        self._peak_overshoot = 0.0

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
            self._peak_overshoot = 0.0
            return contribution

        outward_sign = math.copysign(1.0, wheel.position)
        signed_overshoot = overshoot * outward_sign
        if self._peak_overshoot != 0.0 and (self._peak_overshoot > 0.0) != (signed_overshoot > 0.0):
            # Passed clean through to the other side; that is a new visit.
            self._peak_overshoot = 0.0

        ramp = max(ctx.degrees_to_axis(self.p("ramp_deg")), 1e-4)
        depth = smoothstep(overshoot, 0.0, ramp)

        # Past the ramp the wall keeps stiffening rather than sitting at one
        # value. A force that stops growing is one you learn to push through;
        # a stop should feel firmer the harder you lean on it.
        strength = self.p("strength")
        lean = clamp(overshoot / (ramp * self.LEAN_RAMPS), 0.0, 1.0)
        level = strength + (1.0 - strength) * lean

        # Still setting new ground means still pressing in; the wall holds at
        # full weight. Only once the wheel has given back real distance -- not
        # the small elastic give of a firm hold -- does it count as leaving.
        if abs(signed_overshoot) >= abs(self._peak_overshoot):
            self._peak_overshoot = signed_overshoot
            factor = 1.0
        else:
            hysteresis = self.RELEASE_HYSTERESIS_RAMPS * ramp
            leaving = abs(signed_overshoot) < abs(self._peak_overshoot) - hysteresis
            factor = self.RELEASE_FACTOR if leaving else 1.0

        contribution.constant = -outward_sign * level * depth * factor

        # Damping stiffens with lean too, on the same curve as the wall itself.
        # This channel is a device-side condition effect, computed continuously
        # from the wheel's own velocity sensor rather than sent at the bridge's
        # loop rate, so it is not slowed by anything downstream -- unlike the
        # constant push, which is smoothed for safety and cannot drop fast
        # enough on its own to stop a hard hit picking up return speed.
        damping_base = self.p("damping")
        damping_level = damping_base + (1.0 - damping_base) * lean
        damping = damping_level * depth
        if damping > 1e-4:
            contribution.damper = Damper(coefficient=clamp(damping, 0.0, 1.0), saturation=0.9)

        return contribution
