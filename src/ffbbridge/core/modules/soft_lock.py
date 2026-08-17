"""Soft lock: a wall at the point where the control runs out of travel.

A racing wheel turns much further than a flight control does. Only part of the
rim's travel is mapped to the axis -- about a third of it for ailerons -- and
past that point the surface is already hard over, so turning further does
nothing at all and gives no hint that it is doing nothing.

Both the sim racing convention and the real aeroplane agree on the answer. Sim
racers call it a soft lock: a software end stop at the car's real steering
limit. A yoke calls it a stop, and it is a solid one -- a Cessna's control wheel
simply will not go past full aileron. This gives the rim the same thing: nothing
at all until the control is fully deflected, then a firm wall you can lean on
and immediately recognise.

It is built as a spring with a large deadband, which is the standard way to make
a virtual end stop and costs one device effect slot. No force at all inside the
deadband, so it never interferes with the feel of ordinary flying; resistance
outside it. Because the wheelbase resolves it from its own position sensor, the
resistance is immediate rather than arriving a loop tick later, and it cannot
oscillate the way a high-gain software spring sampled at 100 Hz would.

**How firm it can be.** A condition effect reaches full force only at full
displacement, so the steepest wall available rises in proportion to how far past
the stop you have pushed: a tenth of the rim's travel past it gives a tenth of
maximum force. With the aileron range at a third of a 540-degree wheel the stop
lands near the middle of the travel, and what you feel is firm, unmistakable,
progressive resistance rather than a brick wall. Reducing the wheelbase's
rotation range so less travel is wasted makes it firmer, and is better setup
anyway. There is no way around this with force feedback effects alone, and
faking one in software would risk the wheel buzzing at the boundary.

The lock moves with the ground/air handoff, since the two axes use different
amounts of travel: wider for steering, where fine control at low speed matters,
narrower for ailerons.
"""

from __future__ import annotations

from ..context import TickContext
from ..filters import clamp
from ..forces import Contribution, Spring
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class SoftLock(EffectModule):
    """A firm end stop where the control surface reaches full deflection."""

    id = "soft_lock"
    name = "Soft lock (control stops)"
    description = (
        "A wall at the point where the control is fully deflected, so the rim stops "
        "where the aeroplane's controls would."
    )
    priority = 90
    params = (
        # Full stiffness by default: this is the steepest a condition effect can
        # be, and anything less makes a stop that is easy to miss.
        ParamSpec("strength", 1.0, 0.0, 1.0, "How solid the stop feels", ""),
        ParamSpec("saturation", 0.95, 0.1, 1.0, "Ceiling on the wall's force", ""),
        ParamSpec("margin", 0.02, 0.0, 0.25, "Travel past full deflection before it bites", ""),
        ParamSpec("min_travel", 0.05, 0.01, 1.0, "Closest to centre the wall may sit", ""),
    )

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()

        strength = self.p("strength")
        if strength <= 1e-4:
            return contribution

        # A small margin past the deflection limit keeps the wall from nibbling
        # at the last of the usable travel, so full control authority is
        # reachable without fighting anything.
        lock = ctx.lock_displacement + self.p("margin")
        if lock >= 1.0:
            # The whole rim is mapped to the axis: there is no wasted travel to
            # wall off, and the wheelbase's own rotation limit is the stop.
            return contribution

        contribution.end_stop = Spring(
            coefficient=clamp(strength, 0.0, 1.0),
            center=clamp(ctx.center),
            saturation=self.p("saturation"),
            deadband=clamp(max(lock, self.p("min_travel")), 0.0, 1.0),
        )
        return contribution
