"""Centring assist during a ground/air axis handoff.

The router can rate limit and blend all it likes, but the wheel is a physical
object with your hands on it. If it is held 60 degrees off centre when the axis
changes meaning, software alone cannot fix that. So during a handoff the bridge
asks for the wheel back: a spring toward centre, strongest at the start of the
transition and released as it completes, with extra damping so it walks rather
than snaps.
"""

from __future__ import annotations

from ..context import TickContext
from ..filters import clamp
from ..forces import Contribution, Damper, Spring
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class HandoffAssist(EffectModule):
    """Pulls the wheel to neutral while the axis changes hands."""

    id = "handoff"
    name = "Handoff assist"
    description = "Walks the wheel to centre while the axis swaps between rudder and ailerons."
    priority = 25
    params = (
        ParamSpec("strength", 0.55, 0.0, 1.0, "Centring force during a handoff", ""),
        ParamSpec("damping", 0.30, 0.0, 1.0, "Damping during a handoff", ""),
        ParamSpec("saturation", 0.70, 0.1, 1.0, "Ceiling on the centring force", ""),
    )

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()
        if not ctx.mode.is_transition:
            return contribution

        # Strongest as the handoff begins, gone by the time it completes, so the
        # assist never fights the pilot once the new axis has real authority.
        release = 1.0 - ctx.transition_progress

        strength = self.p("strength") * release
        if strength > 1e-4:
            contribution.spring = Spring(
                coefficient=clamp(strength, 0.0, 1.0),
                center=0.0,
                saturation=self.p("saturation"),
                deadband=0.0,
            )

        damping = self.p("damping") * release
        if damping > 1e-4:
            contribution.damper = Damper(coefficient=clamp(damping, 0.0, 1.0), saturation=0.6)

        return contribution
