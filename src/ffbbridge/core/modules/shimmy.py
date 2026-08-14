"""Nosewheel shimmy: the taxi-speed wobble you feel in the rudder pedals."""

from __future__ import annotations

from ..context import TickContext
from ..filters import clamp, map_range
from ..forces import Contribution, Waveform
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class NosewheelShimmy(EffectModule):
    """A narrow-band wobble when the nosewheel is loaded and deflected.

    Real shimmy appears in a speed band and needs a steering input to excite it,
    which is exactly what makes it satisfying here: it only shows up when you are
    turning off the runway at a brisk taxi, not as constant background noise.
    """

    id = "shimmy"
    name = "Nosewheel shimmy"
    description = "Speed-banded wobble when the nosewheel is loaded and turned."
    priority = 45
    params = (
        ParamSpec("peak_kt", 28.0, 5.0, 80.0, "Speed of worst shimmy", "kt"),
        ParamSpec("band_kt", 22.0, 5.0, 60.0, "Width of the shimmy band", "kt"),
        ParamSpec("hz", 17.0, 6.0, 40.0, "Shimmy frequency", "Hz"),
        ParamSpec("deflection", 0.10, 0.0, 0.8, "Steering input needed to excite it", ""),
        ParamSpec("strength", 0.35, 0.0, 1.0, "Peak magnitude", ""),
    )

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()
        if not tel.weight_on_wheels or ctx.ground_weight <= 0.01:
            return contribution

        # Needs weight on the nose; a taildragger or a rotating aircraft has none.
        nose_load = tel.nose_compression
        if tel.contact_compression and nose_load < 0.03:
            return contribution
        if not tel.contact_compression:
            nose_load = 0.5

        speed = abs(tel.gs_kt)
        peak, band = self.p("peak_kt"), self.p("band_kt")
        distance = abs(speed - peak) / band
        if distance >= 1.0:
            return contribution
        speed_factor = 1.0 - distance * distance

        deflection = abs(wheel.position)
        threshold = self.p("deflection")
        if deflection <= threshold:
            return contribution
        excitation = map_range(deflection, threshold, 0.6, 0.0, 1.0)

        magnitude = clamp(
            self.p("strength") * speed_factor * excitation * clamp(nose_load * 2.0, 0.0, 1.0),
            0.0,
            1.0,
        )
        contribution.add_periodic(
            "shimmy",
            frequency_hz=self.p("hz"),
            magnitude=magnitude * ctx.ground_weight,
            waveform=Waveform.SINE,
            priority=self.priority,
        )
        return contribution
