"""Airframe buffet: stall, Mach and things hanging out in the airflow.

Off by default. The first build was tuned around ground and powerplant feel, so
this ships complete but switched off rather than half-tuned and in the way.
Enable it in the GUI or set ``"enabled": true`` under ``modules.buffet``.
"""

from __future__ import annotations

from ..context import TickContext
from ..filters import clamp, map_range, smoothstep
from ..forces import Contribution, Waveform
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class Buffet(EffectModule):
    """Pre-stall burble, Mach rumble and flap or spoiler airflow shake."""

    id = "buffet"
    name = "Buffet (stall / Mach / flaps)"
    description = "Airframe shake from separated airflow. Off by default."
    priority = 80
    default_enabled = False
    params = (
        ParamSpec("stall_strength", 0.55, 0.0, 1.0, "Stall buffet magnitude", ""),
        ParamSpec("stall_hz", 12.0, 4.0, 30.0, "Stall buffet frequency", "Hz"),
        ParamSpec("alpha_onset", 0.75, 0.3, 1.0, "Fraction of stall alpha where buffet starts", ""),
        ParamSpec("stall_alpha_rad", 0.28, 0.1, 0.6, "Assumed stall angle of attack", "rad"),
        ParamSpec("mach_strength", 0.35, 0.0, 1.0, "Mach buffet magnitude", ""),
        ParamSpec("mach_onset", 0.78, 0.5, 0.99, "Mach number where buffet starts", ""),
        ParamSpec("flap_strength", 0.22, 0.0, 1.0, "Flap and spoiler airflow shake", ""),
        ParamSpec("flap_speed_kt", 110.0, 40.0, 250.0, "Speed for full flap buffet", "kt"),
    )

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()
        if tel.weight_on_wheels:
            return contribution

        # Stall: driven by angle of attack, with the stall warning flag as a
        # backstop for aircraft that do not report a usable alpha.
        alpha_ratio = abs(tel.alpha_rad) / self.p("stall_alpha_rad")
        onset = self.p("alpha_onset")
        stall = smoothstep(alpha_ratio, onset, 1.05)
        if tel.stall_warning:
            stall = max(stall, 0.5)
        if stall > 1e-3:
            contribution.add_periodic(
                "stall_buffet",
                frequency_hz=self.p("stall_hz") * (1.0 + 0.35 * stall),
                magnitude=clamp(self.p("stall_strength") * stall, 0.0, 1.0),
                waveform=Waveform.TRIANGLE,
                priority=self.priority + 5,
            )

        mach = map_range(tel.mach, self.p("mach_onset"), 0.98, 0.0, 1.0)
        if mach > 1e-3:
            contribution.add_periodic(
                "mach_buffet",
                frequency_hz=22.0,
                magnitude=clamp(self.p("mach_strength") * mach, 0.0, 1.0),
                waveform=Waveform.SINE,
                priority=self.priority,
            )

        drag_devices = max(tel.flaps_pct, tel.spoilers_pct)
        if drag_devices > 0.02:
            speed = map_range(tel.ias_kt, 40.0, self.p("flap_speed_kt"), 0.0, 1.0)
            magnitude = self.p("flap_strength") * drag_devices * speed
            if magnitude > 1e-3:
                contribution.add_periodic(
                    "flap_buffet",
                    frequency_hz=16.0,
                    magnitude=clamp(magnitude, 0.0, 1.0),
                    waveform=Waveform.SINE,
                    priority=self.priority - 5,
                )

        return contribution
