"""Brake shudder and anti-skid stutter during the rollout."""

from __future__ import annotations

from ..context import TickContext
from ..filters import clamp, map_range, smoothstep
from ..forces import Contribution, Waveform
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class BrakeShudder(EffectModule):
    """Braking judder, growing as the brakes bite and fading as you slow.

    Two components: a low rumble from the discs and tyres working, and a coarse
    stutter that only shows up under heavy braking, standing in for the wheels
    starting to skip.
    """

    id = "brakes"
    name = "Brake shudder"
    description = "Judder under braking, with a coarser stutter when you stand on them."
    priority = 60
    params = (
        ParamSpec("threshold", 0.06, 0.0, 0.5, "Brake input before anything is felt", ""),
        ParamSpec("base_hz", 11.0, 4.0, 30.0, "Judder frequency", "Hz"),
        ParamSpec("hz_span", 7.0, 0.0, 30.0, "Extra frequency at full brake", "Hz"),
        ParamSpec("skid_threshold", 0.82, 0.3, 1.0, "Brake input where the stutter starts", ""),
        ParamSpec("skid_hz", 6.5, 2.0, 20.0, "Stutter frequency", "Hz"),
        ParamSpec("full_speed_kt", 50.0, 5.0, 150.0, "Speed for full intensity", "kt"),
        ParamSpec("differential", 0.18, 0.0, 1.0, "Pull from uneven braking", ""),
    )

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()
        if not tel.weight_on_wheels:
            return contribution

        brake = tel.brake_input
        if tel.parking_brake:
            brake = max(brake, 1.0)
        threshold = self.p("threshold")
        if brake <= threshold:
            return contribution

        speed = abs(tel.gs_kt)
        # Below a walking pace there is nothing left to shudder against.
        speed_factor = smoothstep(speed, 1.5, self.p("full_speed_kt"))
        if speed_factor <= 1e-3:
            return contribution

        effort = map_range(brake, threshold, 1.0, 0.0, 1.0)
        magnitude = clamp(effort * speed_factor * 0.55, 0.0, 1.0)

        contribution.add_periodic(
            "brake_judder",
            frequency_hz=self.p("base_hz") + self.p("hz_span") * effort,
            magnitude=magnitude,
            waveform=Waveform.SINE,
            priority=self.priority,
        )

        skid_threshold = self.p("skid_threshold")
        if brake > skid_threshold:
            skid = map_range(brake, skid_threshold, 1.0, 0.0, 1.0)
            contribution.add_periodic(
                "brake_skid",
                frequency_hz=self.p("skid_hz"),
                magnitude=clamp(skid * speed_factor * 0.4, 0.0, 1.0),
                waveform=Waveform.SQUARE,
                priority=self.priority + 5,
            )

        # Asymmetric braking pulls the nosewheel toward the braked side. It is a
        # force in the steering rather than one that comes up through the gear,
        # so it goes with the rudder when the wheel is not carrying it.
        differential = tel.brake_right - tel.brake_left
        pull = differential * speed_factor * self.p("differential") * ctx.ground_weight
        contribution.constant += clamp(pull)

        return contribution
