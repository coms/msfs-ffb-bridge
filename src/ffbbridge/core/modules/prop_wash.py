"""Slipstream effects: torque, P-factor and the burble over the tail."""

from __future__ import annotations

from ..context import TickContext
from ..filters import clamp, map_range
from ..forces import Contribution, Waveform
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class PropWash(EffectModule):
    """Asymmetric pull under power, strongest slow and loud.

    On a single-engine piston aircraft, opening the throttle at low airspeed
    swings the nose left: slipstream over the fin, P-factor from the descending
    blade, and engine torque all conspire. This puts that pull into the wheel,
    so a takeoff roll needs a steady boot of right rudder and eases off as the
    tail becomes effective.
    """

    id = "prop_wash"
    name = "Slipstream & torque"
    description = "Left-pulling swing under power at low speed, plus slipstream burble."
    priority = 20
    params = (
        ParamSpec("torque", 0.30, 0.0, 1.0, "Strength of the swing under power", ""),
        ParamSpec("fade_speed_kt", 85.0, 20.0, 200.0, "Speed where the swing has faded", "kt"),
        ParamSpec("air_share", 0.35, 0.0, 1.0, "How much reaches the aileron axis in flight", ""),
        ParamSpec("burble", 0.16, 0.0, 1.0, "Slipstream rumble", ""),
        ParamSpec("burble_hz", 9.0, 3.0, 30.0, "Burble frequency", "Hz"),
        ParamSpec("direction", -1.0, -1.0, 1.0, "Which way the swing pulls", ""),
    )

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()
        if tel.is_helicopter:
            return contribution

        power = tel.max_throttle
        if tel.max_prop_rpm <= 20.0 or power <= 0.02:
            return contribution

        # The swing is a low-speed, high-power phenomenon.
        speed_factor = 1.0 - map_range(tel.ias_kt, 15.0, self.p("fade_speed_kt"), 0.0, 1.0)
        swing = self.p("torque") * power * speed_factor * self.p("direction")

        # In flight the same causes roll rather than yaw, so it still belongs on
        # the wheel, just weaker.
        axis_share = ctx.ground_weight + ctx.air_weight * self.p("air_share")
        contribution.constant += clamp(swing * axis_share)

        burble = self.p("burble") * power * speed_factor
        if burble > 1e-3:
            contribution.add_periodic(
                "prop_wash",
                frequency_hz=self.p("burble_hz"),
                magnitude=clamp(burble, 0.0, 1.0),
                waveform=Waveform.SINE,
                priority=self.priority,
            )
        return contribution
