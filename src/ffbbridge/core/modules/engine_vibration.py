"""Engine and propeller vibration."""

from __future__ import annotations

from ..context import TickContext
from ..filters import clamp, map_range, smoothstep
from ..forces import Contribution, Waveform
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


def fold_to_band(frequency: float, low: float, high: float, *, max_steps: int = 8) -> float:
    """Halve or double a frequency until it lands in a band you can feel.

    A two-blade prop at 2400 RPM has a blade-pass frequency of 80 Hz. A wheel
    motor cannot render that as anything but a faint buzz, and the rim's own
    inertia swallows most of it. Dropping to the 40 Hz subharmonic keeps the
    vibration locked to engine speed, which is the part your hands recognise,
    while putting it where the hardware is actually strong.
    """
    if frequency <= 0.0 or low <= 0.0 or high <= low:
        return 0.0
    steps = 0
    while frequency > high and steps < max_steps:
        frequency *= 0.5
        steps += 1
    while frequency < low and steps < max_steps:
        frequency *= 2.0
        steps += 1
    return clamp(frequency, low, high)


class EngineVibration(EffectModule):
    """Powerplant hum, idle shake and rough running.

    The tone is locked to propeller blade-pass frequency so it rises and falls
    with RPM the way the real thing does, and a prop losing sync or an engine
    running rough is immediately obvious through the rim.
    """

    id = "engine_vibration"
    name = "Engine vibration"
    description = "Blade-pass hum tracking RPM, idle shake, and roughness when running badly."
    priority = 30
    default_gain = 0.8
    params = (
        ParamSpec("blades", 2.0, 1.0, 8.0, "Propeller blades", ""),
        ParamSpec("band_low_hz", 13.0, 5.0, 30.0, "Lowest felt frequency", "Hz"),
        ParamSpec("band_high_hz", 42.0, 15.0, 80.0, "Highest felt frequency", "Hz"),
        ParamSpec("base", 0.18, 0.0, 1.0, "Vibration at cruise power", ""),
        ParamSpec("throttle_share", 0.35, 0.0, 1.0, "Extra vibration with power", ""),
        ParamSpec("idle_shake", 0.30, 0.0, 1.0, "Lumpiness at idle", ""),
        ParamSpec("rough_running", 0.55, 0.0, 1.0, "Roughness with an engine out", ""),
        ParamSpec("idle_rpm", 800.0, 200.0, 2000.0, "RPM considered idle", "rpm"),
        ParamSpec("max_rpm", 2700.0, 800.0, 8000.0, "RPM considered full power", "rpm"),
    )

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()

        rpm = tel.max_prop_rpm
        if rpm <= 20.0:
            return contribution

        idle_rpm, max_rpm = self.p("idle_rpm"), self.p("max_rpm")
        rpm_fraction = map_range(rpm, idle_rpm * 0.5, max_rpm, 0.0, 1.0)

        blade_pass = rpm / 60.0 * self.p("blades")
        frequency = fold_to_band(blade_pass, self.p("band_low_hz"), self.p("band_high_hz"))
        if frequency <= 0.0:
            return contribution

        magnitude = self.p("base") + self.p("throttle_share") * tel.max_throttle * rpm_fraction

        # Below idle the engine shakes on its mounts rather than humming.
        shake = self.p("idle_shake") * (1.0 - smoothstep(rpm, idle_rpm * 0.6, idle_rpm * 1.6))
        if shake > 1e-3:
            contribution.add_periodic(
                "engine_shake",
                frequency_hz=max(frequency * 0.5, self.p("band_low_hz") * 0.6),
                magnitude=clamp(shake, 0.0, 1.0),
                waveform=Waveform.TRIANGLE,
                priority=self.priority + 2,
            )

        # An engine turning without combustion is either starting, windmilling
        # or having a very bad day. All three feel rough.
        if tel.eng_combustion and not tel.any_engine_running:
            magnitude += self.p("rough_running")
        elif tel.eng_combustion and len(tel.eng_combustion) > 1:
            dead = sum(1 for running in tel.eng_combustion if not running)
            if dead:
                magnitude += self.p("rough_running") * dead / len(tel.eng_combustion)

        contribution.add_periodic(
            "engine",
            frequency_hz=frequency,
            magnitude=clamp(magnitude, 0.0, 1.0),
            waveform=Waveform.SINE,
            priority=self.priority,
        )
        return contribution
