"""Landing gear travelling, locking and hanging in the airflow."""

from __future__ import annotations

from ..context import TickContext
from ..filters import EdgeDetector, OneShot, RateOfChange, clamp, smoothstep
from ..forces import Contribution, Waveform
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class GearTransit(EffectModule):
    """Rumble while the gear is in transit, and a clunk when it locks.

    The airflow buffet while the doors are open scales with airspeed, so
    lowering the gear late and fast feels different from doing it properly.
    """

    id = "gear"
    name = "Gear transit"
    description = "Motor rumble while the gear travels, plus a clunk at the locks."
    priority = 55
    params = (
        ParamSpec("transit_hz", 14.0, 4.0, 40.0, "Transit rumble frequency", "Hz"),
        ParamSpec("transit_strength", 0.28, 0.0, 1.0, "Transit rumble magnitude", ""),
        ParamSpec("clunk", 0.45, 0.0, 1.0, "Lock clunk magnitude", ""),
        ParamSpec("buffet", 0.25, 0.0, 1.0, "Airflow buffet with doors open", ""),
        ParamSpec("buffet_speed_kt", 130.0, 40.0, 300.0, "Speed for full buffet", "kt"),
    )

    #: Gear travel slower than this counts as stopped.
    MOVING_RATE = 0.02

    def __init__(self, settings=None) -> None:
        super().__init__(settings)
        self._rate = RateOfChange(smoothing=0.05)
        self._moving_edge = EdgeDetector()
        self._clunk = OneShot(attack=0.006, decay=0.18)

    def reset(self) -> None:
        self._rate.reset()
        self._moving_edge = EdgeDetector()
        self._clunk.reset()

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()

        rate = abs(self._rate.update(tel.gear_pct, dt))
        moving = rate > self.MOVING_RATE

        # A falling edge means the gear just stopped travelling: it locked.
        if self._moving_edge.update(moving) == -1:
            self._clunk.fire(self.p("clunk"))

        if moving:
            speed_factor = smoothstep(tel.ias_kt, 30.0, self.p("buffet_speed_kt"))
            magnitude = self.p("transit_strength") * (1.0 + speed_factor * self.p("buffet") * 2.0)
            contribution.add_periodic(
                "gear_transit",
                frequency_hz=self.p("transit_hz"),
                magnitude=clamp(magnitude, 0.0, 1.0),
                waveform=Waveform.TRIANGLE,
                priority=self.priority,
            )

        clunk = self._clunk.update(dt)
        if clunk > 1e-4:
            contribution.add_periodic(
                "gear_clunk",
                frequency_hz=32.0,
                magnitude=clunk,
                waveform=Waveform.SQUARE,
                priority=self.priority + 10,
            )

        return contribution
