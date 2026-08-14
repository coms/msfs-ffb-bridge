"""The arrival: main gear, nose gear, and how hard you put it on."""

from __future__ import annotations

from ..context import TickContext
from ..filters import EdgeDetector, OneShot, clamp, map_range
from ..forces import Contribution, Waveform
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class Touchdown(EffectModule):
    """Impact thump when the wheels meet the runway.

    The strength comes from the descent rate in the moment *before* contact,
    which is why the module keeps a short peak-hold: by the time the sim reports
    weight on wheels, vertical speed has already collapsed to nearly zero.
    """

    id = "touchdown"
    name = "Touchdown"
    description = "Impact thump scaled by descent rate, with a separate nosewheel arrival."
    priority = 100
    params = (
        ParamSpec(
            "full_scale_fpm", 420.0, 100.0, 1200.0, "Descent rate for a full-scale hit", "fpm"
        ),
        ParamSpec("min_fpm", 40.0, 0.0, 400.0, "Descent rate below which nothing fires", "fpm"),
        ParamSpec("decay_s", 0.22, 0.05, 1.0, "Thump length", "s"),
        ParamSpec("burst_hz", 26.0, 8.0, 60.0, "Impact shudder frequency", "Hz"),
        ParamSpec("nose_share", 0.45, 0.0, 1.0, "Nosewheel thump relative to mains", ""),
        ParamSpec("jolt", 0.45, 0.0, 1.0, "Share of the hit felt as a sideways kick", ""),
    )

    #: How fast a remembered descent-rate peak fades, in fpm per second.
    #: Slow enough that a flare does not erase the approach, fast enough that a
    #: firm arrival is not still remembered on the next landing.
    PEAK_FADE_FPM_PER_S = 160.0

    def __init__(self, settings=None) -> None:
        super().__init__(settings)
        self._main_edge = EdgeDetector()
        self._nose_edge = EdgeDetector()
        self._impulse = OneShot(attack=0.008, decay=0.22)
        self._shudder = OneShot(attack=0.01, decay=0.3)
        self._peak_descent_fpm = 0.0
        self._side_load = 0.0

    def reset(self) -> None:
        self._main_edge = EdgeDetector()
        self._nose_edge = EdgeDetector()
        self._impulse.reset()
        self._shudder.reset()
        self._peak_descent_fpm = 0.0
        self._side_load = 0.0

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()

        self._track_descent(tel, dt)

        main_contact = tel.weight_on_wheels
        nose_contact = tel.nose_compression > 0.05

        if self._main_edge.update(main_contact) == 1:
            self._fire(1.0)
        if self._nose_edge.update(nose_contact) == 1 and main_contact:
            # The nose arriving after the mains is a distinctly smaller, blunter
            # event than the main gear touching down.
            self._fire(self.p("nose_share"))

        self._impulse.decay = self.p("decay_s")
        self._shudder.decay = self.p("decay_s") * 1.4

        impulse = self._impulse.update(dt)
        shudder = self._shudder.update(dt)

        if impulse > 1e-4:
            # Direction follows the side load at the moment of contact, so a
            # crabbed arrival kicks the wheel the way the gear is being dragged.
            # A dead-straight landing still has to be felt as an arrival though,
            # so the kick keeps a floor and only its direction comes from drift.
            drift = clamp(self._side_load * 4.0)
            direction = drift if abs(drift) > 0.05 else 1.0
            weight = 0.45 + 0.55 * abs(drift)
            contribution.constant += impulse * self.p("jolt") * weight * direction

        if shudder > 1e-4:
            contribution.add_periodic(
                "touchdown",
                frequency_hz=self.p("burst_hz"),
                magnitude=shudder,
                waveform=Waveform.SQUARE,
                priority=self.priority,
            )

        return contribution

    def _track_descent(self, tel: FlightTelemetry, dt: float) -> None:
        """Peak-hold the descent rate and the side load seen just before contact.

        By the time the sim reports weight on wheels the descent has already been
        arrested, so the impact has to be judged from what was happening on the
        way down. The peak decays rather than being re-latched: a flare that
        genuinely reduces the descent should soften the arrival, but it must not
        wipe out the memory of how fast you were coming down.
        """
        fade = self.PEAK_FADE_FPM_PER_S * dt
        if tel.weight_on_wheels:
            self._peak_descent_fpm = max(0.0, self._peak_descent_fpm - fade)
            return

        descent = max(0.0, -tel.vs_fpm)
        self._peak_descent_fpm = max(descent, self._peak_descent_fpm - fade)
        self._side_load = clamp(tel.lateral_accel_g * 1.5 + tel.beta_rad * 0.8)

    def _fire(self, scale: float) -> None:
        descent = self._peak_descent_fpm
        if descent < self.p("min_fpm"):
            return
        strength = map_range(descent, self.p("min_fpm"), self.p("full_scale_fpm"), 0.15, 1.0)
        self._impulse.fire(clamp(strength * scale, 0.0, 1.0))
        self._shudder.fire(clamp(strength * scale * 0.85, 0.0, 1.0))
