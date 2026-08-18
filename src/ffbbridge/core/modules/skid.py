"""A wheel that has stopped turning while the aeroplane has not."""

from __future__ import annotations

from ..context import TickContext
from ..filters import LowPass, clamp, map_range, smoothstep
from ..forces import Contribution, Waveform
from ..telemetry import FlightTelemetry, WheelState
from .base import EffectModule, ParamSpec


class LockedWheel(EffectModule):
    """Judder and a pull toward the side that is skidding rather than rolling.

    Brake shudder is what braking feels like when it is working. This is what it
    feels like when it has stopped working: a locked tyre skips rather than
    rolls, which is coarser and slower than disc judder, and a wheel locked on
    one side alone drags that wing back and takes the nose with it.

    The awkward part is knowing what "should be turning" means, because nothing
    reports the tyre radius and it differs by an order of magnitude between a
    Cub and a 747. Two independent readings get around it without a calibration
    step:

    * the mains are the same size as each other, so one turning while the other
      does not is a lock on that side, whatever the radius;
    * for both locking together, the module learns how many RPM a knot is worth
      while the aircraft is rolling with the brakes off, and compares against
      that afterwards.

    An aircraft that does not report wheel RPM learns a ratio of zero, expects
    nothing, and stays silent, which is the right way for this to fail.
    """

    id = "skid"
    name = "Locked wheel"
    description = "Skip and pull when a tyre stops rolling under heavy braking."
    priority = 78
    params = (
        ParamSpec("threshold", 0.35, 0.05, 0.9, "Slip before anything is felt", ""),
        ParamSpec("hz", 7.5, 2.0, 20.0, "Skip frequency", "Hz"),
        ParamSpec("strength", 0.5, 0.0, 1.0, "Peak skip magnitude", ""),
        ParamSpec("pull", 0.3, 0.0, 1.0, "Pull toward the locked side", ""),
        ParamSpec("full_speed_kt", 40.0, 5.0, 150.0, "Speed for full intensity", "kt"),
        ParamSpec("learn_speed_kt", 15.0, 5.0, 80.0, "Speed the reference learns above", "kt"),
    )

    def __init__(self, settings=None) -> None:
        super().__init__(settings)
        # Slow, because it is a property of the aeroplane rather than of the
        # moment: it should not follow a wheel into the skid it is measuring.
        self._rpm_per_kt = LowPass(tau=2.0)
        self._learned = False

    def reset(self) -> None:
        self._rpm_per_kt.reset()
        self._learned = False

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()
        if not tel.weight_on_wheels:
            return contribution

        speed = abs(tel.gs_kt)
        left, right = abs(tel.wheel_rpm_left), abs(tel.wheel_rpm_right)

        # Learn only while rolling freely: under braking the very thing being
        # measured is what is wrong, and a skid would teach it to expect a skid.
        if speed > self.p("learn_speed_kt") and tel.brake_input < 0.05:
            self._rpm_per_kt.update(max(left, right) / speed, dt)
            self._learned = True

        if not self._learned or speed < 1.0:
            return contribution

        expected = self._rpm_per_kt.value * speed
        if expected <= 1e-3:
            # No usable wheel RPM from this aircraft. Nothing to say.
            return contribution

        slip_left = clamp(1.0 - left / expected, 0.0, 1.0)
        slip_right = clamp(1.0 - right / expected, 0.0, 1.0)
        slip = max(slip_left, slip_right)

        threshold = self.p("threshold")
        if slip <= threshold:
            return contribution

        depth = map_range(slip, threshold, 1.0, 0.0, 1.0)
        speed_factor = smoothstep(speed, 2.0, self.p("full_speed_kt"))
        if speed_factor <= 1e-3:
            return contribution

        # A skip is a tyre letting go and grabbing again, so it is square rather
        # than a tone, and coarse enough to count.
        contribution.add_periodic(
            "skid_skip",
            frequency_hz=self.p("hz"),
            magnitude=clamp(self.p("strength") * depth * speed_factor, 0.0, 1.0),
            waveform=Waveform.SQUARE,
            priority=self.priority,
        )

        # One side locked is a yaw; both locked is a straight-ahead skid with
        # nothing to choose between them, and the difference handles both.
        imbalance = slip_right - slip_left
        pull = imbalance * self.p("pull") * speed_factor * ctx.ground_weight
        contribution.constant += clamp(pull)

        return contribution
