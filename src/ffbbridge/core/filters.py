"""Small stateful signal-processing helpers used by the effect modules.

Everything here is frame-rate independent: each ``update`` takes the elapsed time
so the behaviour does not change when the sim frame rate does.
"""

from __future__ import annotations

import math
import random

__all__ = [
    "clamp",
    "lerp",
    "map_range",
    "smoothstep",
    "deadband",
    "expo",
    "LowPass",
    "HighPass",
    "SlewLimiter",
    "RateOfChange",
    "Hysteresis",
    "EdgeDetector",
    "DwellTimer",
    "OneShot",
    "BandNoise",
    "PhaseAccumulator",
]


def clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    """Constrain ``value`` to ``[low, high]``."""
    if value < low:
        return low
    if value > high:
        return high
    return value


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation, with ``t`` clamped to 0..1."""
    return a + (b - a) * clamp(t, 0.0, 1.0)


def map_range(
    value: float,
    in_low: float,
    in_high: float,
    out_low: float,
    out_high: float,
    *,
    clip: bool = True,
) -> float:
    """Rescale ``value`` from one range to another."""
    if in_high == in_low:
        return out_low
    t = (value - in_low) / (in_high - in_low)
    if clip:
        t = clamp(t, 0.0, 1.0)
    return out_low + (out_high - out_low) * t


def smoothstep(value: float, edge0: float, edge1: float) -> float:
    """Cubic ease between two edges, returning 0..1."""
    if edge0 == edge1:
        return 0.0 if value < edge0 else 1.0
    t = clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def deadband(value: float, width: float) -> float:
    """Zero out small inputs and rescale the remainder so the output stays continuous."""
    if width <= 0.0:
        return value
    if abs(value) <= width:
        return 0.0
    scale = 1.0 / (1.0 - width) if width < 1.0 else 1.0
    return math.copysign((abs(value) - width) * scale, value)


def expo(value: float, amount: float) -> float:
    """Blend between a linear and a cubic response.

    ``amount`` 0 is linear, 1 is fully cubic, which softens the centre of the
    wheel's travel without changing the endpoints.
    """
    a = clamp(amount, 0.0, 1.0)
    return (1.0 - a) * value + a * value * value * value


class LowPass:
    """First-order low-pass with a time constant in seconds."""

    __slots__ = ("tau", "value", "_primed")

    def __init__(self, tau: float, initial: float = 0.0) -> None:
        self.tau = max(tau, 0.0)
        self.value = initial
        self._primed = False

    def update(self, value: float, dt: float) -> float:
        if not self._primed:
            self._primed = True
            self.value = value
            return self.value
        if self.tau <= 0.0 or dt <= 0.0:
            self.value = value
        else:
            alpha = 1.0 - math.exp(-dt / self.tau)
            self.value += (value - self.value) * alpha
        return self.value

    def reset(self, value: float = 0.0) -> None:
        self.value = value
        self._primed = False


class HighPass:
    """Complement of :class:`LowPass`; isolates the fast part of a signal.

    Used to turn absolute body accelerations into the jolts you actually feel,
    with steady-state loading such as 1 G of level flight removed.
    """

    __slots__ = ("_lp",)

    def __init__(self, tau: float) -> None:
        self._lp = LowPass(tau)

    def update(self, value: float, dt: float) -> float:
        return value - self._lp.update(value, dt)

    def reset(self, value: float = 0.0) -> None:
        self._lp.reset(value)


class SlewLimiter:
    """Bounds how fast a signal may change, in units per second.

    This is a safety device as much as a smoother: it stops a telemetry glitch
    from becoming a torque step at the wheel rim.
    """

    __slots__ = ("max_rate", "value")

    def __init__(self, max_rate: float, initial: float = 0.0) -> None:
        self.max_rate = max(max_rate, 0.0)
        self.value = initial

    def update(self, target: float, dt: float) -> float:
        if dt <= 0.0 or self.max_rate <= 0.0:
            self.value = target
            return self.value
        max_step = self.max_rate * dt
        delta = target - self.value
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        self.value += delta
        return self.value

    def reset(self, value: float = 0.0) -> None:
        self.value = value


class RateOfChange:
    """Differentiates a signal, with light smoothing to keep it usable."""

    __slots__ = ("_last", "_primed", "_lp")

    def __init__(self, smoothing: float = 0.02) -> None:
        self._last = 0.0
        self._primed = False
        self._lp = LowPass(smoothing)

    def update(self, value: float, dt: float) -> float:
        if not self._primed:
            self._primed = True
            self._last = value
            return 0.0
        rate = (value - self._last) / dt if dt > 0.0 else 0.0
        self._last = value
        return self._lp.update(rate, dt)

    def reset(self) -> None:
        self._primed = False
        self._lp.reset()


class Hysteresis:
    """Boolean with separate rising and falling thresholds."""

    __slots__ = ("on_threshold", "off_threshold", "state")

    def __init__(self, on_threshold: float, off_threshold: float, initial: bool = False) -> None:
        self.on_threshold = on_threshold
        self.off_threshold = off_threshold
        self.state = initial

    def update(self, value: float) -> bool:
        if self.state:
            if value < self.off_threshold:
                self.state = False
        elif value > self.on_threshold:
            self.state = True
        return self.state


class EdgeDetector:
    """Reports the transitions of a boolean signal."""

    __slots__ = ("state", "_primed")

    def __init__(self, initial: bool = False) -> None:
        self.state = initial
        self._primed = False

    def update(self, value: bool) -> int:
        """Return 1 on a rising edge, -1 on a falling edge, 0 otherwise.

        The first call only latches the incoming state, so a bridge that starts
        up with the aircraft already on the runway does not fire a touchdown.
        """
        if not self._primed:
            self._primed = True
            self.state = value
            return 0
        if value == self.state:
            return 0
        self.state = value
        return 1 if value else -1


class DwellTimer:
    """Requires a condition to hold continuously for a set duration."""

    __slots__ = ("duration", "elapsed")

    def __init__(self, duration: float) -> None:
        self.duration = max(duration, 0.0)
        self.elapsed = 0.0

    def update(self, condition: bool, dt: float) -> bool:
        if condition:
            self.elapsed += dt
        else:
            self.elapsed = 0.0
        return self.elapsed >= self.duration

    def reset(self) -> None:
        self.elapsed = 0.0


class OneShot:
    """An attack/decay envelope fired by a trigger, used for transient thumps."""

    __slots__ = ("attack", "decay", "_amplitude", "_elapsed", "_active")

    def __init__(self, attack: float = 0.01, decay: float = 0.25) -> None:
        self.attack = max(attack, 1e-4)
        self.decay = max(decay, 1e-4)
        self._amplitude = 0.0
        self._elapsed = 0.0
        self._active = False

    @property
    def active(self) -> bool:
        return self._active

    def fire(self, amplitude: float) -> None:
        """Start the envelope, keeping the stronger of any overlapping hits.

        While the envelope is still rising its current value is near zero, so
        comparing against that would let a weaker hit arriving in the same tick
        replace a stronger one. During the attack the comparison is therefore
        against the amplitude the envelope is heading for.
        """
        if self._active:
            pending = self._amplitude if self._elapsed < self.attack else self.value()
            if pending >= amplitude:
                return
        self._amplitude = amplitude
        self._elapsed = 0.0
        self._active = True

    def update(self, dt: float) -> float:
        if not self._active:
            return 0.0
        self._elapsed += dt
        if self._elapsed >= self.attack + self.decay:
            self._active = False
            return 0.0
        return self.value()

    def value(self) -> float:
        if not self._active:
            return 0.0
        if self._elapsed < self.attack:
            return self._amplitude * (self._elapsed / self.attack)
        fade = (self._elapsed - self.attack) / self.decay
        return self._amplitude * max(0.0, 1.0 - fade)

    def reset(self) -> None:
        self._active = False
        self._amplitude = 0.0
        self._elapsed = 0.0


class BandNoise:
    """Smoothed pseudo-random noise, for turbulence and surface texture.

    White noise feels like electrical buzz, so it is low-passed into a rumble
    whose character follows ``tau``. A seedable generator keeps tests exact.
    """

    __slots__ = ("_lp", "_rng")

    def __init__(self, tau: float = 0.08, seed: int | None = None) -> None:
        self._lp = LowPass(tau)
        self._rng = random.Random(seed)

    def update(self, dt: float) -> float:
        """Return a smoothed sample, roughly -1..1."""
        raw = self._rng.uniform(-1.0, 1.0)
        # Low-passing shrinks the amplitude, so scale back up to keep the
        # nominal range usable as a gain of 1.0.
        return clamp(self._lp.update(raw, dt) * 2.5)

    def reset(self) -> None:
        self._lp.reset()


class PhaseAccumulator:
    """Tracks the phase of a variable-frequency oscillator.

    Integrating frequency rather than evaluating ``sin(2*pi*f*t)`` keeps the
    waveform continuous when the frequency changes, which matters for ground
    rumble: the tone should glide with groundspeed, not click on every update.
    """

    __slots__ = ("phase",)

    def __init__(self, phase: float = 0.0) -> None:
        self.phase = phase

    def update(self, frequency_hz: float, dt: float) -> float:
        self.phase = (self.phase + frequency_hz * dt) % 1.0
        return self.phase

    def sine(self) -> float:
        return math.sin(self.phase * 2.0 * math.pi)

    def reset(self) -> None:
        self.phase = 0.0
