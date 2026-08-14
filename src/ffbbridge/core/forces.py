"""The vocabulary the effect modules speak.

Sign convention throughout: **positive is clockwise / to the right** from the
pilot's seat. A positive constant force pushes the wheel rim to the right.

All magnitudes are normalised to -1..1, where 1 means the full output the user
has configured. Converting that into device units is the io layer's job, so the
force model never needs to know it is talking to a 5.5 N-m wheelbase.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import StrEnum

from .filters import clamp


class Waveform(StrEnum):
    """Periodic shapes, chosen to match what SDL and DirectInput both provide."""

    SINE = "sine"
    SQUARE = "square"
    TRIANGLE = "triangle"
    SAWTOOTH_UP = "sawtooth_up"
    SAWTOOTH_DOWN = "sawtooth_down"

    def sample(self, phase: float) -> float:
        """Evaluate the waveform at ``phase`` in 0..1, returning -1..1.

        Only needed when a periodic has to be synthesised in software because
        the device ran out of effect slots.
        """
        p = phase % 1.0
        if self is Waveform.SINE:
            return math.sin(p * 2.0 * math.pi)
        if self is Waveform.SQUARE:
            return 1.0 if p < 0.5 else -1.0
        if self is Waveform.TRIANGLE:
            return 4.0 * abs(p - 0.5) - 1.0
        if self is Waveform.SAWTOOTH_UP:
            return 2.0 * p - 1.0
        return 1.0 - 2.0 * p


@dataclass(frozen=True, slots=True)
class Periodic:
    """A vibration. Rendered on the device itself whenever a slot is free.

    Device-side rendering matters: the wheel updates a periodic at its own
    internal rate, so a 30 Hz rumble stays clean even though the bridge only
    revises its parameters 100 times a second.
    """

    label: str
    waveform: Waveform = Waveform.SINE
    frequency_hz: float = 10.0
    magnitude: float = 0.0
    offset: float = 0.0
    priority: int = 0
    """Higher wins when there are more periodics than hardware slots."""

    def scaled(self, gain: float) -> Periodic:
        return Periodic(
            label=self.label,
            waveform=self.waveform,
            frequency_hz=self.frequency_hz,
            magnitude=clamp(self.magnitude * gain, 0.0, 1.0),
            offset=clamp(self.offset * gain),
            priority=self.priority,
        )

    @property
    def is_audible(self) -> bool:
        """Whether this is worth spending a hardware slot on."""
        return self.magnitude > 1e-3 and self.frequency_hz > 0.0


@dataclass(frozen=True, slots=True)
class Spring:
    """A centring force computed on the device from wheel position.

    ``center`` is where the wheel wants to sit, which is how trim is expressed:
    an out-of-trim aircraft pulls toward a displaced neutral rather than centre.
    """

    coefficient: float = 0.0
    center: float = 0.0
    saturation: float = 1.0
    deadband: float = 0.0

    def force_at(self, position: float) -> float:
        """The force this spring produces at a given wheel position.

        Used by the software fallback path and by the tests.
        """
        offset = position - self.center
        if abs(offset) <= self.deadband:
            return 0.0
        offset -= math.copysign(self.deadband, offset)
        return clamp(-self.coefficient * offset, -self.saturation, self.saturation)


@dataclass(frozen=True, slots=True)
class Damper:
    """Resistance proportional to how fast the wheel is being moved."""

    coefficient: float = 0.0
    saturation: float = 1.0

    def force_at(self, velocity: float) -> float:
        return clamp(-self.coefficient * velocity, -self.saturation, self.saturation)


@dataclass(slots=True)
class Contribution:
    """What a single effect module asks for on this tick.

    Modules build one of these and hand it back; they never see the device, the
    other modules, or the user's gain settings.
    """

    constant: float = 0.0
    spring: Spring | None = None
    damper: Damper | None = None
    periodics: list[Periodic] = field(default_factory=list)

    def add_periodic(
        self,
        label: str,
        frequency_hz: float,
        magnitude: float,
        *,
        waveform: Waveform = Waveform.SINE,
        offset: float = 0.0,
        priority: int = 0,
    ) -> None:
        """Convenience for the common case; silently drops inaudible requests."""
        if magnitude <= 1e-4 or frequency_hz <= 0.0:
            return
        self.periodics.append(
            Periodic(
                label=label,
                waveform=waveform,
                frequency_hz=frequency_hz,
                magnitude=clamp(magnitude, 0.0, 1.0),
                offset=clamp(offset),
                priority=priority,
            )
        )

    @property
    def is_empty(self) -> bool:
        return (
            abs(self.constant) < 1e-6
            and self.spring is None
            and self.damper is None
            and not self.periodics
        )


@dataclass(frozen=True, slots=True)
class ForceOutput:
    """The mixed result for one tick, ready for the device.

    ``breakdown`` carries each module's signed contribution so the GUI can show
    which effect is responsible for what the wheel is doing.
    """

    constant: float = 0.0
    spring: Spring | None = None
    damper: Damper | None = None
    periodics: tuple[Periodic, ...] = ()
    breakdown: dict[str, float] = field(default_factory=dict)
    clipped: bool = False

    def total_at(self, position: float, velocity: float) -> float:
        """Total steady force at a given wheel state, for tests and the trace plot."""
        total = self.constant
        if self.spring is not None:
            total += self.spring.force_at(position)
        if self.damper is not None:
            total += self.damper.force_at(velocity)
        return total


ZERO_FORCE = ForceOutput()


def combine_springs(springs: list[Spring]) -> Spring | None:
    """Merge several springs into the single one the device can render.

    Springs in parallel add their stiffness, and the combined rest position is
    the stiffness-weighted average of the individual centres, which is what a
    real linkage carrying two springs would settle at.
    """
    active = [s for s in springs if s.coefficient > 1e-6]
    if not active:
        return None
    total_k = sum(s.coefficient for s in active)
    center = sum(s.center * s.coefficient for s in active) / total_k
    return Spring(
        coefficient=clamp(total_k, 0.0, 1.0),
        center=clamp(center),
        saturation=clamp(max(s.saturation for s in active), 0.0, 1.0),
        deadband=min(s.deadband for s in active),
    )


def combine_dampers(dampers: list[Damper]) -> Damper | None:
    """Merge dampers by summing their coefficients."""
    active = [d for d in dampers if d.coefficient > 1e-6]
    if not active:
        return None
    return Damper(
        coefficient=clamp(sum(d.coefficient for d in active), 0.0, 1.0),
        saturation=clamp(max(d.saturation for d in active), 0.0, 1.0),
    )
