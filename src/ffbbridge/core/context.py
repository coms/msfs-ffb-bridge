"""Per-tick state that modules need beyond the raw telemetry."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class AxisMode(str, Enum):
    """What the wheel is currently steering."""

    GROUND = "ground"
    """Rudder and nosewheel steering."""
    AIR = "air"
    """Ailerons."""
    TO_AIR = "to_air"
    """Handing over from steering to ailerons after takeoff."""
    TO_GROUND = "to_ground"
    """Handing back after touchdown."""

    @property
    def is_transition(self) -> bool:
        return self in (AxisMode.TO_AIR, AxisMode.TO_GROUND)


@dataclass(frozen=True, slots=True)
class TickContext:
    """Shared state handed to every effect module on every tick."""

    mode: AxisMode = AxisMode.GROUND
    ground_weight: float = 1.0
    """1.0 when the wheel is fully a rudder, 0.0 when fully ailerons.

    Modules fade themselves with this rather than testing the mode directly, so
    the handoff blends smoothly instead of switching effects on and off.
    """
    transition_progress: float = 0.0
    """0..1 through a handoff; 0 when not in one."""
    telemetry_stale: bool = False
    seconds: float = 0.0
    """Seconds since the bridge started."""

    @property
    def air_weight(self) -> float:
        return 1.0 - self.ground_weight
