"""Bench tests: prove the hardware without the simulator running.

Being able to feel each effect on its own, in isolation, answers the question
that otherwise takes a whole flight to answer -- is the wheel actually doing
what the bridge asked, and in the right direction? Run these first. If the
sweep feels smooth and left is left, everything after that is tuning.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from ..core.forces import Damper, ForceOutput, Periodic, Spring, Waveform


@dataclass(frozen=True, slots=True)
class BenchTest:
    """One thing to feel, and how to generate it."""

    id: str
    name: str
    description: str
    build: Callable[[float], ForceOutput]
    duration: float = 6.0


def _steady_left(t: float) -> ForceOutput:
    return ForceOutput(constant=-0.4)


def _steady_right(t: float) -> ForceOutput:
    return ForceOutput(constant=0.4)


def _slow_sweep(t: float) -> ForceOutput:
    """A gentle push that crosses through zero, for checking direction and feel."""
    return ForceOutput(constant=0.5 * math.sin(t * 0.6))


def _centring(t: float) -> ForceOutput:
    return ForceOutput(spring=Spring(coefficient=0.45, center=0.0, saturation=0.8))


def _trim_offset(t: float) -> ForceOutput:
    """The neutral point walks off centre and back, which is what trim feels like."""
    return ForceOutput(
        spring=Spring(coefficient=0.45, center=0.4 * math.sin(t * 0.4), saturation=0.8)
    )


def _damping(t: float) -> ForceOutput:
    return ForceOutput(damper=Damper(coefficient=0.5, saturation=0.7))


def _rumble_sweep(t: float) -> ForceOutput:
    """Ground rumble from a crawl to a fast taxi, the range you feel most."""
    frequency = 2.0 + (t % 8.0) * 4.0
    return ForceOutput(
        periodics=(Periodic(label="bench_rumble", frequency_hz=frequency, magnitude=0.4),)
    )


def _engine_hum(t: float) -> ForceOutput:
    return ForceOutput(
        periodics=(Periodic(label="bench_engine", frequency_hz=24.0, magnitude=0.3),)
    )


def _touchdown_thumps(t: float) -> ForceOutput:
    """A thump every two seconds, so you can judge how hard a landing should feel."""
    phase = t % 2.0
    if phase > 0.3:
        return ForceOutput()
    envelope = max(0.0, 1.0 - phase / 0.3)
    return ForceOutput(
        constant=0.35 * envelope,
        periodics=(
            Periodic(
                label="bench_touchdown",
                waveform=Waveform.SQUARE,
                frequency_hz=26.0,
                magnitude=0.7 * envelope,
            ),
        ),
    )


def _everything(t: float) -> ForceOutput:
    """All channels at once, to check the device can hold them simultaneously."""
    return ForceOutput(
        constant=0.15 * math.sin(t * 0.8),
        spring=Spring(coefficient=0.3, saturation=0.6),
        damper=Damper(coefficient=0.2),
        periodics=(
            Periodic(label="bench_a", frequency_hz=6.0, magnitude=0.25),
            Periodic(label="bench_b", frequency_hz=22.0, magnitude=0.2),
            Periodic(label="bench_c", frequency_hz=34.0, magnitude=0.15),
        ),
    )


BENCH_TESTS: tuple[BenchTest, ...] = (
    BenchTest(
        "left",
        "Steady push left",
        "The rim should pull steadily anticlockwise. If it goes the other way, "
        "switch on 'invert force' in the device settings.",
        _steady_left,
    ),
    BenchTest("right", "Steady push right", "The mirror image of the last one.", _steady_right),
    BenchTest(
        "sweep",
        "Slow sweep",
        "A push that eases from left to right and back. Should be smooth throughout, "
        "with no notchiness or sudden reversals.",
        _slow_sweep,
        duration=12.0,
    ),
    BenchTest(
        "centring",
        "Centring spring",
        "The wheel should return to centre when released, and resist being moved away. "
        "If it feels heavier than expected, MOZA's own spring is still switched on.",
        _centring,
    ),
    BenchTest(
        "trim",
        "Trim offset",
        "The rest position drifts off centre and back. This is how trim is expressed: "
        "the wheel wants to sit somewhere other than the middle.",
        _trim_offset,
        duration=16.0,
    ),
    BenchTest(
        "damping",
        "Damping",
        "No centring force, but the wheel should feel like it is moving through oil.",
        _damping,
    ),
    BenchTest(
        "rumble",
        "Runway rumble sweep",
        "A rumble climbing from a crawl to a fast taxi, repeatedly. This is the effect "
        "you will feel most, so set its strength here.",
        _rumble_sweep,
        duration=16.0,
    ),
    BenchTest("engine", "Engine vibration", "A steady hum, as at cruise power.", _engine_hum),
    BenchTest(
        "touchdown",
        "Touchdown thumps",
        "A firm arrival every two seconds. Should be a distinct knock, not a buzz.",
        _touchdown_thumps,
        duration=10.0,
    ),
    BenchTest(
        "everything",
        "Everything at once",
        "Constant force, spring, damper and three vibrations together. If effects drop "
        "out here, the wheel has fewer effect slots than it advertises.",
        _everything,
        duration=12.0,
    ),
)


def find_test(test_id: str) -> BenchTest | None:
    return next((test for test in BENCH_TESTS if test.id == test_id), None)
