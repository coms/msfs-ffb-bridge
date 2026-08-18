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

from ..core.config import BridgeConfig
from ..core.context import TickContext
from ..core.forces import Damper, ForceOutput, Periodic, Spring, Waveform
from ..core.modules.soft_lock import SoftLock
from ..core.telemetry import FlightTelemetry, WheelState


@dataclass(frozen=True, slots=True)
class BenchTest:
    """One thing to feel, and how to generate it."""

    id: str
    name: str
    description: str
    build: Callable[[float, WheelState, BridgeConfig], ForceOutput]
    """Given the time, where the wheel is and the profile, what to feel.

    Most tests use only the first: they are shapes in time. The soft lock is
    the exception, and the reason the other two are passed at all.
    """
    duration: float = 6.0


def _steady_left(t: float, wheel: WheelState, config: BridgeConfig) -> ForceOutput:
    return ForceOutput(constant=-0.4)


def _steady_right(t: float, wheel: WheelState, config: BridgeConfig) -> ForceOutput:
    return ForceOutput(constant=0.4)


def _slow_sweep(t: float, wheel: WheelState, config: BridgeConfig) -> ForceOutput:
    """A gentle push that crosses through zero, for checking direction and feel."""
    return ForceOutput(constant=0.5 * math.sin(t * 0.6))


def _centring(t: float, wheel: WheelState, config: BridgeConfig) -> ForceOutput:
    return ForceOutput(spring=Spring(coefficient=0.45, center=0.0, saturation=0.8))


def _trim_offset(t: float, wheel: WheelState, config: BridgeConfig) -> ForceOutput:
    """The neutral point walks off centre and back, which is what trim feels like."""
    return ForceOutput(
        spring=Spring(coefficient=0.45, center=0.4 * math.sin(t * 0.4), saturation=0.8)
    )


def _damping(t: float, wheel: WheelState, config: BridgeConfig) -> ForceOutput:
    return ForceOutput(damper=Damper(coefficient=0.5, saturation=0.7))


def _rumble_sweep(t: float, wheel: WheelState, config: BridgeConfig) -> ForceOutput:
    """Ground rumble from a crawl to a fast taxi, the range you feel most."""
    frequency = 2.0 + (t % 8.0) * 4.0
    return ForceOutput(
        periodics=(Periodic(label="bench_rumble", frequency_hz=frequency, magnitude=0.4),)
    )


def _engine_hum(t: float, wheel: WheelState, config: BridgeConfig) -> ForceOutput:
    return ForceOutput(
        periodics=(Periodic(label="bench_engine", frequency_hz=24.0, magnitude=0.3),)
    )


def _touchdown_thumps(t: float, wheel: WheelState, config: BridgeConfig) -> ForceOutput:
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


#: Telemetry the position-driven tests hand their module. The soft lock reads
#: nothing from it -- a control stop is a property of the wheel, not the flight.
NO_FLIGHT = FlightTelemetry()


class _SoftLockBench:
    """The real module, driven by where the rim actually is.

    Not an imitation of the stop but the stop itself, which is the point: it
    reads the rotation and the travel from the profile in front of you, so
    where you feel it is where it will be in the air.

    ``BenchTest.build`` is called fresh every tick, but the soft lock's release
    now depends on state carried from one tick to the next -- the deepest lean
    this visit to the wall has reached. A module built from scratch each call
    never remembers a peak, so it never sees itself as having backed off one,
    and the bench would feel exactly like the wall that never lets go. This
    keeps one module alive across a run, and only starts a new one when the
    clock goes backwards, which is what a fresh run of the test looks like.
    """

    def __init__(self) -> None:
        self._module: SoftLock | None = None
        self._last_t: float | None = None

    def __call__(self, t: float, wheel: WheelState, config: BridgeConfig) -> ForceOutput:
        if self._module is None or self._last_t is None or t <= self._last_t:
            self._module = SoftLock(config.module(SoftLock.id))
        self._last_t = t
        context = TickContext(
            wheel_rotation_deg=config.wheel.rotation_deg,
            soft_lock_deg=config.wheel.soft_lock_deg,
        )
        contribution = self._module.update(NO_FLIGHT, wheel, context, 1 / 100)
        return ForceOutput(constant=contribution.constant, damper=contribution.damper)


_soft_lock = _SoftLockBench()


def _everything(t: float, wheel: WheelState, config: BridgeConfig) -> ForceOutput:
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
        "switch on 'invert force' in the device settings. This is the constant force "
        "channel, which carries most of the force model including the soft lock, and "
        "which no Pit House switch can throw away. Start here: if this is silent, "
        "nothing else is worth testing.",
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
        "This is the spring channel: if you feel nothing at all here, the base is "
        "discarding it, and the game spring needs turning up in Pit House. If it feels "
        "heavier than expected, the base's own spring is still switched on.",
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
        "No centring force, but the wheel should feel like it is moving through oil. "
        "The damper channel, and like the spring it can be discarded by the base.",
        _damping,
    ),
    BenchTest(
        "softlock",
        "Soft lock",
        "Turn the wheel slowly to one side. Nothing at all until the stop, then a wall "
        "that firms up the further you lean on it. Where it arrives is what the profile "
        "says: with a 180 degree lock on a 1080 degree wheel, a quarter turn either way. "
        "If it pushes you further out instead of back, the force direction is inverted.",
        _soft_lock,
        duration=30.0,
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
