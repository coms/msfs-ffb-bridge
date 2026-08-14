"""Converting the force model's output into SDL haptic effect structures.

This is the seam between a model that talks in -1..1 and a device that talks in
16-bit integers, milliseconds and hundredths of a degree. It is deliberately
pure: no device handles, no SDL calls, nothing that needs hardware. That makes
the fiddliest and most error-prone part of the hardware path testable anywhere.

Two conventions worth stating, because they are easy to conflate:

**Force inversion** (``force_invert``) is for a wheelbase that pushes the
opposite way to what we asked. It applies to forces we command outright -- the
constant level and a periodic's offset -- and must *not* be applied to spring or
damper coefficients. Those are computed on the device from its own position and
velocity sensors, so they are always self-consistent in its own frame; negating
them would turn a centring spring into one that pushes the wheel to the stops.

**Axis inversion** (``axis_invert``) is for a device whose position axis reads
backwards from our convention. It applies to anything expressed as a *position*
in device units, which is the spring's centre point.
"""

from __future__ import annotations

import ctypes

from sdl2 import haptic as sdl_haptic

from ..core.forces import Damper, Periodic, Spring, Waveform

#: SDL uses signed 16-bit for force levels and magnitudes.
MAX_LEVEL = 32767

#: Periods are expressed in whole milliseconds in a 16-bit field, which sets
#: both the slowest tone we can ask for and the frequency resolution at the
#: fast end: 40 Hz and 41.7 Hz are both 24 ms.
MIN_PERIOD_MS = 1
MAX_PERIOD_MS = 65535

#: Phase is in hundredths of a degree.
PHASE_SCALE = 36000

#: SDL 2 dropped the square wave that DirectInput exposes, so anything asking
#: for one is rendered as a triangle: the closest available shape with the same
#: hard-edged character. The software synthesiser in the mixer still produces a
#: true square when an effect does not get a hardware slot.
WAVEFORM_TO_SDL: dict[Waveform, int] = {
    Waveform.SINE: sdl_haptic.SDL_HAPTIC_SINE,
    Waveform.SQUARE: sdl_haptic.SDL_HAPTIC_TRIANGLE,
    Waveform.TRIANGLE: sdl_haptic.SDL_HAPTIC_TRIANGLE,
    Waveform.SAWTOOTH_UP: sdl_haptic.SDL_HAPTIC_SAWTOOTHUP,
    Waveform.SAWTOOTH_DOWN: sdl_haptic.SDL_HAPTIC_SAWTOOTHDOWN,
}


def to_level(value: float) -> int:
    """Map a -1..1 force onto SDL's signed 16-bit range."""
    return int(round(max(-1.0, min(1.0, value)) * MAX_LEVEL))


def to_magnitude(value: float) -> int:
    """Map a 0..1 magnitude onto SDL's signed 16-bit range."""
    return int(round(max(0.0, min(1.0, value)) * MAX_LEVEL))


def to_unsigned(value: float) -> int:
    """Map a 0..1 value onto the unsigned range SDL uses for saturation."""
    return int(round(max(0.0, min(1.0, value)) * MAX_LEVEL))


def period_ms(frequency_hz: float) -> int:
    """Convert a frequency into the whole-millisecond period SDL wants."""
    if frequency_hz <= 0.0:
        return MAX_PERIOD_MS
    return max(MIN_PERIOD_MS, min(MAX_PERIOD_MS, int(round(1000.0 / frequency_hz))))


def sdl_waveform(waveform: Waveform, supported: int = 0xFFFFFFFF) -> int:
    """Pick the closest waveform the device actually implements.

    Every force feedback device supports a sine, so that is the last resort.
    """
    wanted = WAVEFORM_TO_SDL.get(waveform, sdl_haptic.SDL_HAPTIC_SINE)
    if supported & wanted:
        return wanted
    return sdl_haptic.SDL_HAPTIC_SINE


def _direction() -> sdl_haptic.SDL_HapticDirection:
    """A unit force along the device's first axis, which on a wheel is the rim.

    Cartesian rather than the steering-axis direction type: it is what wheels
    have understood since DirectInput, and the sign of the force we send then
    selects the side rather than the direction vector having to change.
    """
    direction = sdl_haptic.SDL_HapticDirection()
    direction.type = sdl_haptic.SDL_HAPTIC_CARTESIAN
    direction.dir[0] = 1
    direction.dir[1] = 0
    direction.dir[2] = 0
    return direction


def build_constant(level: float, *, force_invert: bool = False) -> sdl_haptic.SDL_HapticEffect:
    """A steady push, the channel most of the force model ends up on."""
    effect = sdl_haptic.SDL_HapticEffect()
    effect.type = sdl_haptic.SDL_HAPTIC_CONSTANT
    effect.constant.type = sdl_haptic.SDL_HAPTIC_CONSTANT
    effect.constant.direction = _direction()
    effect.constant.length = sdl_haptic.SDL_HAPTIC_INFINITY
    effect.constant.level = to_level(-level if force_invert else level)
    return effect


def build_periodic(
    periodic: Periodic,
    *,
    force_invert: bool = False,
    supported: int = 0xFFFFFFFF,
) -> sdl_haptic.SDL_HapticEffect:
    """A vibration, rendered by the device at its own internal rate."""
    waveform = sdl_waveform(periodic.waveform, supported)
    offset = -periodic.offset if force_invert else periodic.offset

    effect = sdl_haptic.SDL_HapticEffect()
    effect.type = waveform
    effect.periodic.type = waveform
    effect.periodic.direction = _direction()
    effect.periodic.length = sdl_haptic.SDL_HAPTIC_INFINITY
    effect.periodic.period = period_ms(periodic.frequency_hz)
    effect.periodic.magnitude = to_magnitude(periodic.magnitude)
    effect.periodic.offset = to_level(offset)
    effect.periodic.phase = 0
    return effect


def build_spring(spring: Spring, *, axis_invert: bool = False) -> sdl_haptic.SDL_HapticEffect:
    """A centring force the device computes from its own position sensor.

    The coefficient is never negated: the device always resolves a spring toward
    its centre in its own frame, and flipping the sign would make it push the
    wheel to the stops instead.
    """
    center = -spring.center if axis_invert else spring.center
    coefficient = to_magnitude(spring.coefficient)
    saturation = to_unsigned(spring.saturation)
    deadband = to_unsigned(spring.deadband)

    effect = sdl_haptic.SDL_HapticEffect()
    effect.type = sdl_haptic.SDL_HAPTIC_SPRING
    effect.condition.type = sdl_haptic.SDL_HAPTIC_SPRING
    effect.condition.direction = _direction()
    effect.condition.length = sdl_haptic.SDL_HAPTIC_INFINITY
    _fill_condition_axis(
        effect.condition,
        coefficient=coefficient,
        saturation=saturation,
        deadband=deadband,
        center=to_level(center),
    )
    return effect


def build_damper(damper: Damper) -> sdl_haptic.SDL_HapticEffect:
    """Resistance the device computes from how fast the wheel is being moved."""
    effect = sdl_haptic.SDL_HapticEffect()
    effect.type = sdl_haptic.SDL_HAPTIC_DAMPER
    effect.condition.type = sdl_haptic.SDL_HAPTIC_DAMPER
    effect.condition.direction = _direction()
    effect.condition.length = sdl_haptic.SDL_HAPTIC_INFINITY
    _fill_condition_axis(
        effect.condition,
        coefficient=to_magnitude(damper.coefficient),
        saturation=to_unsigned(damper.saturation),
        deadband=0,
        center=0,
    )
    return effect


def _fill_condition_axis(
    condition: sdl_haptic.SDL_HapticCondition,
    *,
    coefficient: int,
    saturation: int,
    deadband: int,
    center: int,
    axis: int = 0,
) -> None:
    """Populate one axis of a condition effect, symmetrically left and right.

    Condition effects carry three axes' worth of parameters. A wheel only has
    one, but SDL passes the whole array through to the driver, so the unused
    entries are left at zero rather than mirrored.
    """
    condition.right_sat[axis] = saturation
    condition.left_sat[axis] = saturation
    condition.right_coeff[axis] = coefficient
    condition.left_coeff[axis] = coefficient
    condition.deadband[axis] = deadband
    condition.center[axis] = center


def effects_differ(
    a: sdl_haptic.SDL_HapticEffect | None, b: sdl_haptic.SDL_HapticEffect | None
) -> bool:
    """Whether an effect actually changed, by comparing the raw bytes.

    Updating an effect is a USB round trip, so the loop only sends one when
    something moved. Millisecond period quantisation means small frequency
    changes often compare equal, which conveniently suppresses a lot of traffic.
    """
    if a is None or b is None:
        return a is not b
    return bytes(memoryview(a).cast("B")) != bytes(memoryview(b).cast("B"))


def effect_bytes(effect: sdl_haptic.SDL_HapticEffect) -> bytes:
    """The raw representation, used by the tests and by change detection."""
    return bytes(ctypes.string_at(ctypes.byref(effect), ctypes.sizeof(effect)))
