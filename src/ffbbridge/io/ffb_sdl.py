"""Opening the wheelbase and keeping its effect slots in order.

SDL has a hard rule that shapes the whole application: it must be initialised
and used from a single thread. Everything here therefore belongs to the force
loop's thread and must never be touched from the GUI.

Effect slots are a scarce, physical resource. The device advertises how many it
can play at once, and asking for one more than that fails at creation time
rather than degrading. So slots are allocated by label, reused in place while a
label stays active, and released as soon as it stops -- and the number the
device actually granted is reported back so the mixer can lower its budget.
"""

from __future__ import annotations

import fnmatch
import logging
import time
from dataclasses import dataclass, field

from sdl2 import (
    SDL_INIT_HAPTIC,
    SDL_INIT_JOYSTICK,
    SDL_GetError,
    SDL_InitSubSystem,
    SDL_JoystickClose,
    SDL_JoystickInstanceID,
    SDL_JoystickNameForIndex,
    SDL_JoystickNumAxes,
    SDL_JoystickNumButtons,
    SDL_JoystickOpen,
    SDL_NumJoysticks,
    SDL_Quit,
    SDL_QuitSubSystem,
)
from sdl2 import haptic as sdl_haptic

from ..core.forces import ForceOutput
from . import ffb_effects as fx

LOGGER = logging.getLogger(__name__)

#: Effects run until we stop them.
INFINITE = sdl_haptic.SDL_HAPTIC_INFINITY

#: Fixed slot names for the channels that are always present.
#: How long to leave a refused effect alone before offering it again.
#: Long enough not to hammer a device that means it, short enough that a
#: passing refusal costs a few seconds of feel rather than a whole flight.
RETRY_REFUSED_S = 5.0

CONSTANT_SLOT = "__constant__"
SPRING_SLOT = "__spring__"
DAMPER_SLOT = "__damper__"


class FfbError(RuntimeError):
    """Something went wrong talking to the device."""


def _refusal_hint(label: str) -> str:
    """Point at the setting that most often explains a refused condition effect.

    A wheelbase can be told to discard the spring and damper an application
    asks for, and then says only that it could not create the effect. The
    message that follows is worth more than the one the driver gave us.
    """
    if label in (SPRING_SLOT, DAMPER_SLOT):
        return (
            ". Wheelbases can be set to discard the spring and damper an application asks "
            "for - in MOZA Pit House this is the 'game spring' and 'game damper' setting, "
            "and at zero the centring force never reaches the motor"
        )
    return ""


def _sdl_error() -> str:
    error = SDL_GetError()
    return error.decode("utf-8", "replace") if error else "unknown SDL error"


@dataclass(frozen=True, slots=True)
class DeviceInfo:
    """A joystick SDL can see."""

    index: int
    name: str
    num_axes: int
    num_buttons: int
    is_haptic: bool


@dataclass(slots=True)
class HapticCapabilities:
    """What the wheelbase says it can do.

    Reported by the doctor, and used to pick waveforms the device actually
    implements rather than assuming.
    """

    supported: int = 0
    max_effects: int = 0
    max_playing: int = 0
    num_axes: int = 0

    def has(self, feature: int) -> bool:
        return bool(self.supported & feature)

    def describe(self) -> list[str]:
        """Human-readable feature list for the diagnostics panel."""
        features = {
            "constant force": sdl_haptic.SDL_HAPTIC_CONSTANT,
            "sine": sdl_haptic.SDL_HAPTIC_SINE,
            "triangle": sdl_haptic.SDL_HAPTIC_TRIANGLE,
            "sawtooth up": sdl_haptic.SDL_HAPTIC_SAWTOOTHUP,
            "sawtooth down": sdl_haptic.SDL_HAPTIC_SAWTOOTHDOWN,
            "spring": sdl_haptic.SDL_HAPTIC_SPRING,
            "damper": sdl_haptic.SDL_HAPTIC_DAMPER,
            "inertia": sdl_haptic.SDL_HAPTIC_INERTIA,
            "friction": sdl_haptic.SDL_HAPTIC_FRICTION,
            "ramp": sdl_haptic.SDL_HAPTIC_RAMP,
            "gain control": sdl_haptic.SDL_HAPTIC_GAIN,
            "autocentre control": sdl_haptic.SDL_HAPTIC_AUTOCENTER,
        }
        return [name for name, flag in features.items() if self.has(flag)]


def init_sdl() -> None:
    """Bring up the joystick and haptic subsystems.

    Video is deliberately not initialised: the bridge has no window of its own
    on this thread, and SDL is happy to drive joysticks without one.
    """
    if SDL_InitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC) != 0:
        raise FfbError(f"could not start SDL: {_sdl_error()}")


def shutdown_sdl() -> None:
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC)
    SDL_Quit()


def list_devices() -> list[DeviceInfo]:
    """Every joystick SDL can see, with whether it can produce forces."""
    devices: list[DeviceInfo] = []
    for index in range(SDL_NumJoysticks()):
        raw_name = SDL_JoystickNameForIndex(index)
        name = raw_name.decode("utf-8", "replace") if raw_name else f"device {index}"
        joystick = SDL_JoystickOpen(index)
        if not joystick:
            continue
        try:
            devices.append(
                DeviceInfo(
                    index=index,
                    name=name,
                    num_axes=SDL_JoystickNumAxes(joystick),
                    num_buttons=SDL_JoystickNumButtons(joystick),
                    is_haptic=bool(sdl_haptic.SDL_JoystickIsHaptic(joystick)),
                )
            )
        finally:
            SDL_JoystickClose(joystick)
    return devices


def select_device(devices: list[DeviceInfo], name_match: str) -> DeviceInfo | None:
    """Pick the wheel by name, falling back to any device that can make forces.

    The fallback matters: MOZA has shipped several names for the same base, and
    a user with one force feedback device plugged in should not have to discover
    what string to type.
    """
    pattern = (name_match or "*").lower()
    haptic = [d for d in devices if d.is_haptic]
    for device in haptic:
        if fnmatch.fnmatch(device.name.lower(), pattern):
            return device
    return haptic[0] if haptic else None


class HapticOutput:
    """Owns the device handle and the effects currently loaded on it."""

    def __init__(
        self,
        *,
        name_match: str = "*",
        force_invert: bool = False,
        axis_invert: bool = False,
        disable_autocenter: bool = True,
        slot_budget: int = 0,
    ) -> None:
        self.name_match = name_match
        self.force_invert = force_invert
        self.axis_invert = axis_invert
        self.disable_autocenter = disable_autocenter
        self.requested_budget = slot_budget

        self.device: DeviceInfo | None = None
        self.capabilities = HapticCapabilities()
        self.joystick = None
        self._haptic = None
        self._slots: dict[str, _Slot] = {}
        self._failed_labels: dict[str, float] = {}
        """Effects the device refused, and when, so they can be tried again.

        A refusal is not always permanent. Another application taking the wheel,
        or the device still settling after it was plugged in, refuses an effect
        that would be accepted a moment later -- and giving up for the rest of
        the session costs the centring spring and the damping for the whole
        flight, silently.
        """
        self._retry_after = RETRY_REFUSED_S
        self.periodic_slots = 0
        self.instance_id = -1

    # --- Lifecycle --------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._haptic is not None

    def open(self) -> DeviceInfo:
        """Find and open the wheel, or raise explaining what was found instead."""
        devices = list_devices()
        if not devices:
            raise FfbError("no game controllers found")

        chosen = select_device(devices, self.name_match)
        if chosen is None:
            names = ", ".join(d.name for d in devices) or "none"
            raise FfbError(f"no force feedback device found. Devices seen: {names}")

        joystick = SDL_JoystickOpen(chosen.index)
        if not joystick:
            raise FfbError(f"could not open {chosen.name}: {_sdl_error()}")

        haptic = sdl_haptic.SDL_HapticOpenFromJoystick(joystick)
        if not haptic:
            SDL_JoystickClose(joystick)
            raise FfbError(f"{chosen.name} would not open for force feedback: {_sdl_error()}")

        self.joystick = joystick
        self._haptic = haptic
        self.device = chosen
        self.instance_id = SDL_JoystickInstanceID(joystick)
        self._read_capabilities()
        self._configure()
        LOGGER.info(
            "opened %s: %d effect slots, features: %s",
            chosen.name,
            self.capabilities.max_playing,
            ", ".join(self.capabilities.describe()),
        )
        return chosen

    def _read_capabilities(self) -> None:
        self.capabilities = HapticCapabilities(
            supported=sdl_haptic.SDL_HapticQuery(self._haptic),
            max_effects=sdl_haptic.SDL_HapticNumEffects(self._haptic),
            max_playing=sdl_haptic.SDL_HapticNumEffectsPlaying(self._haptic),
            num_axes=sdl_haptic.SDL_HapticNumAxes(self._haptic),
        )
        # Reserve room for the constant force and the two condition effects,
        # then spend whatever is left on vibration.
        playable = max(self.capabilities.max_playing, self.capabilities.max_effects)
        available = max(0, playable - 3)
        budget = self.requested_budget or available
        self.periodic_slots = max(0, min(budget, available)) if available else 0

    def _configure(self) -> None:
        """Put the device into a state where our forces are the only forces."""
        wants_autocenter_off = self.disable_autocenter and self.capabilities.has(
            sdl_haptic.SDL_HAPTIC_AUTOCENTER
        )
        if wants_autocenter_off and sdl_haptic.SDL_HapticSetAutocenter(self._haptic, 0) != 0:
            LOGGER.warning("could not switch off autocentre: %s", _sdl_error())
        if (
            self.capabilities.has(sdl_haptic.SDL_HAPTIC_GAIN)
            and sdl_haptic.SDL_HapticSetGain(self._haptic, 100) != 0
        ):
            LOGGER.warning("could not set device gain: %s", _sdl_error())

    def close(self) -> None:
        """Release everything. Safe to call twice, and called on the way out."""
        if self._haptic is not None:
            try:
                self.stop_all()
            except Exception:  # never let teardown raise past here
                LOGGER.exception("error while stopping effects")
            sdl_haptic.SDL_HapticClose(self._haptic)
            self._haptic = None
        if self.joystick is not None:
            SDL_JoystickClose(self.joystick)
            self.joystick = None
        self._slots.clear()
        self._failed_labels.clear()

    def __enter__(self) -> HapticOutput:
        self.open()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    # --- Per-tick output --------------------------------------------------

    def apply(self, force: ForceOutput) -> None:
        """Push a tick's forces to the device, sending only what changed."""
        if self._haptic is None:
            return

        self._apply_constant(force)
        self._apply_spring(force)
        self._apply_damper(force)
        self._apply_periodics(force)

    def _apply_constant(self, force: ForceOutput) -> None:
        if not self.capabilities.has(sdl_haptic.SDL_HAPTIC_CONSTANT):
            return
        effect = fx.build_constant(force.constant, force_invert=self.force_invert)
        self._upsert(CONSTANT_SLOT, effect)

    def _apply_spring(self, force: ForceOutput) -> None:
        if not self.capabilities.has(sdl_haptic.SDL_HAPTIC_SPRING):
            return
        if force.spring is None or force.spring.coefficient <= 0.0:
            self._release(SPRING_SLOT)
            return
        self._upsert(SPRING_SLOT, fx.build_spring(force.spring, axis_invert=self.axis_invert))

    def _apply_damper(self, force: ForceOutput) -> None:
        if not self.capabilities.has(sdl_haptic.SDL_HAPTIC_DAMPER):
            return
        if force.damper is None or force.damper.coefficient <= 0.0:
            self._release(DAMPER_SLOT)
            return
        self._upsert(DAMPER_SLOT, fx.build_damper(force.damper))

    def _apply_periodics(self, force: ForceOutput) -> None:
        wanted = {p.label: p for p in force.periodics}
        for label in [k for k in self._slots if k not in wanted and not k.startswith("__")]:
            self._release(label)
        for label, periodic in wanted.items():
            effect = fx.build_periodic(
                periodic,
                force_invert=self.force_invert,
                supported=self.capabilities.supported,
            )
            self._upsert(label, effect)

    # --- Slot bookkeeping -------------------------------------------------

    def _upsert(self, label: str, effect) -> None:
        """Create, or update in place if the effect is already loaded."""
        slot = self._slots.get(label)

        if slot is not None and slot.effect_type == effect.type:
            if not fx.effects_differ(slot.effect, effect):
                return
            if sdl_haptic.SDL_HapticUpdateEffect(self._haptic, slot.effect_id, effect) != 0:
                LOGGER.debug("update of %s failed, recreating: %s", label, _sdl_error())
                self._release(label)
            else:
                slot.effect = effect
                return

        # SDL cannot change an effect's type in place, so a waveform change
        # means dropping the old one first.
        if slot is not None:
            self._release(label)

        refused_at = self._failed_labels.get(label)
        if refused_at is not None:
            if time.monotonic() - refused_at < self._retry_after:
                return
            del self._failed_labels[label]

        effect_id = sdl_haptic.SDL_HapticNewEffect(self._haptic, effect)
        if effect_id < 0:
            # Only the first refusal is worth a line; the retries repeat at the
            # retry interval and would fill the log with the same sentence.
            if label not in self._failed_labels:
                LOGGER.warning(
                    "device refused effect %r: %s%s", label, _sdl_error(), _refusal_hint(label)
                )
            self._failed_labels[label] = time.monotonic()
            self._shrink_budget()
            return
        if sdl_haptic.SDL_HapticRunEffect(self._haptic, effect_id, INFINITE) != 0:
            if label not in self._failed_labels:
                LOGGER.warning("could not start effect %r: %s", label, _sdl_error())
            sdl_haptic.SDL_HapticDestroyEffect(self._haptic, effect_id)
            self._failed_labels[label] = time.monotonic()
            return
        self._slots[label] = _Slot(effect_id=effect_id, effect_type=effect.type, effect=effect)

    def _release(self, label: str) -> None:
        slot = self._slots.pop(label, None)
        if slot is None:
            return
        sdl_haptic.SDL_HapticStopEffect(self._haptic, slot.effect_id)
        sdl_haptic.SDL_HapticDestroyEffect(self._haptic, slot.effect_id)

    def _shrink_budget(self) -> None:
        """Believe the device over its own advertised slot count.

        Wheels have been known to advertise more simultaneous effects than they
        will actually accept, so a refusal is treated as the real limit.
        """
        active = len([k for k in self._slots if not k.startswith("__")])
        if active < self.periodic_slots:
            self.periodic_slots = active
            LOGGER.info("reduced vibration slots to %d", self.periodic_slots)

    def stop_all(self) -> None:
        """Silence the wheel completely and give every slot back."""
        if self._haptic is None:
            return
        for label in list(self._slots):
            self._release(label)
        sdl_haptic.SDL_HapticStopAll(self._haptic)

    @property
    def active_effects(self) -> tuple[str, ...]:
        return tuple(sorted(self._slots))


@dataclass(slots=True)
class _Slot:
    """One effect currently loaded on the device."""

    effect_id: int
    effect_type: int
    effect: object = field(repr=False)
