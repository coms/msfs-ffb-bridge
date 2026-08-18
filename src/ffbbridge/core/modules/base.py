"""The contract every effect module implements."""

from __future__ import annotations

from dataclasses import dataclass

from ..config import ModuleSettings
from ..context import TickContext
from ..forces import Contribution
from ..telemetry import FlightTelemetry, WheelState


@dataclass(frozen=True, slots=True)
class ParamSpec:
    """A tunable knob a module exposes.

    Declaring parameters rather than reading a config schema means the GUI can
    build its sliders automatically and a new module needs no changes anywhere
    else to become tunable.
    """

    name: str
    default: float
    minimum: float
    maximum: float
    label: str
    unit: str = ""

    def clamp(self, value: float) -> float:
        return min(max(value, self.minimum), self.maximum)


class EffectModule:
    """Base class for effects.

    Subclasses set the class attributes, declare any :class:`ParamSpec` knobs,
    and implement :meth:`update`.
    """

    #: Stable identifier used in config files. Never rename it lightly.
    id: str = ""
    #: Human-readable name for the GUI.
    name: str = ""
    #: One-line explanation shown as a tooltip.
    description: str = ""
    #: Higher priority wins when hardware effect slots run short.
    priority: int = 0
    #: Tunable knobs.
    params: tuple[ParamSpec, ...] = ()
    #: Whether the effect is on in a fresh profile.
    default_enabled: bool = True
    #: Exempt this module's steady force from the master strength.
    #:
    #: Master strength is a matter of taste: it makes the whole force model
    #: gentler or firmer to suit the person holding the wheel. A control stop is
    #: not a matter of taste -- softened to a third it is not a stop, it is a
    #: nudge you push through without noticing -- so the few effects that exist
    #: to say "no further" opt out of it. The safety ceiling and the fade-out
    #: envelope still apply, because those are safety rather than taste.
    ignores_master_gain: bool = False
    #: Starting strength in a fresh profile.
    default_gain: float = 1.0

    def __init__(self, settings: ModuleSettings | None = None) -> None:
        self.settings = settings if settings is not None else self.default_settings()
        self._param_defaults = {spec.name: spec.default for spec in self.params}
        self._param_specs = {spec.name: spec for spec in self.params}

    @classmethod
    def default_settings(cls) -> ModuleSettings:
        return ModuleSettings(enabled=cls.default_enabled, gain=cls.default_gain, params={})

    @property
    def enabled(self) -> bool:
        return self.settings.enabled

    @property
    def gain(self) -> float:
        return self.settings.gain

    def p(self, name: str) -> float:
        """Current value of a declared parameter, clamped to its allowed range."""
        spec = self._param_specs[name]
        return spec.clamp(self.settings.params.get(name, spec.default))

    def update(
        self,
        tel: FlightTelemetry,
        wheel: WheelState,
        ctx: TickContext,
        dt: float,
    ) -> Contribution:
        """Return this module's force contribution for the tick.

        ``dt`` is the elapsed time in seconds. Implementations must be pure with
        respect to everything except their own filter state.
        """
        raise NotImplementedError

    def reset(self) -> None:
        """Drop any internal filter state.

        Called when the sim disconnects or the aircraft changes, so that a
        touchdown detector does not fire on the first frame of a new flight.
        """
