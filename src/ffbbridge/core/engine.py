"""The tick loop that ties telemetry, routing and the force model together.

This is the whole bridge, minus the hardware. Give it a telemetry snapshot and a
wheel state and it returns the forces to apply and the axis commands to send,
which makes the entire behaviour of the product reproducible offline: the tests
and the replay tool drive exactly this class.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .config import BridgeConfig, ModuleSettings, ProfileSet
from .context import TickContext
from .forces import ForceOutput
from .mixer import EffectMixer
from .modules import MODULE_REGISTRY
from .modules.base import EffectModule
from .routing import AxisCommand, AxisRouter
from .telemetry import FlightTelemetry, WheelState


@dataclass(frozen=True, slots=True)
class EngineResult:
    """Everything one tick produced."""

    force: ForceOutput
    axis: AxisCommand
    context: TickContext
    dt: float = 0.0
    stale: bool = False
    profile_name: str = ""


@dataclass(slots=True)
class EngineStatus:
    """Coarse state for the GUI and the doctor."""

    profile_name: str = "Default GA"
    aircraft: str = ""
    aircraft_title: str = ""
    """Raw title, kept separate from the display name so it can be matched on."""
    aircraft_model: str = ""
    stale: bool = False
    ticks: int = 0
    module_errors: dict[str, str] = field(default_factory=dict)


class BridgeEngine:
    """Runs the force model at a fixed rate against whatever telemetry arrives."""

    #: Longest step the model will integrate in one go. A stalled loop or a
    #: debugger breakpoint must not produce a single enormous tick.
    MAX_DT = 0.1

    def __init__(
        self,
        profiles: ProfileSet | None = None,
        *,
        periodic_slots: int = 3,
    ) -> None:
        self.profiles = profiles if profiles is not None else ProfileSet()
        self._periodic_slots = periodic_slots
        self._aircraft_key = ""
        self.status = EngineStatus()

        self.config = self._resolve_config("", "")
        self.modules: list[EffectModule] = []
        self.mixer: EffectMixer | None = None
        self.router: AxisRouter | None = None
        self._build(self.config)

        self._last_now = 0.0
        self._last_sample_t = -1.0
        self._last_sample_wall = 0.0
        self._primed = False

    # --- Construction -----------------------------------------------------

    @staticmethod
    def default_module_settings() -> dict[str, ModuleSettings]:
        """Fresh settings for every registered module."""
        return {cls.id: cls.default_settings() for cls in MODULE_REGISTRY}

    def _resolve_config(self, title: str, atc_model: str) -> BridgeConfig:
        config = self.profiles.select(title, atc_model)
        return config.with_module_defaults(self.default_module_settings())

    def _build(self, config: BridgeConfig) -> None:
        """Instantiate modules, mixer and router for a configuration."""
        self.config = config
        self.modules = [cls(config.module(cls.id)) for cls in MODULE_REGISTRY]
        slots = config.device.periodic_slots or self._periodic_slots
        self.mixer = EffectMixer(self.modules, config.safety, periodic_slots=slots)
        self.router = AxisRouter(config.routing, config.wheel)
        self.status.profile_name = config.name

    def set_periodic_slots(self, slots: int) -> None:
        """Tell the engine how many hardware effect slots the device really has."""
        self._periodic_slots = max(0, slots)
        if self.mixer is not None and not self.config.device.periodic_slots:
            self.mixer.periodic_slots = self._periodic_slots

    def apply_config(self, config: BridgeConfig) -> None:
        """Swap in an edited configuration without dropping filter state.

        Gains and parameters are shared objects, so a slider move takes effect on
        the next tick. Only structural changes need a rebuild.
        """
        needs_rebuild = (
            config.name != self.config.name
            or set(config.modules) != set(self.config.modules)
            or config.routing != self.config.routing
            or config.wheel != self.config.wheel
        )
        if needs_rebuild:
            on_ground = self.router.ground_weight > 0.5 if self.router else True
            self._build(config)
            if self.router is not None:
                self.router.reset(on_ground=on_ground)
            return
        self.config = config
        for module in self.modules:
            module.settings = config.module(module.id)
        if self.mixer is not None:
            self.mixer.set_safety(config.safety)
            if config.device.periodic_slots:
                self.mixer.periodic_slots = config.device.periodic_slots

    def reset(self, *, on_ground: bool = True) -> None:
        """Cold start: used on connect, disconnect and aircraft change."""
        if self.mixer is not None:
            self.mixer.reset()
        if self.router is not None:
            self.router.reset(on_ground=on_ground)
        self._primed = False
        self._last_sample_t = -1.0

    # --- Tick -------------------------------------------------------------

    def tick(self, tel: FlightTelemetry, wheel: WheelState, now: float) -> EngineResult:
        """Advance the model. ``now`` is a monotonic wall clock in seconds."""
        assert self.mixer is not None and self.router is not None

        dt = self._elapsed(now)
        self._switch_aircraft_if_needed(tel)
        stale = self._is_stale(tel, now)

        axis = self.router.update(tel, wheel, dt)
        ctx = self.router.context(telemetry_stale=stale, seconds=now)
        force = self.mixer.update(tel, wheel, ctx, dt)

        if stale or not tel.connected:
            # Hold the axes where the sim last saw them rather than commanding
            # a surface deflection from telemetry we no longer trust.
            axis = AxisCommand(
                aileron=axis.aileron,
                rudder=axis.rudder,
                steering=axis.steering,
                mode=axis.mode,
                ground_weight=axis.ground_weight,
                override=axis.override,
            )

        self.status.stale = stale
        self.status.ticks += 1
        self.status.module_errors = dict(self.mixer.diagnostics.module_errors)

        return EngineResult(
            force=force,
            axis=axis,
            context=ctx,
            dt=dt,
            stale=stale,
            profile_name=self.config.name,
        )

    def _elapsed(self, now: float) -> float:
        if not self._primed:
            self._primed = True
            self._last_now = now
            return 0.0
        dt = now - self._last_now
        self._last_now = now
        if dt < 0.0:
            return 0.0
        return min(dt, self.MAX_DT)

    def _is_stale(self, tel: FlightTelemetry, now: float) -> bool:
        """True when telemetry has stopped arriving.

        Tracked from the sample timestamp rather than the connection flag, so a
        sim that has silently stopped feeding data is caught too.
        """
        if not tel.connected:
            return True
        if tel.t != self._last_sample_t:
            self._last_sample_t = tel.t
            self._last_sample_wall = now
            return False
        age_ms = (now - self._last_sample_wall) * 1000.0
        return age_ms > self.config.safety.watchdog_ms

    def _switch_aircraft_if_needed(self, tel: FlightTelemetry) -> None:
        key = f"{tel.title}|{tel.atc_model}"
        if key == self._aircraft_key:
            return
        self._aircraft_key = key
        self.status.aircraft = tel.title or tel.atc_model
        self.status.aircraft_title = tel.title
        self.status.aircraft_model = tel.atc_model
        config = self._resolve_config(tel.title, tel.atc_model)
        self._build(config)
        self.reset(on_ground=tel.on_ground)
