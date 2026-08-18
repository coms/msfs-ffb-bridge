"""Deciding what the single wheel axis controls, and handing over safely.

A racing wheel has one axis; an aeroplane wants a rudder on the ground and
ailerons in the air. This module owns that compromise.

The state is a single continuous number, ``ground_weight``, which slides between
1 (pure rudder and nosewheel steering) and 0 (pure ailerons). Modes are read off
it rather than stored separately, which makes reversals free: an aborted takeoff
or a bounced landing simply turns the number around wherever it happens to be,
with no special case and no discontinuity.

The handoff is deliberately not just a crossfade. Landing in a crosswind you are
holding aileron into wind at the moment the axis wants to become a rudder, and a
naive crossfade would quietly turn that into a bootful of rudder. Three things
guard against it:

* the incoming channel starts from the wheel position at handoff and only
  converges on absolute tracking as the transition completes;
* the axis command is rate limited, so nothing can snap;
* the force model is told to centre the wheel during the transition, so by the
  time the new axis has full authority the wheel is near neutral.

The third is the real protection, and it is why the handoff belongs here rather
than in the output layer: the router and the force model have to cooperate.
"""

from __future__ import annotations

from dataclasses import dataclass

from .config import RoutingConfig, WheelConfig
from .context import AxisMode, TickContext
from .filters import DwellTimer, SlewLimiter, clamp, deadband, expo
from .telemetry import FlightTelemetry, WheelState


class OverrideState:
    """Manual override cycled from a wheel button."""

    AUTO = "auto"
    FORCE_GROUND = "force_ground"
    FORCE_AIR = "force_air"

    ORDER = (AUTO, FORCE_GROUND, FORCE_AIR)

    @classmethod
    def next(cls, current: str) -> str:
        return cls.ORDER[(cls.ORDER.index(current) + 1) % len(cls.ORDER)]


@dataclass(frozen=True, slots=True)
class AxisCommand:
    """What to send to the sim this tick.

    Both channels are always populated. Leaving one unsent would freeze the
    sim's last value there, so an inactive axis is actively held at zero rather
    than abandoned.
    """

    aileron: float = 0.0
    rudder: float = 0.0
    steering: float | None = None
    mode: AxisMode = AxisMode.GROUND
    ground_weight: float = 1.0
    override: str = OverrideState.AUTO


class AxisRouter:
    """Maps wheel position onto the aileron and rudder axes."""

    #: Largest change in an axis command per second. Roughly full travel in a
    #: third of a second: fast enough to fly, slow enough that nothing snaps.
    MAX_AXIS_RATE = 3.0

    def __init__(self, routing: RoutingConfig, wheel_config: WheelConfig) -> None:
        self.routing = routing
        self.wheel_config = wheel_config
        pinned = self._fixed_target()
        start = 1.0 if pinned is None else pinned
        self._ground_weight = start
        self._target_ground = start
        self._handoff_reference = 0.0
        self._air_dwell = DwellTimer(routing.air_dwell_s)
        self._ground_dwell = DwellTimer(routing.ground_dwell_s)
        self._aileron_slew = SlewLimiter(self.MAX_AXIS_RATE)
        self._rudder_slew = SlewLimiter(self.MAX_AXIS_RATE)
        self._override = OverrideState.AUTO
        self._button_was_down = False

    # --- State ------------------------------------------------------------

    @property
    def ground_weight(self) -> float:
        return self._ground_weight

    @property
    def override(self) -> str:
        return self._override

    def set_override(self, override: str) -> None:
        if override in OverrideState.ORDER:
            self._override = override

    def reset(self, *, on_ground: bool = True) -> None:
        """Start again, assuming the aircraft is wherever it says it is.

        Called on connect and on aircraft change so the bridge does not think a
        flight starting on the runway has just landed. A pinned mode starts
        already pinned, so a wheel that is not meant to steer never gets a
        first second of rudder on the way down from a default of one.
        """
        pinned = self._fixed_target()
        if pinned is not None:
            self._ground_weight = pinned
        else:
            self._ground_weight = 1.0 if on_ground else 0.0
        self._target_ground = self._ground_weight
        self._handoff_reference = 0.0
        self._air_dwell.reset()
        self._ground_dwell.reset()
        self._aileron_slew.reset()
        self._rudder_slew.reset()

    def mode(self) -> AxisMode:
        """Current mode, derived from the blend and where it is heading."""
        if self._ground_weight >= 0.999:
            return AxisMode.GROUND
        if self._ground_weight <= 0.001:
            return AxisMode.AIR
        return AxisMode.TO_GROUND if self._target_ground > self._ground_weight else AxisMode.TO_AIR

    def context(self, *, telemetry_stale: bool, seconds: float) -> TickContext:
        """Build the per-tick context the effect modules read."""
        target = self._target_ground
        progress = 1.0 - abs(self._ground_weight - target)
        mode = self.mode()
        return TickContext(
            mode=mode,
            ground_weight=self._ground_weight,
            transition_progress=progress if mode.is_transition else 0.0,
            telemetry_stale=telemetry_stale,
            seconds=seconds,
            wheel_rotation_deg=self.wheel_config.rotation_deg,
            soft_lock_deg=self.wheel_config.soft_lock_deg,
        )

    # --- Update -----------------------------------------------------------

    def update(self, tel: FlightTelemetry, wheel: WheelState, dt: float) -> AxisCommand:
        self._poll_override_button(wheel)
        target = self._resolve_target(tel, dt)

        if target != self._target_ground:
            # A new handoff begins: remember where the wheel is so the incoming
            # axis can start from there instead of jumping to it.
            self._target_ground = target
            self._handoff_reference = wheel.position

        self._advance_blend(dt)

        raw = self._shape(wheel.position)
        gw = self._ground_weight

        ground_range = self._axis_range(self.wheel_config.ground_range)
        air_range = self._axis_range(self.wheel_config.air_range)
        rudder = self._outgoing_or_incoming(raw, ground_range, weight=gw, incoming=target > 0.5)
        aileron = self._outgoing_or_incoming(raw, air_range, weight=1.0 - gw, incoming=target < 0.5)

        aileron = self._aileron_slew.update(clamp(aileron), dt)
        rudder = self._rudder_slew.update(clamp(rudder), dt)

        return AxisCommand(
            aileron=aileron,
            rudder=rudder,
            steering=rudder if self.routing.use_tiller else None,
            mode=self.mode(),
            ground_weight=gw,
            override=self._override,
        )

    # --- Internals --------------------------------------------------------

    def _fixed_target(self) -> float | None:
        """The blend a pinned mode holds the axis at, or None when automatic.

        ``aileron_only`` is the wheel with the rudder taken off it: steering
        stays on the pedals, and the ground effects that are felt through the
        airframe rather than through the steering carry on regardless, because
        they key off weight on wheels rather than off this number.
        """
        if self.routing.mode == "aileron_only":
            return 0.0
        if self.routing.mode == "rudder_only":
            return 1.0
        return None

    def _resolve_target(self, tel: FlightTelemetry, dt: float) -> float:
        """Where the blend should be heading, in 0..1."""
        pinned = self._fixed_target()
        if pinned is not None:
            return pinned
        if self._override == OverrideState.FORCE_GROUND:
            return 1.0
        if self._override == OverrideState.FORCE_AIR:
            return 0.0

        airborne = not tel.weight_on_wheels and tel.agl_ft > self.routing.air_agl_ft
        # Both directions are dwell-gated, so a bounce on landing or a wheel
        # unloading over a bump cannot flip the axis back and forth.
        wants_air = self._air_dwell.update(airborne, dt)
        wants_ground = self._ground_dwell.update(tel.weight_on_wheels, dt)

        if wants_air:
            return 0.0
        if wants_ground:
            return 1.0
        return self._target_ground

    def _advance_blend(self, dt: float) -> None:
        transition_s = max(self.routing.transition_ms / 1000.0, 1e-3)
        step = dt / transition_s
        if self._target_ground > self._ground_weight:
            self._ground_weight = min(self._target_ground, self._ground_weight + step)
        else:
            self._ground_weight = max(self._target_ground, self._ground_weight - step)

    def _outgoing_or_incoming(
        self, raw: float, axis_range: float, *, weight: float, incoming: bool
    ) -> float:
        """Scale one channel's command by its share of the axis.

        The channel being handed control gets the reference-relative treatment;
        the one giving it up simply fades, keeping proportional authority right
        up to the moment it lets go.
        """
        if weight <= 1e-4:
            return 0.0
        value = raw / axis_range if axis_range > 0.0 else raw
        if incoming and weight < 1.0:
            reference = self._shape(self._handoff_reference)
            reference = reference / axis_range if axis_range > 0.0 else reference
            value -= reference * (1.0 - weight)
        return clamp(value) * weight

    def _axis_range(self, axis_range: float) -> float:
        """Travel that earns full deflection, never more than the soft lock allows.

        A soft lock narrower than the configured range would otherwise cost
        authority: the stop would arrive before full rudder did, and you would be
        pushing against the wall to get the last of your steering.
        """
        limit = self.wheel_config.soft_lock_fraction
        return min(axis_range, limit) if limit > 0.0 else axis_range

    def _shape(self, position: float) -> float:
        """Apply calibration to the raw wheel position."""
        cfg = self.wheel_config
        value = position - cfg.center
        if cfg.invert:
            value = -value
        value = deadband(clamp(value), cfg.deadzone)
        return expo(value, cfg.expo)

    def _poll_override_button(self, wheel: WheelState) -> None:
        index = self.routing.override_button
        if index < 0 or index >= len(wheel.buttons):
            self._button_was_down = False
            return
        down = wheel.buttons[index]
        if down and not self._button_was_down:
            self._override = OverrideState.next(self._override)
        self._button_was_down = down
