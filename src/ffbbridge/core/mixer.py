"""Turns a pile of module contributions into one thing the wheel can render.

The device has a small, fixed budget: one spring, one damper, a handful of
periodic effect slots, and a constant force. The mixer spends that budget, then
enforces the safety limits.

Ordering matters and is not arbitrary:

1. Steady forces are summed, gained, clamped and **slew limited**.
2. Vibration that could not get a hardware slot is synthesised and added
   **after** the slew limiter. Rate limiting a 30 Hz rumble would erase it, so
   the limiter only ever sees the slowly-varying part of the signal.
3. The sum is clamped once more, so the ceiling still holds.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .config import SafetyConfig
from .context import TickContext
from .filters import PhaseAccumulator, SlewLimiter, clamp
from .forces import (
    Contribution,
    Damper,
    ForceOutput,
    Periodic,
    Spring,
    combine_dampers,
    combine_springs,
)
from .modules.base import EffectModule
from .telemetry import FlightTelemetry, WheelState


@dataclass(slots=True)
class MixDiagnostics:
    """What the mixer did this tick, for the GUI and the tests."""

    hardware_periodics: tuple[str, ...] = ()
    software_periodics: tuple[str, ...] = ()
    dropped_periodics: tuple[str, ...] = ()
    clipped: bool = False
    envelope: float = 1.0
    """Safety envelope applied this tick; below 1 while fading out."""
    module_errors: dict[str, str] = field(default_factory=dict)


class EffectMixer:
    """Runs the modules and spends the device's effect budget."""

    #: Vibration weaker than this is not worth a slot or a computation.
    AUDIBLE_FLOOR = 1e-3
    #: Vibration weaker than this yields its hardware slot to something stronger.
    #: A faint high-priority effect is better rendered in software than allowed
    #: to displace a rumble you can actually feel.
    SLOT_FLOOR = 0.02

    def __init__(
        self,
        modules: list[EffectModule],
        safety: SafetyConfig,
        *,
        periodic_slots: int = 3,
    ) -> None:
        self.modules = modules
        self.safety = safety
        self.periodic_slots = max(0, periodic_slots)
        self._slew = SlewLimiter(safety.max_slew_per_s)
        self._spring_slew = SlewLimiter(2.0)
        self._phases: dict[str, PhaseAccumulator] = {}
        self._envelope = 1.0
        self.diagnostics = MixDiagnostics()

    def reset(self) -> None:
        """Return to a cold state, as after a disconnect or aircraft change."""
        self._slew.reset()
        self._spring_slew.reset()
        self._phases.clear()
        self._envelope = 1.0
        for module in self.modules:
            module.reset()

    def set_safety(self, safety: SafetyConfig) -> None:
        """Apply new safety limits without dropping filter state."""
        self.safety = safety
        self._slew.max_rate = safety.max_slew_per_s

    def update(
        self,
        tel: FlightTelemetry,
        wheel: WheelState,
        ctx: TickContext,
        dt: float,
    ) -> ForceOutput:
        diagnostics = MixDiagnostics()

        target_envelope = self._target_envelope(tel, ctx)
        self._envelope = self._approach_envelope(target_envelope, dt)
        diagnostics.envelope = self._envelope

        constant = 0.0
        ungained = 0.0
        """Steady force from modules that opt out of the master strength."""
        springs: list[Spring] = []
        dampers: list[Damper] = []
        periodics: list[Periodic] = []
        breakdown: dict[str, float] = {}

        for module in self.modules:
            if not module.enabled:
                continue
            try:
                contribution = module.update(tel, wheel, ctx, dt)
            except Exception as exc:  # a bad module must not take the wheel with it
                diagnostics.module_errors[module.id] = repr(exc)
                module.reset()
                continue
            if contribution is None or contribution.is_empty:
                continue
            gain = module.gain
            if module.ignores_master_gain:
                ungained += contribution.constant * gain
            else:
                constant += contribution.constant * gain
            breakdown[module.id] = contribution.constant * gain
            if contribution.spring is not None:
                springs.append(_scale_spring(contribution.spring, gain))
            if contribution.damper is not None:
                dampers.append(_scale_damper(contribution.damper, gain))
            for periodic in contribution.periodics:
                scaled = periodic.scaled(gain)
                if scaled.is_audible:
                    periodics.append(scaled)

        master = clamp(self.safety.master_gain, 0.0, 1.0) * self._envelope

        # Steady channel: gained, clamped, then rate limited. A control stop
        # skips the master strength but not the envelope, so it still goes quiet
        # with everything else when the sim is paused or the telemetry stops.
        steady = constant * master + ungained * self._envelope
        steady = clamp(steady, -self.safety.max_force, self.safety.max_force)
        steady = self._slew.update(steady, dt)

        hardware, software = self._allocate_periodics(periodics)
        diagnostics.hardware_periodics = tuple(p.label for p in hardware)
        diagnostics.software_periodics = tuple(p.label for p in software)

        vibration = self._synthesise(software, master, dt)
        self._retire_unused_phases({p.label for p in software})

        total = steady + vibration
        clipped = abs(total) > self.safety.max_force
        total = clamp(total, -self.safety.max_force, self.safety.max_force)
        diagnostics.clipped = clipped

        spring = combine_springs(springs)
        if spring is not None:
            coefficient = self._spring_slew.update(spring.coefficient * master, dt)
            spring = Spring(
                coefficient=clamp(coefficient, 0.0, 1.0),
                center=spring.center,
                saturation=clamp(spring.saturation * master, 0.0, 1.0),
                deadband=spring.deadband,
            )
        else:
            self._spring_slew.update(0.0, dt)

        damper = combine_dampers(dampers)
        if damper is not None:
            damper = Damper(
                coefficient=clamp(damper.coefficient * master, 0.0, 1.0),
                saturation=clamp(damper.saturation * master, 0.0, 1.0),
            )

        hardware_scaled = tuple(
            Periodic(
                label=p.label,
                waveform=p.waveform,
                frequency_hz=p.frequency_hz,
                magnitude=clamp(p.magnitude * master, 0.0, 1.0),
                offset=clamp(p.offset * master),
                priority=p.priority,
            )
            for p in hardware
        )

        self.diagnostics = diagnostics
        return ForceOutput(
            constant=total,
            spring=spring,
            damper=damper,
            periodics=hardware_scaled,
            breakdown=breakdown,
            clipped=clipped,
        )

    # --- Safety envelope --------------------------------------------------

    def _target_envelope(self, tel: FlightTelemetry, ctx: TickContext) -> float:
        """1.0 when forces are welcome, 0.0 when they must not be produced."""
        if ctx.telemetry_stale or not tel.connected:
            return 0.0
        if self.safety.zero_when_paused and (tel.paused or tel.slew_active):
            return 0.0
        if self.safety.zero_when_not_in_cockpit and not tel.in_cockpit:
            return 0.0
        return 1.0

    def _approach_envelope(self, target: float, dt: float) -> float:
        """Fade toward the target over ``decay_ms`` rather than stepping.

        Cutting forces instantly would feel like the wheel had been dropped;
        fading also means a single late telemetry frame is not alarming.
        """
        if dt <= 0.0:
            return self._envelope
        step = dt / max(self.safety.decay_ms / 1000.0, 1e-3)
        if target > self._envelope:
            return min(target, self._envelope + step)
        return max(target, self._envelope - step)

    # --- Periodic slot budget --------------------------------------------

    def _allocate_periodics(
        self, periodics: list[Periodic]
    ) -> tuple[list[Periodic], list[Periodic]]:
        """Split vibrations into device-rendered and software-rendered.

        Highest priority first, strongest as the tiebreak, so a touchdown thump
        never loses its slot to a background engine hum. Effects too faint to
        notice are considered only once everything substantial has a slot.
        """
        audible = [p for p in periodics if p.magnitude > self.AUDIBLE_FLOOR]
        if not audible:
            return [], []
        strong = [p for p in audible if p.magnitude >= self.SLOT_FLOOR]
        faint = [p for p in audible if p.magnitude < self.SLOT_FLOOR]
        strong.sort(key=lambda p: (p.priority, p.magnitude), reverse=True)
        faint.sort(key=lambda p: (p.priority, p.magnitude), reverse=True)
        ordered = strong + faint
        return ordered[: self.periodic_slots], ordered[self.periodic_slots :]

    def _synthesise(self, periodics: list[Periodic], master: float, dt: float) -> float:
        """Render overflow vibration into the constant channel.

        Phase is integrated per label so a tone that changes frequency, such as
        ground rumble tracking groundspeed, glides instead of clicking.
        """
        total = 0.0
        for periodic in periodics:
            phase = self._phases.get(periodic.label)
            if phase is None:
                phase = PhaseAccumulator()
                self._phases[periodic.label] = phase
            phase.update(periodic.frequency_hz, dt)
            sample = periodic.waveform.sample(phase.phase)
            total += (sample * periodic.magnitude + periodic.offset) * master
        return total

    def _retire_unused_phases(self, active_labels: set[str]) -> None:
        if len(self._phases) <= len(active_labels):
            return
        for label in [k for k in self._phases if k not in active_labels]:
            del self._phases[label]


def _scale_spring(spring: Spring, gain: float) -> Spring:
    return Spring(
        coefficient=clamp(spring.coefficient * gain, 0.0, 1.0),
        center=spring.center,
        saturation=clamp(spring.saturation * gain, 0.0, 1.0),
        deadband=spring.deadband,
    )


def _scale_damper(damper: Damper, gain: float) -> Damper:
    return Damper(
        coefficient=clamp(damper.coefficient * gain, 0.0, 1.0),
        saturation=clamp(damper.saturation * gain, 0.0, 1.0),
    )


def contributions_of(
    modules: list[EffectModule],
    tel: FlightTelemetry,
    wheel: WheelState,
    ctx: TickContext,
    dt: float,
) -> dict[str, Contribution]:
    """Run modules and return their raw contributions, for tests and the bench."""
    return {m.id: m.update(tel, wheel, ctx, dt) for m in modules if m.enabled}
