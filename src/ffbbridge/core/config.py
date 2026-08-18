"""Configuration and per-aircraft profiles.

Settings are plain JSON so they can be hand-edited, diffed and shared. Anything
the GUI can change lives here, and the GUI writes back through the same types.

This module deliberately knows nothing about which effect modules exist: the
engine passes in the defaults built from the module registry, so adding a module
never means editing a config schema.
"""

from __future__ import annotations

import copy
import fnmatch
import glob
import json
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from .filters import clamp


@dataclass(slots=True)
class ModuleSettings:
    """Per-effect user settings: on/off, strength, and parameter overrides."""

    enabled: bool = True
    gain: float = 1.0
    params: dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {"enabled": self.enabled, "gain": self.gain}
        if self.params:
            out["params"] = dict(self.params)
        return out

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ModuleSettings:
        return cls(
            enabled=bool(data.get("enabled", True)),
            gain=float(data.get("gain", 1.0)),
            params={str(k): float(v) for k, v in (data.get("params") or {}).items()},
        )

    def merged_with(self, other: ModuleSettings) -> ModuleSettings:
        """Overlay ``other`` on top of this one, used to apply a profile over defaults."""
        params = dict(self.params)
        params.update(other.params)
        return ModuleSettings(enabled=other.enabled, gain=other.gain, params=params)


@dataclass(slots=True)
class SafetyConfig:
    """Limits that apply after every effect has had its say.

    These exist because the wheel can produce 5.5 N-m at your wrists. Nothing in
    the force model is trusted to be well-behaved on its own.
    """

    master_gain: float = 0.7
    """Overall strength, 0..1. Deliberately below 1 out of the box."""
    max_force: float = 0.9
    """Hard ceiling on the constant-force channel."""
    max_slew_per_s: float = 8.0
    """Largest change in constant force per second; smooths telemetry glitches."""
    watchdog_ms: float = 500.0
    """Telemetry older than this counts as stale and forces start decaying."""
    decay_ms: float = 500.0
    """How long the fade to zero takes once telemetry is stale."""
    zero_when_paused: bool = True
    zero_when_not_in_cockpit: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "master_gain": self.master_gain,
            "max_force": self.max_force,
            "max_slew_per_s": self.max_slew_per_s,
            "watchdog_ms": self.watchdog_ms,
            "decay_ms": self.decay_ms,
            "zero_when_paused": self.zero_when_paused,
            "zero_when_not_in_cockpit": self.zero_when_not_in_cockpit,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> SafetyConfig:
        base = cls()
        return cls(
            master_gain=clamp(float(data.get("master_gain", base.master_gain)), 0.0, 1.0),
            max_force=clamp(float(data.get("max_force", base.max_force)), 0.0, 1.0),
            max_slew_per_s=max(0.0, float(data.get("max_slew_per_s", base.max_slew_per_s))),
            watchdog_ms=max(0.0, float(data.get("watchdog_ms", base.watchdog_ms))),
            decay_ms=max(1.0, float(data.get("decay_ms", base.decay_ms))),
            zero_when_paused=bool(data.get("zero_when_paused", base.zero_when_paused)),
            zero_when_not_in_cockpit=bool(
                data.get("zero_when_not_in_cockpit", base.zero_when_not_in_cockpit)
            ),
        )


@dataclass(slots=True)
class RoutingConfig:
    """How the single wheel axis is shared between rudder and ailerons."""

    mode: str = "auto"
    """One of ``auto``, ``aileron_only``, ``rudder_only``."""
    transition_ms: float = 1200.0
    """Length of the ground/air handoff, during which the wheel is walked to centre."""
    air_agl_ft: float = 15.0
    """Height that has to be cleared before the axis becomes ailerons."""
    air_dwell_s: float = 1.0
    """How long the airborne condition must hold before handing over."""
    ground_dwell_s: float = 0.7
    """How long weight-on-wheels must hold after touchdown; survives a bounce."""
    use_tiller: bool = False
    """Also drive ``AXIS_STEERING_SET`` for aircraft with a separate tiller."""
    override_button: int = -1
    """Wheel button that toggles the mode by hand; -1 disables."""
    send_rate_hz: float = 60.0
    axis_deadband: float = 0.0008
    """Smallest change worth transmitting, to keep the event rate sane."""

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "transition_ms": self.transition_ms,
            "air_agl_ft": self.air_agl_ft,
            "air_dwell_s": self.air_dwell_s,
            "ground_dwell_s": self.ground_dwell_s,
            "use_tiller": self.use_tiller,
            "override_button": self.override_button,
            "send_rate_hz": self.send_rate_hz,
            "axis_deadband": self.axis_deadband,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> RoutingConfig:
        base = cls()
        mode = str(data.get("mode", base.mode))
        if mode not in ("auto", "aileron_only", "rudder_only"):
            mode = base.mode
        return cls(
            mode=mode,
            transition_ms=max(0.0, float(data.get("transition_ms", base.transition_ms))),
            air_agl_ft=float(data.get("air_agl_ft", base.air_agl_ft)),
            air_dwell_s=max(0.0, float(data.get("air_dwell_s", base.air_dwell_s))),
            ground_dwell_s=max(0.0, float(data.get("ground_dwell_s", base.ground_dwell_s))),
            use_tiller=bool(data.get("use_tiller", base.use_tiller)),
            override_button=int(data.get("override_button", base.override_button)),
            send_rate_hz=max(1.0, float(data.get("send_rate_hz", base.send_rate_hz))),
            axis_deadband=max(0.0, float(data.get("axis_deadband", base.axis_deadband))),
        )


#: Lock-to-lock travel a wheelbase can plausibly be set to, in degrees.
#: The low end is a rally wheel, the high end is a drift base at full range.
ROTATION_MIN_DEG = 90.0
ROTATION_MAX_DEG = 2160.0

#: Ceiling on aileron_curve. 1 already matches the sharpest expo offers the
#: other way; this leaves room to go well past that for someone who wants it.
AILERON_CURVE_MAX = 5.0


@dataclass(slots=True)
class WheelConfig:
    """Calibration of the physical wheel as a flight control."""

    center: float = 0.0
    """Trim for a device that does not read exactly zero at centre."""
    deadzone: float = 0.01
    expo: float = 0.25
    """Softens the centre of travel; 0 is linear."""
    invert: bool = False
    air_range: float = 0.35
    """Fraction of full wheel travel that gives full aileron. 0.35 of a 540-degree
    wheel is about +/-95 degrees, which is a comfortable roll input."""
    aileron_curve: float = 0.0
    """Sharpens the centre of the aileron axis; 0 is linear, the opposite of
    ``expo``. Aileron only -- rudder and steering keep the shared ``expo`` curve.
    Capped at ``AILERON_CURVE_MAX``."""
    ground_range: float = 0.7
    """More travel for steering, where fine control at low speed matters."""
    rotation_deg: float = 540.0
    """Physical lock-to-lock travel the wheelbase is set to, in degrees.

    The bridge only ever sees -1..1, so this is what lets a setting written in
    degrees mean anything. It has to match what the wheel's own software is set
    to; nothing can read it back from the device.
    """
    soft_lock_deg: float = 0.0
    """Lock-to-lock travel the force model defends with an end stop; 0 disables.

    An aeroplane's controls stop somewhere, and a direct-drive base will happily
    keep turning. This is the rotation you want the aircraft to have, expressed
    the way wheel software expresses it: 180 means 90 degrees either side.
    """

    @property
    def soft_lock_fraction(self) -> float:
        """The soft lock as a fraction of travel each way; 0 when disabled.

        Both figures are lock-to-lock, so the ratio is already the half-travel
        fraction that axis units are measured in.
        """
        if self.soft_lock_deg <= 0.0 or self.rotation_deg <= 0.0:
            return 0.0
        return clamp(self.soft_lock_deg / self.rotation_deg, 0.0, 1.0)

    def to_dict(self) -> dict[str, Any]:
        return {
            "center": self.center,
            "deadzone": self.deadzone,
            "expo": self.expo,
            "invert": self.invert,
            "air_range": self.air_range,
            "aileron_curve": self.aileron_curve,
            "ground_range": self.ground_range,
            "rotation_deg": self.rotation_deg,
            "soft_lock_deg": self.soft_lock_deg,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> WheelConfig:
        base = cls()
        return cls(
            center=clamp(float(data.get("center", base.center))),
            deadzone=clamp(float(data.get("deadzone", base.deadzone)), 0.0, 0.5),
            expo=clamp(float(data.get("expo", base.expo)), 0.0, 1.0),
            invert=bool(data.get("invert", base.invert)),
            air_range=clamp(float(data.get("air_range", base.air_range)), 0.05, 1.0),
            aileron_curve=clamp(
                float(data.get("aileron_curve", base.aileron_curve)), 0.0, AILERON_CURVE_MAX
            ),
            ground_range=clamp(float(data.get("ground_range", base.ground_range)), 0.05, 1.0),
            rotation_deg=clamp(
                float(data.get("rotation_deg", base.rotation_deg)),
                ROTATION_MIN_DEG,
                ROTATION_MAX_DEG,
            ),
            soft_lock_deg=max(0.0, float(data.get("soft_lock_deg", base.soft_lock_deg))),
        )


@dataclass(slots=True)
class DeviceConfig:
    """Which wheel to open and how to talk to it."""

    name_match: str = "*MOZA*"
    """Glob against the SDL device name. Falls back to the first haptic device."""
    periodic_slots: int = 0
    """Hardware effect slots to use for vibration; 0 means ask the device."""
    invert_force: bool = False
    """Flip force direction if the wheel pushes the wrong way."""
    disable_autocenter: bool = True
    loop_hz: float = 100.0
    simconnect_dll: str = ""
    """Explicit path to SimConnect.dll; empty means auto-discover."""

    def to_dict(self) -> dict[str, Any]:
        return {
            "name_match": self.name_match,
            "periodic_slots": self.periodic_slots,
            "invert_force": self.invert_force,
            "disable_autocenter": self.disable_autocenter,
            "loop_hz": self.loop_hz,
            "simconnect_dll": self.simconnect_dll,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> DeviceConfig:
        base = cls()
        return cls(
            name_match=str(data.get("name_match", base.name_match)),
            periodic_slots=max(0, int(data.get("periodic_slots", base.periodic_slots))),
            invert_force=bool(data.get("invert_force", base.invert_force)),
            disable_autocenter=bool(data.get("disable_autocenter", base.disable_autocenter)),
            loop_hz=clamp(float(data.get("loop_hz", base.loop_hz)), 20.0, 500.0),
            simconnect_dll=str(data.get("simconnect_dll", base.simconnect_dll)),
        )


@dataclass(slots=True)
class BridgeConfig:
    """A complete profile: one aircraft or family's worth of settings."""

    name: str = "Default GA"
    match: list[str] = field(default_factory=list)
    """Globs tested against the aircraft title and ATC model. Empty is the fallback."""
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    routing: RoutingConfig = field(default_factory=RoutingConfig)
    wheel: WheelConfig = field(default_factory=WheelConfig)
    device: DeviceConfig = field(default_factory=DeviceConfig)
    modules: dict[str, ModuleSettings] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "match": list(self.match),
            "safety": self.safety.to_dict(),
            "routing": self.routing.to_dict(),
            "wheel": self.wheel.to_dict(),
            "device": self.device.to_dict(),
            "modules": {k: v.to_dict() for k, v in sorted(self.modules.items())},
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> BridgeConfig:
        return cls(
            name=str(data.get("name", "Unnamed")),
            match=[str(m) for m in data.get("match", [])],
            safety=SafetyConfig.from_dict(data.get("safety") or {}),
            routing=RoutingConfig.from_dict(data.get("routing") or {}),
            wheel=WheelConfig.from_dict(data.get("wheel") or {}),
            device=DeviceConfig.from_dict(data.get("device") or {}),
            modules={
                str(k): ModuleSettings.from_dict(v or {})
                for k, v in (data.get("modules") or {}).items()
            },
        )

    def module(self, module_id: str) -> ModuleSettings:
        """Settings for one module, creating defaults on first access."""
        settings = self.modules.get(module_id)
        if settings is None:
            settings = ModuleSettings()
            self.modules[module_id] = settings
        return settings

    def with_module_defaults(self, defaults: dict[str, ModuleSettings]) -> BridgeConfig:
        """Return a copy where every known module has settings.

        Values already present win, so a profile only has to state what it wants
        to change and stays readable as modules are added over time.
        """
        merged: dict[str, ModuleSettings] = {}
        for module_id, default in defaults.items():
            override = self.modules.get(module_id)
            merged[module_id] = default.merged_with(override) if override else default
        # Keep unknown entries so a profile written by a newer build survives a
        # round trip through an older one.
        for module_id, settings in self.modules.items():
            merged.setdefault(module_id, settings)
        return replace(self, modules=merged)

    def matches(self, title: str, atc_model: str) -> bool:
        if not self.match:
            return False
        haystack = (title or "").lower(), (atc_model or "").lower()
        return any(
            fnmatch.fnmatch(value, pattern.lower()) for pattern in self.match for value in haystack
        )


def aircraft_pattern(name: str) -> str:
    """A match pattern that means this aeroplane and no other.

    Aircraft titles are not glob-safe -- "Cessna 152 [G1000]" reads as a
    character class and would match nothing, least of all itself -- so the
    wildcards are escaped out of the name before it becomes a pattern.
    """
    return glob.escape(name.strip())


@dataclass(slots=True)
class ProfileSet:
    """A default profile plus any number of aircraft-specific overrides."""

    default: BridgeConfig = field(default_factory=BridgeConfig)
    profiles: list[BridgeConfig] = field(default_factory=list)

    def select(self, title: str, atc_model: str) -> BridgeConfig:
        """First matching profile wins; otherwise the default."""
        for profile in self.profiles:
            if profile.matches(title, atc_model):
                return profile
        return self.default

    def set_for_aircraft(
        self, config: BridgeConfig, title: str, atc_model: str = ""
    ) -> BridgeConfig:
        """Store settings against one aircraft and return what was stored.

        Saving the same aircraft again replaces its profile where it stands
        rather than growing a second one. A new profile goes to the front of the
        list, because the point of tuning one aeroplane is that it should win
        over the family profile that was covering it until now.

        What is stored is a snapshot. ``replace`` alone would leave the profile
        sharing its safety, wheel and module objects with the configuration the
        bridge is still editing, so every later slider move would silently
        rewrite a profile nobody asked to change.
        """
        name = title or atc_model
        pattern = aircraft_pattern(name)
        stored = replace(copy.deepcopy(config), name=name or config.name, match=[pattern])
        for index, existing in enumerate(self.profiles):
            if existing.match == [pattern]:
                self.profiles[index] = stored
                return stored
        self.profiles.insert(0, stored)
        return stored

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": 1,
            "default": self.default.to_dict(),
            "profiles": [p.to_dict() for p in self.profiles],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ProfileSet:
        return cls(
            default=BridgeConfig.from_dict(data.get("default") or {}),
            profiles=[BridgeConfig.from_dict(p) for p in data.get("profiles") or []],
        )

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> ProfileSet:
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    @classmethod
    def load_or_default(cls, path: str | Path) -> ProfileSet:
        """Load a profile file, falling back to defaults if it is missing or broken.

        A corrupt config should never stop the bridge from starting; the doctor
        reports the problem instead.
        """
        try:
            return cls.load(path)
        except (OSError, ValueError):
            return cls()
