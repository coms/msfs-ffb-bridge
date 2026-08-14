"""The telemetry snapshot the whole force model reads from.

One immutable record per sim frame. Every field has a safe default so tests can
build a partial state, and so a SimVar the aircraft does not implement simply
reads as zero rather than breaking the bridge.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

# Vertical acceleration in level flight, ft/s^2. Body accelerations are reported
# in ft/s^2 by SimConnect, so this is the reference for "one G".
G_FT_S2 = 32.174

KT_TO_FT_S = 1.68781


class SurfaceType(IntEnum):
    """MSFS ``SURFACE TYPE`` enumeration."""

    CONCRETE = 0
    GRASS = 1
    WATER = 2
    GRASS_BUMPY = 3
    ASPHALT = 4
    SHORT_GRASS = 5
    LONG_GRASS = 6
    HARD_TURF = 7
    SNOW = 8
    ICE = 9
    URBAN = 10
    FOREST = 11
    DIRT = 12
    CORAL = 13
    GRAVEL = 14
    OIL_TREATED = 15
    STEEL_MATS = 16
    BITUMINOUS = 17
    BRICK = 18
    MACADAM = 19
    PLANKS = 20
    SAND = 21
    SHALE = 22
    TARMAC = 23
    WRIGHT_FLYER_TRACK = 24

    @classmethod
    def from_raw(cls, value: float) -> SurfaceType:
        """Coerce a raw SimVar value, falling back to CONCRETE for anything unknown."""
        try:
            return cls(int(value))
        except ValueError:
            return cls.CONCRETE


class EngineType(IntEnum):
    """MSFS ``ENGINE TYPE`` enumeration."""

    PISTON = 0
    JET = 1
    NONE = 2
    HELO_TURBINE = 3
    UNSUPPORTED = 4
    TURBOPROP = 5

    @classmethod
    def from_raw(cls, value: float) -> EngineType:
        try:
            return cls(int(value))
        except ValueError:
            return cls.PISTON


@dataclass(frozen=True, slots=True)
class FlightTelemetry:
    """An immutable snapshot of the simulated aircraft.

    Units follow SimConnect's native choices so the mapping layer stays a plain
    table: knots for speeds, feet for altitude, radians for angles, ft/s^2 for
    body accelerations, radians/second for body rotation rates.
    """

    # --- Timing -----------------------------------------------------------
    t: float = 0.0
    """Seconds since the bridge started, taken at the moment the sample arrived."""

    # --- Sim state --------------------------------------------------------
    connected: bool = False
    paused: bool = False
    slew_active: bool = False
    in_cockpit: bool = True
    """False while a menu or an external camera is up; forces are cut when False."""

    # --- Aircraft identity ------------------------------------------------
    title: str = ""
    atc_model: str = ""
    is_helicopter: bool = False
    engine_type: EngineType = EngineType.PISTON
    num_engines: int = 1
    total_weight_lb: float = 0.0

    # Design speeds, in knots. VC (cruise) is the reference the control loading
    # model normalises against so one gain setting works across aircraft.
    design_speed_vc_kt: float = 0.0
    design_speed_vs0_kt: float = 0.0
    design_speed_vs1_kt: float = 0.0
    design_takeoff_speed_kt: float = 0.0

    # --- Air data ---------------------------------------------------------
    ias_kt: float = 0.0
    tas_kt: float = 0.0
    gs_kt: float = 0.0
    mach: float = 0.0
    dynamic_pressure_psf: float = 0.0
    alpha_rad: float = 0.0
    beta_rad: float = 0.0
    agl_ft: float = 0.0
    vs_fpm: float = 0.0
    g_force: float = 1.0
    stall_warning: bool = False
    overspeed_warning: bool = False

    # --- Control surfaces and trim ---------------------------------------
    # Positions are -1..1 as the sim sees them, which includes autopilot input.
    aileron_pos: float = 0.0
    elevator_pos: float = 0.0
    rudder_pos: float = 0.0
    aileron_trim_pct: float = 0.0
    rudder_trim_pct: float = 0.0
    elevator_trim_pct: float = 0.0

    brake_left: float = 0.0
    brake_right: float = 0.0
    parking_brake: bool = False

    flaps_pct: float = 0.0
    spoilers_pct: float = 0.0
    gear_pct: float = 1.0
    """Total gear extension, 0 retracted to 1 down."""

    # --- Ground contact ---------------------------------------------------
    on_ground: bool = True
    surface_type: SurfaceType = SurfaceType.CONCRETE
    contact_compression: tuple[float, ...] = ()
    """Per contact point, 0 unloaded to 1 fully compressed. Index 0 is the nose
    or tail wheel, 1 and 2 the mains, matching MSFS contact point ordering."""
    wheel_rpm_center: float = 0.0
    wheel_rpm_left: float = 0.0
    wheel_rpm_right: float = 0.0

    # --- Powerplant -------------------------------------------------------
    eng_rpm: tuple[float, ...] = ()
    prop_rpm: tuple[float, ...] = ()
    eng_combustion: tuple[bool, ...] = ()
    throttle_pct: tuple[float, ...] = ()

    # --- Motion -----------------------------------------------------------
    accel_body: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Body-frame acceleration in ft/s^2 as (lateral, vertical, longitudinal)."""
    rot_velocity_body: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Body-frame rotation rate in rad/s as (pitch, heading, bank)."""

    # --- Environment ------------------------------------------------------
    wind_velocity_kt: float = 0.0
    wind_direction_rad: float = 0.0
    """True direction the wind is coming *from*."""
    heading_true_rad: float = 0.0

    # --- Derived helpers --------------------------------------------------

    @property
    def nose_compression(self) -> float:
        """Compression of the nose or tail contact point, 0 when not reported."""
        return self.contact_compression[0] if self.contact_compression else 0.0

    @property
    def main_compression(self) -> float:
        """Greater of the two main gear compressions."""
        mains = self.contact_compression[1:3]
        return max(mains) if mains else 0.0

    @property
    def weight_on_wheels(self) -> bool:
        """True when the aircraft is carrying weight on any contact point.

        Falls back to ``on_ground`` for aircraft that do not report compression.
        """
        if self.contact_compression:
            return any(c > 0.02 for c in self.contact_compression)
        return self.on_ground

    @property
    def brake_input(self) -> float:
        """Greater of the two brake pedals, 0..1."""
        return max(self.brake_left, self.brake_right)

    @property
    def any_engine_running(self) -> bool:
        return any(self.eng_combustion)

    @property
    def max_prop_rpm(self) -> float:
        """Highest propeller RPM, falling back to engine RPM for jets."""
        if self.prop_rpm and max(self.prop_rpm) > 0.0:
            return max(self.prop_rpm)
        return max(self.eng_rpm) if self.eng_rpm else 0.0

    @property
    def max_throttle(self) -> float:
        return max(self.throttle_pct) if self.throttle_pct else 0.0

    @property
    def lateral_accel_g(self) -> float:
        return self.accel_body[0] / G_FT_S2

    @property
    def vertical_accel_g(self) -> float:
        return self.accel_body[1] / G_FT_S2

    @property
    def roll_rate(self) -> float:
        """Bank rate in rad/s."""
        return self.rot_velocity_body[2]

    @property
    def yaw_rate(self) -> float:
        """Heading rate in rad/s."""
        return self.rot_velocity_body[1]

    @property
    def crosswind_kt(self) -> float:
        """Crosswind component, positive when the wind comes from the right.

        This follows the way pilots talk about it -- "a ten knot crosswind from
        the right" -- rather than the direction the air is pushing, which is the
        opposite sign. Wind direction is reported as where the wind blows *from*,
        so a wind from 090 with the aircraft heading 360 gives ``+V``.
        """
        return self.wind_velocity_kt * math.sin(self.wind_direction_rad - self.heading_true_rad)

    @property
    def headwind_kt(self) -> float:
        """Headwind component, positive on the nose."""
        return self.wind_velocity_kt * math.cos(self.wind_direction_rad - self.heading_true_rad)

    def reference_speed_kt(self) -> float:
        """Speed used to normalise aerodynamic loading across aircraft.

        Prefers the design cruise speed, falls back to a multiple of the stall
        speed, and finally to a GA-sized constant so an aircraft that reports
        nothing still produces sane forces.
        """
        if self.design_speed_vc_kt > 20.0:
            return self.design_speed_vc_kt
        if self.design_speed_vs1_kt > 10.0:
            return self.design_speed_vs1_kt * 2.5
        return 120.0

    def q_ratio(self) -> float:
        """Dynamic pressure as a fraction of the value at the reference speed.

        Proportional to ``V^2``, so it doubles the control force for a 41% speed
        increase, which is how real control loading behaves.
        """
        ref = self.reference_speed_kt()
        if ref <= 0.0:
            return 0.0
        return (self.ias_kt / ref) ** 2


@dataclass(frozen=True, slots=True)
class WheelState:
    """What the physical wheel is doing, read straight from the device."""

    position: float = 0.0
    """Calibrated wheel position, -1 full left to 1 full right."""
    velocity: float = 0.0
    """Rate of change of ``position`` in units per second."""
    buttons: tuple[bool, ...] = field(default_factory=tuple)
    connected: bool = False
