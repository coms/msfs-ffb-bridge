"""The simulation variables the bridge subscribes to, and how they map across.

One flat table drives everything: the data definition sent to SimConnect, the
layout of the block that comes back, and the assembly of a
:class:`~ffbbridge.core.telemetry.FlightTelemetry`.

Two deliberate pieces of defensiveness, because an aircraft is free to implement
whatever subset of these it likes and unit names are easy to get subtly wrong:

* Any variable the simulator rejects is dropped from the definition and the
  layout rebuilt around it, so one bad entry cannot shift every value after it.
  The names of dropped variables are surfaced in the diagnostics rather than
  failing silently.
* Fields that should be fractions are normalised if they arrive as percentages.
  The distinction between the ``Percent`` and ``Percent Over 100`` units is a
  factor of a hundred, and getting it wrong would mean a wheel that thinks it is
  permanently at full brake.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from ..core.telemetry import EngineType, FlightTelemetry, SurfaceType

#: SimConnect data types. Everything here is a double, which keeps the returned
#: block a plain array and avoids hand-packing mixed structures.
DATATYPE_FLOAT64 = 4
DATATYPE_STRING256 = 9

#: Camera states from this value upward are menus, the world map or the hangar
#: rather than a view from which anyone is flying.
CAMERA_STATE_NOT_FLYING = 11

#: How many engines' worth of variables to request.
MAX_ENGINES = 4

#: How many landing gear contact points to request.
MAX_CONTACT_POINTS = 3


@dataclass(frozen=True, slots=True)
class SimVarSpec:
    """One simulation variable and where its value lands."""

    name: str
    unit: str
    key: str
    index: int | None = None
    """Set when the value is one element of a tuple field."""
    fraction: bool = False
    """Normalise a value that arrives as a percentage instead of a fraction."""

    @property
    def is_tuple(self) -> bool:
        return self.index is not None


def _engine_vars() -> tuple[SimVarSpec, ...]:
    specs: list[SimVarSpec] = []
    for engine in range(1, MAX_ENGINES + 1):
        i = engine - 1
        specs += [
            SimVarSpec(f"GENERAL ENG RPM:{engine}", "RPM", "eng_rpm", i),
            SimVarSpec(f"PROP RPM:{engine}", "RPM", "prop_rpm", i),
            SimVarSpec(f"ENG COMBUSTION:{engine}", "Bool", "eng_combustion", i),
            SimVarSpec(
                f"GENERAL ENG THROTTLE LEVER POSITION:{engine}",
                "Percent Over 100",
                "throttle_pct",
                i,
                fraction=True,
            ),
        ]
    return tuple(specs)


def _contact_point_vars() -> tuple[SimVarSpec, ...]:
    return tuple(
        SimVarSpec(
            f"CONTACT POINT COMPRESSION:{point}",
            "Percent Over 100",
            "contact_compression",
            point,
            fraction=True,
        )
        for point in range(MAX_CONTACT_POINTS)
    )


#: Every numeric variable, in the order they are added to the definition.
NUMERIC_VARS: tuple[SimVarSpec, ...] = (
    # Sim state
    SimVarSpec("SIM ON GROUND", "Bool", "on_ground"),
    SimVarSpec("IS SLEW ACTIVE", "Bool", "slew_active"),
    SimVarSpec("CAMERA STATE", "Enum", "camera_state"),
    # Air data
    SimVarSpec("AIRSPEED INDICATED", "Knots", "ias_kt"),
    SimVarSpec("AIRSPEED TRUE", "Knots", "tas_kt"),
    SimVarSpec("GROUND VELOCITY", "Knots", "gs_kt"),
    SimVarSpec("AIRSPEED MACH", "Mach", "mach"),
    SimVarSpec("DYNAMIC PRESSURE", "Pounds per square foot", "dynamic_pressure_psf"),
    SimVarSpec("INCIDENCE ALPHA", "Radians", "alpha_rad"),
    SimVarSpec("INCIDENCE BETA", "Radians", "beta_rad"),
    SimVarSpec("PLANE ALT ABOVE GROUND", "Feet", "agl_ft"),
    SimVarSpec("VERTICAL SPEED", "Feet per minute", "vs_fpm"),
    SimVarSpec("G FORCE", "GForce", "g_force"),
    SimVarSpec("STALL WARNING", "Bool", "stall_warning"),
    SimVarSpec("OVERSPEED WARNING", "Bool", "overspeed_warning"),
    SimVarSpec("PLANE HEADING DEGREES TRUE", "Radians", "heading_true_rad"),
    # Controls
    SimVarSpec("AILERON POSITION", "Position", "aileron_pos"),
    SimVarSpec("ELEVATOR POSITION", "Position", "elevator_pos"),
    SimVarSpec("RUDDER POSITION", "Position", "rudder_pos"),
    SimVarSpec("AILERON TRIM PCT", "Percent Over 100", "aileron_trim_pct", fraction=True),
    SimVarSpec("RUDDER TRIM PCT", "Percent Over 100", "rudder_trim_pct", fraction=True),
    SimVarSpec("ELEVATOR TRIM PCT", "Percent Over 100", "elevator_trim_pct", fraction=True),
    SimVarSpec("BRAKE LEFT POSITION", "Percent Over 100", "brake_left", fraction=True),
    SimVarSpec("BRAKE RIGHT POSITION", "Percent Over 100", "brake_right", fraction=True),
    SimVarSpec("BRAKE PARKING INDICATOR", "Bool", "parking_brake"),
    SimVarSpec("FLAPS HANDLE PERCENT", "Percent Over 100", "flaps_pct", fraction=True),
    SimVarSpec("SPOILERS HANDLE POSITION", "Percent Over 100", "spoilers_pct", fraction=True),
    SimVarSpec("GEAR TOTAL PCT EXTENDED", "Percent Over 100", "gear_pct", fraction=True),
    # Ground contact
    SimVarSpec("SURFACE TYPE", "Enum", "surface_type"),
    *_contact_point_vars(),
    SimVarSpec("CENTER WHEEL RPM", "RPM", "wheel_rpm_center"),
    SimVarSpec("LEFT WHEEL RPM", "RPM", "wheel_rpm_left"),
    SimVarSpec("RIGHT WHEEL RPM", "RPM", "wheel_rpm_right"),
    # Powerplant
    SimVarSpec("NUMBER OF ENGINES", "Number", "num_engines"),
    SimVarSpec("ENGINE TYPE", "Enum", "engine_type"),
    *_engine_vars(),
    # Motion
    SimVarSpec("ACCELERATION BODY X", "Feet per second squared", "accel_body", 0),
    SimVarSpec("ACCELERATION BODY Y", "Feet per second squared", "accel_body", 1),
    SimVarSpec("ACCELERATION BODY Z", "Feet per second squared", "accel_body", 2),
    SimVarSpec("ROTATION VELOCITY BODY X", "Radians per second", "rot_velocity_body", 0),
    SimVarSpec("ROTATION VELOCITY BODY Y", "Radians per second", "rot_velocity_body", 1),
    SimVarSpec("ROTATION VELOCITY BODY Z", "Radians per second", "rot_velocity_body", 2),
    # Environment
    SimVarSpec("AMBIENT WIND VELOCITY", "Knots", "wind_velocity_kt"),
    SimVarSpec("AMBIENT WIND DIRECTION", "Radians", "wind_direction_rad"),
    # Design data, used to scale forces across aircraft
    SimVarSpec("DESIGN SPEED VC", "Knots", "design_speed_vc_kt"),
    SimVarSpec("DESIGN SPEED VS0", "Knots", "design_speed_vs0_kt"),
    SimVarSpec("DESIGN SPEED VS1", "Knots", "design_speed_vs1_kt"),
    SimVarSpec("DESIGN TAKEOFF SPEED", "Knots", "design_takeoff_speed_kt"),
    SimVarSpec("TOTAL WEIGHT", "Pounds", "total_weight_lb"),
)

#: Which telemetry fields are tuples, and how long they are.
TUPLE_FIELDS: dict[str, int] = {
    "eng_rpm": MAX_ENGINES,
    "prop_rpm": MAX_ENGINES,
    "eng_combustion": MAX_ENGINES,
    "throttle_pct": MAX_ENGINES,
    "contact_compression": MAX_CONTACT_POINTS,
    "accel_body": 3,
    "rot_velocity_body": 3,
}

#: Fields that are booleans in the telemetry record.
BOOLEAN_FIELDS = frozenset(
    {"on_ground", "slew_active", "stall_warning", "overspeed_warning", "parking_brake"}
)

#: Fields consumed while assembling rather than assigned straight through.
DERIVED_ONLY = frozenset({"camera_state", "engine_type", "surface_type", "num_engines"})


def normalise(spec: SimVarSpec, value: float) -> float:
    """Rescale a fraction that arrived as a percentage.

    Only applied above 1.5 so a genuine fraction is never touched, and a control
    axis legitimately reaching 1.0 is left alone.
    """
    if spec.fraction and abs(value) > 1.5:
        return value / 100.0
    return value


def assemble(
    specs: Sequence[SimVarSpec],
    values: Sequence[float],
    *,
    t: float,
    title: str = "",
    atc_model: str = "",
    paused: bool = False,
) -> FlightTelemetry:
    """Turn a block of doubles into a telemetry snapshot.

    ``specs`` must be the definition actually in force, which after any
    rejections is not the same as :data:`NUMERIC_VARS`.
    """
    scalars: dict[str, float] = {}
    tuples: dict[str, list[float]] = {key: [0.0] * size for key, size in TUPLE_FIELDS.items()}

    for spec, raw in zip(specs, values, strict=False):
        value = normalise(spec, float(raw))
        if spec.is_tuple:
            slot = tuples.get(spec.key)
            if slot is not None and 0 <= spec.index < len(slot):
                slot[spec.index] = value
        else:
            scalars[spec.key] = value

    engine_count = int(scalars.get("num_engines", 1)) or 1
    engine_count = max(1, min(engine_count, MAX_ENGINES))
    engine_type = EngineType.from_raw(scalars.get("engine_type", 0))
    camera_state = scalars.get("camera_state", 2.0)

    fields: dict[str, object] = {
        key: value
        for key, value in scalars.items()
        if key not in DERIVED_ONLY and key not in BOOLEAN_FIELDS
    }
    for key in BOOLEAN_FIELDS:
        if key in scalars:
            fields[key] = bool(scalars[key])

    fields.update(
        t=t,
        connected=True,
        paused=paused,
        in_cockpit=camera_state < CAMERA_STATE_NOT_FLYING,
        title=title,
        atc_model=atc_model,
        num_engines=engine_count,
        engine_type=engine_type,
        is_helicopter=engine_type is EngineType.HELO_TURBINE,
        surface_type=SurfaceType.from_raw(scalars.get("surface_type", 0)),
        contact_compression=tuple(tuples["contact_compression"]),
        accel_body=tuple(tuples["accel_body"][:3]),
        rot_velocity_body=tuple(tuples["rot_velocity_body"][:3]),
        eng_rpm=tuple(tuples["eng_rpm"][:engine_count]),
        prop_rpm=tuple(tuples["prop_rpm"][:engine_count]),
        eng_combustion=tuple(bool(v) for v in tuples["eng_combustion"][:engine_count]),
        throttle_pct=tuple(tuples["throttle_pct"][:engine_count]),
    )
    return FlightTelemetry(**fields)  # type: ignore[arg-type]
