"""A scripted flight, so the force model can be exercised without a simulator.

This is the workhorse of the test suite. It flies a complete light-aircraft
sortie -- start, taxi, run-up, takeoff, climb, cruise, approach, flare,
touchdown, rollout, shutdown -- and produces the same telemetry the SimConnect
layer would. Every ground and touchdown effect gets driven through its real
trigger conditions, in order, on every test run.
"""

from __future__ import annotations

import math
from collections.abc import Iterator
from dataclasses import dataclass

from .telemetry import G_FT_S2, FlightTelemetry, SurfaceType


@dataclass(frozen=True, slots=True)
class Keyframe:
    """A moment in the scripted flight. Scalars interpolate, flags step."""

    t: float
    gs_kt: float = 0.0
    ias_kt: float = 0.0
    agl_ft: float = 0.0
    vs_fpm: float = 0.0
    rpm: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    flaps: float = 0.0
    gear: float = 1.0
    on_ground: bool = True
    running: bool = False
    surface: SurfaceType = SurfaceType.CONCRETE


#: A Cessna-sized circuit. Times are seconds from engine start.
DEFAULT_SCRIPT: tuple[Keyframe, ...] = (
    Keyframe(0.0),
    Keyframe(3.0, rpm=300.0, throttle=0.15),
    Keyframe(6.0, rpm=750.0, throttle=0.15, running=True),
    Keyframe(12.0, rpm=800.0, throttle=0.15, running=True),
    # Taxi out, including a stretch of grass to feel the difference.
    Keyframe(20.0, gs_kt=12.0, ias_kt=8.0, rpm=1000.0, throttle=0.25, running=True),
    Keyframe(
        32.0,
        gs_kt=14.0,
        ias_kt=9.0,
        rpm=1000.0,
        throttle=0.25,
        running=True,
        surface=SurfaceType.GRASS,
    ),
    Keyframe(
        40.0,
        gs_kt=10.0,
        ias_kt=7.0,
        rpm=1000.0,
        throttle=0.2,
        running=True,
        surface=SurfaceType.GRASS,
    ),
    # Hold short, run-up against the brakes.
    Keyframe(46.0, gs_kt=0.0, ias_kt=0.0, rpm=1000.0, throttle=0.2, brake=1.0, running=True),
    Keyframe(52.0, gs_kt=0.0, ias_kt=0.0, rpm=1800.0, throttle=0.6, brake=1.0, running=True),
    Keyframe(58.0, gs_kt=0.0, ias_kt=0.0, rpm=1000.0, throttle=0.2, brake=1.0, running=True),
    # Takeoff roll.
    Keyframe(62.0, gs_kt=0.0, ias_kt=0.0, rpm=2500.0, throttle=1.0, running=True),
    Keyframe(78.0, gs_kt=55.0, ias_kt=55.0, rpm=2650.0, throttle=1.0, running=True),
    Keyframe(82.0, gs_kt=62.0, ias_kt=62.0, rpm=2650.0, throttle=1.0, running=True),
    # Rotate and climb away.
    Keyframe(
        84.0,
        gs_kt=66.0,
        ias_kt=66.0,
        agl_ft=8.0,
        vs_fpm=350.0,
        rpm=2650.0,
        throttle=1.0,
        on_ground=False,
        running=True,
    ),
    Keyframe(
        95.0,
        gs_kt=78.0,
        ias_kt=78.0,
        agl_ft=500.0,
        vs_fpm=700.0,
        rpm=2600.0,
        throttle=1.0,
        gear=1.0,
        on_ground=False,
        running=True,
    ),
    Keyframe(
        140.0,
        gs_kt=90.0,
        ias_kt=88.0,
        agl_ft=2800.0,
        vs_fpm=650.0,
        rpm=2500.0,
        throttle=0.95,
        on_ground=False,
        running=True,
    ),
    # Cruise.
    Keyframe(
        155.0,
        gs_kt=118.0,
        ias_kt=112.0,
        agl_ft=3000.0,
        vs_fpm=0.0,
        rpm=2350.0,
        throttle=0.72,
        on_ground=False,
        running=True,
    ),
    Keyframe(
        210.0,
        gs_kt=120.0,
        ias_kt=114.0,
        agl_ft=3000.0,
        vs_fpm=0.0,
        rpm=2350.0,
        throttle=0.72,
        on_ground=False,
        running=True,
    ),
    # Descend, configure.
    Keyframe(
        240.0,
        gs_kt=105.0,
        ias_kt=100.0,
        agl_ft=1800.0,
        vs_fpm=-600.0,
        rpm=2100.0,
        throttle=0.45,
        on_ground=False,
        running=True,
    ),
    Keyframe(
        255.0,
        gs_kt=88.0,
        ias_kt=85.0,
        agl_ft=1200.0,
        vs_fpm=-550.0,
        rpm=1900.0,
        throttle=0.35,
        flaps=0.5,
        on_ground=False,
        running=True,
    ),
    Keyframe(
        268.0,
        gs_kt=75.0,
        ias_kt=72.0,
        agl_ft=700.0,
        vs_fpm=-500.0,
        rpm=1800.0,
        throttle=0.3,
        flaps=1.0,
        on_ground=False,
        running=True,
    ),
    Keyframe(
        282.0,
        gs_kt=68.0,
        ias_kt=66.0,
        agl_ft=120.0,
        vs_fpm=-450.0,
        rpm=1700.0,
        throttle=0.25,
        flaps=1.0,
        on_ground=False,
        running=True,
    ),
    # Flare.
    Keyframe(
        286.0,
        gs_kt=62.0,
        ias_kt=60.0,
        agl_ft=12.0,
        vs_fpm=-300.0,
        rpm=1500.0,
        throttle=0.1,
        flaps=1.0,
        on_ground=False,
        running=True,
    ),
    Keyframe(
        288.0,
        gs_kt=58.0,
        ias_kt=56.0,
        agl_ft=2.0,
        vs_fpm=-160.0,
        rpm=1200.0,
        throttle=0.05,
        flaps=1.0,
        on_ground=False,
        running=True,
    ),
    # Still descending as the wheels arrive: contact is what stops the descent,
    # not the flare, and the touchdown effect depends on that being modelled.
    Keyframe(
        288.55,
        gs_kt=56.5,
        ias_kt=54.5,
        agl_ft=0.2,
        vs_fpm=-145.0,
        rpm=1100.0,
        throttle=0.0,
        flaps=1.0,
        on_ground=False,
        running=True,
    ),
    # Touchdown and rollout.
    Keyframe(
        288.6,
        gs_kt=56.0,
        ias_kt=54.0,
        agl_ft=0.0,
        vs_fpm=0.0,
        rpm=1000.0,
        throttle=0.0,
        flaps=1.0,
        on_ground=True,
        running=True,
    ),
    Keyframe(
        300.0, gs_kt=30.0, ias_kt=28.0, rpm=900.0, throttle=0.0, brake=0.7, flaps=1.0, running=True
    ),
    Keyframe(308.0, gs_kt=8.0, ias_kt=6.0, rpm=900.0, throttle=0.0, brake=0.5, running=True),
    # Taxi in and shut down.
    Keyframe(316.0, gs_kt=10.0, ias_kt=7.0, rpm=1000.0, throttle=0.2, running=True),
    Keyframe(326.0, gs_kt=0.0, ias_kt=0.0, rpm=900.0, throttle=0.15, brake=0.9, running=True),
    Keyframe(332.0, gs_kt=0.0, ias_kt=0.0, rpm=200.0, throttle=0.0, brake=1.0, running=False),
    Keyframe(336.0, gs_kt=0.0, ias_kt=0.0, rpm=0.0, throttle=0.0, brake=1.0, running=False),
)

#: Times the script is designed to produce a specific event, for the tests.
SCRIPT_EVENTS = {
    "engine_start": 6.0,
    "grass_taxi": 34.0,
    "takeoff_roll": 70.0,
    "liftoff": 84.0,
    "cruise": 180.0,
    "gear_down": 255.0,
    "touchdown": 288.6,
    "rollout_braking": 300.0,
    "shutdown": 334.0,
}


@dataclass(slots=True)
class SyntheticFlight:
    """Turns the keyframe script into a stream of telemetry samples."""

    script: tuple[Keyframe, ...] = DEFAULT_SCRIPT
    rate_hz: float = 60.0
    title: str = "Cessna Skyhawk G1000 Asobo"
    atc_model: str = "C172"
    wind_kt: float = 8.0
    wind_from_rad: float = math.radians(240.0)
    heading_rad: float = math.radians(270.0)
    turbulence: float = 0.35
    """Scales the synthetic gust content, 0 for perfectly smooth air."""

    @property
    def duration(self) -> float:
        return self.script[-1].t

    def sample_at(self, t: float) -> FlightTelemetry:
        """Build the telemetry for one instant."""
        before, after, blend = self._bracket(t)
        gs = _lerp(before.gs_kt, after.gs_kt, blend)
        ias = _lerp(before.ias_kt, after.ias_kt, blend)
        agl = _lerp(before.agl_ft, after.agl_ft, blend)
        vs = _lerp(before.vs_fpm, after.vs_fpm, blend)
        rpm = _lerp(before.rpm, after.rpm, blend)
        throttle = _lerp(before.throttle, after.throttle, blend)
        brake = _lerp(before.brake, after.brake, blend)
        flaps = _lerp(before.flaps, after.flaps, blend)
        gear = _lerp(before.gear, after.gear, blend)

        on_ground = before.on_ground
        running = before.running
        surface = before.surface

        # Longitudinal acceleration from the speed the script is asking for.
        ahead = self.sample_scalar(t + 0.25, "gs_kt")
        behind = self.sample_scalar(t - 0.25, "gs_kt")
        long_accel = (ahead - behind) / 0.5 * 1.68781

        gust = self._gust(t) * self.turbulence * (0.0 if on_ground else 1.0)
        lateral = gust * 4.0
        vertical = G_FT_S2 + gust * 6.0
        roll_rate = gust * 0.12

        # Gear compression eases off as the wings take the weight. It is capped
        # short of zero: right up to rotation the wheels are still in contact,
        # and a model that reports no contact at all while rolling would look
        # like the aircraft had already flown.
        if on_ground:
            lift_fraction = min(0.9, (ias / 62.0) ** 2) if ias > 0 else 0.0
            load = max(0.0, 1.0 - lift_fraction)
            compression = (0.45 * load, 0.6 * load, 0.6 * load)
        else:
            compression = (0.0, 0.0, 0.0)

        return FlightTelemetry(
            t=t,
            connected=True,
            title=self.title,
            atc_model=self.atc_model,
            num_engines=1,
            design_speed_vc_kt=120.0,
            design_speed_vs0_kt=40.0,
            design_speed_vs1_kt=48.0,
            design_takeoff_speed_kt=62.0,
            total_weight_lb=2300.0,
            ias_kt=ias,
            tas_kt=ias * 1.03,
            gs_kt=gs,
            mach=ias / 660.0,
            dynamic_pressure_psf=0.00339 * ias * ias,
            alpha_rad=_alpha_for(ias, on_ground),
            beta_rad=gust * 0.05,
            agl_ft=agl,
            vs_fpm=vs,
            g_force=vertical / G_FT_S2,
            stall_warning=(not on_ground) and 0 < ias < 47.0,
            aileron_trim_pct=0.0,
            rudder_trim_pct=0.02,
            brake_left=brake,
            brake_right=brake,
            parking_brake=False,
            flaps_pct=flaps,
            gear_pct=gear,
            on_ground=on_ground,
            surface_type=surface,
            contact_compression=compression,
            wheel_rpm_center=gs * 12.0 if on_ground else 0.0,
            eng_rpm=(rpm,),
            prop_rpm=(rpm,),
            eng_combustion=(running,),
            throttle_pct=(throttle,),
            accel_body=(lateral, vertical, long_accel),
            rot_velocity_body=(0.0, 0.0, roll_rate),
            wind_velocity_kt=self.wind_kt,
            wind_direction_rad=self.wind_from_rad,
            heading_true_rad=self.heading_rad,
        )

    def sample_scalar(self, t: float, field_name: str) -> float:
        """Interpolate one scripted scalar, used for finite differences."""
        before, after, blend = self._bracket(t)
        return _lerp(getattr(before, field_name), getattr(after, field_name), blend)

    def stream(self, *, start: float = 0.0, end: float | None = None) -> Iterator[FlightTelemetry]:
        """Yield samples at the configured rate."""
        step = 1.0 / self.rate_hz
        stop = self.duration if end is None else end
        t = start
        while t <= stop + 1e-9:
            yield self.sample_at(t)
            t += step

    def _bracket(self, t: float) -> tuple[Keyframe, Keyframe, float]:
        t = max(self.script[0].t, min(t, self.script[-1].t))
        previous = self.script[0]
        for frame in self.script:
            if frame.t >= t:
                span = frame.t - previous.t
                blend = (t - previous.t) / span if span > 1e-9 else 0.0
                return previous, frame, blend
            previous = frame
        return previous, previous, 0.0

    def _gust(self, t: float) -> float:
        """Deterministic pseudo-turbulence: a few incommensurate sine waves."""
        return (
            math.sin(t * 1.7) * 0.5 + math.sin(t * 4.3 + 1.1) * 0.3 + math.sin(t * 9.1 + 2.7) * 0.2
        )


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _alpha_for(ias: float, on_ground: bool) -> float:
    """A plausible angle of attack: high when slow, low when fast."""
    if ias < 1.0:
        return 0.0
    alpha = 0.30 * (48.0 / max(ias, 30.0)) ** 2
    return min(alpha, 0.32) * (0.4 if on_ground else 1.0)
