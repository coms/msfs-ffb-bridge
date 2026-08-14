"""Runway and taxiway texture felt through the nosewheel."""

from __future__ import annotations

from ..context import TickContext
from ..filters import BandNoise, clamp, map_range, smoothstep
from ..forces import Contribution, Waveform
from ..telemetry import FlightTelemetry, SurfaceType, WheelState
from .base import EffectModule, ParamSpec

#: How rough each surface feels, 0 (glass) to 1 (ploughed field).
#: These are the numbers you will most likely want to tweak by feel; a slider on
#: the module's gain scales all of them together.
SURFACE_ROUGHNESS: dict[SurfaceType, float] = {
    SurfaceType.CONCRETE: 0.17,
    SurfaceType.ASPHALT: 0.15,
    SurfaceType.BITUMINOUS: 0.17,
    SurfaceType.TARMAC: 0.15,
    SurfaceType.MACADAM: 0.22,
    SurfaceType.OIL_TREATED: 0.18,
    SurfaceType.BRICK: 0.24,
    SurfaceType.STEEL_MATS: 0.30,
    SurfaceType.PLANKS: 0.32,
    SurfaceType.HARD_TURF: 0.26,
    SurfaceType.SHORT_GRASS: 0.34,
    SurfaceType.GRASS: 0.42,
    SurfaceType.LONG_GRASS: 0.52,
    SurfaceType.GRASS_BUMPY: 0.68,
    SurfaceType.DIRT: 0.46,
    SurfaceType.GRAVEL: 0.55,
    SurfaceType.SAND: 0.50,
    SurfaceType.SHALE: 0.45,
    SurfaceType.CORAL: 0.40,
    SurfaceType.SNOW: 0.28,
    SurfaceType.ICE: 0.06,
    SurfaceType.URBAN: 0.22,
    SurfaceType.FOREST: 0.55,
    SurfaceType.WATER: 0.18,
    SurfaceType.WRIGHT_FLYER_TRACK: 0.35,
}

#: Surfaces made of slabs or planks produce a regular beat rather than a hiss.
JOINTED_SURFACES = frozenset(
    {
        SurfaceType.CONCRETE,
        SurfaceType.BRICK,
        SurfaceType.PLANKS,
        SurfaceType.STEEL_MATS,
        SurfaceType.MACADAM,
    }
)


class GroundRoll(EffectModule):
    """Rolling texture, from smooth concrete hum to a grass strip's chatter.

    Two things happen at once, which is what makes a surface identifiable:
    a periodic tone whose frequency tracks groundspeed (on concrete this is
    literally the slab joints going past) and a broadband tug on the rim from
    the nosewheel wandering over an uneven surface.
    """

    id = "ground_roll"
    name = "Runway rumble"
    description = "Surface texture through the rolling gear, scaled by speed and surface type."
    priority = 75
    params = (
        ParamSpec("hz_per_kt", 0.09, 0.01, 0.4, "Slab-joint rate per knot", "Hz/kt"),
        ParamSpec("chatter_scale", 3.2, 1.0, 8.0, "Frequency multiplier on loose surfaces", ""),
        ParamSpec("max_hz", 32.0, 5.0, 60.0, "Highest tone frequency", "Hz"),
        ParamSpec("min_hz", 3.0, 0.5, 15.0, "Lowest tone frequency", "Hz"),
        ParamSpec("full_speed_kt", 45.0, 10.0, 120.0, "Speed for full intensity", "kt"),
        ParamSpec("texture", 0.55, 0.0, 2.0, "Broadband wander vs. pure tone", ""),
    )

    def __init__(self, settings=None) -> None:
        super().__init__(settings)
        self._noise = BandNoise(tau=0.05, seed=1701)

    def reset(self) -> None:
        self._noise.reset()

    def update(
        self, tel: FlightTelemetry, wheel: WheelState, ctx: TickContext, dt: float
    ) -> Contribution:
        contribution = Contribution()
        if not tel.weight_on_wheels:
            self._noise.update(dt)
            return contribution

        speed = abs(tel.gs_kt)
        if speed < 0.5:
            self._noise.update(dt)
            return contribution

        roughness = SURFACE_ROUGHNESS.get(tel.surface_type, 0.2)

        # Intensity ramps in from a walking pace and keeps growing slowly beyond
        # the reference speed, so a takeoff roll builds rather than plateauing.
        intensity = smoothstep(speed, 1.0, self.p("full_speed_kt"))
        intensity *= map_range(speed, self.p("full_speed_kt"), 140.0, 1.0, 1.25)

        # Weight on the wheels modulates it: as the wings take the load during
        # rotation the rumble fades out on its own.
        load = clamp(max(tel.nose_compression, tel.main_compression) * 2.0, 0.0, 1.0)
        if not tel.contact_compression:
            load = 1.0

        amplitude = clamp(roughness * intensity * load, 0.0, 1.0)
        if amplitude <= 1e-3:
            self._noise.update(dt)
            return contribution

        # Jointed surfaces beat at the rate slabs pass under the wheel, which is
        # a slow, countable thud. Loose surfaces have no such structure and are
        # felt as a much finer chatter, so they get a higher frequency for the
        # same speed. This difference is most of what makes grass identifiable.
        jointed = tel.surface_type in JOINTED_SURFACES
        rate = self.p("hz_per_kt") * (1.0 if jointed else self.p("chatter_scale"))
        frequency = clamp(speed * rate, self.p("min_hz"), self.p("max_hz"))
        contribution.add_periodic(
            "ground_roll",
            frequency_hz=frequency,
            magnitude=amplitude * (0.85 if jointed else 0.7),
            waveform=Waveform.TRIANGLE if jointed else Waveform.SINE,
            priority=self.priority,
        )

        # The wander is what separates grass from concrete: an irregular sideways
        # tug rather than a clean tone.
        texture = self.p("texture") * roughness * intensity * load
        if texture > 1e-3:
            contribution.constant += self._noise.update(dt) * texture * 0.35
        else:
            self._noise.update(dt)

        return contribution
