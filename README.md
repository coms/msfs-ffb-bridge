# MSFS 2024 → MOZA R5 force feedback bridge

Microsoft Flight Simulator 2024 has no native force feedback support, so everything the
simulator knows about the forces acting on your aeroplane stops at the screen. This bridge
reads that telemetry over SimConnect, runs it through a force model, and drives a MOZA R5
wheelbase with it — runway texture, touchdown, brakes, engine vibration, gusts and control
loading, all felt through the rim.

The wheel is also your flight control: **rudder and nosewheel steering on the ground,
ailerons in the air**, with an automatic handoff between the two.

> **Status: complete but unflown.** Everything is built and the test suite passes on
> Linux and Windows, and CI produces a working Windows executable. But the hardware half
> has never met a real wheelbase or a running simulator — it was developed on Linux, where
> neither exists. Expect a tuning round or two. Start with
> [the bench tests](docs/setup-pithouse.md#checking-it-worked), which prove the hardware
> path on its own before the simulator is involved.

## How it works

```
MSFS 2024 ──SimConnect──► telemetry ──► force model ──► SDL2 haptics ──► MOZA R5
                              │                                              ▲
                              └──► axis router ──► AXIS_RUDDER_SET  /        │
                                                   AXIS_AILERONS_SET  ───────┘
                                                                     wheel position
```

The wheel is *not* bound in the simulator's control settings. The bridge reads its position
directly from the device and injects the aileron or rudder axis over SimConnect, which is
what lets a single physical axis change meaning between the ground and the air.

### The ground/air handoff

Landing in a crosswind you are holding aileron into wind at the exact moment the axis wants
to become a rudder. Three things stop that from becoming a bootful of rudder:

- the incoming channel starts from the wheel position at handoff and converges on absolute
  tracking as the transition completes;
- the axis command is rate limited, so nothing can snap;
- the force model centres the wheel during the transition, so by the time the new axis has
  full authority the wheel is near neutral.

Both directions are dwell-gated, so a bounce on landing or a wheel unloading over a bump
cannot flip the axis back and forth.

## Effects

| Effect | What it does |
| --- | --- |
| `ground_roll` | Surface texture: a slow thud over concrete slabs, a fine chatter on grass and gravel, scaled by speed and wheel loading |
| `touchdown` | Impact thump scaled by the descent rate you actually arrived with, with a separate nosewheel arrival |
| `brakes` | Judder under braking, with a coarser stutter when you stand on them |
| `shimmy` | Speed-banded nosewheel wobble when the wheel is loaded and turned |
| `steering_feel` | Nosewheel scrub when stopped, caster centring at speed, rudder trim |
| `gear` | Motor rumble while the gear travels, clunk at the locks, airflow buffet |
| `engine_vibration` | Blade-pass hum tracking RPM, idle shake, roughness with an engine out |
| `turbulence` | Gust jolts high-passed from body accelerations, plus wind-scaled chop |
| `prop_wash` | The left swing under power at low speed, plus slipstream burble |
| `crosswind` | Weathervaning on the ground, sideslip-driven roll in the air |
| `control_loading` | Centring force growing with the square of airspeed, offset by trim |
| `handoff` | Walks the wheel to centre while the axis changes hands |
| `buffet` | Stall, Mach and flap buffet. Off by default |

Every effect has an on/off switch, a strength slider and its own parameters, all tunable
live and saved per aircraft.

## Safety

The R5 can produce 5.5 N·m at your wrists, so nothing in the force model is trusted:

- a master gain that ships at 70% and a hard ceiling on total force;
- rate limiting on the steady force channel, so a telemetry glitch cannot become a jolt;
- a watchdog that fades forces to zero when telemetry stops arriving;
- automatic zeroing when the simulator is paused, in a menu, or in slew mode;
- a module that raises an exception is dropped for that tick rather than taking the wheel
  with it.

## Getting started

```
ffbbridge doctor      # check the setup and explain anything missing
ffbbridge bench       # feel each effect on its own, with the simulator closed
ffbbridge             # open the tuning interface and fly
```

Two things to do before the first flight:

- [Set up MOZA Pit House](docs/setup-pithouse.md). The wheelbase applies its own spring
  and friction on top of anything an application sends, and they are on by default. This
  is the most common reason the bridge appears not to work.
- [Leave the wheel unbound in the simulator](docs/setup-msfs.md). The bridge reads the
  device directly and injects the axis, which is what lets one axis change meaning
  between the ground and the air.

Then [tuning](docs/tuning.md) — every slider applies while you fly.

Record a flight and replay it as often as you like with different settings:

```
ffbbridge replay flight.jsonl.gz --csv trace.csv
```

`ffbbridge simulate --csv trace.csv` flies a scripted sortie through the force model with
no simulator and no hardware, which is the quickest way to see what a change actually did.

## Development

The force model is pure Python with no operating system dependencies, so it runs and is
tested anywhere:

```bash
python -m venv .venv && .venv/bin/pip install -e ".[dev]"
.venv/bin/python -m pytest
```

Tests drive a scripted flight — start, taxi, run-up, takeoff, climb, cruise, approach,
flare, touchdown, rollout, shutdown — through the real engine, so every ground and
touchdown effect is exercised through its actual trigger conditions on every run.

## Licence

MIT. See [LICENSE](LICENSE).
