# How it fits together

```
                    ┌──────────────────── loop thread ─────────────────────┐
                    │                                                       │
  MSFS 2024 ────────┼──► SimConnectClient ──► FlightTelemetry ──┐          │
   (SimConnect)     │                                            │          │
                    │    WheelReader ──────► WheelState ─────────┤          │
                    │        ▲                                   ▼          │
                    │        │                            BridgeEngine      │
   MOZA R5 ◄────────┼── HapticOutput ◄── ForceOutput ◄────┬──────┴───────┐  │
   (SDL2 haptics)   │                                      │              │  │
                    │    AxisOutput ──► AXIS_*_SET ────────┘         AxisRouter
                    │        │                                                │
                    └────────┼────────────────────────────────────────────────┘
                             ▼
                       MSFS 2024 controls        snapshot ──► Dear PyGui (main thread)
```

## The layers

**`ffbbridge.core`** is the entire force model and has no operating system
dependencies at all. It can be imported, run and tested on any platform, which
is why the test suite covers it thoroughly: a complete scripted flight runs
through the real engine on every test run.

**`ffbbridge.io`** is the two hardware edges, kept as thin as possible.
`ffb_effects` is the exception — it converts the force model's output into SDL
structures and is pure data, so it is fully tested despite being part of the
hardware path. That is deliberate: the conversions are where the mistakes that
are hardest to diagnose by feel would live.

**`ffbbridge.app`** is the runtime. One thread owns SDL and SimConnect; the
interface reads an immutable snapshot and posts commands back through a queue.

## Why one thread

SDL requires that everything touching it happens on the thread that initialised
it. The SimConnect message pump wants to be somewhere a redraw can never block
it. Both constraints point the same way, so both live on the force loop thread
and the interface never calls into either.

The loop runs at 100 Hz by default. That is enough because vibration is rendered
*on the device*: a periodic effect is described once and the wheelbase plays it
at its own internal rate, so a 30 Hz rumble stays clean without the bridge
having to update at 30 Hz. Only the slowly-varying steady force needs the loop.

## The mixer's budget

A wheelbase can play a small, fixed number of effects at once. The mixer spends
that budget every tick:

- one constant force, which is where most of the model ends up;
- one spring and one damper, computed on the device from its own sensors;
- as many periodics as there are slots left, allocated by priority with strength
  as the tiebreak.

Vibration that cannot get a slot is not dropped — it is synthesised in software
and added to the constant channel. That happens *after* the slew limiter, which
matters: rate limiting exists so a telemetry glitch cannot become a jolt at the
rim, but applying it to a 30 Hz rumble would erase the rumble. So the limiter
only ever sees the slowly-varying part.

## Sign conventions

Positive is clockwise, to the right, from the pilot's seat, throughout.

Two different kinds of inversion exist and are easy to conflate:

- **Force inversion** is for a base that pushes the opposite way to what was
  asked. It applies to forces we command outright, and must *not* be applied to
  spring or damper coefficients — those are resolved on the device from its own
  position sensor and are already self-consistent. Negating one would turn a
  centring spring into one that drives the wheel to the stops.
- **Axis inversion** is for a device whose position axis reads backwards. It
  applies to positions in device units, which means the spring's centre.

## Failure behaviour

Every layer is expected to fail and none of them may take the wheel with them:

- a module that raises is skipped for that tick, reset, and reported;
- a simulation variable the aircraft does not implement is dropped from the
  subscription and the block layout rebuilt around it, because the block is
  positional and a stale entry would misattribute every value after it;
- telemetry that stops arriving fades the forces out over half a second rather
  than cutting them, which is both safer and less startling;
- the loop thread's teardown runs on every exit path, including an unhandled
  exception, so the wheel is always released.

## Testing

The force model is tested by flying it. `core.synthetic` scripts a complete
sortie and the tests assert behaviour through it: that the axis hands over once
in each direction at the right moments, that the touchdown fires exactly once
and scales with descent rate, that rumble frequency tracks groundspeed, that
nothing ever exceeds the configured ceiling.

What cannot be tested here is how any of it feels, and whether the MOZA driver
behaves as its documentation suggests. That needs a wheel, a simulator and a
circuit.
