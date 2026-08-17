# Tuning

Every slider takes effect on the next tick. There is no apply button, and
nothing needs restarting: move a slider while flying and feel what it does.

## The order to do it in

1. **Master strength** first, with nothing else touched. Fly a circuit and set
   it so the strongest thing you feel — usually a firm touchdown — is firm but
   not alarming. Everything else scales from here.
2. **Runway rumble** next, because it is what you feel most. Taxi on concrete,
   then on grass. They should feel like different surfaces, not the same effect
   at two volumes: concrete is a slow, countable thud, grass a finer chatter.
3. **Touchdown** third. Land deliberately firmly once and see whether it reads
   as an arrival. Then grease one on and check it is noticeably gentler.
4. **Engine vibration** last, and quieter than you think. It is present the
   whole time you are flying, so what feels right in a five-second test is often
   tiring after twenty minutes.

Leave the rest alone until something specific bothers you.

## What the sliders mean

**Strength** on an effect scales that effect alone. Anything above 1.0 is
pushing past the level the effect was designed around, which is fine, but if you
find yourself at 2.0 the master strength is probably too low.

**Effect parameters** change character rather than volume. A few worth knowing:

- `ground_roll` → *Slab-joint rate per knot*: how fast the concrete thud comes
  with speed. Real slabs pass at about 1 Hz at taxi speed. Raise it if the
  rumble feels too slow to register.
- `ground_roll` → *Frequency multiplier on loose surfaces*: how much finer grass
  and gravel feel than concrete. This is most of what makes a surface
  identifiable; lower it and everything starts feeling the same.
- `touchdown` → *Descent rate for a full-scale hit*: the arrival that produces
  maximum thump, in feet per minute. Lower it if your landings all feel soft.
- `engine_vibration` → *Propeller blades*: set it to match the aircraft. The
  tone is locked to blade-pass frequency, so a four-blade aircraft with this set
  to two hums at the wrong pitch.
- `steering_feel` → *Stationary scrub resistance*: how heavy the nosewheel is
  when you are stopped. Raise it if taxiing feels weightless.
- `control_loading` → *Centring force at reference speed*: the weight of the
  ailerons in cruise. This one is normalised against the aircraft's design
  cruise speed, so it should carry across aircraft without adjustment.

## Per-aircraft profiles

Profiles match on the aircraft title or its ATC model, with wildcards. The first
match wins, and the default is used when nothing matches:

```json
{
  "name": "Warbirds",
  "match": ["*spitfire*", "*mustang*"],
  "modules": {
    "control_loading": { "enabled": true, "gain": 1.3 },
    "engine_vibration": { "enabled": true, "gain": 1.1, "params": { "blades": 4.0 } },
    "shimmy": { "enabled": false }
  }
}
```

A profile only states what it changes; everything else comes from the defaults.
The shipped file has worked examples for airliners and warbirds.

## Tuning without flying

`ffbbridge simulate` flies a scripted sortie — start, taxi, run-up, takeoff,
climb, cruise, approach, flare, touchdown, rollout — through the force model and
prints what it produced. `--csv trace.csv` writes every tick, which is the
fastest way to see whether a change did what you expected without leaving your
desk.

```
ffbbridge simulate --csv trace.csv
```

## The ground/air handover

The wheel is a rudder on the ground and ailerons in the air, and the changeover
is where the sharp edges are.

`routing.transition_ms` is how long the handover takes. Longer feels smoother
but leaves you with reduced authority on both axes for longer. The default of
1.2 seconds is a compromise; below about 600 ms it starts to feel abrupt.

`routing.air_dwell_s` and `ground_dwell_s` are how long the conditions must hold
before a handover starts. They exist so a bounce on landing or a wheel unloading
over a bump cannot flip the axis. Raise `ground_dwell_s` if you bounce often.

**The crosswind case is worth understanding.** Landing in a crosswind you are
holding aileron into wind at the moment the axis wants to become a rudder. The
bridge does three things about it: the incoming axis starts from where the wheel
already is rather than jumping, the command is rate limited, and the force model
actively pulls the wheel toward centre during the handover. That last one is the
real protection, and you will feel it. If you fight it, the aileron input you
are holding does partly become rudder input — so let the wheel come back to
centre as the wheels touch, which is roughly what you would do anyway.

If you would rather do it by hand, bind a rim button to the override with
`routing.override_button`, or set `routing.mode` to `aileron_only` or
`rudder_only` to switch the automation off entirely.

## Safety

- **Master strength** ships at 70% and **force ceiling** at 90%. The ceiling is
  a hard clamp on total force; lower it if the wheel ever surprises you.
- **PANIC** in the interface cuts all force immediately and keeps it cut until
  pressed again.
- Forces are cut automatically when the simulator is paused, in a menu, in slew
  mode, or when telemetry stops arriving for half a second.
- Closing the bridge releases every effect. So does a crash — the teardown runs
  on every exit path.

## The soft lock

Only part of the rim's travel is mapped to a flight control — by default about a
third of it for ailerons, and 70% for ground steering. Past that point the
surface is already hard over and turning further does nothing. The soft lock
puts a control stop exactly there, so the rim stops where the aeroplane's
controls would, and it moves as the axis hands over between ground and air.

Nothing at all is felt inside the usable range, so it never interferes with any
other effect.

**What it can and cannot be.** A force feedback condition effect reaches full
force only at full displacement, so the resistance grows in proportion to how far
past the stop you have pushed: a tenth of the travel past it gives a tenth of
maximum force. What you feel is firm, unmistakable, progressive resistance
rather than a brick wall. Nothing can change that with force feedback alone —
faking a hard wall in software would mean a high-gain loop running at 100 Hz,
which on a direct drive wheel risks it buzzing at the boundary.

What does help is **reducing the wheelbase's rotation angle** so less travel is
wasted, which puts the stop nearer the rim's own limit. See
[the note in the Pit House settings](setup-pithouse.md#a-note-on-rotation-angle).

Parameters:

- *How solid the stop feels* — full stiffness by default, which is as steep as a
  condition effect goes. Lower it if you would rather it were only a hint.
- *Travel past full deflection before it bites* — a small margin so the stop
  never eats the last of the travel the axis needs.

The master strength slider deliberately does **not** scale it. A stop softened to
a third is one you push straight through without noticing, which is worse than
not having one. It still disappears with everything else when the simulator is
paused or telemetry stops.

Feel where it sits with `ffbbridge bench softlock`.
