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

## Response curves

`expo` and `aileron_curve` shape the wheel's raw position before it becomes an
axis, and go the opposite way from each other.

- `expo` (0..1, shared by aileron and rudder/steering) softens the centre: a
  cubic blend that makes small inputs near neutral move less than they would
  linearly, without changing where full deflection arrives. Useful when the
  centre feels twitchy. Not on the Tuning tab -- set it in the profile JSON,
  under `wheel`, the same way as `air_range` and `ground_range`.
- `aileron_curve` (0..5, aileron only) does the reverse: it sharpens the
  centre, so small aileron inputs move further than linear while full
  deflection stays where it was. Rudder and steering are unaffected -- they
  keep whatever `expo` is set to. Useful when small roll corrections feel like
  they need more wheel than they should, without wanting to change how much
  wheel gives full aileron. On the Tuning tab under *Wheel*; 1 is already about
  as sharp as `expo` goes the other way, so most of the range past that is for
  someone who wants a very small dead-feeling centre to disappear entirely.

Both apply on top of `deadzone` and `center`. If both are non-zero at once on
the aileron axis, `expo` softens first and `aileron_curve` then sharpens what
is left -- in practice, set one or the other rather than both.

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
- `skid` → *Slip before anything is felt*: how far a wheel has to fall behind
  the others before it counts as locked. Nothing reports tyre radius, so the
  module learns RPM per knot while you roll with the brakes off; an aircraft
  that reports no wheel RPM at all stays silent rather than guessing.
- `control_loading` → *Centring force at reference speed*: the weight of the
  ailerons in cruise. This one is normalised against the aircraft's design
  cruise speed, so it should carry across aircraft without adjustment.

## The soft lock

An aeroplane's controls stop somewhere. A direct-drive base does not, so the
bridge puts the stop back in software:

```json
"wheel": {
  "rotation_deg": 540.0,
  "soft_lock_deg": 180.0
}
```

Both live on the **Tuning** tab under *Wheel*, where the two sliders take effect
on the next tick like every other slider, and a line underneath says what they
add up to in degrees: where the stop is, and where full aileron and full rudder
arrive. Move rotation and a soft lock wider than the wheel follows it down.
Nothing is written to disk until *Save profile*.

Both are lock-to-lock, the way wheel software states them. `rotation_deg` is
what the wheelbase itself is set to and has to match it — nothing can read it
back from the device. `soft_lock_deg` is the travel the aircraft gets, so 180
means 90 degrees either side of centre. Set it to 0 for no stop.

Ships at 180°, which is roughly a light aircraft's yoke and comfortable to reach
without shuffling your hands.

Two things follow from turning it on:

- `air_range` and `ground_range` are capped at the soft lock, so full rudder and
  full aileron still arrive **at** the stop rather than through it. A 180° lock
  therefore makes the ground axis noticeably more sensitive than the 0.7 of a
  540° wheel it would otherwise use — that is the trade, and it is why a narrow
  lock wants a little more `expo`. This cap follows the `soft_lock` effect's own
  enabled switch, so turning that effect off in the module list gives the full
  configured range back rather than leaving the axis capped at where a wall no
  longer stands.
- The stop ignores the **master strength**, alone among the steady forces. Turn
  the force model down to something comfortable and a stop scaled with it stops
  being a stop — at 30% it is a nudge you push straight through, and your
  control travel has quietly grown. It is still bounded by the **force ceiling**,
  which is what makes it soft: lean hard and it yields. If the stop is too
  strong, lower the ceiling or the effect's own *Force at the stop*; turning the
  master down will not do it. Nothing here can hold the rim against a determined
  arm, and it should not try to.

- The wall pushes at full weight only while you are still leaning further into
  it. It remembers the deepest lean each visit to the stop reaches, and once
  the rim has backed off that peak by a real margin, the wall lets go almost
  entirely -- down to a twentieth of itself -- and leaves the damping to bring
  it home, rather than shoving all the way back through neutral when you
  relax. The small give-and-take of a firm hold does not count as leaving, so
  a wheel held against the stop still gets the whole wall and cannot creep
  outward.

The `soft_lock` effect has the usual three knobs: *Force at the stop*, *Travel
the stop builds over* (8 degrees by default — the wall ramps in rather than
arriving as a step), and *Resistance past the stop*, which stiffens with lean
the same way the wall itself does and is what actually stops a hard hit from
bouncing. It is worth turning up if you are still feeling a kick on a fast flick
into the stop: it runs on the device's own velocity sensor rather than
however often the bridge can send an update, so it is the more effective of
the two channels for arresting a fast impact rather than reacting to one after
the fact. Turn the effect off to keep the axis scaling but lose the wall.

## Per-aircraft profiles

The short way: load the aeroplane, tune it, and press **Save for this aircraft**
on the Tuning tab. That writes a profile matched to the title the simulator
reports and puts it in front of everything else, so it wins over any family
profile that was covering the aircraft until now. Saving the same aircraft again
updates that profile rather than adding a second one.

**Save as default** is the other button, and it is the one to use for settings
that should apply to anything you have not tuned individually — master strength,
wheel rotation, which axis the wheel is.

Both write a snapshot. Sliders moved after a save are not in the file until you
press it again.

The long way, and what the buttons are writing: profiles match on the aircraft
title or its ATC model, with wildcards. The first match wins, and the default is
used when nothing matches:

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
`routing.override_button`, or pick a fixed axis on the Tuning tab under *Axis*
— the same thing as setting `routing.mode` to `aileron_only` or `rudder_only`,
which switches the automation off entirely.

### Flying with pedals

*Ailerons only* is the wheel with the rudder taken off it: the wheel is
ailerons the whole flight, steering and rudder stay on the pedals, and no
handoff ever happens. It is pinned from the first tick, so there is no second
of rudder on the way down from a ground default.

What that costs and what it does not is worth being precise about, because it
is the usual worry:

- **Silenced**, because they are forces in the steering: nosewheel shimmy,
  stationary scrub, ground weathervaning, and the ground share of prop wash.
- **Untouched**, because they key off weight on wheels rather than off the axis
  blend: runway rumble, touchdown, brakes, gear and engine vibration. Taxiing
  still feels like the surface you are taxiing on.

Control loading also runs at full weight throughout, since the wheel is the
ailerons even while you are on the ground.

## Safety

- **Master strength** ships at 70% and **force ceiling** at 90%. The ceiling is
  a hard clamp on total force; lower it if the wheel ever surprises you.
- **PANIC** in the interface cuts all force immediately and keeps it cut until
  pressed again.
- Forces are cut automatically when the simulator is paused, in a menu, in slew
  mode, or when telemetry stops arriving for half a second.
- Closing the bridge releases every effect. So does a crash — the teardown runs
  on every exit path.
