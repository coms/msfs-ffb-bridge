# MOZA Pit House settings

**Do this first.** It is the single most common reason the bridge appears not to
work, and no amount of correct code on the bridge's side can compensate.

## Why it matters

A MOZA wheelbase applies its own spring, damping, friction and inertia *on the
base itself*, on top of whatever an application asks for. Those settings are
enabled by default, because for a racing game they are a feature: they stand in
for the self-centring a car's steering geometry provides.

For flight simulation they are the enemy. An aeroplane's controls are not
springy in a fixed way — they load up with airspeed, they go slack near the
stall, and trim moves where they want to rest. If the base is adding a constant
spring underneath, you feel the base's opinion rather than the aircraft's.

MOZA has acknowledged that the base does not fully honour an application's
request to disable its spring. So it has to be turned off by hand.

## Two different settings that are both called "spring"

This is the one to get right, and the names do not help:

- The base's **own spring** is the base adding centring of its own, the same
  amount whatever is happening. This is the one to turn **off**.
- The **game spring** is how much of an application's *requested* spring the
  base passes through, from 0 to 100%. This is the bridge's centring force —
  control loading, the weight of the ailerons that grows with airspeed — and at
  0 it is thrown away before it reaches the motor. This one has to be **on**.

Turn the first off and the second off and the wheel goes dead in the air: the
bridge asks for exactly the right centring force and the base discards it. The
same switch exists for damping, friction and inertia; the bridge uses the
damper, so leave that one on too.

The bridge's own effects are still worth understanding when you set these. Only
control loading, steering feel and the soft lock's damping ride on the spring
and damper channels. Everything else — runway rumble, touchdown, the soft lock
itself — is a constant force or a vibration, and arrives regardless.

## Settings

In MOZA Pit House, on the wheelbase page:

| Setting | Value | Why |
| --- | --- | --- |
| Force feedback mode | **DirectInput** | Anything else can hide effects from applications entirely |
| Spring (the base's own) | **0** | The bridge provides centring itself, scaled by airspeed |
| Game spring | **100%** | How much of the *bridge's* centring gets through. At 0 there is none |
| Game damper | **100%** | Carries the soft lock's damping and the aerodynamic damping |
| Friction | **0** | Masks the small, fast effects like runway texture |
| Inertia | **0** | Makes the rim feel heavy and swallows sharp transients such as touchdown |
| Damping | **5–10** | A little is good: it takes the edge off without dulling anything |
| Steering angle | **360–540°** | Comfortable for both taxiing and roll input |
| Soft lock / end stop | **off** | The bridge does this, at the aircraft's travel rather than the wheel's |
| Overall strength | **40–60%** | Start here. The R5 can produce 5.5 N·m, which is a lot at the wrists |
| Road sensitivity / effect equaliser | flat, or off | The bridge has already shaped the effects; a second equaliser fights it |
| Natural inertia / natural friction | **0** | Same reasoning as above |

Save it as a profile named something like "Flight" so it is one click to switch
back and forth with your racing settings.

Whatever steering angle you settle on, tell the bridge about it:
`wheel.rotation_deg` has to match, because it is the only way a setting written
in degrees — the soft lock — can mean anything. Nothing can read the angle back
from the wheelbase. It is the *Wheel rotation* slider on the Tuning tab, or
`wheel.rotation_deg` in the profile file; either way, press *Save profile* so it
is still right next time.

## Checking it worked

Run the bridge's bench tests with the simulator closed:

```
ffbbridge bench centring
```

Let go of the wheel while it plays. It should return to centre firmly but not
violently. Then:

```
ffbbridge bench damping
```

This one has **no** centring force at all — only damping. If the wheel still
springs back to centre during this test, the base's own spring is still on, and
Pit House needs another look.

```
ffbbridge bench sweep
```

A slow push from left to right and back. It should be smooth throughout. If it
feels notchy or there is a dead patch in the middle, friction or a deadzone is
still enabled somewhere.

## Direction

If `ffbbridge bench left` pulls the rim clockwise rather than anticlockwise, set
`invert_force` to `true` under `device` in your profile, or tick the equivalent
box in the interface. This costs nothing to get wrong — it is just a sign.
