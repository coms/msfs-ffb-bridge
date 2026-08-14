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

## Settings

In MOZA Pit House, on the wheelbase page:

| Setting | Value | Why |
| --- | --- | --- |
| Force feedback mode | **DirectInput** | Anything else can hide effects from applications entirely |
| Spring | **0** | The bridge provides centring itself, scaled by airspeed |
| Friction | **0** | Masks the small, fast effects like runway texture |
| Inertia | **0** | Makes the rim feel heavy and swallows sharp transients such as touchdown |
| Damping | **5–10** | A little is good: it takes the edge off without dulling anything |
| Steering angle | **360–540°** | Comfortable for both taxiing and roll input |
| Overall strength | **40–60%** | Start here. The R5 can produce 5.5 N·m, which is a lot at the wrists |
| Road sensitivity / effect equaliser | flat, or off | The bridge has already shaped the effects; a second equaliser fights it |
| Natural inertia / natural friction | **0** | Same reasoning as above |

Save it as a profile named something like "Flight" so it is one click to switch
back and forth with your racing settings.

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
