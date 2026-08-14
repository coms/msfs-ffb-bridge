# Setting up Microsoft Flight Simulator

## Leave the wheel unbound

This is the important one, and it is the opposite of what you would normally do.

**Do not bind the MOZA wheel to anything in the simulator's control settings.**

The bridge reads the wheel directly from the device and injects the aileron or
rudder axis over SimConnect. That indirection is the whole reason a single
physical axis can be a rudder while you are on the ground and ailerons once you
are flying — a binding cannot change its meaning mid-flight, but an injected
axis can.

If the wheel is *also* bound in the simulator, both paths fight each other and
the control will jitter or stick.

To check: open **Options → Controls**, select the MOZA base in the device list,
and clear any assignment on its steering axis. Buttons on the rim are fine to
bind normally — the bridge only claims the steering axis.

Your pedals are unaffected. Bind them as usual, for brakes, throttle or rudder.

## SimConnect

The bridge needs `SimConnect.dll`. It looks in, in order:

1. the path set in your profile under `device.simconnect_dll`;
2. next to the bridge's own executable;
3. the `SimConnect` Python package, if installed;
4. an installed MSFS SDK, via the `MSFS2024_SDK` or `MSFS_SDK` environment
   variable.

If you downloaded a packaged build, the DLL travels with it and there is nothing
to do. Otherwise the quickest route is:

```
pip install SimConnect
```

which exists to obtain Microsoft's DLL — the bridge does not use any of that
package's Python code.

`ffbbridge doctor` reports which of these it found, or every place it looked if
it found none.

## What the bridge does to the simulator

Only two things:

- it subscribes to about sixty simulation variables, read only;
- it sends `AXIS_AILERONS_SET`, `AXIS_RUDDER_SET` and, if you enable the tiller
  option, `AXIS_STEERING_SET`.

Nothing is written to your aircraft, your settings or your flight. Closing the
bridge leaves the simulator exactly as it was, except that the axes it was
driving stop moving.

## First flight

1. Run `ffbbridge doctor` and clear anything it reports.
2. Work through [the Pit House settings](setup-pithouse.md) and the bench tests.
3. Start the simulator, load a Cessna 172 on a runway, and start the bridge.
4. Check the status line: simulator connected, wheel connected, axis **ground**.
5. Taxi. You should feel the surface through the rim, and the wheel should steer.
6. Take off. About a second after you are properly airborne the axis label
   changes to **air** and the wheel becomes ailerons — you will feel it centre
   itself during the handover.
7. Land. The reverse happens shortly after the wheels are down.

If the forces feel too weak or too strong, that is the master strength slider,
and it is meant to be moved while flying. See [tuning](tuning.md).

## If SimConnect.dll cannot be found

`ffbbridge doctor` lists every path it checked. The DLL is almost certainly
already on your machine — the simulator ships one, and so does the SDK. Search
your drive for `SimConnect.dll`, then either copy it next to `ffbbridge.exe` or
point the profile at it:

```json
{ "default": { "device": { "simconnect_dll": "C:\\path\\to\\SimConnect.dll" } } }
```

The profile lives in `%LOCALAPPDATA%\msfs-ffb-bridge\profiles.json`.
