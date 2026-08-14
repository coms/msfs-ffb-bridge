# Running it

Two ways: download a build, or run from source. The download needs nothing
installed; running from source needs Python 3.11 or newer.

Either way, `run.bat` in the repository root does the right thing — it uses a
packaged `ffbbridge.exe` if one is sitting next to it, and otherwise sets up a
Python environment on first use. Arguments pass straight through, so
`run.bat doctor` and `ffbbridge.exe doctor` are the same thing.

## Download a build

1. Open the [Actions tab](https://github.com/coms/msfs-ffb-bridge/actions) and
   click the newest green run.
2. Scroll to **Artifacts** and download `ffbbridge-windows`.
3. Unzip it. You get `ffbbridge.exe`, the docs and the default profiles.

Open a terminal in that folder — Shift + right-click in Explorer, "Open
PowerShell window here" — and work through the three commands below.

## Run from source

```bat
git clone https://github.com/coms/msfs-ffb-bridge
cd msfs-ffb-bridge
run.bat doctor
```

The first run creates `.venv` and installs everything, which takes a minute.
Every run after that is immediate.

On Linux or macOS, `./run.sh` does the same. The wheel and the simulator are
Windows-only, so only the offline tools (`simulate`, `replay`) are useful there
— which is exactly what they are for.

## The first three commands, in order

### 1. Check the setup

```
ffbbridge.exe doctor
```

Reports what it found and what to do about anything missing. The two it is
likely to raise:

- **No controllers.** Turn the wheelbase on, and in MOZA Pit House set the
  force feedback mode to **DirectInput**.
- **SimConnect.dll not found.** It lists every path it checked. The library is
  almost certainly already on your machine; copy it next to `ffbbridge.exe`. See
  [setting up the simulator](setup-msfs.md#if-simconnectdll-cannot-be-found).

### 2. Prove the hardware, with the simulator closed

```
ffbbridge.exe bench            # list the tests
ffbbridge.exe bench sweep      # then run them one at a time
ffbbridge.exe bench centring
ffbbridge.exe bench damping
ffbbridge.exe bench rumble
ffbbridge.exe bench touchdown
```

This is the step worth not skipping. It answers "is the wheel doing what the
bridge asked, in the right direction" before a flight is involved, and it is how
you catch a Pit House setting that is still fighting you.

`bench damping` is the important one: that effect has **no** centring force at
all. If the wheel still springs back to centre during it, MOZA's own spring is
still enabled and needs turning off — see
[the Pit House settings](setup-pithouse.md).

If `bench left` pulls the rim clockwise instead of anticlockwise, set
`invert_force` under `device` in the profile.

### 3. Fly

```
ffbbridge.exe
```

Opens the tuning window and starts the bridge. Then start the simulator, load a
Cessna 172 on a runway, and check the status line reads *simulator connected ·
wheel connected · axis ground*.

Before that first flight, two things that are not optional:

- [Set up MOZA Pit House](setup-pithouse.md). Spring, friction and inertia to 0;
  damping 5–10; strength 40–60% to start.
- [Unbind the wheel in the simulator's controls](setup-msfs.md). The bridge
  injects the axis itself, and a binding would fight it.

Then [tuning](tuning.md) — every slider applies while you fly.

## Trying it with no wheel and no simulator

Works on any machine, including the one you are reading this on:

```
ffbbridge.exe simulate
```

Flies a scripted circuit — start, taxi, run-up, takeoff, climb, cruise,
approach, flare, touchdown, rollout — through the real force model and prints
what the wheel would have done at each stage. `--csv trace.csv` writes every
tick. It is the quickest way to see what a configuration change actually did.

## Command summary

| Command | What it does |
| --- | --- |
| `ffbbridge` | Open the tuning interface and run the bridge |
| `ffbbridge run --console` | Run with a status line instead of a window |
| `ffbbridge doctor` | Check the setup and explain anything missing |
| `ffbbridge devices` | List every game controller Windows can see |
| `ffbbridge bench [test]` | Play one effect, with the simulator closed |
| `ffbbridge simulate` | Fly a scripted sortie through the force model offline |
| `ffbbridge replay <file>` | Replay a recorded flight through the force model |

Add `--csv trace.csv` to `simulate` or `replay` for a full per-tick trace, and
`-v` to any of them for debug logging. Logs also go to
`%LOCALAPPDATA%\msfs-ffb-bridge\ffbbridge.log`.
