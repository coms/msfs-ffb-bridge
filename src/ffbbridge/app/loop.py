"""The force loop: one thread that owns the wheel and the simulator connection.

SDL requires that everything touching it happens on the thread that initialised
it, and the SimConnect message pump wants to be somewhere that is never blocked
by a redraw. Both live here. The interface never calls into either directly; it
reads an immutable snapshot and posts commands to a queue, and this thread
applies them between ticks.

If this thread stops, the wheel must go quiet. Every exit path -- a clean stop,
an unhandled exception, interpreter shutdown -- runs through the same teardown.
"""

from __future__ import annotations

import contextlib
import logging
import queue
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from ..core.config import BridgeConfig, ProfileSet
from ..core.context import AxisMode
from ..core.engine import BridgeEngine, EngineResult
from ..core.forces import ZERO_FORCE, ForceOutput
from ..core.recording import TelemetryRecorder
from ..core.routing import AxisCommand, OverrideState
from ..core.telemetry import FlightTelemetry, WheelState
from ..io.axis_out import AxisOutput
from ..io.ffb_sdl import FfbError, HapticOutput, init_sdl, shutdown_sdl
from ..io.simconnect_client import SimConnectClient
from ..io.wheel_input import WheelReader

LOGGER = logging.getLogger(__name__)

#: How often to retry a connection that is not up yet.
RECONNECT_INTERVAL_S = 2.0

#: Sleeping this close to the deadline and spinning the rest keeps the loop
#: steady without burning a core: Windows timers are only good to about a
#: millisecond even after asking for the fastest resolution.
SPIN_MARGIN_S = 0.0015


@dataclass(frozen=True, slots=True)
class RuntimeSnapshot:
    """A consistent view of the bridge for the interface to render."""

    running: bool = False
    sim_connected: bool = False
    device_connected: bool = False
    device_name: str = ""
    sim_error: str = ""
    device_error: str = ""
    aircraft: str = ""
    profile: str = ""
    mode: AxisMode = AxisMode.GROUND
    ground_weight: float = 1.0
    override: str = OverrideState.AUTO
    stale: bool = True
    panic: bool = False
    loop_hz: float = 0.0
    telemetry: FlightTelemetry = field(default_factory=FlightTelemetry)
    wheel: WheelState = field(default_factory=WheelState)
    force: ForceOutput = ZERO_FORCE
    axis: AxisCommand = field(default_factory=AxisCommand)
    effect_labels: tuple[str, ...] = ()
    software_labels: tuple[str, ...] = ()
    withheld: str = ""
    """Why the wheel is getting nothing, when it is getting nothing."""
    unavailable_vars: tuple[str, ...] = ()
    module_errors: dict[str, str] = field(default_factory=dict)
    recording_path: str = ""
    recorded_samples: int = 0


class BridgeRuntime:
    """Owns the loop thread and everything that talks to hardware."""

    def __init__(
        self,
        profiles: ProfileSet | None = None,
        *,
        config_path: Path | None = None,
    ) -> None:
        self.profiles = profiles if profiles is not None else ProfileSet()
        self.config_path = config_path
        self.engine = BridgeEngine(self.profiles)

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._commands: queue.Queue[Callable[[], None]] = queue.Queue()
        self._snapshot = RuntimeSnapshot()
        self._lock = threading.Lock()

        self._panic = False
        self._sim = SimConnectClient(dll_path=self.engine.config.device.simconnect_dll)
        self._axis_out: AxisOutput | None = None
        self._haptic: HapticOutput | None = None
        self._wheel_reader: WheelReader | None = None
        self._bench: Callable[[float, WheelState], ForceOutput] | None = None
        self._recorder: TelemetryRecorder | None = None
        self._last_recorded_t = -1.0

    # --- Public, callable from any thread ---------------------------------

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ffb-loop", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        """Ask the loop to finish and wait for the wheel to be released."""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout)
        self._thread = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def snapshot(self) -> RuntimeSnapshot:
        with self._lock:
            return self._snapshot

    def post(self, command: Callable[[], None]) -> None:
        """Run something on the loop thread, between ticks."""
        self._commands.put(command)

    @property
    def haptic(self) -> HapticOutput | None:
        """The open device, for settings that have to reach it while it is open."""
        return self._haptic

    def set_panic(self, panic: bool) -> None:
        """Cut all force immediately, or restore it."""

        def apply() -> None:
            self._panic = panic
            if panic and self._haptic is not None:
                self._haptic.stop_all()

        self.post(apply)

    def toggle_panic(self) -> None:
        self.set_panic(not self._panic)

    def apply_config(self, config: BridgeConfig) -> None:
        self.post(lambda: self.engine.apply_config(config))

    def set_override(self, override: str) -> None:
        def apply() -> None:
            if self.engine.router is not None:
                self.engine.router.set_override(override)

        self.post(apply)

    def set_bench(self, generator: Callable[[float, WheelState], ForceOutput] | None) -> None:
        """Drive the wheel from a generator instead of the simulator.

        This is what makes the bench test possible: the same output path, fed by
        something other than a flight, so the hardware can be proved on its own.

        The generator is given where the wheel is as well as the time, because
        the effects worth feeling on a bench are not all functions of time: a
        control stop is a function of where the rim has been turned to.
        """

        def apply() -> None:
            self._bench = generator

        self.post(apply)

    def start_recording(self, path: Path, *, note: str = "") -> None:
        """Begin writing telemetry to a file for later replay."""

        def apply() -> None:
            self._stop_recording_now()
            recorder = TelemetryRecorder(path, note=note)
            try:
                recorder.open()
            except OSError as exc:
                LOGGER.error("could not start recording: %s", exc)
                return
            self._recorder = recorder
            self._last_recorded_t = -1.0
            LOGGER.info("recording to %s", path)

        self.post(apply)

    def stop_recording(self) -> None:
        self.post(self._stop_recording_now)

    def _stop_recording_now(self) -> None:
        if self._recorder is None:
            return
        LOGGER.info("recorded %d samples to %s", self._recorder.samples, self._recorder.path)
        self._recorder.close()
        self._recorder = None

    def _record(self, telemetry: FlightTelemetry) -> None:
        """Write each distinct sample once.

        The loop runs faster than telemetry arrives, so without the timestamp
        check a recording would be mostly duplicates.
        """
        if self._recorder is None or not telemetry.connected:
            return
        if telemetry.t == self._last_recorded_t:
            return
        self._last_recorded_t = telemetry.t
        self._recorder.write(telemetry)

    # --- The loop ---------------------------------------------------------

    def _run(self) -> None:
        timer_resolution = _request_fine_timers()
        try:
            init_sdl()
        except FfbError as exc:
            LOGGER.error("SDL would not start: %s", exc)
            self._publish(device_error=str(exc))
            return

        try:
            self._loop()
        except Exception:
            LOGGER.exception("the force loop stopped unexpectedly")
        finally:
            # Whatever happened, the wheel does not keep pulling.
            self._teardown()
            _release_fine_timers(timer_resolution)

    def _loop(self) -> None:
        period = 1.0 / max(self.engine.config.device.loop_hz, 20.0)
        next_tick = time.perf_counter()
        last_reconnect = 0.0
        measured_hz = 0.0
        last_time = time.perf_counter()

        while not self._stop.is_set():
            self._drain_commands()
            now = time.perf_counter()

            if now - last_reconnect > RECONNECT_INTERVAL_S:
                last_reconnect = now
                self._ensure_device()
                self._ensure_sim(now)

            self._sim.pump(now)
            wheel = self._read_wheel(period)
            telemetry = self._sim.latest or FlightTelemetry(connected=False)
            self._record(telemetry)
            result = self.engine.tick(telemetry, wheel, now)
            force = self._output_force(result, now, wheel)
            self._send_axes(result, now)

            elapsed = now - last_time
            last_time = now
            if elapsed > 0:
                measured_hz = measured_hz * 0.9 + (1.0 / elapsed) * 0.1

            self._publish_state(result, wheel, telemetry, force, measured_hz)

            period = 1.0 / max(self.engine.config.device.loop_hz, 20.0)
            next_tick += period
            self._sleep_until(next_tick)
            if next_tick < time.perf_counter() - period:
                # The loop fell behind; start again from now rather than trying
                # to catch up with a burst of ticks.
                next_tick = time.perf_counter()

    def _drain_commands(self) -> None:
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return
            try:
                command()
            except Exception:
                LOGGER.exception("a queued command failed")

    def _sleep_until(self, deadline: float) -> None:
        remaining = deadline - time.perf_counter()
        if remaining > SPIN_MARGIN_S:
            self._stop.wait(remaining - SPIN_MARGIN_S)
        while time.perf_counter() < deadline:
            if self._stop.is_set():
                return

    # --- Connections ------------------------------------------------------

    def _ensure_device(self) -> None:
        if self._haptic is not None and self._haptic.is_open:
            return
        device_config = self.engine.config.device
        haptic = HapticOutput(
            name_match=device_config.name_match,
            force_invert=device_config.invert_force,
            axis_invert=self.engine.config.wheel.invert,
            disable_autocenter=device_config.disable_autocenter,
            slot_budget=device_config.periodic_slots,
        )
        try:
            info = haptic.open()
        except FfbError as exc:
            self._publish(device_error=str(exc))
            return
        self._haptic = haptic
        self._wheel_reader = WheelReader(haptic.joystick)
        self.engine.set_periodic_slots(haptic.periodic_slots)
        LOGGER.info("wheel ready: %s", info.name)
        self._publish(device_error="")

    def _ensure_sim(self, now: float) -> None:
        if self._sim.connected:
            return
        if self._sim.connect(now=now):
            self._axis_out = AxisOutput(self._sim, self.engine.config.routing)
            self.engine.reset()
            LOGGER.info("simulator connected")

    def _read_wheel(self, dt: float) -> WheelState:
        if self._wheel_reader is None:
            return WheelState(connected=False)
        try:
            return self._wheel_reader.read(dt)
        except Exception:
            LOGGER.exception("could not read the wheel")
            return WheelState(connected=False)

    # --- Output -----------------------------------------------------------

    def _output_force(self, result: EngineResult, now: float, wheel: WheelState) -> ForceOutput:
        force = result.force
        if self._bench is not None:
            force = self._bench(now, wheel)
        if self._panic:
            force = ZERO_FORCE
        if self._haptic is not None and self._haptic.is_open:
            try:
                self._haptic.apply(force)
            except Exception:
                LOGGER.exception("could not send forces; releasing the wheel")
                self._close_device()
        return force

    def _send_axes(self, result: EngineResult, now: float) -> None:
        if self._axis_out is None or self._bench is not None:
            return
        if not self._sim.connected:
            self._axis_out.on_reconnect()
            return
        try:
            self._axis_out.send(result.axis, now)
        except Exception:
            LOGGER.exception("could not send an axis position")

    # --- Publishing -------------------------------------------------------

    def _publish_state(
        self,
        result: EngineResult,
        wheel: WheelState,
        telemetry: FlightTelemetry,
        force: ForceOutput,
        loop_hz: float,
    ) -> None:
        mixer = self.engine.mixer
        snapshot = RuntimeSnapshot(
            running=True,
            sim_connected=self._sim.connected,
            device_connected=self._haptic is not None and self._haptic.is_open,
            device_name=self._haptic.device.name if self._haptic and self._haptic.device else "",
            sim_error=self._sim.status.last_error,
            device_error=self.snapshot().device_error,
            aircraft=self.engine.status.aircraft,
            profile=result.profile_name,
            mode=result.context.mode,
            ground_weight=result.context.ground_weight,
            override=result.axis.override,
            stale=result.stale,
            panic=self._panic,
            loop_hz=loop_hz,
            telemetry=telemetry,
            wheel=wheel,
            force=force,
            axis=result.axis,
            effect_labels=mixer.diagnostics.hardware_periodics if mixer else (),
            software_labels=mixer.diagnostics.software_periodics if mixer else (),
            withheld=mixer.diagnostics.withheld if mixer else "",
            unavailable_vars=tuple(self._sim.status.unavailable_vars),
            module_errors=dict(self.engine.status.module_errors),
            recording_path=str(self._recorder.path) if self._recorder else "",
            recorded_samples=self._recorder.samples if self._recorder else 0,
        )
        with self._lock:
            self._snapshot = snapshot

    def _publish(self, **changes) -> None:
        """Update a few snapshot fields without a full tick."""
        with self._lock:
            current = self._snapshot
            merged = {
                key: getattr(current, key)
                for key in RuntimeSnapshot.__dataclass_fields__
                if key not in changes
            }
            merged.update(changes)
            self._snapshot = RuntimeSnapshot(**merged)

    # --- Teardown ---------------------------------------------------------

    def _close_device(self) -> None:
        if self._haptic is not None:
            with contextlib.suppress(Exception):
                self._haptic.close()
        self._haptic = None
        self._wheel_reader = None

    def _teardown(self) -> None:
        LOGGER.info("releasing the wheel and closing the connection")
        with contextlib.suppress(Exception):
            self._stop_recording_now()
        self._close_device()
        with contextlib.suppress(Exception):
            self._sim.close()
        with contextlib.suppress(Exception):
            shutdown_sdl()
        self._publish(running=False, device_connected=False, sim_connected=False, force=ZERO_FORCE)


def _request_fine_timers() -> bool:
    """Ask Windows for millisecond timer resolution.

    Without it ``sleep`` rounds up to about 15 ms, which would turn a 100 Hz
    loop into a 60 Hz one with visible jitter.
    """
    try:
        import ctypes

        winmm = getattr(ctypes, "WinDLL", None)
        if winmm is None:
            return False
        ctypes.WinDLL("winmm").timeBeginPeriod(1)
        return True
    except Exception:  # noqa: BLE001 - absent on anything but Windows
        return False


def _release_fine_timers(acquired: bool) -> None:
    if not acquired:
        return
    with contextlib.suppress(Exception):
        import ctypes

        ctypes.WinDLL("winmm").timeEndPeriod(1)
