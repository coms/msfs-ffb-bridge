"""The tuning interface.

Built around the assumption that tuning happens while flying: every slider takes
effect on the next tick, with no apply button and no restart, and the force
readout shows which effect is responsible for what you are feeling. The bench
and diagnostics tabs are for the setup session before the first flight.

Dear PyGui owns the main thread and never touches SDL or SimConnect. It reads
the runtime's snapshot and posts changes back through the command queue.
"""

from __future__ import annotations

import logging
from pathlib import Path

import dearpygui.dearpygui as dpg

from ..core.config import ProfileSet
from ..core.modules import MODULE_REGISTRY
from . import paths
from .bench import BENCH_TESTS
from .doctor import format_report, run_checks
from .loop import BridgeRuntime, RuntimeSnapshot

LOGGER = logging.getLogger(__name__)

WINDOW_WIDTH = 1120
WINDOW_HEIGHT = 780

#: How much of the trace plot to keep, in samples at the refresh rate.
TRACE_LENGTH = 400

COLOUR_OK = (110, 200, 130)
COLOUR_WARN = (235, 190, 90)
COLOUR_BAD = (225, 110, 110)
COLOUR_DIM = (150, 150, 155)


class GuiApp:
    """Wires the widgets to the runtime."""

    def __init__(self, runtime: BridgeRuntime) -> None:
        self.runtime = runtime
        self.profile_path = runtime.config_path or paths.profiles_path()
        self._trace_t: list[float] = []
        self._trace_force: list[float] = []
        self._bench_active: str | None = None
        self._bench_started = 0.0

    # --- Building ---------------------------------------------------------

    def build(self) -> None:
        with dpg.window(tag="root"):
            self._build_status()
            with dpg.tab_bar():
                with dpg.tab(label="Flying"):
                    self._build_flying_tab()
                with dpg.tab(label="Tuning"):
                    self._build_tuning_tab()
                with dpg.tab(label="Bench test"):
                    self._build_bench_tab()
                with dpg.tab(label="Diagnostics"):
                    self._build_diagnostics_tab()

    def _build_status(self) -> None:
        with dpg.group(horizontal=True):
            dpg.add_text("Simulator:")
            dpg.add_text("connecting", tag="status_sim", color=COLOUR_DIM)
            dpg.add_text(" | Wheel:")
            dpg.add_text("searching", tag="status_device", color=COLOUR_DIM)
            dpg.add_text(" | Axis:")
            dpg.add_text("ground", tag="status_mode", color=COLOUR_DIM)
            dpg.add_text(" | ")
            dpg.add_text("", tag="status_aircraft", color=COLOUR_DIM)
        with dpg.group(horizontal=True):
            dpg.add_button(label="PANIC - cut all force", callback=self._on_panic, width=190)
            dpg.add_button(
                label="Force ground axis", callback=lambda: self._override("force_ground")
            )
            dpg.add_button(label="Force air axis", callback=lambda: self._override("force_air"))
            dpg.add_button(label="Automatic", callback=lambda: self._override("auto"))
            dpg.add_text("", tag="status_loop", color=COLOUR_DIM)
        dpg.add_separator()

    def _build_flying_tab(self) -> None:
        with dpg.group(horizontal=True):
            with dpg.child_window(width=430, height=610):
                dpg.add_text("Aircraft")
                for tag, label in (
                    ("tel_ias", "Indicated airspeed"),
                    ("tel_gs", "Groundspeed"),
                    ("tel_agl", "Height above ground"),
                    ("tel_vs", "Vertical speed"),
                    ("tel_wow", "Weight on wheels"),
                    ("tel_surface", "Surface"),
                    ("tel_rpm", "Propeller RPM"),
                    ("tel_throttle", "Throttle"),
                    ("tel_brakes", "Brakes"),
                    ("tel_flaps", "Flaps / gear"),
                    ("tel_wind", "Wind"),
                    ("tel_accel", "Body acceleration"),
                ):
                    with dpg.group(horizontal=True):
                        dpg.add_text(f"{label:22}", color=COLOUR_DIM)
                        dpg.add_text("-", tag=tag)

                dpg.add_separator()
                dpg.add_text("Axis routing")
                with dpg.group(horizontal=True):
                    dpg.add_text(f"{'Wheel position':22}", color=COLOUR_DIM)
                    dpg.add_text("-", tag="axis_wheel")
                with dpg.group(horizontal=True):
                    dpg.add_text(f"{'Aileron command':22}", color=COLOUR_DIM)
                    dpg.add_text("-", tag="axis_aileron")
                with dpg.group(horizontal=True):
                    dpg.add_text(f"{'Rudder command':22}", color=COLOUR_DIM)
                    dpg.add_text("-", tag="axis_rudder")
                with dpg.group(horizontal=True):
                    dpg.add_text(f"{'Ground / air blend':22}", color=COLOUR_DIM)
                    dpg.add_text("-", tag="axis_blend")

            with dpg.child_window(width=650, height=610):
                dpg.add_text("Force output")
                dpg.add_slider_float(
                    tag="force_total",
                    label="steady force",
                    min_value=-1.0,
                    max_value=1.0,
                    enabled=False,
                    width=380,
                )
                with dpg.plot(label="Force trace", height=180, width=-1):
                    dpg.add_plot_axis(dpg.mvXAxis, label="seconds", tag="trace_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="force", tag="trace_y"):
                        dpg.add_line_series([], [], label="steady", tag="trace_series")
                    dpg.set_axis_limits("trace_y", -1.0, 1.0)

                dpg.add_separator()
                dpg.add_text("Contribution by effect")
                for module in MODULE_REGISTRY:
                    dpg.add_slider_float(
                        tag=f"bar_{module.id}",
                        label=module.name,
                        min_value=-1.0,
                        max_value=1.0,
                        enabled=False,
                        width=300,
                    )
                dpg.add_separator()
                dpg.add_text("Vibration on the wheel: -", tag="force_effects", wrap=620)
                dpg.add_text("Mixed into steady force: -", tag="force_software", wrap=620)

    def _build_tuning_tab(self) -> None:
        config = self.runtime.engine.config
        with dpg.group(horizontal=True):
            dpg.add_button(label="Save profile", callback=self._on_save)
            dpg.add_button(label="Reload profile", callback=self._on_reload)
            dpg.add_text(str(self.profile_path), color=COLOUR_DIM)

        dpg.add_separator()
        dpg.add_text("Overall")
        dpg.add_slider_float(
            label="Master strength",
            default_value=config.safety.master_gain,
            min_value=0.0,
            max_value=1.0,
            width=300,
            callback=lambda _s, value: self._set_safety("master_gain", value),
        )
        dpg.add_slider_float(
            label="Force ceiling",
            default_value=config.safety.max_force,
            min_value=0.1,
            max_value=1.0,
            width=300,
            callback=lambda _s, value: self._set_safety("max_force", value),
        )
        dpg.add_separator()

        with dpg.child_window(height=520):
            for module in MODULE_REGISTRY:
                settings = config.module(module.id)
                with dpg.collapsing_header(label=module.name, default_open=False):
                    dpg.add_text(module.description, color=COLOUR_DIM, wrap=900)
                    dpg.add_checkbox(
                        label="enabled",
                        default_value=settings.enabled,
                        callback=self._module_toggle(module.id),
                    )
                    dpg.add_slider_float(
                        label="strength",
                        default_value=settings.gain,
                        min_value=0.0,
                        max_value=2.0,
                        width=320,
                        callback=self._module_gain(module.id),
                    )
                    for spec in module.params:
                        dpg.add_slider_float(
                            label=f"{spec.label}{f' ({spec.unit})' if spec.unit else ''}",
                            default_value=settings.params.get(spec.name, spec.default),
                            min_value=spec.minimum,
                            max_value=spec.maximum,
                            width=320,
                            callback=self._module_param(module.id, spec.name),
                        )

    def _build_bench_tab(self) -> None:
        dpg.add_text(
            "Play one effect at a time with the simulator closed. Start here: if the "
            "sweep is smooth and left really is left, everything after this is tuning.",
            wrap=1000,
            color=COLOUR_DIM,
        )
        dpg.add_separator()
        for test in BENCH_TESTS:
            with dpg.group(horizontal=True):
                dpg.add_button(label=test.name, width=220, callback=self._bench_callback(test.id))
                dpg.add_text(test.description, wrap=740, color=COLOUR_DIM)
        dpg.add_separator()
        dpg.add_button(label="Stop", callback=lambda: self._stop_bench(), width=220)
        dpg.add_text("", tag="bench_status")

    def _build_diagnostics_tab(self) -> None:
        dpg.add_button(label="Run checks", callback=self._on_doctor)
        dpg.add_separator()
        dpg.add_text("", tag="doctor_output", wrap=1060)
        dpg.add_separator()
        dpg.add_text("Simulation variables the aircraft did not provide:", color=COLOUR_DIM)
        dpg.add_text("none", tag="doctor_missing", wrap=1060)
        dpg.add_text("Effect modules that raised an error:", color=COLOUR_DIM)
        dpg.add_text("none", tag="doctor_errors", wrap=1060)

    # --- Callbacks --------------------------------------------------------

    def _on_panic(self) -> None:
        self.runtime.toggle_panic()

    def _override(self, mode: str) -> None:
        self.runtime.set_override(mode)

    def _set_safety(self, field: str, value: float) -> None:
        config = self.runtime.engine.config

        def apply() -> None:
            setattr(config.safety, field, value)
            if self.runtime.engine.mixer is not None:
                self.runtime.engine.mixer.set_safety(config.safety)

        self.runtime.post(apply)

    def _module_toggle(self, module_id: str):
        def callback(_sender, value):
            settings = self.runtime.engine.config.module(module_id)
            self.runtime.post(lambda: setattr(settings, "enabled", bool(value)))

        return callback

    def _module_gain(self, module_id: str):
        def callback(_sender, value):
            settings = self.runtime.engine.config.module(module_id)
            self.runtime.post(lambda: setattr(settings, "gain", float(value)))

        return callback

    def _module_param(self, module_id: str, name: str):
        def callback(_sender, value):
            settings = self.runtime.engine.config.module(module_id)
            self.runtime.post(lambda: settings.params.__setitem__(name, float(value)))

        return callback

    def _bench_callback(self, test_id: str):
        def callback():
            self._start_bench(test_id)

        return callback

    def _start_bench(self, test_id: str) -> None:
        from .bench import find_test

        test = find_test(test_id)
        if test is None:
            return
        import time

        started = time.perf_counter()
        self._bench_active = test_id
        self.runtime.set_bench(lambda now: test.build(now - started))
        dpg.set_value("bench_status", f"Playing: {test.name}. Press Stop when you are done.")

    def _stop_bench(self) -> None:
        self._bench_active = None
        self.runtime.set_bench(None)
        dpg.set_value("bench_status", "Stopped.")

    def _on_save(self) -> None:
        profiles = self.runtime.profiles
        profiles.default = self.runtime.engine.config
        try:
            profiles.save(self.profile_path)
            LOGGER.info("saved profile to %s", self.profile_path)
        except OSError as exc:
            LOGGER.error("could not save the profile: %s", exc)

    def _on_reload(self) -> None:
        path = Path(self.profile_path)
        profiles = ProfileSet.load_or_default(path)
        self.runtime.profiles = profiles
        self.runtime.apply_config(
            profiles.default.with_module_defaults(self.runtime.engine.default_module_settings())
        )

    def _on_doctor(self) -> None:
        dpg.set_value("doctor_output", format_report(run_checks(self.runtime.engine.config)))

    # --- Per-frame refresh ------------------------------------------------

    def refresh(self) -> None:
        snapshot = self.runtime.snapshot()
        self._refresh_status(snapshot)
        self._refresh_telemetry(snapshot)
        self._refresh_forces(snapshot)
        self._refresh_diagnostics(snapshot)

    def _refresh_status(self, s: RuntimeSnapshot) -> None:
        if s.sim_connected:
            dpg.set_value("status_sim", "connected")
            dpg.configure_item("status_sim", color=COLOUR_OK)
        else:
            dpg.set_value("status_sim", "not running")
            dpg.configure_item("status_sim", color=COLOUR_WARN)

        if s.device_connected:
            dpg.set_value("status_device", s.device_name or "connected")
            dpg.configure_item("status_device", color=COLOUR_OK)
        else:
            dpg.set_value("status_device", s.device_error or "not found")
            dpg.configure_item("status_device", color=COLOUR_BAD)

        label = s.mode.value.replace("_", " ")
        if s.override != "auto":
            label += f"  ({s.override.replace('_', ' ')})"
        dpg.set_value("status_mode", label)
        dpg.configure_item("status_mode", color=COLOUR_WARN if s.mode.is_transition else COLOUR_OK)
        dpg.set_value("status_aircraft", f"{s.aircraft or 'no aircraft'} [{s.profile}]")
        dpg.set_value(
            "status_loop",
            f"  {s.loop_hz:.0f} Hz{'  PANIC - force is cut' if s.panic else ''}"
            f"{'  telemetry stale' if s.stale and s.sim_connected else ''}",
        )

    def _refresh_telemetry(self, s: RuntimeSnapshot) -> None:
        t = s.telemetry
        dpg.set_value("tel_ias", f"{t.ias_kt:6.1f} kt")
        dpg.set_value("tel_gs", f"{t.gs_kt:6.1f} kt")
        dpg.set_value("tel_agl", f"{t.agl_ft:7.0f} ft")
        dpg.set_value("tel_vs", f"{t.vs_fpm:+7.0f} fpm")
        dpg.set_value("tel_wow", "yes" if t.weight_on_wheels else "no")
        dpg.set_value("tel_surface", t.surface_type.name.replace("_", " ").title())
        dpg.set_value("tel_rpm", f"{t.max_prop_rpm:6.0f}")
        dpg.set_value("tel_throttle", f"{t.max_throttle * 100:5.0f} %")
        dpg.set_value(
            "tel_brakes", f"L {t.brake_left * 100:3.0f} %   R {t.brake_right * 100:3.0f} %"
        )
        dpg.set_value(
            "tel_flaps", f"flaps {t.flaps_pct * 100:3.0f} %   gear {t.gear_pct * 100:3.0f} %"
        )
        dpg.set_value(
            "tel_wind",
            f"{t.wind_velocity_kt:4.1f} kt, crosswind {t.crosswind_kt:+5.1f} kt",
        )
        dpg.set_value(
            "tel_accel",
            f"lat {t.lateral_accel_g:+.2f} g   vert {t.vertical_accel_g:+.2f} g",
        )
        dpg.set_value("axis_wheel", f"{s.wheel.position:+.3f}")
        dpg.set_value("axis_aileron", f"{s.axis.aileron:+.3f}")
        dpg.set_value("axis_rudder", f"{s.axis.rudder:+.3f}")
        dpg.set_value(
            "axis_blend",
            f"{s.ground_weight * 100:3.0f} % rudder / {(1 - s.ground_weight) * 100:3.0f} % aileron",
        )

    def _refresh_forces(self, s: RuntimeSnapshot) -> None:
        dpg.set_value("force_total", s.force.constant)
        for module in MODULE_REGISTRY:
            dpg.set_value(f"bar_{module.id}", s.force.breakdown.get(module.id, 0.0))

        effects = ", ".join(
            f"{p.label} {p.frequency_hz:.0f} Hz at {p.magnitude * 100:.0f}%"
            for p in s.force.periodics
        )
        dpg.set_value("force_effects", f"Vibration on the wheel: {effects or 'none'}")
        dpg.set_value(
            "force_software",
            f"Mixed into steady force: {', '.join(s.software_labels) or 'none'}",
        )

        self._trace_t.append(s.telemetry.t)
        self._trace_force.append(s.force.constant)
        if len(self._trace_t) > TRACE_LENGTH:
            del self._trace_t[: len(self._trace_t) - TRACE_LENGTH]
            del self._trace_force[: len(self._trace_force) - TRACE_LENGTH]
        dpg.set_value("trace_series", [list(self._trace_t), list(self._trace_force)])
        if self._trace_t:
            dpg.set_axis_limits(
                "trace_x", self._trace_t[0], max(self._trace_t[-1], self._trace_t[0] + 1)
            )

    def _refresh_diagnostics(self, s: RuntimeSnapshot) -> None:
        dpg.set_value("doctor_missing", ", ".join(s.unavailable_vars) or "none")
        dpg.set_value(
            "doctor_errors",
            "; ".join(f"{k}: {v}" for k, v in s.module_errors.items()) or "none",
        )


def run_gui(runtime: BridgeRuntime) -> int:
    """Open the window and run until it is closed."""
    dpg.create_context()
    dpg.create_viewport(
        title="MSFS to MOZA force feedback bridge",
        width=WINDOW_WIDTH,
        height=WINDOW_HEIGHT,
    )

    app = GuiApp(runtime)
    app.build()

    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("root", True)

    runtime.start()
    try:
        while dpg.is_dearpygui_running():
            app.refresh()
            dpg.render_dearpygui_frame()
    finally:
        # The wheel is released before the window disappears, not after.
        runtime.stop()
        dpg.destroy_context()
    return 0
