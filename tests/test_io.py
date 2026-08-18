"""Tests for the hardware and simulator adapters.

The device itself cannot be tested without a wheel plugged into a Windows box,
but the conversions can: the SDL effect structures, the SimVar block layout and
the axis injection are all pure data, and they are where the mistakes that are
hardest to spot by feel would live.
"""

from __future__ import annotations

import ctypes
import logging

import pytest

from ffbbridge.core.config import RoutingConfig
from ffbbridge.core.context import AxisMode
from ffbbridge.core.forces import Damper, ForceOutput, Periodic, Spring, Waveform
from ffbbridge.core.routing import AxisCommand
from ffbbridge.io import ffb_effects as fx
from ffbbridge.io.axis_out import AXIS_FULL_SCALE, AxisOutput, to_axis_units
from ffbbridge.io.ffb_sdl import DeviceInfo, HapticCapabilities, select_device
from ffbbridge.io.simconnect_client import (
    EVENT_FLAG_GROUPID_IS_PRIORITY,
    GROUP_PRIORITY_HIGHEST,
    SIMCONNECT_RECV_SIMOBJECT_DATA,
    SIMOBJECT_DATA_OFFSET,
    SimConnectClient,
    find_simconnect_dll,
)
from ffbbridge.io.simvars import (
    MAX_ENGINES,
    NUMERIC_VARS,
    SimVarSpec,
    assemble,
    normalise,
)

sdl_haptic = pytest.importorskip("sdl2.haptic")


# --- SDL effect construction --------------------------------------------


def test_force_maps_onto_the_full_signed_range():
    assert fx.to_level(1.0) == 32767
    assert fx.to_level(-1.0) == -32767
    assert fx.to_level(0.0) == 0
    assert fx.to_level(5.0) == 32767  # clamped, not wrapped


def test_constant_effect_carries_level_and_runs_forever():
    effect = fx.build_constant(0.5)
    assert effect.type == sdl_haptic.SDL_HAPTIC_CONSTANT
    assert effect.constant.level == pytest.approx(16384, abs=2)
    assert effect.constant.length == sdl_haptic.SDL_HAPTIC_INFINITY


def test_force_inversion_flips_the_commanded_direction():
    assert fx.build_constant(0.5, force_invert=True).constant.level < 0


def test_period_is_whole_milliseconds():
    assert fx.period_ms(100.0) == 10
    assert fx.period_ms(1.2) == 833
    assert fx.period_ms(0.0) == fx.MAX_PERIOD_MS
    assert fx.period_ms(5000.0) == fx.MIN_PERIOD_MS  # cannot go below 1 ms


def test_square_falls_back_because_sdl_has_no_square():
    """SDL 2 dropped the square wave DirectInput exposes."""
    assert not hasattr(sdl_haptic, "SDL_HAPTIC_SQUARE")
    assert fx.sdl_waveform(Waveform.SQUARE) == sdl_haptic.SDL_HAPTIC_TRIANGLE


def test_waveform_falls_back_to_sine_when_unsupported():
    only_sine = sdl_haptic.SDL_HAPTIC_SINE
    assert fx.sdl_waveform(Waveform.SAWTOOTH_UP, only_sine) == sdl_haptic.SDL_HAPTIC_SINE
    assert fx.sdl_waveform(Waveform.SINE, only_sine) == sdl_haptic.SDL_HAPTIC_SINE


def test_periodic_effect_carries_frequency_and_magnitude():
    periodic = Periodic(
        label="rumble", waveform=Waveform.TRIANGLE, frequency_hz=25.0, magnitude=0.5
    )
    effect = fx.build_periodic(periodic)
    assert effect.type == sdl_haptic.SDL_HAPTIC_TRIANGLE
    assert effect.periodic.period == 40
    assert effect.periodic.magnitude == pytest.approx(16384, abs=2)


def test_spring_keeps_a_positive_coefficient_when_forces_are_inverted():
    """A negated spring coefficient would drive the wheel to the stops.

    Condition effects are resolved on the device from its own position sensor,
    so they are already self-consistent. Only the centre point is a position in
    our frame, and only that follows an inverted axis.
    """
    spring = Spring(coefficient=0.5, center=0.3, saturation=0.8)
    normal = fx.build_spring(spring)
    inverted = fx.build_spring(spring, axis_invert=True)
    assert normal.condition.right_coeff[0] > 0
    assert inverted.condition.right_coeff[0] == normal.condition.right_coeff[0]
    assert inverted.condition.center[0] == -normal.condition.center[0]


def test_spring_is_symmetric_about_its_centre():
    effect = fx.build_spring(Spring(coefficient=0.5, saturation=0.8, deadband=0.1))
    condition = effect.condition
    assert condition.right_coeff[0] == condition.left_coeff[0]
    assert condition.right_sat[0] == condition.left_sat[0]
    assert condition.deadband[0] > 0


def test_condition_effects_only_touch_the_first_axis():
    """A wheel has one axis; the other two entries must stay untouched."""
    effect = fx.build_spring(Spring(coefficient=0.5, center=0.5))
    for axis in (1, 2):
        assert effect.condition.right_coeff[axis] == 0
        assert effect.condition.center[axis] == 0


def test_damper_is_a_condition_effect_of_its_own_type():
    effect = fx.build_damper(Damper(coefficient=0.4, saturation=0.5))
    assert effect.type == sdl_haptic.SDL_HAPTIC_DAMPER
    assert effect.condition.right_coeff[0] > 0


def test_change_detection_compares_the_whole_structure():
    a = fx.build_constant(0.5)
    assert not fx.effects_differ(a, fx.build_constant(0.5))
    assert fx.effects_differ(a, fx.build_constant(0.6))
    assert fx.effects_differ(a, None)


def test_close_frequencies_quantise_to_the_same_period():
    """Millisecond periods usefully suppress pointless device traffic."""
    a = fx.build_periodic(Periodic(label="x", frequency_hz=40.0, magnitude=0.5))
    b = fx.build_periodic(Periodic(label="x", frequency_hz=40.4, magnitude=0.5))
    assert not fx.effects_differ(a, b)


# --- Device selection ----------------------------------------------------


def _device(name, haptic=True, index=0):
    return DeviceInfo(index=index, name=name, num_axes=6, num_buttons=20, is_haptic=haptic)


def test_device_selection_prefers_a_name_match():
    devices = [_device("Generic Pad", haptic=False), _device("MOZA R5 Wheel Base", index=1)]
    assert select_device(devices, "*MOZA*").name == "MOZA R5 Wheel Base"


def test_device_selection_falls_back_to_any_haptic_device():
    """A user with one wheel plugged in should not have to guess its name."""
    devices = [_device("Some Other Wheel")]
    assert select_device(devices, "*MOZA*").name == "Some Other Wheel"


def test_device_selection_ignores_devices_without_force_feedback():
    assert select_device([_device("Gamepad", haptic=False)], "*") is None
    assert select_device([], "*") is None


def test_capability_reporting_lists_features():
    caps = HapticCapabilities(
        supported=sdl_haptic.SDL_HAPTIC_CONSTANT | sdl_haptic.SDL_HAPTIC_SPRING
    )
    described = caps.describe()
    assert "constant force" in described
    assert "spring" in described
    assert "damper" not in described


# --- SimVar block --------------------------------------------------------


def test_every_variable_has_a_home_on_the_telemetry_record():
    from ffbbridge.core.telemetry import FlightTelemetry
    from ffbbridge.io.simvars import DERIVED_ONLY, TUPLE_FIELDS

    fields = set(FlightTelemetry.__dataclass_fields__)
    for spec in NUMERIC_VARS:
        assert spec.key in fields or spec.key in DERIVED_ONLY, spec.name
        if spec.is_tuple:
            assert spec.key in TUPLE_FIELDS
            assert spec.index < TUPLE_FIELDS[spec.key]


def test_variable_names_are_unique():
    names = [spec.name for spec in NUMERIC_VARS]
    assert len(names) == len(set(names))


def test_percentages_are_normalised_but_fractions_are_left_alone():
    spec = SimVarSpec("BRAKE LEFT POSITION", "Percent Over 100", "brake_left", fraction=True)
    assert normalise(spec, 85.0) == pytest.approx(0.85)
    assert normalise(spec, 0.85) == pytest.approx(0.85)
    assert normalise(spec, 1.0) == pytest.approx(1.0)  # a legitimate full deflection

    plain = SimVarSpec("AIRSPEED INDICATED", "Knots", "ias_kt")
    assert normalise(plain, 120.0) == 120.0


def test_assembling_a_block_produces_usable_telemetry():
    values = []
    lookup = {
        "SIM ON GROUND": 1.0,
        "AIRSPEED INDICATED": 65.0,
        "GROUND VELOCITY": 70.0,
        "SURFACE TYPE": 1.0,
        "NUMBER OF ENGINES": 1.0,
        "CAMERA STATE": 2.0,
        "PROP RPM:1": 2400.0,
        "ENG COMBUSTION:1": 1.0,
        "CONTACT POINT COMPRESSION:0": 45.0,
    }
    values = [lookup.get(spec.name, 0.0) for spec in NUMERIC_VARS]
    telemetry = assemble(NUMERIC_VARS, values, t=1.0, title="Cessna 172")

    assert telemetry.connected is True
    assert telemetry.ias_kt == 65.0
    assert telemetry.on_ground is True
    assert telemetry.title == "Cessna 172"
    assert telemetry.prop_rpm == (2400.0,)
    assert telemetry.eng_combustion == (True,)
    assert telemetry.contact_compression[0] == pytest.approx(0.45)


def test_a_dropped_variable_does_not_shift_the_rest():
    """The block is positional, so a rejected variable must leave the layout.

    Getting this wrong would silently attribute every subsequent value to the
    wrong field, which is far worse than losing one variable.
    """
    values = [float(i) for i in range(len(NUMERIC_VARS))]
    full = assemble(NUMERIC_VARS, values, t=0.0)

    index = next(i for i, s in enumerate(NUMERIC_VARS) if s.name == "AIRSPEED INDICATED")
    trimmed_specs = [s for i, s in enumerate(NUMERIC_VARS) if i != index]
    trimmed_values = [v for i, v in enumerate(values) if i != index]
    trimmed = assemble(trimmed_specs, trimmed_values, t=0.0)

    assert trimmed.ias_kt == 0.0  # the dropped one reads as its default
    assert trimmed.gs_kt == full.gs_kt  # everything after it stays put
    assert trimmed.total_weight_lb == full.total_weight_lb


def test_engine_tuples_are_trimmed_to_the_engine_count():
    values = [4.0 if spec.name == "NUMBER OF ENGINES" else 1.0 for spec in NUMERIC_VARS]
    telemetry = assemble(NUMERIC_VARS, values, t=0.0)
    assert len(telemetry.prop_rpm) == MAX_ENGINES

    values = [1.0 if spec.name == "NUMBER OF ENGINES" else 1.0 for spec in NUMERIC_VARS]
    telemetry = assemble(NUMERIC_VARS, values, t=0.0)
    assert len(telemetry.prop_rpm) == 1


def test_menus_and_the_world_map_are_not_the_cockpit():
    def in_cockpit(camera_state):
        values = [camera_state if spec.name == "CAMERA STATE" else 0.0 for spec in NUMERIC_VARS]
        return assemble(NUMERIC_VARS, values, t=0.0).in_cockpit

    assert in_cockpit(2.0) is True  # cockpit
    assert in_cockpit(3.0) is True  # external view
    assert in_cockpit(12.0) is False  # world map
    assert in_cockpit(14.0) is False  # hangar


def test_simobject_payload_offset_matches_the_struct():
    """A wrong offset would read the header as flight data."""
    assert ctypes.sizeof(SIMCONNECT_RECV_SIMOBJECT_DATA) == SIMOBJECT_DATA_OFFSET


def test_dll_discovery_returns_nothing_rather_than_raising():
    assert find_simconnect_dll("/definitely/not/here/SimConnect.dll") is None


# --- Talking to the simulator --------------------------------------------


class FakeDll:
    """Records the arguments the client passes to SimConnect."""

    def __init__(self, result=0):
        self.result = result
        self.calls: list[tuple] = []
        self.packet_id = 100

    def __getattr__(self, name):
        def call(*args):
            self.calls.append((name, args))
            if name == "SimConnect_GetLastSentPacketID":
                self.packet_id += 1
                args[1]._obj.value = self.packet_id
                return 0
            return self.result

        return call


def connected_client(dll):
    client = SimConnectClient()
    client._dll = dll
    client._handle = ctypes.c_void_p(1)
    return client


def test_axis_events_are_sent_at_a_priority_not_to_a_notification_group():
    """The group argument is a priority only if it is flagged as one.

    Without the flag the simulator reads it as the id of a notification group
    this client never creates, answers every event with UNRECOGNIZED_ID, and
    drops it -- silently, as far as the wheel is concerned, sixty times a
    second.
    """
    dll = FakeDll()
    client = connected_client(dll)
    event_id = client.map_event("AXIS_AILERONS_SET")
    assert client.transmit(event_id, 4096)

    _, args = next(c for c in dll.calls if c[0] == "SimConnect_TransmitClientEvent")
    assert args[4] == GROUP_PRIORITY_HIGHEST
    assert args[5] == EVENT_FLAG_GROUPID_IS_PRIORITY


def test_a_rejected_packet_is_named_rather_than_numbered():
    dll = FakeDll()
    client = connected_client(dll)
    event_id = client.map_event("AXIS_AILERONS_SET")
    client.transmit(event_id, 4096)

    assert client._describe(dll.packet_id) == "AXIS_AILERONS_SET"
    assert client._describe(999999) == "send id 999999"


def test_a_fault_in_the_send_loop_is_not_logged_forty_thousand_times(caplog):
    """It repeats at the send rate, and buries its own first occurrence."""
    client = SimConnectClient()
    with caplog.at_level(logging.WARNING):
        for _ in range(5000):
            client._complain("simulator reported %s for %s", "UNRECOGNIZED_ID", "AXIS_RUDDER_SET")

    assert len(caplog.records) == 4  # the first, then 10, 100 and 1000
    assert "UNRECOGNIZED_ID" in caplog.records[0].message


# --- Axis injection ------------------------------------------------------


class FakeClient:
    """Stands in for the simulator, recording what would have been sent."""

    def __init__(self, connected=True):
        self.connected = connected
        self.sent: list[tuple[str, int]] = []
        self._events: dict[str, int] = {}

    def map_event(self, name):
        return self._events.setdefault(name, 100 + len(self._events))

    def transmit(self, event_id, value):
        name = next(n for n, i in self._events.items() if i == event_id)
        self.sent.append((name, value))
        return True


def test_axis_scaling_uses_the_simulator_range():
    assert to_axis_units(1.0) == AXIS_FULL_SCALE
    assert to_axis_units(-1.0) == -AXIS_FULL_SCALE
    assert to_axis_units(0.0) == 0
    assert to_axis_units(2.0) == AXIS_FULL_SCALE


def test_both_axes_are_sent_every_time():
    """An unsent axis would keep whatever the simulator last had."""
    client = FakeClient()
    output = AxisOutput(client, RoutingConfig())
    output.send(AxisCommand(aileron=0.0, rudder=0.5, mode=AxisMode.GROUND), now=0.0)
    assert {name for name, _ in client.sent} == {"AXIS_AILERONS_SET", "AXIS_RUDDER_SET"}


def test_unchanged_axes_are_not_resent_every_tick():
    client = FakeClient()
    output = AxisOutput(client, RoutingConfig(send_rate_hz=1000.0))
    command = AxisCommand(aileron=0.25, rudder=0.0, mode=AxisMode.AIR)
    output.send(command, now=0.0)
    before = len(client.sent)
    for step in range(1, 20):
        output.send(command, now=step * 0.001)
    assert len(client.sent) == before


def test_an_unchanged_axis_is_refreshed_eventually():
    """A reload can reset the simulator's stored value, so it is re-asserted."""
    client = FakeClient()
    output = AxisOutput(client, RoutingConfig(send_rate_hz=1000.0))
    command = AxisCommand(aileron=0.25, rudder=0.0, mode=AxisMode.AIR)
    output.send(command, now=0.0)
    before = len(client.sent)
    output.send(command, now=2.0)
    assert len(client.sent) > before


def test_the_send_rate_is_limited():
    client = FakeClient()
    output = AxisOutput(client, RoutingConfig(send_rate_hz=10.0))
    for step in range(100):
        value = step / 100.0
        output.send(AxisCommand(aileron=value, rudder=0.0), now=step * 0.001)
    # A tenth of a second of ticks at 10 Hz is one opportunity to send.
    assert len({name for name, _ in client.sent}) <= 2
    assert len(client.sent) <= 4


def test_the_tiller_is_only_sent_when_enabled():
    client = FakeClient()
    output = AxisOutput(client, RoutingConfig(use_tiller=True))
    output.send(AxisCommand(aileron=0.0, rudder=0.4, steering=0.4), now=0.0)
    assert any(name == "AXIS_STEERING_SET" for name, _ in client.sent)


def test_nothing_is_sent_while_disconnected():
    client = FakeClient(connected=False)
    output = AxisOutput(client, RoutingConfig())
    output.send(AxisCommand(aileron=0.5, rudder=0.5), now=0.0)
    assert client.sent == []


def test_negative_values_are_transmitted_as_signed():
    """The event parameter is unsigned; a raw negative would peg the control."""
    client = FakeClient()
    output = AxisOutput(client, RoutingConfig())
    output.send(AxisCommand(aileron=-1.0, rudder=0.0), now=0.0)
    aileron = next(value for name, value in client.sent if name == "AXIS_AILERONS_SET")
    assert aileron == -AXIS_FULL_SCALE
    assert (aileron & 0xFFFFFFFF) == 0xFFFFC001


def test_force_output_survives_the_round_trip_to_effects():
    """Everything the mixer can produce has to be expressible as SDL effects."""
    force = ForceOutput(
        constant=0.3,
        spring=Spring(coefficient=0.4, center=0.1, saturation=0.7),
        damper=Damper(coefficient=0.2),
        periodics=(
            Periodic(label="a", frequency_hz=12.0, magnitude=0.4),
            Periodic(label="b", waveform=Waveform.SQUARE, frequency_hz=30.0, magnitude=0.2),
        ),
    )
    built = [
        fx.build_constant(force.constant),
        fx.build_spring(force.spring),
        fx.build_damper(force.damper),
        *[fx.build_periodic(p) for p in force.periodics],
    ]
    assert all(effect.type != 0 for effect in built)
    assert len({fx.effect_bytes(effect) for effect in built}) == len(built)
