"""Tests for recording flights and replaying them through the model."""

from __future__ import annotations

import json

from ffbbridge.core.engine import BridgeEngine
from ffbbridge.core.recording import (
    TelemetryRecorder,
    from_dict,
    read_recording,
    recording_header,
    to_dict,
    write_recording,
)
from ffbbridge.core.synthetic import SyntheticFlight
from ffbbridge.core.telemetry import EngineType, FlightTelemetry, SurfaceType, WheelState


def test_a_snapshot_survives_a_round_trip():
    original = FlightTelemetry(
        t=12.5,
        ias_kt=95.0,
        surface_type=SurfaceType.GRAVEL,
        engine_type=EngineType.TURBOPROP,
        contact_compression=(0.1, 0.4, 0.4),
        eng_combustion=(True, False),
        accel_body=(1.0, 32.0, -0.5),
        title="Some Aeroplane",
    )
    restored = from_dict(to_dict(original))
    assert restored == original


def test_records_are_plain_json():
    payload = to_dict(FlightTelemetry(surface_type=SurfaceType.GRASS))
    text = json.dumps(payload)
    assert json.loads(text)["surface_type"] == int(SurfaceType.GRASS)


def test_unknown_fields_from_a_newer_version_are_ignored():
    """A recording made by a later build should still replay."""
    record = to_dict(FlightTelemetry(ias_kt=80.0))
    record["some_future_field"] = 42
    assert from_dict(record).ias_kt == 80.0


def test_recording_and_replay_preserves_the_flight(tmp_path):
    path = tmp_path / "flight.jsonl"
    flight = SyntheticFlight()
    original = list(flight.stream(start=60.0, end=90.0))
    written = write_recording(path, original, note="takeoff")

    replayed = list(read_recording(path))
    assert written == len(original) == len(replayed)
    assert replayed[0].t == original[0].t
    assert replayed[-1] == original[-1]


def test_the_header_carries_a_note(tmp_path):
    path = tmp_path / "flight.jsonl"
    write_recording(path, [FlightTelemetry()], note="grass strip, gusty")
    header = recording_header(path)
    assert header["note"] == "grass strip, gusty"
    assert header["format"] >= 1


def test_the_header_is_not_replayed_as_a_sample(tmp_path):
    path = tmp_path / "flight.jsonl"
    write_recording(path, [FlightTelemetry(t=1.0), FlightTelemetry(t=2.0)])
    assert [sample.t for sample in read_recording(path)] == [1.0, 2.0]


def test_a_truncated_recording_still_replays(tmp_path):
    """Recordings end when a session ends, which is not always tidily."""
    path = tmp_path / "flight.jsonl"
    write_recording(path, list(SyntheticFlight().stream(start=0.0, end=5.0)))

    text = path.read_text(encoding="utf-8")
    path.write_text(text[: int(len(text) * 0.8)], encoding="utf-8")

    samples = list(read_recording(path))
    assert len(samples) > 100
    assert all(isinstance(sample, FlightTelemetry) for sample in samples)


def test_each_sample_is_flushed_as_it_is_written(tmp_path):
    """A recording of a crash is only useful if it survives the crash."""
    path = tmp_path / "flight.jsonl"
    recorder = TelemetryRecorder(path)
    recorder.open()
    recorder.write(FlightTelemetry(t=1.0))
    assert len(path.read_text(encoding="utf-8").splitlines()) == 2  # header plus one
    recorder.close()


def test_recordings_can_be_compressed(tmp_path):
    """An hour of flying is a quarter of a gigabyte otherwise."""
    samples = list(SyntheticFlight().stream(start=0.0, end=30.0))
    plain = tmp_path / "flight.jsonl"
    packed = tmp_path / "flight.jsonl.gz"
    write_recording(plain, samples)
    write_recording(packed, samples, note="compressed")

    assert packed.stat().st_size * 4 < plain.stat().st_size
    assert list(read_recording(packed)) == list(read_recording(plain))
    assert recording_header(packed)["note"] == "compressed"


def test_writing_to_a_closed_recorder_is_harmless():
    recorder = TelemetryRecorder("/tmp/never-created.jsonl")
    recorder.write(FlightTelemetry())
    assert recorder.samples == 0


def test_a_replayed_flight_produces_the_same_forces(tmp_path):
    """The point of recording: identical input must give identical output."""
    flight = SyntheticFlight()
    wheel = WheelState(position=0.1, connected=True)

    live_engine = BridgeEngine()
    live = [live_engine.tick(t, wheel, t.t).force.constant for t in flight.stream()]

    path = tmp_path / "flight.jsonl"
    write_recording(path, flight.stream())
    replay_engine = BridgeEngine()
    replayed = [replay_engine.tick(t, wheel, t.t).force.constant for t in read_recording(path)]

    assert len(live) == len(replayed)
    assert live == replayed
