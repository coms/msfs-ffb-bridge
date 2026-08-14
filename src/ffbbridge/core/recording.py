"""Recording flights and replaying them through the force model.

This is the tuning loop that makes the hardware half tractable. Fly with the
recorder on, and the resulting file can be replayed through the engine as many
times as you like, with different settings, without going near an aircraft --
and it can be sent to someone else who can do the same.

The format is JSON Lines: one telemetry sample per line, so a recording can be
truncated, concatenated or inspected with ordinary tools, and a crash mid-flight
still leaves everything written up to that point.
"""

from __future__ import annotations

import gzip
import json
from collections.abc import Iterable, Iterator
from dataclasses import fields
from pathlib import Path
from typing import Any, TextIO

from .telemetry import EngineType, FlightTelemetry, SurfaceType

FORMAT_VERSION = 1

#: Recordings are verbose -- around 80 kB a second uncompressed, which is a
#: quarter of a gigabyte for an hour's flying. A ``.gz`` suffix compresses that
#: by about nine to one and costs nothing to read back. Flushing every line
#: gives up some of the ratio a plain archive would reach, which is the right
#: trade: a recording of something going wrong has to survive it.
COMPRESSED_SUFFIX = ".gz"


def _open_text(path: Path, mode: str) -> TextIO:
    """Open a recording, transparently compressed if the name says so."""
    if path.suffix == COMPRESSED_SUFFIX:
        return gzip.open(path, mode + "t", encoding="utf-8", newline="")  # type: ignore[return-value]
    return path.open(mode, encoding="utf-8")


#: Fields that need converting on the way in or out of JSON.
_ENUM_FIELDS = {"surface_type": SurfaceType, "engine_type": EngineType}
_TUPLE_FIELDS = frozenset(
    {
        "contact_compression",
        "eng_rpm",
        "prop_rpm",
        "eng_combustion",
        "throttle_pct",
        "accel_body",
        "rot_velocity_body",
    }
)


def to_dict(telemetry: FlightTelemetry) -> dict[str, Any]:
    """Flatten a snapshot into something JSON can hold."""
    record: dict[str, Any] = {}
    for field in fields(telemetry):
        value = getattr(telemetry, field.name)
        if isinstance(value, tuple):
            value = list(value)
        elif isinstance(value, SurfaceType | EngineType):
            value = int(value)
        record[field.name] = value
    return record


def from_dict(record: dict[str, Any]) -> FlightTelemetry:
    """Rebuild a snapshot, ignoring anything a newer version wrote."""
    known = {field.name for field in fields(FlightTelemetry)}
    kwargs: dict[str, Any] = {}
    for key, value in record.items():
        if key not in known:
            continue
        if key in _ENUM_FIELDS:
            kwargs[key] = _ENUM_FIELDS[key].from_raw(value)
        elif key in _TUPLE_FIELDS:
            kwargs[key] = tuple(value)
        else:
            kwargs[key] = value
    return FlightTelemetry(**kwargs)


class TelemetryRecorder:
    """Appends telemetry to a file as it arrives.

    Every line is flushed, because the interesting recordings are the ones made
    while something is going wrong and the session may not end tidily.
    """

    def __init__(self, path: str | Path, *, note: str = "") -> None:
        self.path = Path(path)
        self.note = note
        self._handle: TextIO | None = None
        self.samples = 0

    def __enter__(self) -> TelemetryRecorder:
        self.open()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._handle is not None

    def open(self) -> None:
        if self._handle is not None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = _open_text(self.path, "w")
        header = {"format": FORMAT_VERSION, "note": self.note}
        self._handle.write(json.dumps(header) + "\n")
        self._handle.flush()

    def write(self, telemetry: FlightTelemetry) -> None:
        if self._handle is None:
            return
        self._handle.write(json.dumps(to_dict(telemetry), separators=(",", ":")) + "\n")
        self._handle.flush()
        self.samples += 1

    def close(self) -> None:
        if self._handle is not None:
            self._handle.close()
            self._handle = None


def read_recording(path: str | Path) -> Iterator[FlightTelemetry]:
    """Replay a recording, skipping the header and any unreadable line.

    A truncated final line is normal -- it means the recording ended with the
    session rather than tidily -- and is not worth failing over.
    """
    with _open_text(Path(path), "r") as handle:
        for index, line in enumerate(handle):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except ValueError:
                continue
            if index == 0 and "format" in record:
                continue
            if "t" not in record:
                continue
            yield from_dict(record)


def recording_header(path: str | Path) -> dict[str, Any]:
    """The first line's metadata, or an empty dict if there is none."""
    with _open_text(Path(path), "r") as handle:
        first = handle.readline().strip()
    try:
        record = json.loads(first)
    except ValueError:
        return {}
    return record if "format" in record else {}


def write_recording(path: str | Path, samples: Iterable[FlightTelemetry], *, note: str = "") -> int:
    """Write a whole recording at once, for tests and for exporting a flight."""
    with TelemetryRecorder(path, note=note) as recorder:
        for sample in samples:
            recorder.write(sample)
        return recorder.samples
