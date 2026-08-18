"""A small SimConnect client, built directly on the native DLL with ctypes.

Written rather than borrowed for two reasons. The obvious one is performance:
the bridge needs about sixty variables at frame rate, and the usual Python
wrapper polls each variable separately, which is an order of magnitude more work
than one packed block on a periodic subscription. The other is licensing --
the popular wrapper is AGPL, and this only needs a few hundred lines.

Messages are drained with ``GetNextDispatch`` rather than the callback API, so
the simulator never calls into Python from its own thread and everything stays
on the loop thread that also owns SDL.
"""

from __future__ import annotations

import ctypes
import logging
import os
import sys
from ctypes import byref, c_char_p, c_double, c_float, c_int, c_long, c_void_p
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

from ..core.telemetry import FlightTelemetry
from .simvars import (
    DATATYPE_FLOAT64,
    DATATYPE_STRING256,
    NUMERIC_VARS,
    SimVarSpec,
    assemble,
)

LOGGER = logging.getLogger(__name__)

# ctypes.wintypes only exists on Windows, and this module has to import
# everywhere so the rest of the package and its tests stay platform neutral.
# These are the same types by definition.
DWORD = ctypes.c_uint32
HANDLE = ctypes.c_void_p

S_OK = 0

# Identifiers. Any distinct values will do; these are just readable.
DEFINE_NUMERIC = 1
DEFINE_IDENT = 2
REQUEST_NUMERIC = 1
REQUEST_IDENT = 2

#: How many recently sent packets to keep named, for reporting rejections.
RECENT_PACKETS = 64

EVENT_PAUSE = 1000
EVENT_SIM = 1001
FIRST_AXIS_EVENT = 2000

#: Priority to transmit axis events at, ahead of anything else claiming them.
GROUP_PRIORITY_HIGHEST = 1

#: Tells the simulator to read the group argument of TransmitClientEvent as a
#: priority rather than as the id of a notification group.
#:
#: Without it the same 1 means "notification group number one", which is a group
#: this client never creates, and every event sent is answered with
#: UNRECOGNIZED_ID and dropped. Transmitting at a priority needs no group.
EVENT_FLAG_GROUPID_IS_PRIORITY = 0x00000010

OBJECT_ID_USER = 0

PERIOD_NEVER = 0
PERIOD_ONCE = 1
PERIOD_VISUAL_FRAME = 2
PERIOD_SIM_FRAME = 3
PERIOD_SECOND = 4

RECV_ID_EXCEPTION = 1
RECV_ID_OPEN = 2
RECV_ID_QUIT = 3
RECV_ID_EVENT = 4
RECV_ID_SIMOBJECT_DATA = 8

#: Offset of the payload inside SIMCONNECT_RECV_SIMOBJECT_DATA: the three header
#: fields plus seven of its own, all DWORDs.
SIMOBJECT_DATA_OFFSET = 40

#: Exception codes worth naming in a log message.
EXCEPTION_NAMES = {
    0: "NONE",
    1: "ERROR",
    2: "SIZE_MISMATCH",
    3: "UNRECOGNIZED_ID",
    4: "UNOPENED",
    5: "VERSION_MISMATCH",
    6: "TOO_MANY_GROUPS",
    7: "NAME_UNRECOGNIZED",
    8: "TOO_MANY_EVENT_NAMES",
    9: "EVENT_ID_DUPLICATE",
    10: "TOO_MANY_MAPS",
    11: "TOO_MANY_OBJECTS",
    12: "TOO_MANY_REQUESTS",
    26: "INVALID_DATA_TYPE",
    27: "INVALID_DATA_SIZE",
    28: "DATA_ERROR",
    29: "INVALID_ARRAY",
}


class SimConnectError(RuntimeError):
    """The simulator could not be reached, or refused something fundamental."""


class SIMCONNECT_RECV(ctypes.Structure):
    _fields_ = [("dwSize", DWORD), ("dwVersion", DWORD), ("dwID", DWORD)]


class SIMCONNECT_RECV_EXCEPTION(ctypes.Structure):
    _fields_ = [
        ("dwSize", DWORD),
        ("dwVersion", DWORD),
        ("dwID", DWORD),
        ("dwException", DWORD),
        ("dwSendID", DWORD),
        ("dwIndex", DWORD),
    ]


class SIMCONNECT_RECV_EVENT(ctypes.Structure):
    _fields_ = [
        ("dwSize", DWORD),
        ("dwVersion", DWORD),
        ("dwID", DWORD),
        ("uGroupID", DWORD),
        ("uEventID", DWORD),
        ("dwData", DWORD),
    ]


class SIMCONNECT_RECV_SIMOBJECT_DATA(ctypes.Structure):
    _fields_ = [
        ("dwSize", DWORD),
        ("dwVersion", DWORD),
        ("dwID", DWORD),
        ("dwRequestID", DWORD),
        ("dwObjectID", DWORD),
        ("dwDefineID", DWORD),
        ("dwFlags", DWORD),
        ("dwentrynumber", DWORD),
        ("dwoutof", DWORD),
        ("dwDefineCount", DWORD),
    ]


def simconnect_search_paths(explicit: str = "") -> list[Path]:
    """Everywhere the DLL might be, in order of how likely each is to be right.

    Exposed separately so the doctor can show exactly where it looked rather
    than just reporting a failure.
    """
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))

    # Bundled beside a frozen executable, or in the source tree's lib folder.
    frozen_dir = getattr(sys, "_MEIPASS", None)
    if frozen_dir:
        candidates.append(Path(frozen_dir) / "SimConnect.dll")
    candidates.append(Path(sys.argv[0]).resolve().parent / "SimConnect.dll")
    candidates.append(Path(__file__).resolve().parents[3] / "lib" / "SimConnect.dll")

    # The SimConnect pip package ships Microsoft's DLL; the Python code around
    # it is AGPL and unused here, but the binary is the same one the SDK
    # installs and is a convenient way to obtain it.
    try:
        import SimConnect as _simconnect_pkg  # type: ignore[import-not-found]

        candidates.append(Path(_simconnect_pkg.__file__).parent / "SimConnect.dll")
    except Exception:  # noqa: BLE001 - absence is the normal case
        pass

    # An installed MSFS SDK, either by environment variable or at the path its
    # installer defaults to.
    for variable in ("MSFS2024_SDK", "MSFS_SDK"):
        root = os.environ.get(variable)
        if root:
            candidates.append(Path(root) / "SimConnect SDK" / "lib" / "SimConnect.dll")
    for root in (r"C:\MSFS 2024 SDK", r"C:\MSFS SDK"):
        candidates.append(Path(root) / "SimConnect SDK" / "lib" / "SimConnect.dll")

    candidates.extend(_simulator_install_candidates())
    return candidates


def find_simconnect_dll(explicit: str = "") -> Path | None:
    """The first place the DLL actually turns up, or None."""
    for path in simconnect_search_paths(explicit):
        try:
            if path.is_file():
                return path
        except OSError:
            continue
    return None


def _simulator_install_candidates() -> list[Path]:
    """Places the simulator's own copy of the DLL tends to sit.

    Worth searching before asking anyone to install a software development kit
    to fly an aeroplane: if the simulator is on the machine, the library usually
    already is too.
    """
    candidates: list[Path] = []
    program_files = [
        os.environ.get("PROGRAMFILES", r"C:\Program Files"),
        os.environ.get("PROGRAMFILES(X86)", r"C:\Program Files (x86)"),
    ]
    steam_libraries = [
        Path(base) / "Steam" / "steamapps" / "common" for base in program_files if base
    ]
    steam_libraries.append(Path(r"C:\SteamLibrary\steamapps\common"))
    # Steam has used both spellings across the two titles, and a wrong guess
    # only costs a path that does not exist.
    for library in steam_libraries:
        for title in (
            "Microsoft Flight Simulator 2024",
            "MicrosoftFlightSimulator2024",
            "Microsoft Flight Simulator",
            "MicrosoftFlightSimulator",
        ):
            candidates.append(library / title / "SimConnect.dll")

    local = os.environ.get("LOCALAPPDATA")
    if local:
        packages = Path(local) / "Packages"
        for package in (
            "Microsoft.Limitless_8wekyb3d8bbwe",  # MSFS 2024, Microsoft Store
            "Microsoft.FlightSimulator_8wekyb3d8bbwe",  # MSFS 2020, Microsoft Store
        ):
            candidates.append(packages / package / "LocalCache" / "SimConnect.dll")
            candidates.append(packages / package / "LocalState" / "SimConnect.dll")

    return candidates


@dataclass(slots=True)
class SimConnectStatus:
    """What the doctor needs to explain the current state of the connection."""

    connected: bool = False
    dll_path: str = ""
    unavailable_vars: list[str] = field(default_factory=list)
    last_error: str = ""
    samples: int = 0
    exceptions: int = 0


class SimConnectClient:
    """Subscribes to the aircraft's state and injects control axes."""

    def __init__(self, *, dll_path: str = "", app_name: str = "MSFS FFB Bridge") -> None:
        self.app_name = app_name
        self.configured_dll = dll_path
        self.status = SimConnectStatus()

        self._dll = None
        self._handle = HANDLE()
        self._specs: list[SimVarSpec] = list(NUMERIC_VARS)
        self._pending: dict[int, SimVarSpec] = {}
        """Send IDs of AddToDataDefinition calls, so a rejection names its variable."""
        self._recent: deque[tuple[int, str]] = deque(maxlen=RECENT_PACKETS)
        """Send IDs of everything else, so a rejection can name that too."""
        self._complaints: dict[str, int] = {}
        """How many times each distinct rejection has been seen."""
        self._events: dict[str, int] = {}
        self._event_names: dict[int, str] = {}
        self._next_event_id = FIRST_AXIS_EVENT

        self.latest: FlightTelemetry | None = None
        self.paused = False
        self.title = ""
        self.atc_model = ""
        self._ident_fields = 0
        """How many of TITLE/ATC MODEL actually registered. Reading a field that
        was rejected would mean reading past the end of what the simulator
        actually sent back -- whatever heap memory happens to follow, which
        changes from message to message and looks exactly like the aircraft's
        identity flickering every second."""
        self._elapsed = 0.0

    # --- Connection -------------------------------------------------------

    @property
    def connected(self) -> bool:
        return bool(self._handle)

    def connect(self, *, now: float = 0.0) -> bool:
        """Open the connection and subscribe. False if the simulator is not up."""
        if self.connected:
            return True
        if self._dll is None and not self._load_dll():
            return False

        handle = HANDLE()
        result = self._dll.SimConnect_Open(byref(handle), self.app_name.encode(), None, 0, None, 0)
        if result != S_OK or not handle:
            self.status.last_error = "simulator not running or refused the connection"
            self.status.connected = False
            return False

        self._handle = handle
        self.status.connected = True
        self.status.last_error = ""
        self._specs = list(NUMERIC_VARS)
        self.status.unavailable_vars = []
        self._elapsed = now

        self._build_definitions()
        self._subscribe()
        LOGGER.info("connected to the simulator with %d variables", len(self._specs))
        return True

    def _load_dll(self) -> bool:
        path = find_simconnect_dll(self.configured_dll)
        if path is None:
            self.status.last_error = (
                "SimConnect.dll not found. Install the MSFS SDK, or place the DLL "
                "next to the bridge, or set its path in the configuration."
            )
            return False
        loader = getattr(ctypes, "WinDLL", None)
        if loader is None:
            self.status.last_error = "SimConnect is only available on Windows"
            return False
        try:
            self._dll = loader(str(path))
        except OSError as exc:
            self.status.last_error = f"could not load {path}: {exc}"
            return False
        self.status.dll_path = str(path)
        self._declare_signatures()
        return True

    def _declare_signatures(self) -> None:
        """Pin down argument types so ctypes cannot guess them wrongly."""
        dll = self._dll
        assert dll is not None
        dll.SimConnect_Open.restype = c_long
        dll.SimConnect_Open.argtypes = [
            ctypes.POINTER(HANDLE),
            c_char_p,
            c_void_p,
            DWORD,
            HANDLE,
            DWORD,
        ]
        dll.SimConnect_Close.restype = c_long
        dll.SimConnect_Close.argtypes = [HANDLE]

        dll.SimConnect_AddToDataDefinition.restype = c_long
        dll.SimConnect_AddToDataDefinition.argtypes = [
            HANDLE,
            DWORD,
            c_char_p,
            c_char_p,
            c_int,
            c_float,
            DWORD,
        ]
        dll.SimConnect_ClearDataDefinition.restype = c_long
        dll.SimConnect_ClearDataDefinition.argtypes = [HANDLE, DWORD]

        dll.SimConnect_RequestDataOnSimObject.restype = c_long
        dll.SimConnect_RequestDataOnSimObject.argtypes = [
            HANDLE,
            DWORD,
            DWORD,
            DWORD,
            c_int,
            DWORD,
            DWORD,
            DWORD,
            DWORD,
        ]

        dll.SimConnect_GetNextDispatch.restype = c_long
        dll.SimConnect_GetNextDispatch.argtypes = [
            HANDLE,
            ctypes.POINTER(ctypes.POINTER(SIMCONNECT_RECV)),
            ctypes.POINTER(DWORD),
        ]

        dll.SimConnect_MapClientEventToSimEvent.restype = c_long
        dll.SimConnect_MapClientEventToSimEvent.argtypes = [HANDLE, DWORD, c_char_p]

        dll.SimConnect_TransmitClientEvent.restype = c_long
        dll.SimConnect_TransmitClientEvent.argtypes = [
            HANDLE,
            DWORD,
            DWORD,
            DWORD,
            DWORD,
            DWORD,
        ]

        dll.SimConnect_SubscribeToSystemEvent.restype = c_long
        dll.SimConnect_SubscribeToSystemEvent.argtypes = [HANDLE, DWORD, c_char_p]

        dll.SimConnect_SetNotificationGroupPriority.restype = c_long
        dll.SimConnect_SetNotificationGroupPriority.argtypes = [HANDLE, DWORD, DWORD]

        dll.SimConnect_GetLastSentPacketID.restype = c_long
        dll.SimConnect_GetLastSentPacketID.argtypes = [HANDLE, ctypes.POINTER(DWORD)]

    def close(self) -> None:
        if self._handle and self._dll is not None:
            self._dll.SimConnect_Close(self._handle)
        self._handle = HANDLE()
        self.status.connected = False
        self.latest = None

    # --- Subscriptions ----------------------------------------------------

    def _build_definitions(self) -> None:
        """Register the block of variables and the aircraft identity strings."""
        dll = self._dll
        assert dll is not None
        self._pending.clear()
        dll.SimConnect_ClearDataDefinition(self._handle, DEFINE_NUMERIC)

        for index, spec in enumerate(self._specs):
            result = dll.SimConnect_AddToDataDefinition(
                self._handle,
                DEFINE_NUMERIC,
                spec.name.encode(),
                spec.unit.encode(),
                DATATYPE_FLOAT64,
                0.0,
                index,
            )
            if result != S_OK:
                LOGGER.warning("could not add %s to the definition", spec.name)
                continue
            send_id = DWORD()
            if dll.SimConnect_GetLastSentPacketID(self._handle, byref(send_id)) == S_OK:
                self._pending[send_id.value] = spec

        dll.SimConnect_ClearDataDefinition(self._handle, DEFINE_IDENT)
        self._ident_fields = 0
        for name in (b"TITLE", b"ATC MODEL"):
            result = dll.SimConnect_AddToDataDefinition(
                self._handle, DEFINE_IDENT, name, None, DATATYPE_STRING256, 0.0, 0
            )
            if result != S_OK:
                LOGGER.warning("could not add %s to the identity definition", name.decode())
                break
            self._ident_fields += 1

    def _subscribe(self) -> None:
        dll = self._dll
        assert dll is not None
        dll.SimConnect_RequestDataOnSimObject(
            self._handle,
            REQUEST_NUMERIC,
            DEFINE_NUMERIC,
            OBJECT_ID_USER,
            PERIOD_SIM_FRAME,
            0,
            0,
            0,
            0,
        )
        self._note_packet("the request for the telemetry block")
        # The aircraft only changes when a flight is loaded, so once a second is
        # ample and keeps the strings out of the fast path.
        dll.SimConnect_RequestDataOnSimObject(
            self._handle, REQUEST_IDENT, DEFINE_IDENT, OBJECT_ID_USER, PERIOD_SECOND, 0, 0, 0, 0
        )
        self._note_packet("the request for the aircraft name")
        dll.SimConnect_SubscribeToSystemEvent(self._handle, EVENT_PAUSE, b"Pause")
        self._note_packet("the subscription to Pause")
        dll.SimConnect_SubscribeToSystemEvent(self._handle, EVENT_SIM, b"Sim")
        self._note_packet("the subscription to Sim")

    # --- Message pump -----------------------------------------------------

    def pump(self, now: float) -> None:
        """Drain everything the simulator has queued for us."""
        if not self.connected or self._dll is None:
            return
        self._elapsed = now
        pointer = ctypes.POINTER(SIMCONNECT_RECV)()
        size = DWORD()
        rebuild = False

        for _ in range(256):  # bounded, so a flood cannot stall the force loop
            if (
                self._dll.SimConnect_GetNextDispatch(self._handle, byref(pointer), byref(size))
                != S_OK
            ):
                break
            if not pointer:
                break
            message = pointer.contents
            if message.dwID == RECV_ID_SIMOBJECT_DATA:
                self._on_simobject_data(pointer)
            elif message.dwID == RECV_ID_EVENT:
                self._on_event(pointer)
            elif message.dwID == RECV_ID_EXCEPTION:
                rebuild |= self._on_exception(pointer)
            elif message.dwID == RECV_ID_QUIT:
                LOGGER.info("the simulator closed the connection")
                self.close()
                return

        if rebuild:
            self._rebuild_without_rejected()

    def _on_simobject_data(self, pointer) -> None:
        data = ctypes.cast(pointer, ctypes.POINTER(SIMCONNECT_RECV_SIMOBJECT_DATA)).contents
        base = ctypes.addressof(data) + SIMOBJECT_DATA_OFFSET

        if data.dwRequestID == REQUEST_NUMERIC:
            count = min(data.dwDefineCount, len(self._specs))
            values = (c_double * count).from_address(base)
            self.latest = assemble(
                self._specs[:count],
                list(values),
                t=self._elapsed,
                title=self.title,
                atc_model=self.atc_model,
                paused=self.paused,
            )
            self.status.samples += 1
        elif data.dwRequestID == REQUEST_IDENT:
            if self._ident_fields >= 1:
                self.title = _read_fixed_string(base, 256)
            if self._ident_fields >= 2:
                self.atc_model = _read_fixed_string(base + 256, 256)

    def _on_event(self, pointer) -> None:
        event = ctypes.cast(pointer, ctypes.POINTER(SIMCONNECT_RECV_EVENT)).contents
        if event.uEventID == EVENT_PAUSE:
            self.paused = bool(event.dwData)
        elif event.uEventID == EVENT_SIM:
            # "Sim" going false means the aircraft is not being simulated: a
            # menu, a loading screen, or the end of a flight.
            self.paused = not bool(event.dwData)

    def _note_packet(self, description: str) -> None:
        """Remember what the packet just sent was, so a rejection can name it.

        Without this a rejected axis event is a bare send id, which says only
        that something the simulator did not recognise happened sixty times a
        second. It is one call and it is what turns a mystery into a sentence.
        """
        dll = self._dll
        if dll is None:
            return
        send_id = DWORD()
        if dll.SimConnect_GetLastSentPacketID(self._handle, byref(send_id)) == S_OK:
            self._recent.append((send_id.value, description))

    def _describe(self, send_id: int) -> str:
        for known_id, description in self._recent:
            if known_id == send_id:
                return description
        return f"send id {send_id}"

    def _complain(self, message: str, *args) -> None:
        """Log a repeating fault once, then only on powers of ten.

        A fault in the send loop repeats at the send rate. Logging every one of
        them buries the reason it started under a hundred thousand copies of
        itself, and a log nobody can read is a log nobody reads.
        """
        rendered = message % args
        seen = self._complaints.get(rendered, 0) + 1
        self._complaints[rendered] = seen
        if seen == 1:
            LOGGER.warning("%s", rendered)
        elif seen in (10, 100, 1000, 10000):
            LOGGER.warning("%s (%d times now)", rendered, seen)

    def _on_exception(self, pointer) -> bool:
        """Note a rejected variable. Returns whether the definition needs rebuilding."""
        exception = ctypes.cast(pointer, ctypes.POINTER(SIMCONNECT_RECV_EXCEPTION)).contents
        self.status.exceptions += 1
        spec = self._pending.get(exception.dwSendID)
        name = EXCEPTION_NAMES.get(exception.dwException, str(exception.dwException))

        if spec is None:
            self._complain("simulator reported %s for %s", name, self._describe(exception.dwSendID))
            return False

        LOGGER.warning("the simulator rejected %r (%s); dropping it", spec.name, name)
        self.status.unavailable_vars.append(spec.name)
        self._specs = [s for s in self._specs if s is not spec]
        return True

    def _rebuild_without_rejected(self) -> None:
        """Re-register the definition after dropping variables the sim refused.

        This has to happen as a whole: the returned block is positional, so a
        rejected variable that stayed in the layout would shift every value
        after it and quietly produce forces from the wrong numbers.
        """
        LOGGER.info("rebuilding the data definition with %d variables", len(self._specs))
        self._build_definitions()
        self._subscribe()

    # --- Output -----------------------------------------------------------

    def map_event(self, name: str) -> int:
        """Register a simulator event by name and return its id."""
        if name in self._events:
            return self._events[name]
        if not self.connected or self._dll is None:
            return -1
        event_id = self._next_event_id
        self._next_event_id += 1
        if (
            self._dll.SimConnect_MapClientEventToSimEvent(self._handle, event_id, name.encode())
            != S_OK
        ):
            LOGGER.warning("could not map the event %s", name)
            return -1
        self._note_packet(f"the mapping of {name}")
        self._events[name] = event_id
        self._event_names[event_id] = name
        return event_id

    def transmit(self, event_id: int, value: int) -> bool:
        """Send an event with a signed value.

        The parameter is an unsigned field, so a negative axis position has to
        be passed as its two's complement or the simulator reads it as a very
        large positive number and slams the control to the stop.
        """
        if not self.connected or self._dll is None or event_id < 0:
            return False
        sent = (
            self._dll.SimConnect_TransmitClientEvent(
                self._handle,
                OBJECT_ID_USER,
                event_id,
                DWORD(value & 0xFFFFFFFF),
                GROUP_PRIORITY_HIGHEST,
                EVENT_FLAG_GROUPID_IS_PRIORITY,
            )
            == S_OK
        )
        if sent:
            self._note_packet(self._event_names.get(event_id, f"event {event_id}"))
        return sent


def _read_fixed_string(address: int, size: int) -> str:
    """Read a fixed-width, NUL-padded string out of a returned data block.

    Seen in the wild on ATC MODEL for some default aircraft: the field
    registers and the simulator answers every request for it, but the bytes
    it sends back are uninitialised memory rather than a clean, NUL-padded
    blank. That is indistinguishable from a real string at this level, so it
    has to be caught here -- unprintable content or a UTF-8 decode failure
    is treated as no value at all rather than trusted, since acting on it
    would mean an aircraft's identity, and everything keyed off it, changing
    on every refresh for no real reason.
    """
    raw = ctypes.string_at(address, size)
    text = raw.split(b"\0", 1)[0].decode("utf-8", "replace").strip()
    if not text.isprintable() or "�" in text:
        return ""
    return text


__all__ = [
    "SimConnectClient",
    "SimConnectError",
    "SimConnectStatus",
    "find_simconnect_dll",
]
