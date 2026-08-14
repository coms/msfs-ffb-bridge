"""Injecting the wheel's position into the simulator as a control axis.

The wheel is deliberately left unbound in the simulator's own control settings.
Instead the bridge reads it and sends the value to whichever axis the router has
decided the wheel currently is. That indirection is the whole reason a single
axis can be a rudder on the ground and ailerons in the air.

This is a well-trodden path -- the established external control tools all drive
MSFS axes through the same events -- but it does mean the bridge is responsible
for not flooding the connection, hence the send rate limit and the deadband.
"""

from __future__ import annotations

import logging

from ..core.config import RoutingConfig
from ..core.routing import AxisCommand
from .simconnect_client import SimConnectClient

LOGGER = logging.getLogger(__name__)

#: Full deflection in simulator axis units.
AXIS_FULL_SCALE = 16383

AILERON_EVENT = "AXIS_AILERONS_SET"
RUDDER_EVENT = "AXIS_RUDDER_SET"
STEERING_EVENT = "AXIS_STEERING_SET"

#: Resend an unchanged axis at least this often. The simulator holds the last
#: value it was given, so a reload or a control reset would otherwise leave a
#: stale deflection sitting there until the pilot happened to move the wheel.
KEEPALIVE_S = 1.0


def to_axis_units(value: float) -> int:
    """Map -1..1 onto the simulator's signed axis range."""
    return int(round(max(-1.0, min(1.0, value)) * AXIS_FULL_SCALE))


class AxisOutput:
    """Sends aileron, rudder and tiller positions, no more often than needed."""

    def __init__(self, client: SimConnectClient, routing: RoutingConfig) -> None:
        self.client = client
        self.routing = routing
        self._event_ids: dict[str, int] = {}
        self._last_sent: dict[str, int] = {}
        self._last_send_time: dict[str, float] = {}
        self._next_allowed = 0.0
        self.sends = 0

    def reset(self) -> None:
        """Forget what was last sent, so the next tick transmits unconditionally."""
        self._last_sent.clear()
        self._last_send_time.clear()
        self._next_allowed = 0.0

    def on_reconnect(self) -> None:
        """Event ids belong to a connection and do not survive it."""
        self._event_ids.clear()
        self.reset()

    def send(self, command: AxisCommand, now: float) -> None:
        """Transmit the tick's axis positions, subject to the rate limit."""
        if not self.client.connected:
            return
        if now < self._next_allowed:
            return
        self._next_allowed = now + 1.0 / max(self.routing.send_rate_hz, 1.0)

        self._send_axis(AILERON_EVENT, command.aileron, now)
        self._send_axis(RUDDER_EVENT, command.rudder, now)
        if command.steering is not None:
            self._send_axis(STEERING_EVENT, command.steering, now)

    def _send_axis(self, event_name: str, value: float, now: float) -> None:
        units = to_axis_units(value)
        previous = self._last_sent.get(event_name)
        due = now - self._last_send_time.get(event_name, -KEEPALIVE_S) >= KEEPALIVE_S
        threshold = int(self.routing.axis_deadband * AXIS_FULL_SCALE)

        if previous is not None and abs(units - previous) <= threshold and not due:
            return

        event_id = self._event_ids.get(event_name)
        if event_id is None:
            event_id = self.client.map_event(event_name)
            if event_id < 0:
                return
            self._event_ids[event_name] = event_id

        if self.client.transmit(event_id, units):
            self._last_sent[event_name] = units
            self._last_send_time[event_name] = now
            self.sends += 1
