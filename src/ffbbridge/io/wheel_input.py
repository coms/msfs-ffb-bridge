"""Reading the wheel's position and buttons.

The bridge reads the wheel directly rather than letting the simulator do it.
That is what makes the ground/air axis switch possible at all -- the same
physical input has to be routed to different simulator axes depending on the
phase of flight, which cannot be expressed as a binding.

Raw values only: calibration lives in :class:`~ffbbridge.core.config.WheelConfig`
and is applied by the router, so the same reading can be shaped differently for
the steering and aileron axes.
"""

from __future__ import annotations

import logging

from sdl2 import (
    SDL_JoystickGetAxis,
    SDL_JoystickGetButton,
    SDL_JoystickNumAxes,
    SDL_JoystickNumButtons,
    SDL_JoystickUpdate,
)

from ..core.filters import RateOfChange
from ..core.telemetry import WheelState

LOGGER = logging.getLogger(__name__)

#: SDL reports axes as signed 16-bit. Dividing by 32767 rather than 32768 means
#: a wheel at full lock reads exactly 1.0.
AXIS_SCALE = 32767.0

#: More buttons than this are ignored; wheel rims do not have hundreds and the
#: tuple is rebuilt every tick.
MAX_BUTTONS = 64


class WheelReader:
    """Samples one joystick's steering axis and buttons."""

    def __init__(self, joystick, *, axis_index: int = 0) -> None:
        self.joystick = joystick
        self.axis_index = axis_index
        self._velocity = RateOfChange(smoothing=0.02)
        self._num_axes = SDL_JoystickNumAxes(joystick) if joystick else 0
        self._num_buttons = min(SDL_JoystickNumButtons(joystick) if joystick else 0, MAX_BUTTONS)
        if self.axis_index >= self._num_axes:
            LOGGER.warning(
                "axis %d requested but the device reports only %d; using axis 0",
                self.axis_index,
                self._num_axes,
            )
            self.axis_index = 0

    @property
    def num_buttons(self) -> int:
        return self._num_buttons

    def read(self, dt: float) -> WheelState:
        """Sample the device. Must be called from the thread that owns SDL."""
        if self.joystick is None or self._num_axes == 0:
            return WheelState(connected=False)

        SDL_JoystickUpdate()
        raw = SDL_JoystickGetAxis(self.joystick, self.axis_index)
        position = max(-1.0, min(1.0, raw / AXIS_SCALE))
        velocity = self._velocity.update(position, dt)
        buttons = tuple(
            bool(SDL_JoystickGetButton(self.joystick, index)) for index in range(self._num_buttons)
        )
        return WheelState(
            position=position,
            velocity=velocity,
            buttons=buttons,
            connected=True,
        )

    def reset(self) -> None:
        self._velocity.reset()
