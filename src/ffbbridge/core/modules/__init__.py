"""Effect modules. Each one turns telemetry into a force contribution.

A module is deliberately small and independent: it sees the telemetry, the wheel
state and the tick context, and returns what it wants the wheel to do. It never
sees the device, the user's gain settings or the other modules.
"""

from __future__ import annotations

from .base import EffectModule, ParamSpec
from .brakes import BrakeShudder
from .buffet import Buffet
from .control_loading import ControlLoading
from .crosswind import Crosswind
from .engine_vibration import EngineVibration
from .gear import GearTransit
from .ground_roll import GroundRoll
from .handoff import HandoffAssist
from .prop_wash import PropWash
from .shimmy import NosewheelShimmy
from .soft_lock import SoftLock
from .steering_feel import SteeringFeel
from .touchdown import Touchdown
from .turbulence import Turbulence

#: Every module the bridge knows about, in the order the GUI lists them.
#: Ground and touchdown effects come first because they are what this build
#: was tuned for.
MODULE_REGISTRY: tuple[type[EffectModule], ...] = (
    GroundRoll,
    Touchdown,
    BrakeShudder,
    NosewheelShimmy,
    SteeringFeel,
    GearTransit,
    EngineVibration,
    Turbulence,
    PropWash,
    Crosswind,
    ControlLoading,
    SoftLock,
    HandoffAssist,
    Buffet,
)

__all__ = [
    "EffectModule",
    "ParamSpec",
    "MODULE_REGISTRY",
    "HandoffAssist",
    "SoftLock",
    "GroundRoll",
    "Touchdown",
    "BrakeShudder",
    "NosewheelShimmy",
    "SteeringFeel",
    "GearTransit",
    "EngineVibration",
    "Turbulence",
    "PropWash",
    "Crosswind",
    "ControlLoading",
    "Buffet",
]
