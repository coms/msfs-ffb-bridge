"""Diagnostics: answer "why isn't it working" without guesswork.

Every check reports what it looked for and what it found, so a failure points at
the fix rather than just saying no. This is also where the MOZA-specific traps
get called out, because they are configuration problems on the wheelbase that no
amount of correct code on this side can work around.
"""

from __future__ import annotations

import platform
import sys
from dataclasses import dataclass
from enum import StrEnum

from ..core.config import BridgeConfig
from ..io.ffb_sdl import HapticCapabilities, init_sdl, list_devices, select_device
from ..io.simconnect_client import find_simconnect_dll


class Level(StrEnum):
    OK = "ok"
    WARN = "warn"
    FAIL = "fail"
    INFO = "info"


@dataclass(frozen=True, slots=True)
class Finding:
    level: Level
    title: str
    detail: str
    fix: str = ""

    def format(self) -> str:
        marks = {
            Level.OK: "[ ok ]",
            Level.WARN: "[warn]",
            Level.FAIL: "[fail]",
            Level.INFO: "[info]",
        }
        text = f"{marks[self.level]} {self.title}\n        {self.detail}"
        if self.fix:
            text += f"\n        fix: {self.fix}"
        return text


def run_checks(config: BridgeConfig | None = None) -> list[Finding]:
    """Everything that can be checked without a simulator running."""
    config = config or BridgeConfig()
    findings: list[Finding] = [
        Finding(
            Level.INFO,
            "Environment",
            f"Python {sys.version.split()[0]} on {platform.system()} {platform.release()}",
        )
    ]
    findings += _check_platform()
    findings += _check_device(config)
    findings += _check_simconnect(config)
    findings.append(_pit_house_reminder())
    return findings


def _check_platform() -> list[Finding]:
    if platform.system() == "Windows":
        return [Finding(Level.OK, "Platform", "Windows, which is where MSFS and the wheel live.")]
    return [
        Finding(
            Level.WARN,
            "Platform",
            f"{platform.system()} is not Windows. The force model runs anywhere, but "
            "SimConnect and the MOZA driver are Windows only.",
            "Run the bridge on the machine the simulator is on.",
        )
    ]


def _check_device(config: BridgeConfig) -> list[Finding]:
    try:
        init_sdl()
    except Exception as exc:  # noqa: BLE001 - reported, not raised
        return [
            Finding(
                Level.FAIL,
                "SDL",
                f"could not start SDL: {exc}",
                "Reinstall the bridge, or check that SDL2 is present alongside it.",
            )
        ]

    devices = list_devices()
    if not devices:
        return [
            Finding(
                Level.FAIL,
                "Controllers",
                "no game controllers found at all.",
                "Check the wheelbase is powered on and connected, and that Windows "
                "lists it under 'Set up USB game controllers'.",
            )
        ]

    findings = [
        Finding(
            Level.INFO,
            "Controllers",
            "\n        ".join(
                f"{d.name} ({d.num_axes} axes, {d.num_buttons} buttons"
                f"{', force feedback' if d.is_haptic else ', no force feedback'})"
                for d in devices
            ),
        )
    ]

    chosen = select_device(devices, config.device.name_match)
    if chosen is None:
        findings.append(
            Finding(
                Level.FAIL,
                "Wheelbase",
                f"none of the devices report force feedback (looking for "
                f"{config.device.name_match!r}).",
                "In MOZA Pit House, set the force feedback mode to DirectInput.",
            )
        )
        return findings

    findings.append(Finding(Level.OK, "Wheelbase", f"using {chosen.name}."))
    return findings


def describe_capabilities(capabilities: HapticCapabilities) -> list[Finding]:
    """Report what an open device turned out to support.

    Separate from the offline checks because it needs the device open, which the
    running bridge already has.
    """
    findings = [
        Finding(
            Level.INFO,
            "Device features",
            ", ".join(capabilities.describe()) or "none reported",
        ),
        Finding(
            Level.INFO,
            "Effect slots",
            f"{capabilities.max_playing} effects can play at once "
            f"({capabilities.max_effects} can be loaded).",
        ),
    ]
    missing = [
        name
        for name, present in (
            ("constant force", capabilities.has(0x0001)),
            ("spring", capabilities.has(0x0080)),
        )
        if not present
    ]
    if missing:
        findings.append(
            Finding(
                Level.WARN,
                "Missing features",
                f"the wheel does not report: {', '.join(missing)}.",
                "Check the force feedback mode in Pit House; some modes hide "
                "DirectInput effects from applications.",
            )
        )
    if capabilities.max_playing and capabilities.max_playing < 4:
        findings.append(
            Finding(
                Level.WARN,
                "Effect slots",
                f"only {capabilities.max_playing} effects can play at once, so some "
                "vibration will be mixed into the steady force channel instead.",
                "Nothing to fix; the bridge already handles it.",
            )
        )
    return findings


def _check_simconnect(config: BridgeConfig) -> list[Finding]:
    path = find_simconnect_dll(config.device.simconnect_dll)
    if path is None:
        return [
            Finding(
                Level.FAIL,
                "SimConnect",
                "SimConnect.dll was not found in any of the usual places.",
                "Install the free MSFS SDK from inside the simulator, or run "
                "'pip install SimConnect' to obtain the DLL, or set its path in "
                "the configuration.",
            )
        ]
    return [Finding(Level.OK, "SimConnect", f"found {path}.")]


def _pit_house_reminder() -> Finding:
    """The trap that catches everyone.

    MOZA's own spring, damper, friction and inertia are applied on the wheelbase
    itself and override what an application asks for. They are on by default,
    and no amount of correct DirectInput on this side can switch them off.
    """
    return Finding(
        Level.INFO,
        "MOZA Pit House settings",
        "The wheelbase applies its own spring, damping, friction and inertia on top "
        "of anything the bridge sends, and they are enabled by default.",
        "Set force feedback mode to DirectInput; spring, friction and inertia to 0; "
        "damping to about 5-10; rotation to 360-540 degrees; overall strength to "
        "40-60% to begin with.",
    )


def format_report(findings: list[Finding]) -> str:
    lines = ["MSFS FFB Bridge diagnostics", "=" * 52]
    lines += [finding.format() for finding in findings]
    failures = sum(1 for f in findings if f.level is Level.FAIL)
    warnings = sum(1 for f in findings if f.level is Level.WARN)
    lines.append("=" * 52)
    lines.append(
        "All clear."
        if not failures and not warnings
        else f"{failures} problem(s), {warnings} warning(s)."
    )
    return "\n".join(lines)
