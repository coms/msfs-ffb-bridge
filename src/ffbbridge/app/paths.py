"""Where the bridge keeps its settings and recordings."""

from __future__ import annotations

import os
import sys
from pathlib import Path

APP_DIR_NAME = "msfs-ffb-bridge"


def config_dir() -> Path:
    """Per-user configuration directory, created on demand."""
    if sys.platform == "win32":
        base = Path(os.environ.get("LOCALAPPDATA", Path.home() / "AppData" / "Local"))
    else:
        base = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    path = base / APP_DIR_NAME
    path.mkdir(parents=True, exist_ok=True)
    return path


def profiles_path() -> Path:
    return config_dir() / "profiles.json"


def recordings_dir() -> Path:
    path = config_dir() / "recordings"
    path.mkdir(parents=True, exist_ok=True)
    return path


def log_path() -> Path:
    return config_dir() / "ffbbridge.log"


def bundled_profile() -> Path | None:
    """The default profile shipped with the application, if it can be found."""
    candidates = [
        Path(getattr(sys, "_MEIPASS", "")) / "profiles" / "default_ga.json",
        Path(__file__).resolve().parents[3] / "profiles" / "default_ga.json",
    ]
    for candidate in candidates:
        try:
            if candidate.is_file():
                return candidate
        except OSError:
            continue
    return None
