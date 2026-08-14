#!/usr/bin/env bash
# Launcher for development and for the offline tools.
#
# The wheel and the simulator are Windows-only, so on Linux and macOS this is
# for `simulate`, `replay` and the tests -- which is what they exist for.
# Arguments pass straight through: ./run.sh simulate --csv trace.csv

set -euo pipefail
cd "$(dirname "$0")"

PY=.venv/bin/python

if [ ! -x "$PY" ]; then
    echo "Setting up a Python environment. This happens once and takes a minute."
    if ! command -v python3 >/dev/null 2>&1; then
        echo "python3 was not found. Install Python 3.11 or newer and try again." >&2
        exit 1
    fi
    python3 -m venv .venv
    "$PY" -m pip install --quiet --upgrade pip
    "$PY" -m pip install --quiet -e ".[dev]"
    echo
fi

exec "$PY" -m ffbbridge.app.main "$@"
