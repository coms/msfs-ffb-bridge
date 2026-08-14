"""Entry point for the packaged build.

PyInstaller needs a real script rather than a module, and running the package's
``main`` directly would break its relative imports.
"""

from ffbbridge.app.main import main

if __name__ == "__main__":
    raise SystemExit(main())
