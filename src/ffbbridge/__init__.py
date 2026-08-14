"""Force feedback bridge from Microsoft Flight Simulator 2024 to a MOZA R5 wheelbase.

Layout:
    ffbbridge.core  Pure-Python force model, routing and configuration. No OS
                    dependencies, so it runs and is tested on any platform.
    ffbbridge.io    Windows-facing adapters: SDL2 haptics, wheel input, SimConnect.
    ffbbridge.app   Runtime loop, GUI, bench test and diagnostics.
"""

__version__ = "0.1.0"
