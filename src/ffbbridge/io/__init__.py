"""Adapters to the outside world: the wheel, and the simulator.

Everything in here is platform-specific and mostly untestable without hardware,
so it is kept as thin as possible. The one exception is :mod:`ffb_effects`,
which is pure data conversion and is fully covered by tests.
"""
