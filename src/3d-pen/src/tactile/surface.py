"""Legacy import shim. Active implementation is pen_surface_geometry.surface."""
import importlib
import sys
sys.modules[__name__] = importlib.import_module("pen_surface_geometry.surface")
