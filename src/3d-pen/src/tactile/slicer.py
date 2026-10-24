"""Legacy import shim. Active implementation is pen_toolpath.legacy_slicer."""
import importlib
import sys
sys.modules[__name__] = importlib.import_module("pen_toolpath.legacy_slicer")
