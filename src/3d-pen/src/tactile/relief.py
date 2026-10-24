"""Legacy import shim. Active implementation is pen_image_processing.relief."""
import importlib
import sys
sys.modules[__name__] = importlib.import_module("pen_image_processing.relief")
