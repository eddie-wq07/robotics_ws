"""Small JSON boundary between the image pipeline and ROS."""
import json
from pathlib import Path


def write_json(path, data, *, compact=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=None if compact else 2, allow_nan=False) + "\n")
    temporary.replace(path)
