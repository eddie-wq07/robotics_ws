"""Throttled, atomic progress updates for an offline preparation worker."""
import json
from pathlib import Path
import time


class ProgressWriter:
    def __init__(self, output):
        self.path = Path(output) / 'progress.json'
        self.stage = None
        self.started = time.monotonic()
        self.last_write = 0.

    def __call__(self, stage, completed=0, total=None):
        now = time.monotonic()
        changed = stage != self.stage
        if changed:
            self.stage, self.started = stage, now
        if not changed and now-self.last_write < .5 and completed != total:
            return
        elapsed = now-self.started
        fraction = min(1., max(0., completed/total)) if total else None
        remaining = elapsed*(total-completed)/completed if total and completed and elapsed >= 2 else None
        data = {'stage': stage, 'completed': completed, 'total': total,
                'fraction': fraction, 'elapsed_seconds': elapsed,
                'eta_seconds': max(0., remaining) if remaining is not None else None}
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(data, allow_nan=False))
        temporary.replace(self.path)
        self.last_write = now
