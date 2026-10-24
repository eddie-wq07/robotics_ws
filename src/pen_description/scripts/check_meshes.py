#!/usr/bin/env python3
"""Verify installed CAD against the canonical provenance manifest."""
import hashlib
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
manifest = json.loads((root / 'manifest.json').read_text())
known = {item['file']: item for item in manifest['assets']}
for name in ('jig', 'pen_holder', 'pen_holder_combined', 'pen', 'sphere', 'flat', 'tab'):
    filename = name + '.stl'
    item = known[filename]
    if item['units'] != 'm':
        raise ValueError(filename + ': expected metres')
    if hashlib.sha256((root / filename).read_bytes()).hexdigest() != item['sha256']:
        raise ValueError(filename + ': hash differs from provenance manifest')
print('Verified seven pen-cell meshes in metres against manifest.')
