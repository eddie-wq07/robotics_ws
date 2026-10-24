"""Reload an explicitly supplied, fingerprinted prepared job without replanning."""
import hashlib
import json
import math
from pathlib import Path
import shutil

from .preview_cache import FILES, REQUIRED, checksum


def planner_revision(root):
    files = {}
    for name in ('pen_toolpath', 'pen_surface_geometry', 'pen_image_processing', 'pen_description'):
        for path in sorted((Path(root) / 'src' / name).rglob('*')):
            if path.is_file() and path.suffix in ('.py', '.xacro', '.yaml') and 'test' not in path.parts:
                files[str(path.relative_to(root))] = checksum(path)
    return hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()


def restore_prepared(root, source, request, input_hash, profile_hash, output):
    source = Path(source).resolve()
    if not source.is_relative_to((Path(root) / 'assets/outputs').resolve()):
        raise ValueError('Prepared trajectory must be inside assets/outputs')
    manifest = json.loads((source / 'prepared-manifest.json').read_text())
    signature = {key: request.get(key) for key in
                 ('size_mm', 'surface', 'fill', 'execution_mode', 'execution_backend')}
    if (manifest['request'] != signature or manifest['input_sha256'] != input_hash or
            manifest['profile_sha256'] != profile_hash or
            manifest['planner_revision'] != planner_revision(root)):
        return False
    context = request.get('robot_preview', {})
    joints = context.get('initial_positions', {})
    if context.get('joints') != manifest['joints']:
        return False
    for name, expected in manifest['start_positions'].items():
        value = joints.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or abs(value - expected) > .005:
            return False
    records = manifest['files']
    if not REQUIRED <= records.keys() or not records.keys() <= FILES:
        raise ValueError('Prepared trajectory manifest has invalid files')
    for name, digest in records.items():
        if checksum(source / name) != digest:
            raise ValueError('Saved trajectory changed; prepare it again')
    for name, digest in records.items():
        shutil.copyfile(source / name, Path(output) / name)
        if checksum(Path(output) / name) != digest:
            raise ValueError('Saved trajectory changed while loading')
    return True
