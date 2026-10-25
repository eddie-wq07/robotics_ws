"""Persistent cache of verified preparation artifacts, never execution approval."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import uuid


FILES = {'job.json', 'robot_preview.json', 'toolpath.json', 'report.json', 'drawing.json',
         'picture.png', 'relief.png', 'relief.json', 'drawing.png',
         'joint_trajectory.json', 'unretimed_robot_preview.json'}
REQUIRED = {'job.json', 'toolpath.json', 'report.json', 'drawing.json'}
START_POSITION_TOLERANCE_RAD = .005


def start_positions_match(expected, actual):
    """Use the same bounded start check for automatic and explicit saved jobs."""
    if not isinstance(expected, dict) or not expected or not isinstance(actual, dict):
        return False
    if expected.keys() != actual.keys():
        return False
    for name, target in expected.items():
        value = actual[name]
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
               for x in (target, value)):
            return False
        if abs(value-target) > START_POSITION_TOLERANCE_RAD:
            return False
    return True


def checksum(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


class PreviewCache:
    def __init__(self, root):
        self.root = Path(root)
        self.directory = self.root / 'assets/outputs/.preview_cache'

    def key(self, request, input_hash, profile_hash):
        value = deepcopy(request)
        # Provenance timestamps do not alter geometry. Preview-only starts are
        # exact; real starts use the bounded comparison against the manifest.
        value.get('robot_preview', {}).pop('initial_source', None)
        # Real jobs already support bounded saved-start reuse. Compare the actual
        # joint values against the manifest on restore, never rounded key bins.
        if request.get('execution_mode') == 'real':
            value.get('robot_preview', {}).pop('initial_positions', None)
        sources = {}
        for package in ('pen_toolpath', 'pen_surface_geometry', 'pen_image_processing', 'pen_description'):
            base = self.root / 'src' / package / package
            for file in sorted(base.rglob('*')):
                if file.is_file() and file.suffix in ('.py', '.yaml'):
                    sources[str(file.relative_to(self.root))] = checksum(file)
        metadata = {}
        for site in (self.root / '.venv-pen/lib').glob('python*/site-packages'):
            for name in ('numpy', 'scipy', 'shapely', 'trimesh', 'pillow', 'opencv_python_headless'):
                for path in site.glob(name+'-*.dist-info/METADATA'):
                    metadata[path.parent.name] = checksum(path)
        body = {'schema': 2, 'request': value, 'input': input_hash, 'profile': profile_hash,
                'sources': sources, 'dependencies': metadata, 'python': sys.version}
        return hashlib.sha256(json.dumps(body, sort_keys=True, allow_nan=False).encode()).hexdigest()

    def restore(self, key, output, request=None):
        entry, output = self.directory / key, Path(output)
        copied = []
        try:
            manifest = json.loads((entry / 'manifest.json').read_text())
            records = manifest['files']
            if not isinstance(records, dict) or manifest.get('key') != key or not REQUIRED <= records.keys() or not records.keys() <= FILES:
                return False
            if (request or {}).get('execution_mode') == 'real':
                positions = request.get('robot_preview', {}).get('initial_positions', {})
                if manifest.get('execution_mode') != 'real' or not start_positions_match(
                        manifest.get('start_positions'), positions):
                    return False
            elif manifest.get('execution_mode') == 'real':
                return False
            job = json.loads((entry / 'job.json').read_text())
            if not isinstance(job, dict) or (job.get('robot_preview_sha256') and 'robot_preview.json' not in records):
                return False
            for name, digest in records.items():
                if checksum(entry / name) != digest:
                    return False
            for name, digest in records.items():
                copied.append(output / name)
                shutil.copyfile(entry / name, output / name)
                if checksum(output / name) != digest:
                    raise ValueError('Cache changed while restoring')
            return True
        except (OSError, ValueError, KeyError, TypeError):
            for file in copied:
                file.unlink(missing_ok=True)
            return False

    def save(self, key, output, request=None):
        output = Path(output)
        if not all((output / name).is_file() for name in REQUIRED):
            return
        positions = (request or {}).get('robot_preview', {}).get('initial_positions', {})
        if (request or {}).get('execution_mode') == 'real' and not start_positions_match(positions, positions):
            return
        self.directory.mkdir(parents=True, exist_ok=True)
        temporary = self.directory / ('.tmp-'+uuid.uuid4().hex)
        entry = self.directory / key
        temporary.mkdir()
        try:
            records = {}
            for name in sorted(FILES):
                if (output / name).is_file():
                    shutil.copyfile(output / name, temporary / name)
                    records[name] = checksum(temporary / name)
            manifest = {'key': key, 'files': records,
                        'execution_mode': (request or {}).get('execution_mode'),
                        'start_positions': positions}
            (temporary / 'manifest.json').write_text(json.dumps(manifest))
            if entry.exists():
                shutil.rmtree(entry)
            temporary.rename(entry)
            # Keep only the most recent eight results. Original job folders
            # remain untouched when an internal cache entry expires.
            entries = sorted((p for p in self.directory.iterdir() if not p.name.startswith('.')),
                             key=lambda p: p.stat().st_mtime, reverse=True)
            for old in entries[8:]:
                shutil.rmtree(old)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)
