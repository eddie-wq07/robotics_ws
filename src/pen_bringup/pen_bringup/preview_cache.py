"""Persistent cache of verified offline preview artifacts, never execution approval."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import uuid


FILES = {'job.json', 'robot_preview.json', 'toolpath.json', 'report.json', 'drawing.json',
         'picture.png', 'relief.png', 'relief.json', 'drawing.png',
         'joint_trajectory.json', 'unretimed_robot_preview.json'}
REQUIRED = {'job.json', 'toolpath.json', 'report.json', 'drawing.json'}


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
        # Provenance timestamps do not alter geometry; exact starting joint
        # positions, model revision and all planning options remain in the key.
        value.get('robot_preview', {}).pop('initial_source', None)
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
        body = {'schema': 1, 'request': value, 'input': input_hash, 'profile': profile_hash,
                'sources': sources, 'dependencies': metadata, 'python': sys.version}
        return hashlib.sha256(json.dumps(body, sort_keys=True, allow_nan=False).encode()).hexdigest()

    def restore(self, key, output):
        entry, output = self.directory / key, Path(output)
        copied = []
        try:
            manifest = json.loads((entry / 'manifest.json').read_text())
            records = manifest['files']
            if not isinstance(records, dict) or manifest.get('key') != key or not REQUIRED <= records.keys() or not records.keys() <= FILES:
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

    def save(self, key, output):
        output = Path(output)
        if not all((output / name).is_file() for name in REQUIRED):
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
            (temporary / 'manifest.json').write_text(json.dumps({'key': key, 'files': records}))
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
