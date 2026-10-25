"""Discover saved drawing profiles and resolve selections without touching hardware."""
import hashlib
import os
from pathlib import Path
import re
import tempfile

import yaml


class ProfileCatalog:
    def __init__(self, root, startup=None):
        self.directory = Path(root) / 'src/pen_description/config'
        self.startup = Path(startup).resolve() if startup else None
        self._signature = None
        self._records = {}

    def records(self):
        paths = sorted(set(self.directory.glob('*.yaml')) | set(self.directory.glob('*.yml')))
        paths = [p.resolve() for p in paths if p.resolve().parent == self.directory.resolve()]
        if self.startup and self.startup.is_file() and self.startup not in paths:
            paths.append(self.startup)
        signature = tuple((str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in paths)
        if signature == self._signature:
            return self._records
        records = {}
        for path in paths:
            try:
                raw = path.read_bytes()
                data = yaml.safe_load(raw)
                if not isinstance(data, dict):
                    continue
                selection = data.get('selection') or {}
                if not isinstance(selection, dict) or (not selection and path != self.startup):
                    continue
                geometry, workpiece = data.get('surface'), data.get('workpiece') or {}
                if not isinstance(geometry, dict) or not isinstance(workpiece, dict):
                    continue
                kind = geometry.get('type')
                surface = selection.get('surface', kind)
                if surface not in ('plane', 'sphere', 'cylinder', 'photo_board'):
                    continue
                if kind != ('plane' if surface == 'photo_board' else surface):
                    continue
                record = {'id': path.name, 'label': workpiece.get('label') or path.stem,
                          'surface': surface, 'selection': {k: selection[k] for k in ('image', 'size_mm', 'fill') if k in selection},
                          'images': selection.get('images', [selection['image']] if selection.get('image') else []),
                          'sha256': hashlib.sha256(raw).hexdigest(), 'path': path, 'data': data}
                if not isinstance(record['images'], list) or any(not isinstance(n, str) for n in record['images']):
                    continue
                if record['id'] in records:
                    raise ValueError('Duplicate profile filename')
                records[record['id']] = record
            except (OSError, ValueError, TypeError, yaml.YAMLError):
                # Unrelated setup/configuration files are not drawing profiles.
                continue
        self._signature, self._records = signature, records
        return records

    @staticmethod
    def unavailable(record, mode):
        data = record['data']
        if mode == 'real' and (data.get('calibrated') is not True or data.get('fake_only', True)):
            return 'This profile is preview-only; a measured real profile is required.'
        if mode == 'fake' and not (data.get('fake_only') is True and data.get('synthetic') is True):
            return 'This profile requires preview or real mode.'
        return None

    def public(self, mode):
        return [{**{k: r[k] for k in ('id', 'label', 'surface', 'selection', 'images', 'sha256')},
                 'calibrated': r['data'].get('calibrated', False),
                 'fake_only': r['data'].get('fake_only', True),
                 'execution_blockers': self.execution_blockers(r, mode),
                 'available': self.unavailable(r, mode) is None,
                 'unavailable_reason': self.unavailable(r, mode)} for r in self.records().values()]

    @staticmethod
    def execution_blockers(record, mode):
        if mode != 'real':
            return []
        data = record['data']
        reasons = []
        if data.get('calibrated') is not True:
            reasons.append('The profile is marked uncalibrated.')
        if data.get('fake_only') is not False:
            reasons.append('The profile is marked fake-only.')
        if data.get('synthetic', False):
            reasons.append('The profile contains synthetic geometry.')
        if record['surface'] == 'photo_board' and any(
                data.get('routes', {}).get(name, {}).get('qualified') is not True
                for name in ('approach', 'park')):
            reasons.append('Photo-board approach and return routes have not been qualified.')
        missing = [name for name in ('rt', 'clearance', 'pen', 'tool')
                   if data.get('qualification', {}).get(name) is not True]
        if missing:
            reasons.append('Missing physical qualification: ' + ', '.join(missing) + '.')
        if any(data.get('qualified', {}).get(name) is not True for name in ('cartesian', 'rt')):
            reasons.append('Cartesian and timing qualification are required.')
        return reasons

    def update_flags(self, identifier, changes, expected_hash):
        if not isinstance(identifier, str) or not identifier:
            raise ValueError('Select a saved profile before editing flags')
        if (not isinstance(changes, dict) or not changes or
                set(changes) - {'calibrated', 'fake_only'} or
                any(type(value) is not bool for value in changes.values())):
            raise ValueError('Only boolean calibrated and fake_only fields may be updated')
        record = self.resolve({'profile': identifier})
        path = record['path']
        original = path.read_bytes()
        if not isinstance(expected_hash, str) or hashlib.sha256(original).hexdigest() != expected_hash:
            raise ValueError('Profile changed; refresh before editing its flags')
        text = original.decode('utf-8')
        for key, value in changes.items():
            text, count = re.subn(r'^(%s:[ \t]*)(?:true|false)([ \t]*(?:#[^\n]*)?)$' % key,
                                 lambda m: m[1] + str(value).lower() + m[2], text, flags=re.MULTILINE)
            if count != 1:
                raise ValueError(f'{key} must be an explicit top-level YAML boolean')
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, prefix='.profile-flags-',
                                             delete=False, encoding='utf-8') as output:
                temporary = Path(output.name)
                output.write(text)
                output.flush()
                os.fsync(output.fileno())
            temporary.chmod(path.stat().st_mode & 0o777)
            if path.read_bytes() != original:
                raise ValueError('Profile changed; refresh before editing its flags')
            os.replace(temporary, path)
            self._signature = None
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        return self.resolve({'profile': identifier})

    def resolve(self, request):
        records = self.records()
        if request.get('profile'):
            try:
                return records[request['profile']]
            except (KeyError, TypeError) as exc:
                raise ValueError('Unknown saved drawing profile') from exc
        candidates = list(records.values())
        if request.get('surface'):
            candidates = [r for r in candidates if r['surface'] == request['surface']]
        matches = [r for r in candidates if request.get('image') in r['images']]
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            raise ValueError('This photo has multiple saved profiles; choose a profile.')
        fallback = records.get(request.get('fallback_profile'))
        if fallback in candidates:
            return fallback
        # A requested surface wins over a photo association, e.g. apple on a plane.
        canonical = {'plane': 'flat.yaml', 'sphere': 'sphere.yaml', 'cylinder': 'tylenol_sun.yaml', 'photo_board': 'photo_board.yaml'}
        default = records.get(canonical.get(request.get('surface')))
        if default and default in candidates:
            return default
        if len(candidates) == 1:
            return candidates[0]
        startup = next((r for r in candidates if r['path'] == self.startup), None)
        if startup:
            return startup
        if candidates:
            raise ValueError('Choose a saved drawing profile for this photo.')
        raise ValueError('No saved drawing profile matches this selection')
