"""Read-only URDF model for the browser; no hardware is constructed."""
from pathlib import Path
import hashlib
import math
import subprocess
import xml.etree.ElementTree as ET
import yaml


def vec(value, default):
    return [float(x) for x in value.split()] if value else default


def origin(element):
    item = element.find('origin')
    return {'xyz': vec(item.get('xyz'), [0, 0, 0]) if item is not None else [0, 0, 0],
            'rpy': vec(item.get('rpy'), [0, 0, 0]) if item is not None else [0, 0, 0]}


class Model:
    def __init__(self, root, profile=None, mode='preview'):
        self.root, self.profile, self.mode = Path(root), profile, mode
        self.cached = None
        self.profile_key = None

    def get(self):
        try:
            profile = self.profile or self.root / 'src/pen_description/config/fake.yaml'
            key = hashlib.sha256(Path(profile).read_bytes()).hexdigest()
            display_path = self.root / 'src/pen_description/config/visualization.yaml'
            measured_path = self.root / 'src/pen_description/config/calibration.mocha.yaml'
            display_bytes = display_path.read_bytes() if display_path.is_file() else b''
            measured_bytes = measured_path.read_bytes() if measured_path.is_file() else b''
            revision = hashlib.sha256(key.encode() + display_bytes + measured_bytes).hexdigest()
            if self.cached is not None and revision == self.profile_key:
                return self.cached
            result = subprocess.run(['ros2', 'run', 'pen_description', 'render_description',
                                     '--profile', str(profile), '--mode', self.mode],
                                    capture_output=True, text=True, timeout=15)
            if result.returncode:
                raise ValueError(result.stderr[-800:])
            robot = ET.fromstring(result.stdout)
            materials = {item.get('name'): item.find('color') for item in robot.findall('.//material')
                         if item.find('color') is not None}
            profile_data = yaml.safe_load(Path(profile).read_text())
            links, joints = [], []
            for link in robot.findall('link'):
                visuals = []
                for visual in link.findall('visual'):
                    geometry = visual.find('geometry')
                    if geometry is None or not len(geometry):
                        continue
                    shape = geometry[0]
                    data = {'kind': shape.tag, 'name': visual.get('name', ''),
                            **shape.attrib, **origin(visual)}
                    color = visual.find('material/color')
                    if color is None:
                        material = visual.find('material')
                        if material is not None:
                            color = materials.get(material.get('name'))
                    if color is not None:
                        data['rgba'] = vec(color.get('rgba'), [1, 1, 1, 1])
                    if shape.tag == 'mesh':
                        uri = shape.get('filename', '')
                        if not uri.startswith(('package://pen_description/', 'package://flexiv_description/',
                                               'package://realsense2_description/')):
                            continue
                        data['url'] = '/api/mesh/' + uri.removeprefix('package://')
                    visuals.append(data)
                links.append({'name': link.get('name'), 'visuals': visuals})
            for joint in robot.findall('joint'):
                axis = joint.find('axis')
                limit = joint.find('limit')
                joints.append({'name': joint.get('name'), 'type': joint.get('type'),
                               'parent': joint.find('parent').get('link'),
                               'child': joint.find('child').get('link'),
                               'axis': vec(axis.get('xyz'), [1, 0, 0]) if axis is not None else [1, 0, 0],
                               'limit': {key: float(limit.get(key)) for key in ('lower', 'upper', 'velocity')
                                         if limit is not None and limit.get(key) is not None},
                               **origin(joint)})
            synthetic = bool(profile_data.get('synthetic') or profile_data.get('fake_only'))
            display = yaml.safe_load(display_bytes) or {}
            measured = yaml.safe_load(measured_bytes) or {}
            mount = display.get('holder_mount') if synthetic else None
            sphere_mount = display.get('sphere_mount') if synthetic and profile_data.get('surface', {}).get('type') == 'sphere' else None
            flat_mount = display.get('flat_mount') if synthetic and profile_data.get('surface', {}).get('type') == 'plane' else None
            joint_snapshot = display.get('joint_snapshot') if synthetic else None
            home = profile_data.get('home', {})
            if not joint_snapshot and home.get('joint_positions_rad'):
                joint_snapshot = {
                    'robot_sn': profile_data['robot_sn'],
                    'positions_rad': home['joint_positions_rad'],
                    'recorded_utc': home.get('recorded_utc'),
                    'source': 'Recorded Home pose (offline preview, not live feedback)',
                }
            if joint_snapshot:
                positions = joint_snapshot.get('positions_rad')
                if (not isinstance(positions, list) or len(positions) != 7 or
                        any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                            for v in positions) or
                        joint_snapshot.get('robot_sn', '').rsplit('-', 1)[0] != profile_data['robot_sn'].rsplit('-', 1)[0]):
                    raise ValueError('Joint snapshot must contain seven finite angles for this robot model')
                joint_snapshot = {**joint_snapshot, 'joints': {
                    f'{profile_data["robot_sn"]}_joint{i+1}': value for i, value in enumerate(positions)}}
            if mount:
                holder_name = mount.get('holder_mesh_file', 'pen_holder.stl')
                holder_file = (self.root / 'assets/meshes' / holder_name).resolve()
                if holder_file.parent != (self.root / 'assets/meshes').resolve():
                    raise ValueError('Holder mesh must be in the canonical mesh directory')
                flange_file = self.root / 'src/flexiv_description/meshes/Rizon4s/visual/link7.obj'
                for file, field in ((holder_file, 'holder_mesh_sha256'),
                                    (flange_file, 'flange_mesh_sha256')):
                    if hashlib.sha256(file.read_bytes()).hexdigest() != mount[field]:
                        raise ValueError('Display mounting registration does not match the CAD meshes')
            for surface_name, surface_mount in (('sphere', sphere_mount), ('flat', flat_mount)):
                if not surface_mount:
                    continue
                for name in (surface_name, 'jig'):
                    file = self.root / f'assets/meshes/{name}.stl'
                    if hashlib.sha256(file.read_bytes()).hexdigest() != surface_mount[f'{name}_mesh_sha256']:
                        raise ValueError('Surface mounting registration does not match the CAD meshes')
            self.cached = {'available': True, 'links': links, 'joints': joints,
                           'synthetic': synthetic,
                           'robot_sn': profile_data.get('robot_sn'), 'profile_source_sha256': key,
                           'surface': profile_data.get('surface'), 'model_revision': revision,
                           'flange_to_pen_tip': profile_data.get('T_flange_pen_tip'),
                           'visualization': {'holder_mount': mount,
                               'joint_snapshot': joint_snapshot,
                               'sphere_mount': sphere_mount,
                               'flat_mount': flat_mount,
                               'jig_mount': measured.get('T_base_jig') if synthetic else None,
                               'measurements': {'profile_id': measured.get('profile_id'),
                                   'jig_far_edge_m': measured.get('measurements', {}).get('jig_far_edge_in_base_m'),
                                   'jig_note': measured.get('measurements', {}).get('jig_registration_note')}}}
            self.profile_key = revision
        except (OSError, ValueError, subprocess.TimeoutExpired, ET.ParseError) as exc:
            return {'available': False, 'message': str(exc)}
        return self.cached
