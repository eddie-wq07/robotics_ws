#!/usr/bin/env python3
"""Create a texture-reduced GLB from this scan; requires Pillow and numpy."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import struct

import numpy as np
from PIL import Image


def digest(data):
    return hashlib.sha256(data).hexdigest()


def prepare(source, destination, texture_size):
    if source.resolve() == destination.resolve():
        raise ValueError('Source and destination must differ')
    raw = source.read_bytes()
    if struct.unpack_from('<4sII', raw) != (b'glTF', 2, len(raw)):
        raise ValueError('Invalid GLB header')
    size, kind = struct.unpack_from('<I4s', raw, 12)
    if kind != b'JSON':
        raise ValueError('Missing JSON chunk')
    doc = json.loads(raw[20:20 + size])
    length, kind = struct.unpack_from('<I4s', raw, 20 + size)
    binary = raw[28 + size:]
    if kind != b'BIN\0' or length != len(binary) or len(doc['buffers']) != 1:
        raise ValueError('Expected one embedded binary buffer')
    images = {im['bufferView']: im for im in doc['images']}
    normal_views = {doc['images'][doc['textures'][m['normalTexture']['index']]['source']]['bufferView']
                    for m in doc['materials'] if 'normalTexture' in m}
    output = bytearray()
    textures, geometry = [], []
    for index, view in enumerate(doc['bufferViews']):
        if view.get('buffer', 0) != 0 or 'extensions' in view:
            raise ValueError('Expected uncompressed buffer views')
        start = view.get('byteOffset', 0)
        data = binary[start:start + view['byteLength']]
        if len(data) != view['byteLength']:
            raise ValueError('Truncated buffer view')
        if index in images:
            with Image.open(io.BytesIO(data)) as image:
                image.verify()
            with Image.open(io.BytesIO(data)) as image:
                original_size = list(image.size)
                image = image.convert('RGB')
                image.thumbnail((texture_size, texture_size), Image.Resampling.LANCZOS)
                if index in normal_views:
                    vectors = np.asarray(image, dtype=np.float32) / 127.5 - 1
                    vectors /= np.maximum(np.linalg.norm(vectors, axis=2, keepdims=True), 1e-8)
                    image = Image.fromarray(np.rint((vectors + 1) * 127.5).clip(0, 255).astype(np.uint8))
                encoded = io.BytesIO()
                image.save(encoded, format='PNG')
                data = encoded.getvalue()
                textures.append({'name': images[index].get('name'), 'source_size_px': original_size,
                                 'size_px': list(image.size), 'bytes': len(data)})
                images[index]['mimeType'] = 'image/png'
        else:
            geometry.append({'buffer_view': index, 'bytes': len(data), 'sha256': digest(data)})
        output.extend(b'\0' * (-len(output) % 4))
        view['byteOffset'], view['byteLength'] = len(output), len(data)
        output.extend(data)
    doc['buffers'][0]['byteLength'] = len(output)
    encoded = json.dumps(doc, separators=(',', ':')).encode()
    encoded += b' ' * (-len(encoded) % 4)
    output.extend(b'\0' * (-len(output) % 4))
    result = (struct.pack('<4sII', b'glTF', 2, 28 + len(encoded) + len(output))
              + struct.pack('<I4s', len(encoded), b'JSON') + encoded
              + struct.pack('<I4s', len(output), b'BIN\0') + output)
    destination.write_bytes(result)
    primitives = [p for mesh in doc['meshes'] for p in mesh['primitives']]
    manifest = {
        'asset': 'tylenol', 'status': 'validated_visual_asset',
        'source': source.name, 'source_sha256': digest(raw), 'source_bytes': len(raw),
        'file': destination.name, 'sha256': digest(result), 'bytes': len(result),
        'generator': doc['asset'].get('generator'), 'import_date': '2026-10-24',
        'units': 'meters by glTF convention; physical dimensions unverified',
        'up_axis': 'Y for the glTF scene; preserve embedded node rotation',
        'vertex_count': sum(doc['accessors'][p['attributes']['POSITION']]['count'] for p in primitives),
        'triangle_count': sum(doc['accessors'][p['indices']]['count'] // 3 for p in primitives),
        'geometry': geometry, 'textures': textures, 'nodes': doc['nodes'],
        'mesh_local_bounds': [{'min': doc['accessors'][p['attributes']['POSITION']]['min'],
                               'max': doc['accessors'][p['attributes']['POSITION']]['max']}
                              for p in primitives],
        'collision': None, 'mass_kg': None, 'inertia': None,
        'notes': ['Geometry, UVs, normals, tangents, indices and material settings preserved.',
                  'All source textures verified and decoded; normal vectors renormalized after resizing.',
                  'Static visual asset only; no physical scale, collision or dynamics qualification.'],
    }
    destination.with_name('manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({k: manifest[k] for k in ('file', 'bytes', 'sha256')}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--texture-size', type=int, default=2048)
    args = parser.parse_args()
    if args.texture_size < 1:
        parser.error('--texture-size must be positive')
    prepare(args.source, args.destination, args.texture_size)
