#!/usr/bin/env python3
"""Create a Z-up textured photo board whose drawing surface is at local Z=0."""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image
import trimesh


def create_photo_board(image, output, size_mm=120., thickness_mm=5.35):
    size, thickness = size_mm / 1000, thickness_mm / 1000
    if not np.isfinite([size, thickness]).all() or min(size, thickness) <= 0:
        raise ValueError('Board size and thickness must be positive and finite')
    board = trimesh.creation.box([size, size, thickness])
    board.apply_translation([0, 0, -thickness / 2])
    board.update_faces(board.face_normals[:, 2] < .9)
    board.visual = trimesh.visual.ColorVisuals(board, face_colors=[168, 126, 76, 255])
    vertices = [[-size/2, -size/2, 0], [size/2, -size/2, 0],
                [size/2, size/2, 0], [-size/2, size/2, 0]]
    photo = trimesh.Trimesh(vertices=vertices, faces=[[0, 1, 2], [0, 2, 3]], process=False)
    texture = Image.open(image).convert('RGB')
    texture.thumbnail((1024, 1024))
    material = trimesh.visual.material.PBRMaterial(baseColorTexture=texture,
                  metallicFactor=0, roughnessFactor=.9, doubleSided=True)
    # Planar drawing right is +Y, image up is -X. Trimesh UVs have V=1 at top;
    # its glTF exporter converts these to glTF's upper-left texture origin.
    photo.visual = trimesh.visual.TextureVisuals(uv=[[0, 1], [0, 0], [1, 0], [1, 1]], material=material)
    scene = trimesh.Scene()
    scene.add_geometry(board, node_name='cardboard')
    scene.add_geometry(photo, node_name='photo')
    Path(output).write_bytes(scene.export(file_type='glb'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--size-mm', type=float, default=120.)
    parser.add_argument('--thickness-mm', type=float, default=5.35)
    args = parser.parse_args()
    create_photo_board(args.image, args.output, args.size_mm, args.thickness_mm)
