"""One-off measurement script (Phase 1): AABB of each distinct HX5-D20 right-hand
finger STL, in the mesh's local frame (matches MJCF <mesh> local frame since MJCF
applies scale but no rotation to these meshes). Used to derive capsule collision
geoms in models/hand_only.xml.

Run: python3 tests/measure_hand_meshes.py
"""

import pathlib
import trimesh
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
MESH_DIR = REPO_ROOT / "assets" / "robotis_ffw" / "assets" / "hx5_d20" / "hx5_d20_right"
SCALE = 0.001  # matches MJCF <mesh scale="0.001 0.001 0.001">

MESHES = [
    "hx5_d20_base_unit.stl",
    "hx5_d20_thumb_mcp.stl",
    "hx5_d20_thumb_mcp2.stl",
    "hx5_d20_thumb_ip.stl",
    "hx5_d20_thumb_tip.stl",
    "hx5_d20_finger_mcp.stl",
    "hx5_d20_finger_pip.stl",
    "hx5_d20_finger_dip.stl",
    "hx5_d20_finger_tip.stl",
]


def main():
    for fname in MESHES:
        path = MESH_DIR / fname
        mesh = trimesh.load(path, force="mesh")
        verts = mesh.vertices * SCALE
        mn = verts.min(axis=0)
        mx = verts.max(axis=0)
        extent = mx - mn
        center = (mn + mx) / 2
        print(f"{fname}")
        print(f"  min={mn.round(5).tolist()}  max={mx.round(5).tolist()}")
        print(f"  extent={extent.round(5).tolist()}  center={center.round(5).tolist()}")


if __name__ == "__main__":
    main()
