#!/usr/bin/env python3
"""Inspect STL meshes and suggest the best URDF primitive for each.

Usage:
    python inspect_mesh.py                 # all .STL files
    python inspect_mesh.py chassis lidar   # only the named ones
"""
import sys
from pathlib import Path

import trimesh

# Folder layout assumed:
#   mobile_robot_description/
#     ├── scripts/inspect_mesh.py   <- this file
#     └── meshes/*.STL
MESHES_DIR = Path(__file__).resolve().parent.parent / "meshes"


def inspect(mesh_path):
    """Print bounding box, center of mass, and a suggested URDF primitive."""
    # force="mesh" flattens multi-part STLs so .volume / .area are available.
    mesh = trimesh.load(mesh_path, force="mesh")
    bx, by, bz = mesh.bounding_box.extents
    cx, cy, cz = mesh.center_mass

    # Bounding box CENTER in the mesh's own frame.
    #   mesh.bounds -> 2x3 array: [[xmin, ymin, zmin], [xmax, ymax, zmax]]
    # The center is the midpoint; we need it because primitives in URDF are
    # placed at their <origin>, so if the mesh isn't centered on (0,0,0)
    # the collision primitive needs that same offset to cover it.
    bbox_lo, bbox_hi = mesh.bounds
    ox, oy, oz = (bbox_lo + bbox_hi) / 2.0

    print(f"\n[{mesh_path.name}]")
    print(f"  Bounding box (x,y,z) [m]:  {bx:.4f}  {by:.4f}  {bz:.4f}")
    print(f"  Bounding box center [m]:  {ox:+.4f}  {oy:+.4f}  {oz:+.4f}")
    print(f"  Center of mass      [m]:  {cx:+.4f}  {cy:+.4f}  {cz:+.4f}")
    print(f"  Bounding sphere R   [m]:  {mesh.bounding_sphere.primitive.radius:.4f}")
    print(f"  Surface area       [m²]:  {mesh.area:.6f}")
    # Volume is only meaningful when watertight == True.
    print(f"  Volume             [m³]:  {mesh.volume:.6f}")
    print(f"  Watertight             :  {mesh.is_watertight}")

    suggest_primitive(bx, by, bz, ox, oy, oz)


def suggest_primitive(bx, by, bz, ox=0.0, oy=0.0, oz=0.0):
    """Pick BOX / CYLINDER / SPHERE from bounding-box proportions.

    Heuristic: two extents are "equal" if they agree to within 2% of the
    largest extent. That scales from a 4 cm lidar (~1 mm tol) up to a
    1.3 m chassis (~2.6 cm tol).

      • all three equal   -> SPHERE
      • exactly two equal -> CYLINDER (the odd one out = axis = length)
      • all three differ  -> BOX

    (ox, oy, oz) is the bounding-box center in the mesh frame — used as the
    URDF <origin> so the suggested primitive actually overlaps the mesh.
    """
    tol = 0.02 * max(bx, by, bz)

    def near(a, b):
        return abs(a - b) < tol

    origin_xyz = f'{ox:.4f} {oy:.4f} {oz:.4f}'

    # ---- Sphere: all three roughly equal ------------------------------------
    if near(bx, by) and near(by, bz):
        r = (bx + by + bz) / 6.0  # average of half-extents
        print(f"  Suggested primitive    :  SPHERE  radius={r:.4f}")
        print("  URDF snippet           :  <collision>")
        print(f"                              <origin xyz=\"{origin_xyz}\" rpy=\"0 0 0\"/>")
        print(f"                              <geometry><sphere radius=\"{r:.4f}\"/></geometry>")
        print("                            </collision>")
        return

    # ---- Cylinder: two extents equal => the third is the axis (length) ------
    # URDF cylinders are oriented along +Z by default; X/Y need an rpy rotation.
    if near(bx, by):          # axis = Z
        radius, length, axis_note, rpy = (bx + by) / 4.0, bz, "Z (URDF default)", "0 0 0"
    elif near(by, bz):        # axis = X
        radius, length, axis_note, rpy = (by + bz) / 4.0, bx, "X (rotated)", "0 1.5708 0"
    elif near(bx, bz):        # axis = Y
        radius, length, axis_note, rpy = (bx + bz) / 4.0, by, "Y (rotated)", "1.5708 0 0"
    else:
        # ---- Box: nothing matched -> all three dimensions differ -----------
        print(f"  Suggested primitive    :  BOX  {bx:.4f} x {by:.4f} x {bz:.4f}")
        print("  URDF snippet           :  <collision>")
        print(f"                              <origin xyz=\"{origin_xyz}\" rpy=\"0 0 0\"/>")
        print(
            f"                              <geometry>"
            f"<box size=\"{bx:.4f} {by:.4f} {bz:.4f}\"/>"
            f"</geometry>")
        print("                            </collision>")
        return

    print(
        f"  Suggested primitive    :  CYLINDER  radius={
            radius:.4f}  length={
            length:.4f}  axis={axis_note}")
    print("  URDF snippet           :  <collision>")
    print(f"                              <origin xyz=\"{origin_xyz}\" rpy=\"{rpy}\"/>")
    print(
        f"                              <geometry>"
        f"<cylinder radius=\"{radius:.4f}\" length=\"{length:.4f}\"/>"
        f"</geometry>")
    print("                            </collision>")


# Pick which meshes to inspect: command-line names, or every *.STL by default.
names = sys.argv[1:]
if names:
    targets = [MESHES_DIR / (n if n.lower().endswith(".stl") else f"{n}.STL") for n in names]
else:
    targets = sorted(MESHES_DIR.glob("*.STL"))

print(f"Inspecting {len(targets)} mesh(es) from {MESHES_DIR}")
for path in targets:
    if path.exists():
        inspect(path)
    else:
        print(f"warning: {path} not found", file=sys.stderr)
