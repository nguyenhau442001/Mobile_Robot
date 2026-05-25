"""Spawn 100 mobile robots in one Genesis scene using parallel environments.

Genesis replicates the scene N times and runs all environments in a single
batched simulation step — no loop over 100 separate entities needed.
"""
import sys
from pathlib import Path

import torch
import genesis as gs
from xacro_loader import urdf_from_xacro

REPO    = Path(__file__).resolve().parents[2]
ROS_PKG = REPO / "mobile_robot_description"
XACRO   = ROS_PKG / "urdf" / "mobile_robot.urdf.xacro"

N_ENVS   = 100
GRID_COLS = 10                  # 10 × 10 grid
SPACING   = 2.0                 # metres between robots

gs.init(backend=gs.metal)

scene = gs.Scene(
    show_viewer=True,
    viewer_options=gs.options.ViewerOptions(
        camera_pos    =(0.0, -30.0, 25.0),
        camera_lookat =(0.0,   0.0,  0.0),
        camera_fov    =60,
    ),
    sim_options=gs.options.SimOptions(dt=0.01),
)

scene.add_entity(gs.morphs.Plane())

with urdf_from_xacro(XACRO, {"mobile_robot_description": ROS_PKG}) as urdf_path:
    robot = scene.add_entity(
        gs.morphs.URDF(file=urdf_path, pos=(0, 0, 0.1)),
    )

    # build N parallel copies of the scene
    scene.build(n_envs=N_ENVS)

    # arrange robots in a 10×10 grid
    cols = torch.arange(N_ENVS) % GRID_COLS
    rows = torch.arange(N_ENVS) // GRID_COLS
    xs   = (cols - GRID_COLS / 2) * SPACING
    ys   = (rows - GRID_COLS / 2) * SPACING
    zs   = torch.full((N_ENVS,), 0.1)
    origins = torch.stack([xs, ys, zs], dim=1).float()  # (100, 3)

    robot.set_pos(origins)

    print(f"Simulating {N_ENVS} robots — press ESC or Ctrl+C to exit")
    while scene.viewer.is_alive():
        scene.step()
