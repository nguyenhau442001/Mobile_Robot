from pathlib import Path

import genesis as gs
from xacro_loader import urdf_from_xacro

REPO = Path(__file__).resolve().parents[2]
ROS_PKG = REPO / "mobile_robot_description"
XACRO = ROS_PKG / "urdf" / "mobile_robot.urdf.xacro"

gs.init(backend=gs.metal)

scene = gs.Scene(
    show_viewer=True,
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(3.5, -2.0, 2.5),
        camera_lookat=(0.0, 0.0, 0.3),
        camera_fov=40,
    ),
    sim_options=gs.options.SimOptions(dt=0.01),
)

plane = scene.add_entity(gs.morphs.Plane())

with urdf_from_xacro(XACRO, {"mobile_robot_description": ROS_PKG}) as urdf_path:
    robot = scene.add_entity(
        gs.morphs.URDF(
            file=urdf_path,
            pos=(0, 0, 0.1),
            euler=(0, 0, 0),
        )
    )

    scene.build()

    print("Robot loaded — press ESC or Ctrl+C to exit")
    while scene.viewer.is_alive():
        scene.step()
