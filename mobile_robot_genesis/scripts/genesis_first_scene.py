import genesis as gs

gs.init(backend=gs.metal)

# Create scene
scene = gs.Scene(
    show_viewer = True,
    viewer_options = gs.options.ViewerOptions(
        camera_pos    = (3.5, -1.0, 2.5),
        camera_lookat = (0.0,  0.0, 0.5),
        camera_fov    = 40,
    ),
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
)

# Add ground
plane = scene.add_entity(gs.morphs.Plane())

# Add robot Franka
robot = scene.add_entity(
    gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
)

# Build scene
scene.build()

# Run simulation
print("Scene running — press ESC to escape")
for i in range(1000):
    scene.step()