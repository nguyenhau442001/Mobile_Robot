from pathlib import Path

import genesis as gs
import torch
import numpy as np
from enum import Enum
from xacro_loader import urdf_from_xacro

REPO = Path(__file__).resolve().parents[2]
ROS_PKG = REPO / "mobile_robot_description"
XACRO = ROS_PKG / "urdf" / "mobile_robot.urdf.xacro"

# ── Robot States ────────────────────────────────────────


class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    CHARGING = "charging"
    DONE = "done"

# ── Task ────────────────────────────────────────────────


class Task:
    def __init__(self, task_id, pickup, dropoff):
        self.task_id = task_id
        self.pickup = torch.tensor(pickup, dtype=torch.float32)
        self.dropoff = torch.tensor(dropoff, dtype=torch.float32)
        self.assigned = False

# ── Fleet Manager ────────────────────────────────────────


class FleetManager:
    def __init__(self, robots, n_robots):
        self.robots = robots
        self.n_robots = n_robots

        # state tracking for each robot
        self.states = [RobotState.IDLE] * n_robots
        self.goals = [None] * n_robots
        self.tasks = [None] * n_robots

        # task queue
        self.task_queue = []

        # metrics
        self.completed = 0
        self.step_count = 0

    def add_task(self, task):
        self.task_queue.append(task)

    def _get_robot_positions(self):
        # get all robot positions at once (batched)
        return self.robots.get_pos()   # shape: (N, 3)

    def _assign_tasks(self):
        """Assign tasks from queue to idle robots."""
        positions = self._get_robot_positions()

        for i in range(self.n_robots):
            if self.states[i] == RobotState.IDLE and self.task_queue:
                # find nearest unassigned task
                best_task = None
                best_dist = float('inf')

                for task in self.task_queue:
                    if not task.assigned:
                        dist = torch.norm(
                            positions[i, :2] - task.pickup[:2]
                        ).item()
                        if dist < best_dist:
                            best_dist = dist
                            best_task = task

                if best_task:
                    best_task.assigned = True
                    self.tasks[i] = best_task
                    self.goals[i] = best_task.pickup
                    self.states[i] = RobotState.MOVING
                    self.task_queue.remove(best_task)
                    print(f"  [Fleet] Robot {i:03d} → Task {best_task.task_id}")

    def _check_arrivals(self):
        """Check if any robot reached its goal."""
        positions = self._get_robot_positions()

        for i in range(self.n_robots):
            if self.states[i] != RobotState.MOVING:
                continue

            goal = self.goals[i]
            dist = torch.norm(positions[i, :2] - goal[:2]).item()

            if dist < 0.3:  # arrived threshold
                task = self.tasks[i]

                # reached pickup → now go to dropoff
                if torch.allclose(goal, task.pickup, atol=0.3):
                    self.goals[i] = task.dropoff
                    print(f"  [Fleet] Robot {i:03d} picked up → heading to dropoff")

                # reached dropoff → task complete
                else:
                    self.states[i] = RobotState.IDLE
                    self.tasks[i] = None
                    self.goals[i] = None
                    self.completed += 1
                    print(f"  [Fleet] Robot {i:03d} ✅ Task done! "
                          f"Total completed: {self.completed}")

    def _move_robots(self):
        """Step all robots one frame toward their goals (kinematic)."""
        positions = self._get_robot_positions()
        new_pos = positions.clone()

        for i in range(self.n_robots):
            if self.states[i] == RobotState.MOVING and self.goals[i] is not None:
                direction = self.goals[i][:2] - positions[i, :2]
                dist = torch.norm(direction)
                if dist > 0.01:
                    speed = min(1.5, dist.item())   # max 1.5 m/s
                    step_dist = speed * 0.01             # dt = 0.01 s
                    unit_dir = direction / dist
                    new_pos[i, 0] += unit_dir[0] * step_dist
                    new_pos[i, 1] += unit_dir[1] * step_dist

        self.robots.set_pos(new_pos)

    def step(self):
        """Called every simulation step."""
        self.step_count += 1

        # every 50 steps: assign new tasks
        if self.step_count % 50 == 0:
            self._assign_tasks()

        self._check_arrivals()
        self._move_robots()

        # print status every 500 steps
        if self.step_count % 500 == 0:
            idle = sum(1 for s in self.states if s == RobotState.IDLE)
            moving = sum(1 for s in self.states if s == RobotState.MOVING)
            print(f"\n[Step {self.step_count}] "
                  f"Idle: {idle} | Moving: {moving} | "
                  f"Queue: {len(self.task_queue)} | "
                  f"Completed: {self.completed}\n")


# ── Main ─────────────────────────────────────────────────
def main():
    N_ROBOTS = 100
    N_TASKS = 200

    gs.init(backend=gs.metal)

    scene = gs.Scene(
        show_viewer=True,
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0, -20, 25),
            camera_lookat=(0, 0, 0),
            camera_fov=60,
        ),
        sim_options=gs.options.SimOptions(dt=0.01),
    )

    scene.add_entity(gs.morphs.Plane())

    with urdf_from_xacro(XACRO, {"mobile_robot_description": ROS_PKG}) as urdf_path:
        robots = scene.add_entity(
            gs.morphs.URDF(file=urdf_path, pos=(0, 0, 0.1)),
        )

        scene.build(n_envs=N_ROBOTS)

        # set initial positions — 10×10 grid
        cols = torch.arange(N_ROBOTS) % 10
        rows = torch.arange(N_ROBOTS) // 10
        init_pos = torch.stack([
            cols * 1.5 - 7.5,
            rows * 1.5 - 7.5,
            torch.full((N_ROBOTS,), 0.1),
        ], dim=1).float()
        robots.set_pos(init_pos)

        fleet = FleetManager(robots, N_ROBOTS)

        print(f"Generating {N_TASKS} tasks...")
        for t in range(N_TASKS):
            pickup = [np.random.uniform(-8, 8), np.random.uniform(-8, 8), 0.1]
            dropoff = [np.random.uniform(-8, 8), np.random.uniform(-8, 8), 0.1]
            fleet.add_task(Task(task_id=t, pickup=pickup, dropoff=dropoff))

        print(f"Fleet of {N_ROBOTS} robots ready — press ESC or Ctrl+C to exit\n")

        while scene.viewer.is_alive():
            fleet.step()
            scene.step()

            if len(fleet.task_queue) < 20:
                for _ in range(50):
                    pickup = [np.random.uniform(-8, 8), np.random.uniform(-8, 8), 0.1]
                    dropoff = [np.random.uniform(-8, 8), np.random.uniform(-8, 8), 0.1]
                    fleet.add_task(Task(
                        task_id=fleet.completed + len(fleet.task_queue),
                        pickup=pickup,
                        dropoff=dropoff,
                    ))


if __name__ == "__main__":
    main()
