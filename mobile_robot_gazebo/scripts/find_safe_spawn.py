#!/usr/bin/env python3
"""Find a safe spawn pose in a Gazebo .world by maximizing clearance from obstacles.

Parses every <model><pose>...</pose></model> in the given .world file, treats
each one as a point obstacle (excluding ground/wall/roof/lamp), then grid-searches
for the (x, y) that maximizes the minimum distance to any obstacle.

Useful for picking x_pos/y_pos launch args when dropping a robot into an
unfamiliar Gazebo world.
"""

import argparse
import re
import sys
from pathlib import Path

NON_OBSTACLE_KEYWORDS = ("Ground", "Wall", "Roof", "Lamp", "sun")


def extract_obstacles(world_text: str):
    text = re.sub(r"<!--.*?-->", "", world_text, flags=re.S)
    obstacles = []
    for match in re.finditer(r'<model name="([^"]+)">(.*?)</model>', text, re.S):
        name, body = match.group(1), match.group(2)
        if any(kw in name for kw in NON_OBSTACLE_KEYWORDS):
            continue
        pose_match = re.search(r"<pose[^>]*>([^<]+)</pose>", body)
        if not pose_match:
            continue
        parts = pose_match.group(1).split()
        if len(parts) < 2:
            continue
        try:
            x, y = float(parts[0]), float(parts[1])
        except ValueError:
            continue
        obstacles.append((name, x, y))
    return obstacles


def find_best_spawn(obstacles, x_range, y_range, step):
    best_xy = None
    best_clearance = -1.0
    x = x_range[0]
    while x <= x_range[1]:
        y = y_range[0]
        while y <= y_range[1]:
            if obstacles:
                clearance = min(
                    ((x - ox) ** 2 + (y - oy) ** 2) ** 0.5
                    for _, ox, oy in obstacles
                )
            else:
                clearance = float("inf")
            if clearance > best_clearance:
                best_clearance = clearance
                best_xy = (x, y)
            y += step
        x += step
    return best_xy, best_clearance


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("world", help="Path to .world file")
    parser.add_argument("--x-range", nargs=2, type=float, default=None,
                        metavar=("MIN", "MAX"),
                        help="Search range in X (default: obstacle bounding box)")
    parser.add_argument("--y-range", nargs=2, type=float, default=None,
                        metavar=("MIN", "MAX"),
                        help="Search range in Y (default: obstacle bounding box)")
    parser.add_argument("--step", type=float, default=0.5,
                        help="Grid step in metres (default: 0.5)")
    args = parser.parse_args()

    world_path = Path(args.world)
    if not world_path.is_file():
        print(f"error: {world_path} not found", file=sys.stderr)
        sys.exit(1)

    obstacles = extract_obstacles(world_path.read_text())

    print(f"Obstacles ({len(obstacles)}):")
    print(f"{'name':55s} {'x':>7s} {'y':>7s}")
    print("-" * 75)
    for name, x, y in obstacles:
        print(f"{name:55s} {x:7.2f} {y:7.2f}")

    if obstacles:
        xs = [o[1] for o in obstacles]
        ys = [o[2] for o in obstacles]
        x_range = tuple(args.x_range) if args.x_range else (min(xs), max(xs))
        y_range = tuple(args.y_range) if args.y_range else (min(ys), max(ys))
    else:
        x_range = tuple(args.x_range) if args.x_range else (-10.0, 10.0)
        y_range = tuple(args.y_range) if args.y_range else (-10.0, 10.0)

    (bx, by), clearance = find_best_spawn(obstacles, x_range, y_range, args.step)
    print()
    print(f"Best spawn: x_pos:={bx:.2f} y_pos:={by:.2f}  (clearance={clearance:.2f} m)")


if __name__ == "__main__":
    main()
