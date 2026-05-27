#!/usr/bin/env python3
"""
Benchmark RTF for ODE/TPE/Bullet/DART in gz-sim.
On Harmonic use gz.transport13 / gz.msgs10.world_stats_pb2 instead.
"""
import time
import argparse
import statistics
from gz.transport import Node
from gz.msgs.world_stats_pb2 import WorldStatistics

rtf_samples = []


def on_stats(msg: WorldStatistics):
    rtf_samples.append(msg.real_time_factor)


def main(duration: int, world: str = "default"):
    node = Node()
    topic = f"/world/{world}/stats"
    node.subscribe(WorldStatistics, topic, on_stats)
    print(f"Collecting RTF samples for {duration}s on {topic}...")
    time.sleep(duration)

    if rtf_samples:
        print("\n=== RTF Benchmark Results ===")
        print(f"  Samples     : {len(rtf_samples)}")
        print(f"  Mean RTF    : {statistics.mean(rtf_samples):.4f}")
        print(f"  Median RTF  : {statistics.median(rtf_samples):.4f}")
        print(f"  Stdev RTF   : {statistics.stdev(rtf_samples):.4f}")
        print(f"  Min RTF     : {min(rtf_samples):.4f}")
        print(f"  Max RTF     : {max(rtf_samples):.4f}")
    else:
        print("No samples received. Is the simulation running?")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=60)
    parser.add_argument("--world", type=str, default="default")
    args = parser.parse_args()
    main(args.duration, args.world)
