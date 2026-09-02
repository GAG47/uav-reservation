#!/usr/bin/env python3
"""Plot reference trajectory-debug CSV for manual geometry inspection."""

import csv
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


INPUT_PATH = Path("results/trajectory_debug.csv")
OUTPUT_PATH = Path("results/reference_trajectory_debug.png")


def load_trajectories():
    trajectories = defaultdict(list)
    with INPUT_PATH.open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            trajectories[int(row["uav_id"])].append(
                {
                    "time": float(row["time"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row["z"]),
                    "movement": row["movement"],
                    "source": row["source_direction"],
                    "target": row["target_direction"],
                }
            )
    if not trajectories:
        raise RuntimeError("trajectory_debug.csv contains no samples")
    return trajectories


def main():
    trajectories = load_trajectories()
    figure, (xy_axis, altitude_axis) = plt.subplots(1, 2, figsize=(12, 5))
    for uav_id, samples in sorted(trajectories.items()):
        label = (
            f"UAV {uav_id}: {samples[0]['source']}→{samples[0]['target']} "
            f"{samples[0]['movement']}"
        )
        xy_axis.plot(
            [sample["x"] for sample in samples],
            [sample["y"] for sample in samples],
            label=label,
        )
        altitude_axis.plot(
            [sample["time"] for sample in samples],
            [sample["z"] for sample in samples],
            label=label,
        )

    xy_axis.set_title("Reference trajectories: XY")
    xy_axis.set_xlabel("x (m)")
    xy_axis.set_ylabel("y (m)")
    xy_axis.set_aspect("equal", adjustable="box")
    xy_axis.grid(True, alpha=0.3)

    altitude_axis.set_title("Reference trajectories: altitude")
    altitude_axis.set_xlabel("time (s)")
    altitude_axis.set_ylabel("z (m)")
    altitude_axis.grid(True, alpha=0.3)
    altitude_axis.legend(fontsize="small")

    figure.tight_layout()
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUTPUT_PATH, dpi=150)
    plt.close(figure)
    print(f"Wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
