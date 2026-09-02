#!/usr/bin/env python3
"""Plot the C++ benchmark summary without recomputing benchmark statistics."""

import csv
from pathlib import Path

import matplotlib.pyplot as plt


SUMMARY_PATH = Path("results/benchmark_summary.csv")
RESULTS_DIR = Path("results")
MODE_LABELS = {
    "middle_only": "2D-FCFS / MiddleOnly",
    "three_layers": "3D-FCFS / ThreeLayers",
}


def load_summary():
    grouped = {mode: [] for mode in MODE_LABELS}
    with SUMMARY_PATH.open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            mode = row["layer_mode"]
            if mode in grouped:
                grouped[mode].append(row)
    for rows in grouped.values():
        rows.sort(key=lambda row: float(row["arrival_rate"]))
    if any(not rows for rows in grouped.values()):
        raise RuntimeError("benchmark summary must contain both layer modes")
    return grouped


def plot_metric(grouped, mean_field, std_field, ylabel, output_name):
    plt.figure()
    for mode, rows in grouped.items():
        x_values = [float(row["arrival_rate"]) for row in rows]
        means = [float(row[mean_field]) for row in rows]
        standard_deviations = [float(row[std_field]) for row in rows]
        plt.errorbar(
            x_values,
            means,
            yerr=standard_deviations,
            marker="o",
            capsize=3,
            label=MODE_LABELS[mode],
        )
    plt.xlabel("Total arrival rate (UAV/s)")
    plt.ylabel(ylabel)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(RESULTS_DIR / output_name, dpi=150)
    plt.close()


def plot_throughput(grouped):
    plt.figure()
    for mode, rows in grouped.items():
        x_values = [float(row["arrival_rate"]) for row in rows]
        means = [float(row["mean_throughput"]) for row in rows]
        standard_deviations = [float(row["std_throughput"]) for row in rows]
        plt.errorbar(
            x_values,
            means,
            yerr=standard_deviations,
            marker="o",
            capsize=3,
            label=MODE_LABELS[mode],
        )
    ideal_rates = sorted(
        {float(row["arrival_rate"]) for rows in grouped.values() for row in rows}
    )
    plt.plot(ideal_rates, ideal_rates, linestyle="--", label="Ideal throughput")
    plt.xlabel("Total arrival rate (UAV/s)")
    plt.ylabel("Mean departure throughput (UAV/s)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(RESULTS_DIR / "throughput_vs_arrival_rate.png", dpi=150)
    plt.close()


def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    grouped = load_summary()
    plot_metric(
        grouped,
        "mean_average_delay",
        "std_average_delay",
        "Mean completed-UAV delay (s)",
        "average_delay_vs_arrival_rate.png",
    )
    plot_metric(
        grouped,
        "mean_p95_delay",
        "std_p95_delay",
        "Mean P95 completed-UAV delay (s)",
        "p95_delay_vs_arrival_rate.png",
    )
    plot_throughput(grouped)
    print("Wrote three benchmark plots to results/")


if __name__ == "__main__":
    main()
