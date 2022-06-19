#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt

DATA_FILES = ["out/1655223535.397669-algo_comparison_experiment-BEST.csv",
              "out/1655591787.63358-algo_comparison_experiment-2-lane.csv",
              "out/1655591666.1699452-algo_comparison_experiment-3-lane.csv"]
DATA_FILE = "out/1655223535.397669-algo_comparison_experiment-BEST.csv"

algo_map = {
    "stip": "STIP",
    "qb_im": "AIM",
    "tl": "Traffic Lights"
}


def main():
    dfs = [pd.read_csv(f) for f in DATA_FILES]
    df = pd.read_csv(DATA_FILE)
    plot_x_against_y(df, "algo", "vpm", "delay", ["qb_im", "stip"], x_title="Traffic Density (vpm)",
                     y_title="Delay (s)")
    plot_x_against_y(df, "algo", "vpm", "delay", ["qb_im", "stip", "tl"], x_title="Traffic Density (vpm)",
                     y_title="Delay (s)")
    plot_x_against_y(df, "algo", "vpm", "messages_exchanged", ["qb_im", "stip"], x_title="Traffic Density (vpm)",
                     y_title="Avg # Messages/s")
    plot_x_against_y(df, "algo", "vpm", "time_per_step", ["qb_im", "stip"], x_title="Traffic Density (vpm)",
                     y_title="Avg Time per Step (ms)")
    plot_x_against_y(df, "vpm", "granularity", "time_per_step", x_title="Granularity", y_title="Avg Time per Step (ms)")
    plot_x_against_y(df, "vpm", "granularity", "delay", x_title="Granularity", y_title="Delay (s)")
    plot_multi_frames_x_against_y(dfs, ["1", "2", "3"], "vpm", "delay", x_title="Traffic Density (vpm) / # Lanes",
                                  y_title="Delay (s)")


def plot_multi_frames_x_against_y(dfs, names, x, y, x_title=None, y_title=None, legend_title=None):
    plt.figure(figsize=(4, 3))
    for i, df in enumerate(dfs):
        plt.plot(df[x], df[y], marker='.', label=f"{names[i]}")

    plt.xlabel(x_title if x_title else x)
    plt.ylabel(y_title if y_title else y)

    plt.grid(True)
    if legend_title:
        plt.legend(title=legend_title)
    else:
        plt.legend()
    plt.savefig(f"out/graphs/{x}-vs-{y}-{'-'.join([str(name) for name in names])}.pdf", bbox_inches="tight")
    plt.show()


def plot_x_against_y(df: pd.DataFrame, partition_by, x, y, partition_filter=None, x_title=None, y_title=None,
                     legend_title=None):
    """
    Partitions the dataframe by the column 'partition_by', drawing a separate line
    for each partition, of x against y.

    Can optionally provide a partition_filter, so that only certain partitions have
    a line drawn for them
    """
    plt.figure(figsize=(4, 3))
    partitions = partition_filter if partition_filter else df[partition_by].unique()
    for partition in partitions:
        rows = df[df[partition_by] == partition]
        plt.plot(rows[x], rows[y], marker='.', label=f"{algo_map[partition]}")

    plt.xlabel(x_title if x_title else x)
    plt.ylabel(y_title if y_title else y)

    plt.grid(True)
    if legend_title:
        plt.legend(title=legend_title)
    else:
        plt.legend()
    plt.savefig(f"out/graphs/{x}-vs-{y}-{'-'.join([str(partition) for partition in partitions])}.pdf",
                bbox_inches="tight")
    plt.show()


if __name__ == "__main__":
    main()
