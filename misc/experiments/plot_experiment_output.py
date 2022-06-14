#!/usr/bin/env python
import pandas as pd
import matplotlib.pyplot as plt

DATA_FILE = "../out.txt"


def main():
    df = pd.read_csv(DATA_FILE)
    print(df)
    plot_x_against_y(df, ["qb_im", "stip"], "vpm", "delay")
    plot_x_against_y(df, ["qb_im", "stip", "tl"], "vpm", "delay")
    plot_x_against_y(df, ["qb_im", "stip"], "vpm", "messages_exchanged")
    plot_x_against_y(df, ["qb_im", "stip"], "vpm", "time_per_step")


def plot_x_against_y(df, algos, x, y):
    for algo in algos:
        algo_rows = df[df["algo"] == algo]
        print(algo_rows)
        plt.plot(algo_rows[x], algo_rows[y], marker='o', label=algo)

    plt.title(f"{x} vs {y}")
    plt.xlabel(x)
    plt.ylabel(y)
    plt.grid(True)
    plt.legend()
    plt.savefig(f"{x}-vs-{y}-{'-'.join(algos)}.pdf")
    plt.show()


if __name__ == "__main__":
    main()
