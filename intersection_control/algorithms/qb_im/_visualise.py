import math
from typing import Dict, Tuple, FrozenSet

import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms

from intersection_control.algorithms.utils.discretised_intersection import InternalVehicle, Intersection
from intersection_control.environments.sumo.sumo_intersection_handler import PointBasedTrajectory


def draw(intersection: Intersection, vehicle: InternalVehicle, reservations: Dict[int, FrozenSet[Tuple[int, int]]],
         safety_buffer: Tuple[float, float]):
    def animate(i):
        if not vehicle.is_in_intersection():
            return
        for p in list(ax.patches):
            p.remove()
        x, y = vehicle.position
        x -= vehicle.width / 2
        y -= vehicle.length / 2
        car = patches.Rectangle((x, y), vehicle.width, vehicle.length, linewidth=1, edgecolor='b',
                                facecolor='none', zorder=5)
        t = transforms.Affine2D().rotate_around(vehicle.position[0], vehicle.position[1],
                                                vehicle.angle + math.radians(90)) + ax.transData
        car.set_transform(t)

        ax.add_patch(car)

        x -= safety_buffer[0] / 2
        y -= safety_buffer[1] / 2
        buffer = patches.Rectangle((x, y), vehicle.width + safety_buffer[0], vehicle.length + safety_buffer[1],
                                   linewidth=1, edgecolor='k', facecolor='none', zorder=5)
        t = transforms.Affine2D().rotate_around(vehicle.position[0], vehicle.position[1],
                                                vehicle.angle + math.radians(90)) + ax.transData
        buffer.set_transform(t)
        ax.add_patch(buffer)

        tiles = intersection.get_tiles_for_vehicle(vehicle, (2, 2))
        for tile in tiles:
            color_tile(ax, tile, intersection, "gray")

        if i in reservations:
            for reserved_tile in reservations[i]:
                color_tile(ax, reserved_tile, intersection, "lightcoral")

        vehicle.update(0.25)

    fig, ax = plt.subplots()
    fig.canvas.manager.set_window_title('Intersection')
    ax.axis('equal')
    ax.spines['left'].set_position('zero')
    ax.spines['right'].set_color('none')
    ax.spines['bottom'].set_position('zero')
    ax.spines['top'].set_color('none')
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')
    ax.set_xticks(np.arange(-intersection.size[0] / 2, intersection.size[0] / 2,
                            intersection.size[0] / intersection.granularity))
    ax.set_yticks(np.arange(-intersection.size[1] / 2, intersection.size[1] / 2,
                            intersection.size[1] / intersection.granularity))
    ax.set_yticklabels([])
    ax.set_xticklabels([])
    ax.set_xlim((-intersection.size[0] / 2 - 1, intersection.size[0] / 2 + 1))
    ax.set_ylim((-intersection.size[1] / 2 - 1, intersection.size[1] / 2 + 1))
    ax.grid()

    _ = FuncAnimation(fig, animate, frames=30, interval=500, repeat=False)

    plt.show()


def color_tile(ax, tile, intersection: Intersection, color):
    i, j = tile
    x_coord = (i * (intersection.size[0] / intersection.granularity)) - intersection.size[0] / 2
    y_coord = (j * (intersection.size[1] / intersection.granularity)) - intersection.size[1] / 2
    rect = patches.Rectangle((x_coord, y_coord), intersection.size[0] / intersection.granularity,
                             intersection.size[1] / intersection.granularity, linewidth=1, edgecolor='none',
                             facecolor=color, zorder=1)
    ax.add_patch(rect)


def main():
    trajectories = {
        "SN": PointBasedTrajectory(10, [np.array((10., -30.)), np.array((10., 30.))]),
        "NS": PointBasedTrajectory(10, [np.array((-10., 30.)), np.array((-10., -30.))]),
        "EW": PointBasedTrajectory(10, [np.array((30., 10.)), np.array((-30., 10.))]),
        "WE": PointBasedTrajectory(10, [np.array((-30., -10.)), np.array((30., -10.))]),
        "WN": PointBasedTrajectory(10, [np.array((-30., -10.)), np.array((-8., -4.)),
                                        np.array((6., 12.)), np.array((10., 30.))])
    }
    intersection = Intersection(60, 60, (0, 0), 25, trajectories)
    reserved_vehicle = InternalVehicle(10,  # velocity
                                       5,  # length
                                       2,  # width
                                       "EW",  # arrival_lane
                                       intersection)  # intersection
    safety_buffer = (0.5, 1)

    reservations: Dict[int, FrozenSet[Tuple[int, int]]] = {}
    i = 0
    while reserved_vehicle.is_in_intersection():
        reservations[i] = intersection.get_tiles_for_vehicle(reserved_vehicle, safety_buffer)
        reserved_vehicle.update(0.25)
        i += 1

    vehicle = InternalVehicle(10,  # velocity
                              5,  # length
                              2,  # width
                              "WN",  # arrival_lane
                              intersection)  # intersection

    draw(intersection, vehicle, reservations, safety_buffer)


if __name__ == "__main__":
    main()
