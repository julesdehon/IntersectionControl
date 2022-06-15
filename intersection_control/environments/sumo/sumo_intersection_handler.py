from collections import namedtuple
from typing import Dict, List, Tuple, Optional, Set
import numpy as np
import traci
import sumolib
from intersection_control.core.environment import IntersectionHandler, Trajectory


class PointBasedTrajectory(Trajectory):
    def __init__(self, speed_limit: float, points: List[np.ndarray]):
        self._speed_limit = speed_limit
        self.points = points
        self.length = None

    @property
    def speed_limit(self) -> float:
        return self._speed_limit

    def point_at(self, distance: float) -> Tuple[Tuple[float, float], float]:
        if distance < 0:
            raise ValueError("Cannot find point at negative distance")

        (x, y), alpha, _ = self._point_at(distance)
        return (x, y), alpha

    def _point_at(self, distance: float) -> Tuple[Tuple[float, float], float, float]:
        """Same as point_at but also returns the distance moved
        """
        curr_trajectory_slice = 0
        curr = self.points[0].copy()
        total_distance_moved = 0

        while total_distance_moved < distance and curr_trajectory_slice < len(self.points) - 1:
            vector_to_waypoint = self.points[curr_trajectory_slice + 1] - self.points[curr_trajectory_slice]
            dist_to_waypoint = np.linalg.norm(vector_to_waypoint)
            dist_moved = min(distance - total_distance_moved, dist_to_waypoint)
            curr += vector_to_waypoint * (dist_moved / dist_to_waypoint)
            total_distance_moved += dist_moved
            if total_distance_moved != distance:
                curr_trajectory_slice += 1

        if curr_trajectory_slice == len(self.points) - 1:
            vector = self.points[curr_trajectory_slice] - self.points[curr_trajectory_slice - 1]
        else:
            vector = self.points[curr_trajectory_slice + 1] - self.points[curr_trajectory_slice]
        alpha = np.arctan2(vector[1], vector[0])

        return (curr[0], curr[1]), alpha, total_distance_moved

    def get_length(self):
        if self.length is None:
            _, _, self.length = self._point_at(np.inf)
        return self.length


class SumoIntersectionHandler(IntersectionHandler):
    def __init__(self, net: sumolib.net.Net, routes: Dict[str, List[str]]):
        self.net = net
        self.routes = routes
        # TODO: The notion of a trajectory here is capturing two things: routes through the intersection (does the car
        #  want to go left, straight or right?), and also the actual path the vehicle might follow through the
        #  the intersection - but these are actually two distinct things that need to be captured separately:
        #  what are the directions I can go - and if I say I am going one of those directions, how many ways (what are
        #  the trajectories I can follow) to do this? The reason this works here is because this is a single lane
        #  intersection. This would not extend to a multi-lane intersection so this needs to change!
        self.trajectories: Dict[str, Dict[str, Trajectory]] = {
            intersection.getID():
                {
                    f"{self._get_route_through_edge(edge.getID())}-{edge.getIncoming()[0].getID()}":
                        PointBasedTrajectory(edge.getSpeed(), [np.array(point) for point in edge.getShape()])
                    for edge in [self.net.getLane(laneId) for laneId in intersection.getInternal()]
                }
            for intersection in self.net.getNodes() if intersection.getType() == "traffic_light"
        }
        # Can assume the junctions will not move or change shape, so can do this to reduce the number of traci calls
        self._positions = {i_id: traci.junction.getPosition(i_id) for i_id in self.get_ids()}
        self._shapes = {i_id: traci.junction.getShape(i_id) for i_id in self.get_ids()}

    def get_ids(self) -> List[str]:
        return [node.getID() for node in self.net.getNodes() if node.getType() == "traffic_light"]

    def get_width(self, intersection_id: str) -> float:
        shape = self._shapes[intersection_id]
        xs = [x for (x, _) in shape]
        return max(xs) - min(xs)

    def get_height(self, intersection_id: str) -> float:
        shape = self._shapes[intersection_id]
        ys = [y for (y, _) in shape]
        return max(ys) - min(ys)

    def get_position(self, intersection_id: str) -> Tuple[float, float]:
        return self._positions[intersection_id]

    def get_trajectories(self, intersection_id: str) -> Dict[str, Trajectory]:
        return self.trajectories[intersection_id]

    def set_traffic_light_phase(self, intersection_id: str, phases: Tuple[Set[str], Set[str], Set[str]]):
        (g, y, r) = phases
        links = [self._get_route_through_edge(link.getID()) for [(_, _, link)] in
                 traci.trafficlight.getControlledLinks(intersection_id)]
        phase = ["r" if link in r else "y" if link in y else "g" for link in links]
        traci.trafficlight.setRedYellowGreenState(intersection_id, "".join(phase))

    def _get_route_through_edge(self, edge) -> Optional[str]:
        for route, edges in self.routes.items():
            inner_connections = self.net.getEdge(edges[0]).getConnections(self.net.getEdge(edges[1]))
            if edge in {self.net.getLane(ic.getViaLaneID()).getID() for ic in inner_connections}:
                return route
        return None
