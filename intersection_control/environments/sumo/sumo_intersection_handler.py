from typing import Dict, List, Tuple, Optional
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
    def __init__(self, net_file: str, route_file: str):
        self.net = sumolib.net.readNet(net_file, withInternal=True)
        self.routes = list(sumolib.xml.parse_fast(route_file, 'route', ['id', 'edges']))
        # TODO: The notion of a trajectory here is capturing two things: routes through the intersection (does the car
        #  want to go left, straight or right?), and also the actual path the vehicle might follow through the
        #  the intersection - but these are actually two distinct things that need to be captured separately:
        #  what are the directions I can go - and if I say I am going one of those directions, how many ways (what are
        #  the trajectories I can follow) to do this? The reason this works here is because this is a single lane
        #  intersection. This would not extend to a multi-lane intersection so this needs to change!
        self.trajectories: Dict[str, Dict[str, Trajectory]] = {
            intersection.getID():
                {
                    self._get_route_through_edge(edge):
                        PointBasedTrajectory(edge.getSpeed(), [np.array(point) for point in edge.getShape()])
                    for edge in [self.net.getLane(laneId) for laneId in intersection.getInternal()]
                }
            for intersection in self.net.getNodes() if intersection.getType() == "traffic_light"
        }

    def get_ids(self) -> List[str]:
        return [node.getID() for node in self.net.getNodes() if node.getType() == "traffic_light"]

    def get_width(self, intersection_id: str) -> float:
        shape = traci.junction.getShape(intersection_id)
        xs = [x for (x, _) in shape]
        return max(xs) - min(xs)

    def get_height(self, intersection_id: str) -> float:
        shape = traci.junction.getShape(intersection_id)
        ys = [y for (y, _) in shape]
        return max(ys) - min(ys)

    def get_position(self, intersection_id: str) -> Tuple[float, float]:
        return traci.junction.getPosition(intersection_id)

    def get_trajectories(self, intersection_id: str) -> Dict[str, Trajectory]:
        return self.trajectories[intersection_id]

    def _get_route_through_edge(self, edge) -> Optional[str]:
        for route in self.routes:
            edges = route.edges.split()
            inner_connection = self.net.getEdge(edges[0]).getConnections(self.net.getEdge(edges[1]))[0]
            if self.net.getLane(inner_connection.getViaLaneID()).getID() == edge.getID():
                return route.id
        return None
