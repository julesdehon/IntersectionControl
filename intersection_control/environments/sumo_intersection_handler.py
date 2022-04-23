from typing import Dict, List, Tuple
import numpy as np
import traci
import sumolib
from intersection_control.core.environment import IntersectionHandler, Trajectory


class SumoIntersectionHandler(IntersectionHandler):
    def __init__(self):
        self.net = sumolib.net.readNet("network/intersection.net.xml", withInternal=True)
        self.routes = list(sumolib.xml.parse_fast("network/intersection.rou.xml", 'route', ['id', 'edges']))
        self.trajectories: Dict[str, Dict[str, Trajectory]] = {
            intersection.getID():
                {
                    self._get_route_through_edge(edge):
                        Trajectory(edge.getMaxSpeed(), [np.array(point) for point in edge.getShape()])
                    for edge in intersection.getInternal()
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

    def _get_route_through_edge(self, edge):
        for route in self.routes:
            edges = route.edges.split()
            inner_connection = self.net.getEdge(edges[0]).getConnections(self.net.getEdge(edges[1]))[0]
            if self.net.getLane(inner_connection.getViaLaneID()).getEdge().getID() == edge.getID():
                return route.id
        return None
