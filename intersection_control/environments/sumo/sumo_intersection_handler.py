from typing import Dict, List, Tuple, Optional
import numpy as np
import traci
import sumolib
from intersection_control.core.environment import IntersectionHandler, Trajectory


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
                        Trajectory(edge.getSpeed(), [np.array(point) for point in edge.getShape()])
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
