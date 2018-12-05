from typing import List, Tuple, Optional

from commonroad.scenario.trajectory import State
from numpy.core.umath import subtract
from numpy.linalg import norm


class DijkstraNode:
    def __init__(self, state: State, is_goal_node: bool = False, costs: float = -1, previous=None):
        """
        Represents a node in a dijkstra search.
        :param state: The commonroad state this node represents.
        :param is_goal_node: If set to True this node represents a goal node.
        :param costs: The current costs to reach this node from the start node. Costs of -1 represent unknown costs.
        :param previous: The predecessor of this node in the Dijkstra search graph.
        """
        self.state = state
        self.is_goal_node = is_goal_node
        self.costs = costs
        self.previous = previous


def dijkstra_search(start_state: State, goal_states: List[State], states: List[State]) \
        -> Optional[Tuple[List[State], float]]:
    # NOTE Per definition there is an edge from state A to state B if and only if A.time_step == B.time_step - 1
    if start_state in states:
        states.remove(start_state)

    # NOTE White and grey nodes refer to their previous node (second element in tuple)
    white_nodes: List[DijkstraNode] = []
    grey_nodes: List[DijkstraNode] = [DijkstraNode(start_state, costs=0)]
    black_nodes: List[DijkstraNode] = []
    for goal_state in goal_states:
        if goal_state in states:
            states.remove(goal_state)
        black_nodes.append(DijkstraNode(goal_state, is_goal_node=True))
    for state in states:
        black_nodes.append(DijkstraNode(state))

    next_white: DijkstraNode = DijkstraNode(None)
    while grey_nodes != [] and not next_white.is_goal_node:
        next_white: DijkstraNode = min(grey_nodes, key=lambda gp: gp.costs)
        grey_nodes.remove(next_white)
        white_nodes.append(next_white)
        for black in black_nodes:
            if next_white.state.time_step == black.state.time_step - 1:
                costs: float = black.costs \
                               + norm(next_white.state.position - black.state.position) \
                               * abs(next_white.state.orientation - black.state.orientation)
                grey_nodes.append(
                    DijkstraNode(black.state, is_goal_node=black.is_goal_node, costs=costs, previous=next_white))
                black_nodes.remove(black)

    if next_white.is_goal_node:
        costs: float = next_white.costs
        path: List[State] = []
        current: DijkstraNode = next_white
        while current.previous:
            path.append(current.state)
            current = current.previous
        return path, costs
