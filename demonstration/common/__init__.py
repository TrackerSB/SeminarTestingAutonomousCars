import os
from typing import Tuple, Optional, List, Dict, TypeVar, Any

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario

from common.coords import CoordsHelp
from common.draw import DrawConfig
from common.types import drawable_types


_T = TypeVar('_T')


def flatten_dict_values(dictionary: Dict[Any, List[_T]]) -> List[_T]:
    """
    Returns a flat list of all values of the given dictionary.
    :param dictionary: The dictionary whose value to take.
    :return: The flat list of all values of the dictionary.
    """
    return [item for sublist in dictionary.values() for item in sublist]


def load_scenario(path: str) -> Tuple[Scenario, PlanningProblem]:
    """
    Loads the given common road scenario.
    :param path: The relative path to the common road file to load. The base of the relative path is based on
    os.getcwd().
    :type path: str
    :return: A tuple returning the loaded scenario and the first planning problem found.
    """
    scenario_path: os.path = os.path.join(os.getcwd(), path)
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path) \
        .open()
    if planning_problem_set:
        for _, planning_problem in planning_problem_set.planning_problem_dict.items():
            return scenario, planning_problem


def is_valid(to_check: drawable_types, scenario: Scenario) -> Optional[bool]:
    """
    Checks whether the given position is valid position within the scenario. A position is valid only if there is no
    collision with other traffic participants or any obstacles.
    :param to_check The object to check.
    :param scenario: The scenario where the position and intersection of the object has to be checked in.
    :return True only if all the given positions are allowed in terms of collision freedom in the scenario.
    """
    positions: List[Tuple[float, float]] = CoordsHelp.get_all_pos(to_check, DrawConfig.car_length, DrawConfig.car_width)
    # noinspection PyTypeChecker
    lanes: List[List[int]] = scenario.lanelet_network.find_lanelet_by_position(np.array(positions))
    is_within_lane: bool = all(map(lambda lane: lane != [], lanes))
    intersects_with_obstacle: bool = False
    for obstacle in scenario.obstacles:
        # FIXME Recognize current time step for checking validity
        if any(map(lambda pos: obstacle.occupancy_at_time(0).shape.contains_point(np.array(pos)), positions)):
            intersects_with_obstacle = True
            break
    return is_within_lane and not intersects_with_obstacle
