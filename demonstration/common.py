import os
from typing import Tuple

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario


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


def is_valid_position(position: np.array, scenario: Scenario) -> bool:
    """
    Checks whether the given position is valid position within the scenario. A position is valid only if there is no
    collision with other traffic participants or any obstacles.
    :param position The position to check.
    :type position: ndarray
    :param scenario: The scenario where the position has to be checked in.
    :type scenario: Scenario
    :return True only if the given position is allowed in terms of collision freedom in the scenario.
    """
    is_within_lane: bool = scenario.lanelet_network.find_lanelet_by_position(np.array([position])) != [[]]
    intersects_with_obstacle: bool = False
    for obstacle in scenario.obstacles:
        if obstacle.occupancy_at_time(0).shape.contains_point(position):
            intersects_with_obstacle = True
            break
    return is_within_lane and not intersects_with_obstacle
