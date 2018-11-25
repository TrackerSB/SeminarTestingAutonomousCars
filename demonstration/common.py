import os
from typing import Tuple, Optional, List

import matplotlib.patches as pltpat
import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad.visualization.draw_dispatch_cr import draw_object
from matplotlib.artist import Artist


class DrawConfig:
    shape_params = {
    }
    draw_params = {
        'time_begin': 0,
        'antialiased': True,
        'scenario': {
            'dynamic_obstacle': {
                'occupancy': {
                    'draw_occupancies': 1
                },
                'shape': shape_params,
                'draw_shape': True,
                'draw_icon': True,
                'draw_bounding_box': True,
                'show_label': False,
                'trajectory_steps': 40,
                'zorder': 20
            }
        }
    }
    car_width: float = 0.5
    car_length: float = 1
    # For the following variables see CenterToLeftBottom.ggb
    translation_rho: float = 0.5 * np.sqrt(np.square(car_length) + np.square(car_width))  # h
    gamma: float = np.cos((0.5 * car_length) / translation_rho)  # cos(g / h)


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
    :rtype bool
    """
    # noinspection PyTypeChecker
    is_within_lane: bool = scenario.lanelet_network.find_lanelet_by_position(np.array([position])) != [[]]
    intersects_with_obstacle: bool = False
    for obstacle in scenario.obstacles:
        if obstacle.occupancy_at_time(0).shape.contains_point(position):
            intersects_with_obstacle = True
            break
    return is_within_lane and not intersects_with_obstacle


def pol2cart(rho: float, phi: float) -> np.array:
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return [x, y]


def convert_and_draw(to_draw: object, color_index: int) -> Optional[Artist]:
    """
    Converts the given object to a drawable object, draws and returns it.
    :param to_draw: The object to draw.
    :type to_draw: object
    :return The object drawn on the current plot or None if it could not be drawn.
    :rtype Optional[Artist]
    """
    # Convert
    if isinstance(to_draw, (Scenario, LaneletNetwork, List)):
        # TODO Check whether it is List[plottable_types]
        # TODO Check whether it is plottable_types
        artist = draw_object(to_draw, draw_params=DrawConfig.draw_params)
    else:
        if isinstance(to_draw, State):
            # Map center position to right bottom position of rectangle
            translation_phi: float = np.pi + to_draw.orientation + DrawConfig.gamma
            pos = to_draw.position + pol2cart(DrawConfig.translation_rho, translation_phi)

            colors = ['#000000', '#ff0000', '#00ff00', '#0000ff', '#ffff00', '#00ffff', '#ff00ff']
            # colors = ['#000000', '#111111', '#222222', '#333333', '#444444', '#555555', '#666666', '#777777', '#888888', '#999999']
            converted = pltpat.Rectangle(
                pos, DrawConfig.car_length, DrawConfig.car_width, np.math.degrees(to_draw.orientation), fill=False,
                edgecolor=colors[to_draw.time_step % len(colors)])
                #edgecolor=colors[color_index % len(colors)])
        else:
            os.error("Could not convert an object of type " + str(type(to_draw)) + ".")
            converted = None

        # Draw
        if isinstance(converted, pltpat.Patch):
            artist = plt.gca().add_patch(converted)
        else:
            os.error("Could not draw a converted object of type " + str(type(converted)) + ".")
            artist = None

    return artist
