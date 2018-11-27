import os
from logging import error, warning
from typing import Tuple, Optional, List, Union

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
from numpy.core.multiarray import ndarray
from shapely.geometry import Polygon
from shapely.ops import unary_union

drawable_types = Union[pltpat.Patch, Scenario, LaneletNetwork]
convertible_types = Union[Scenario, LaneletNetwork, List, State]


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
                'draw_icon': True,  # FIXME Without drawing the icon the initial view is to small
                'draw_bounding_box': True,
                'show_label': False,
                'zorder': 2
            },
            'static_obstacle': {
                'zorder': 1
            }
        }
    }
    car_width: float = 0.5
    car_length: float = 1
    # For the following variables see CenterToLeftBottom.ggb
    translation_rho: float = 0.5 * np.sqrt(np.square(car_length) + np.square(car_width))  # h
    gamma: float = np.cos((0.5 * car_length) / translation_rho)  # cos(g / h)


class MathHelp:
    @staticmethod
    def pol2cart(rho: float, phi: float) -> np.array:
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return [x, y]

    @staticmethod
    def center_to_right_bottom_pos(center_pos: ndarray, orientation: float) -> Tuple[float, float]:
        """
        Calculates the coordinates of the right bottom point of a rectangle representing a car.
        :param center_pos: The coordinate of the center point of the rectangle representing a car.
        :param orientation: The orientation in radians.
        :return: The position of the right bottom point of a rectangle representing a car.
        """
        translation_phi: float = np.pi + orientation + DrawConfig.gamma
        return tuple(center_pos + MathHelp.pol2cart(DrawConfig.translation_rho, translation_phi))

    @staticmethod
    def get_all_pos(right_bottom_pos: ndarray, orientation: float) -> List[ndarray]:
        """
        Returns all four corner points of a rectangle representing a car having the given right bottom position and
        orientation. The points are return anti-clockwise starting with the right bottom point.
        :param right_bottom_pos: The right bottom position of the rectangle.
        :param orientation: The orientation of the rectangle.
        :return: The list of the four corners of the rectangle representing a car.
        """
        length_vec: ndarray = MathHelp.pol2cart(DrawConfig.car_length, orientation)
        width_vec: ndarray = MathHelp.pol2cart(DrawConfig.car_width, orientation + np.pi / 2)
        return [right_bottom_pos,
                right_bottom_pos + length_vec,
                right_bottom_pos + length_vec + width_vec,
                right_bottom_pos + width_vec]


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


def is_valid(to_check: object, scenario: Scenario) -> Optional[bool]:
    """
    Checks whether the given position is valid position within the scenario. A position is valid only if there is no
    collision with other traffic participants or any obstacles.
    :param to_check The object to check.
    :param scenario: The scenario where the position and intersection of the object has to be checked in.
    :return True only if all the given positions are allowed in terms of collision freedom in the scenario.
    """
    if isinstance(to_check, pltpat.Rectangle):
        # positions: List[ndarray] = to_check.get_bbox().get_points()
        positions: List[ndarray] = np.array([[to_check.get_x(), to_check.get_y()]])
    else:
        error("Can not check validity of objects of type " + str(type(to_check)))
        return None
    is_within_lane: bool = scenario.lanelet_network.find_lanelet_by_position(positions) != [[]]
    intersects_with_obstacle: bool = False
    for obstacle in scenario.obstacles:
        # FIXME Recognize current time step for checking validity
        if any(map(lambda pos: obstacle.occupancy_at_time(0).shape.contains_point(pos), positions)):
            intersects_with_obstacle = True
            break
    return is_within_lane and not intersects_with_obstacle


def convert_to_drawable(to_draw: convertible_types) -> Optional[drawable_types]:
    """
    Converts the given object to a representation which can be draw on a plot using draw(...).
    :param to_draw: The object to draw.
    :return: The plottable representation of the given object.
    """
    if isinstance(to_draw, (Scenario, LaneletNetwork, List)):
        # TODO Check whether it is List[plottable_types]
        # TODO Check whether it is plottable_types
        converted = to_draw
    elif isinstance(to_draw, State):
        pos = MathHelp.center_to_right_bottom_pos(to_draw.position, to_draw.orientation)

        colors = ['#000000', '#ff0000', '#00ff00', '#0000ff', '#ffff00', '#00ffff', '#ff00ff']
        # colors = ['#000000', '#111111', '#222222', '#333333', '#444444', '#555555', '#666666', '#777777', '#888888', '#999999']
        converted = pltpat.Rectangle(
            pos, DrawConfig.car_length, DrawConfig.car_width, np.math.degrees(to_draw.orientation), fill=False,
            edgecolor=colors[to_draw.time_step % len(colors)])
        # edgecolor=colors[color_index % len(colors)])
    else:
        error("Could not convert an object of type " + str(type(to_draw)) + ".")
        converted = None
    return converted


def draw(converted: drawable_types) -> Optional[Artist]:
    """
    Converts the given object to a drawable object, draws and returns it.
    :param converted: The object to draw.
    :return The object drawn on the current plot or None if it could not be drawn.
    """
    if isinstance(converted, (Scenario, LaneletNetwork, List)):
        # TODO Check whether it is List[plottable_types]
        # TODO Check whether it is plottable_types
        artist = draw_object(converted, draw_params=DrawConfig.draw_params)
    elif isinstance(converted, pltpat.Patch):
        artist = plt.gca().add_patch(converted)
    else:
        error("Could not draw a converted object of type " + str(type(converted)) + ".")
        artist = None

    return artist


def union_to_polygon(drawables: List[drawable_types]) -> Polygon:
    polygons: List[Polygon] = []
    for drawable in drawables:
        if isinstance(drawable, pltpat.Rectangle):
            # noinspection PyTypeChecker
            points: List[ndarray] = MathHelp.get_all_pos(np.asanyarray(drawable.get_xy()), np.radians(drawable.angle))
            points_tuple: List[Tuple[int], ...] = list(map(tuple, points))
            polygon: Polygon = Polygon(points_tuple)
            if polygon.is_valid:
                polygons.append(polygon)
            else:
                warning("Created an invalid polygon.")
        else:
            warning("Can not convert " + str(type(drawable)) + " to a shapely representation.")
    return unary_union(polygons)
