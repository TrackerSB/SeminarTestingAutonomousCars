import os
from copy import deepcopy
from logging import error, warning
from queue import Queue
from threading import Thread
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
from matplotlib.lines import Line2D
from numpy.core.multiarray import ndarray
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union

from StatesQueue import StatesQueue

drawable_types = Union[pltpat.Patch, Scenario, LaneletNetwork, MultiPolygon]
convertible_types = Union[Scenario, LaneletNetwork, List, State]


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
    def get_all_pos(drawable: drawable_types) -> List[Tuple[float, float]]:
        """
        Returns all corner points in anti-clockwise order.
        :param drawable: The objects whose corner positions have to be calculated.
        :return: The corner positions of the given object.
        """
        if isinstance(drawable, pltpat.Rectangle):
            right_bottom_pos: ndarray = np.asanyarray(drawable.get_xy())
            orientation: float = np.radians(drawable.angle)
            length_vec: ndarray = MathHelp.pol2cart(DrawConfig.car_length, orientation)
            width_vec: ndarray = MathHelp.pol2cart(DrawConfig.car_width, orientation + np.pi / 2)
            positions = [tuple(right_bottom_pos),
                         tuple(right_bottom_pos + length_vec),
                         tuple(right_bottom_pos + length_vec + width_vec),
                         tuple(right_bottom_pos + width_vec)]
        else:
            raise Exception("Objects of type " + str(type(drawable)) + " are not supported.")
        return positions


class GenerationConfig:
    max_yaw: float = np.pi / 16  # 11.25°
    yaw_steps: int = 8
    num_threads: int = 8
    position_threshold = 0.5
    angle_threshold = max_yaw * 0.85  # NOTE Needs to be way smaller than max_yaw otherwise the car tends to the right.


class GenerationHelp:
    @staticmethod
    def generate_states(scenario: Scenario, planning_problem: PlanningProblem, time_steps: int) \
            -> Tuple[List[drawable_types], int]:
        """
        Generates all positions the ego vehicle can have within the next steps in the given scenario and considering the
        given preplanning problem.
        :param scenario: The scenario the ego vehicle is driving in.
        :param planning_problem: The preplanning problem to solve.
        :param time_steps: The number of steps to simulate.
        :return: A tuple returning all generated valid states and the number of total states processed.
        """
        num_states_processed: int = 0
        valid_converted: List = []

        def generate_next_states() -> None:
            nonlocal num_states_processed
            while True:
                state: State = current_states.get()
                if state.time_step < time_steps:
                    yaw_steps: Union[ndarray, Tuple[ndarray, Optional[float]]] \
                        = np.linspace(-GenerationConfig.max_yaw, GenerationConfig.max_yaw,
                                      num=GenerationConfig.yaw_steps, endpoint=True)
                    for yaw in yaw_steps:
                        transformed: State = deepcopy(state)
                        transformed.orientation += yaw
                        delta_x: float = np.cos(transformed.orientation) * transformed.velocity * scenario.dt
                        delta_y: float = np.sin(transformed.orientation) * transformed.velocity * scenario.dt
                        transformed.position[0] += delta_x
                        transformed.position[1] += delta_y
                        if transformed not in current_states:
                            converted: drawable_types = DrawHelp.convert_to_drawable(transformed)
                            if is_valid(converted, scenario):
                                valid_converted.append(converted)
                                transformed.time_step += 1
                                current_states.put(transformed)
                                DrawHelp.draw(converted)
                current_states.task_done()
                num_states_processed += 1

        current_states: Queue = StatesQueue(GenerationConfig.position_threshold, GenerationConfig.angle_threshold)
        current_states.put(planning_problem.initial_state)
        for i in range(GenerationConfig.num_threads):
            worker: Thread = Thread(target=generate_next_states, args=(), daemon=True)
            worker.start()

        current_states.join()
        return valid_converted, num_states_processed


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


class DrawHelp:
    @staticmethod
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

    @staticmethod
    def draw(to_draw: drawable_types) -> Optional[Artist]:
        """
        Converts the given object to a drawable object, draws and returns it.
        :param to_draw: The object to draw.
        :return The object drawn on the current plot or None if it could not be drawn.
        """
        if isinstance(to_draw, (Scenario, LaneletNetwork, List)):
            # TODO Check whether it is List[plottable_types]
            # TODO Check whether it is plottable_types
            artist = draw_object(to_draw, draw_params=DrawConfig.draw_params)
        elif isinstance(to_draw, MultiPolygon):
            for line_string in to_draw.boundary:
                coords = line_string.coords
                x_data: List[Tuple[float, float]] = list(map(lambda l: l[0], coords))
                y_data: List[Tuple[float, float]] = list(map(lambda l: l[1], coords))
                artist = plt.gca().add_line(Line2D(x_data, y_data))
        elif isinstance(to_draw, pltpat.Patch):
            artist = plt.gca().add_patch(to_draw)
        else:
            error("Could not draw a converted object of type " + str(type(to_draw)) + ".")
            artist = None

        return artist

    @staticmethod
    def union_to_polygon(drawables: List[drawable_types]) -> MultiPolygon:
        polygons: List[Polygon] = []
        for drawable in drawables:
            if isinstance(drawable, pltpat.Rectangle):
                # noinspection PyTypeChecker
                points: List[ndarray] = MathHelp.get_all_pos(drawable)
                points_tuple: List[Tuple[int], ...] = list(map(tuple, points))
                polygon: Polygon = Polygon(points_tuple)
                if polygon.is_valid:
                    polygons.append(polygon)
                else:
                    warning("Created an invalid polygon.")
            else:
                warning("Can not convert " + str(type(drawable)) + " to a shapely representation.")
        return unary_union(polygons)


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
    positions: List[Tuple[float, float]] = MathHelp.get_all_pos(to_check)
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
