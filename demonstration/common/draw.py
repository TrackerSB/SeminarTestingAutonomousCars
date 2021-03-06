from logging import error, warning
from typing import Optional, List, Tuple, Union

import commonroad
from commonroad.geometry.shape import Shape
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet
from commonroad.scenario.obstacle import StaticObstacle, DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad.visualization.draw_dispatch_cr import draw_object
from matplotlib.artist import Artist
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle, Patch
from matplotlib.pyplot import gca
from numpy.core.multiarray import ndarray
from numpy.core.umath import degrees, pi
from shapely.geometry import MultiPolygon, Polygon, LineString, MultiLineString
from shapely.ops import unary_union

from common.coords import CoordsHelp
from common.types import convertible_types, drawable_types


class DrawConfig:
    shape_params = {
    }
    draw_params = {
        'time_begin': 0,
        'antialiased': True,
        'scenario': {
            'dynamic_obstacle': {
                'occupancy': {
                    'draw_occupancies': 0
                },
                'shape': shape_params,
                'draw_shape': True,
                'draw_icon': True,  # FIXME Without drawing the icon the initial view is to small
                'draw_bounding_box': True,
                'show_label': True,
                'zorder': 2
            },
            'static_obstacle': {
                'zorder': 1
            }
        }
    }
    colors = ['#000000', '#ff0000', '#00ff00', '#0000ff', '#ffff00', '#00ffff', '#ff00ff']
    car_width: float = 1.5
    car_length: float = 2.5


class DrawHelp:
    @staticmethod
    def convert_to_drawable(to_draw: convertible_types, time_step: int = 0) -> Optional[drawable_types]:
        """
        Converts the given object to a representation which can be draw on a plot using draw(...).
        :param to_draw: The object to draw.
        :param time_step: This parameter defines the time step to calculate the occupancy for in the case of
        DynamicObstacles.
        :return: The plottable representation of the given object.
        """
        from common import MyState
        if isinstance(to_draw, (Scenario, LaneletNetwork, Lanelet, StaticObstacle, GoalRegion, List)):  # FIXME Use List[plottable_types]
            converted = to_draw
        elif isinstance(to_draw, DynamicObstacle):
            shape: Shape = to_draw.occupancy_at_time(time_step).shape
            left, bottom = shape.center / 2
            angle: float = to_draw.initial_state.orientation * 360 / 2 / pi
            converted = Rectangle((left, bottom), shape.length, shape.width, angle=angle)
        elif isinstance(to_draw, MyState):
            converted = DrawHelp.convert_to_drawable(to_draw.state)
        elif isinstance(to_draw, State):
            pos = CoordsHelp.center_to_right_bottom_pos(
                to_draw.position, to_draw.orientation, DrawConfig.car_length, DrawConfig.car_width)

            converted = Rectangle(
                pos, DrawConfig.car_length, DrawConfig.car_width, degrees(to_draw.orientation), fill=False,
                edgecolor=DrawConfig.colors[to_draw.time_step % len(DrawConfig.colors)])
        else:
            error("Could not convert an object of type " + str(type(to_draw)) + ".")
            converted = None
        return converted

    @staticmethod
    def draw(to_draw: drawable_types) -> Union[Optional[Artist], list]:
        """
        Converts the given object to a drawable object, draws and returns it.
        :param to_draw: The object to draw.
        :return The object drawn on the current plot or None if it could not be drawn.
        """
        if not to_draw:
            artist = None
        elif isinstance(to_draw, (Scenario, LaneletNetwork, List, GoalRegion, StaticObstacle, commonroad.geometry.shape.Rectangle)):  # FIXME Use List[plottable_types]
            artist = draw_object(to_draw, draw_params=DrawConfig.draw_params)
        elif isinstance(to_draw, DynamicObstacle):
            error("Pass an occupancy of a DynamicObstacle instead of the obstacle itself.")
            artist = None
        elif isinstance(to_draw, MultiLineString):
            artist = []  # In case the MultiLineString had no boundary
            for line_string in to_draw.boundary:
                artist.append(DrawHelp.draw(line_string))
        elif isinstance(to_draw, LineString):
            coords = to_draw.coords
            x_data: List[Tuple[float, float]] = list(map(lambda l: l[0], coords))
            y_data: List[Tuple[float, float]] = list(map(lambda l: l[1], coords))
            artist = gca().add_line(Line2D(x_data, y_data, color='orange'))
        elif isinstance(to_draw, MultiPolygon):
            artist = []  # In case the polygon had no boundary
            for line_string in to_draw.boundary:
                artist.append(DrawHelp.draw(line_string))
        elif isinstance(to_draw, Polygon):
            artist = DrawHelp.draw(to_draw.boundary)
        elif isinstance(to_draw, Patch):
            artist = gca().add_patch(to_draw)
        else:
            error("Drawing " + str(type(to_draw)) + " is not implemented, yet.")
            artist = None

        if isinstance(artist, list) and artist == []:
            artist = None
        return artist

    @staticmethod
    def union_to_polygon(drawables: List[drawable_types]) -> MultiPolygon:
        polygons: List[Polygon] = []
        for drawable in drawables:
            if isinstance(drawable, Rectangle):
                # noinspection PyTypeChecker
                points: List[ndarray] = CoordsHelp.get_all_pos(drawable, DrawConfig.car_length, DrawConfig.car_width)
                points_tuple: List[Tuple[int], ...] = list(map(tuple, points))
                polygon: Polygon = Polygon(points_tuple)
                if polygon.is_valid:
                    polygons.append(polygon)
                else:
                    warning("Created an invalid polygon.")
            elif isinstance(drawable, Polygon):
                polygons.append(drawable)
            elif isinstance(drawable, MultiPolygon):
                for polygon in drawable:
                    polygons.append(polygon)
            else:
                warning("Union with " + str(type(drawable)) + " not implemented, yet.")
        return unary_union(polygons)
