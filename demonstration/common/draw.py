from logging import error, warning
from typing import Optional, List, Tuple

from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad.visualization.draw_dispatch_cr import draw_object, plottable_types
from matplotlib.artist import Artist
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle, Patch
from matplotlib.pyplot import gca
from numpy.core.multiarray import ndarray
from numpy.core.umath import degrees
from shapely.geometry import MultiPolygon, Polygon
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
    colors = ['#000000', '#ff0000', '#00ff00', '#0000ff', '#ffff00', '#00ffff', '#ff00ff']
    car_width: float = 0.5
    car_length: float = 1


class DrawHelp:
    @staticmethod
    def convert_to_drawable(to_draw: convertible_types) -> Optional[drawable_types]:
        """
        Converts the given object to a representation which can be draw on a plot using draw(...).
        :param to_draw: The object to draw.
        :return: The plottable representation of the given object.
        """
        if isinstance(to_draw, (Scenario, LaneletNetwork, List)):  # FIXME Use List[plottable_types]
            converted = to_draw
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
    def draw(to_draw: drawable_types) -> Optional[Artist]:
        """
        Converts the given object to a drawable object, draws and returns it.
        :param to_draw: The object to draw.
        :return The object drawn on the current plot or None if it could not be drawn.
        """
        if isinstance(to_draw, (Scenario, LaneletNetwork, List)):  # FIXME Use List[plottable_types]
            artist = draw_object(to_draw, draw_params=DrawConfig.draw_params)
        elif isinstance(to_draw, MultiPolygon):
            for line_string in to_draw.boundary:
                coords = line_string.coords
                x_data: List[Tuple[float, float]] = list(map(lambda l: l[0], coords))
                y_data: List[Tuple[float, float]] = list(map(lambda l: l[1], coords))
                artist = gca().add_line(Line2D(x_data, y_data))
        elif isinstance(to_draw, Patch):
            artist = gca().add_patch(to_draw)
        else:
            error("Could not draw a converted object of type " + str(type(to_draw)) + ".")
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
            else:
                warning("Can not convert " + str(type(drawable)) + " to a shapely representation.")
        return unary_union(polygons)
