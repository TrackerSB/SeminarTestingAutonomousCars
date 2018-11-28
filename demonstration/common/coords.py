from typing import Tuple, List

from matplotlib.patches import Rectangle
from numpy import asanyarray, array
from numpy.core.multiarray import ndarray
from numpy.core.umath import cos, sin, pi, radians, sqrt, square

from common.types import drawable_types


class CoordsHelp:
    @staticmethod
    def pol2cart(rho: float, phi: float) -> ndarray:
        x = rho * cos(phi)
        y = rho * sin(phi)
        return array([x, y])

    @staticmethod
    def center_to_right_bottom_pos(center_pos: ndarray, orientation: float, car_length: float, car_width: float) \
            -> Tuple[float, float]:
        """
        Calculates the coordinates of the right bottom point of a rectangle representing a car.
        :param center_pos: The coordinate of the center point of the rectangle representing a car.
        :param orientation: The orientation in radians.
        :param car_length: The length of the car.
        :param car_width: The width of the car.
        :return: The position of the right bottom point of a rectangle representing a car.
        """
        # For the following variables see CenterToLeftBottom.ggb
        translation_rho: float = 0.5 * sqrt(square(car_length) + square(car_width))  # h
        gamma: float = cos((0.5 * car_length) / translation_rho)  # cos(g / h)
        translation_phi: float = pi + orientation + gamma
        return tuple(center_pos + CoordsHelp.pol2cart(translation_rho, translation_phi))

    @staticmethod
    def get_all_pos(drawable: drawable_types, car_length: float = None, car_width: float = None) \
            -> List[Tuple[float, float]]:
        """
        Returns all corner points in anti-clockwise order.
        :param drawable: The objects whose corner positions have to be calculated.
        :param car_length: Only used if the drawable represents a car.
        :param car_width: Only used if the drawable represents a car.
        :return: The corner positions of the given object.
        """
        if isinstance(drawable, Rectangle):
            if car_length and car_width:
                right_bottom_pos: ndarray = asanyarray(drawable.get_xy())
                orientation: float = radians(drawable.angle)
                length_vec: ndarray = CoordsHelp.pol2cart(car_length, orientation)
                width_vec: ndarray = CoordsHelp.pol2cart(car_width, orientation + pi / 2)
                positions = [tuple(right_bottom_pos),
                             tuple(right_bottom_pos + length_vec),
                             tuple(right_bottom_pos + length_vec + width_vec),
                             tuple(right_bottom_pos + width_vec)]
            else:
                raise Exception("For calculating the points of a car the car length and width is needed.")
        else:
            raise Exception("Objects of type " + str(type(drawable)) + " are not supported.")
        return positions
