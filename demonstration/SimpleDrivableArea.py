import os
from time import sleep
from typing import Tuple, Any, List
from warnings import warn

from commonroad.common.util import Interval
from commonroad.prediction.prediction import Occupancy
import numpy as np

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Rectangle, Shape, Polygon
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.draw_dispatch_cr import draw_object
from matplotlib.animation import FuncAnimation, ArtistAnimation
from matplotlib.artist import Artist
import matplotlib.patches as pltpat
from matplotlib.collections import PathCollection
from matplotlib.lines import Line2D

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
            'trajectory_steps': 0,
            'zorder': 20
        }
    }
}


def convert_dynamic_obstacle_to_artist(obstacle: DynamicObstacle, timestep: int):
    shape: Shape = obstacle.occupancy_at_time(timestep).shape
    artist: Artist = None
    if isinstance(shape, Rectangle):
        left, bottom = shape.center/2
        angle = obstacle.initial_state.orientation * 360 / 2 / np.pi
        artist = plt.gca().add_patch(pltpat.Rectangle((left, bottom), shape.length, shape.width, angle=angle))
    elif isinstance(shape, Polygon):
        artist = plt.gca().add_patch(pltpat.Polygon(np.array(shape.vertices)))
    else:
        warn("Shapes of type " + str(type(shape)) + " are not supported yet")
    return artist


def main():
    figure = plt.figure()
    file_path: os.path = os.path.join(os.getcwd(), 'scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')
    scenario: Scenario = CommonRoadFileReader(file_path).open()[0]
    ego_vehicle: DynamicObstacle = scenario.dynamic_obstacles[1]

    # Draw lanes
    draw_object(scenario.lanelet_network, draw_params=draw_params, plot_limits=[-200, 250, -100, 100])

    # Draw static obstacles
    for obstacle in scenario.static_obstacles:
        draw_object(obstacle, draw_params=draw_params)

    # Animate dynamic obstacles
    artists: List[List[Artist]] = []
    for timestep in range(0, 51):
        frame = []
        for obstacle in scenario.dynamic_obstacles:
            frame.append(convert_dynamic_obstacle_to_artist(obstacle, timestep))
            # frame.append(draw_object(obstacle.occupancy_at_time(timestep)))
        artists.append(frame)

    ani = ArtistAnimation(figure, artists, blit=True, interval=100)  # NOTE The assignment is needed to force execution
    plt.gca().set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
