import os
from functools import partial
from typing import Tuple, List, Optional
from warnings import warn

import matplotlib.patches as pltpat
import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Rectangle, Shape, Polygon
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad.visualization.draw_dispatch_cr import draw_object
from matplotlib.animation import ArtistAnimation
from matplotlib.artist import Artist

draw_object_part = partial(draw_object, draw_params={
    'time_begin': 0,
    'antialiased': True,
    'scenario': {
        'dynamic_obstacle': {
            'occupancy': {
                'draw_occupancies': 1
            },
            'shape': {},
            'draw_shape': True,
            'draw_icon': True,
            'draw_bounding_box': True,
            'show_label': False,
            'trajectory_steps': 0,
            'zorder': 20
        }
    }
})


def convert_to_artist(obstacle: DynamicObstacle, timestep: int) -> Artist:
    shape: Shape = obstacle.occupancy_at_time(timestep).shape
    artist: Artist = None
    if isinstance(shape, Rectangle):
        left, bottom = shape.center / 2
        angle: float = obstacle.initial_state.orientation * 360 / 2 / np.pi
        artist = plt.gca().add_patch(pltpat.Rectangle((left, bottom), shape.length, shape.width, angle=angle))
    elif isinstance(shape, Polygon):
        artist = plt.gca().add_patch(pltpat.Polygon(np.array(shape.vertices)))
    else:
        warn("Shapes of type " + str(type(shape)) + " are not supported yet")
    return artist


def createDynamicObstaclesArtists(dynamic_obstacles: List[DynamicObstacle]) -> List[List[Artist]]:
    artists: List[List[Artist]] = []
    for timestep in range(0, 51):
        frame = []
        for obstacle in dynamic_obstacles:
            frame.append(convert_to_artist(obstacle, timestep))
            # frame.append(draw_object(obstacle.occupancy_at_time(timestep)))
        artists.append(frame)

    return artists


def extendFramesWithPrediction(initial_state: State, frames: List[List[Artist]], delay_time_step_sec: float) \
        -> List[List[Artist]]:
    draw_object_part(initial_state)
    current_state: State = initial_state
    for frame in frames:
        state_point: pltpat.Circle = pltpat.Circle(current_state.position, 2)
        frame.append(plt.gca().add_patch(state_point))
        delta_x: float = np.cos(current_state.orientation) * current_state.velocity * delay_time_step_sec
        delta_y: float = np.sin(current_state.orientation) * current_state.velocity * delay_time_step_sec

        translation: np.ndarray = np.array([delta_x, delta_y])
        current_state = current_state.translate_rotate(translation, 0)

    return frames


def main() -> None:
    figure = plt.figure()
    file_path: os.path = os.path.join(os.getcwd(), 'scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')
    common_road_input: Tuple[Scenario, PlanningProblemSet] = CommonRoadFileReader(file_path).open()
    scenario: Scenario = common_road_input[0]
    planning_problem: Optional[PlanningProblem] = common_road_input[1].planning_problem_dict.get(800)

    # Draw goal region
    draw_object_part(planning_problem.goal)

    # Draw lanes
    draw_object_part(scenario.lanelet_network, plot_limits=[-200, 250, -100, 100])

    # Draw static obstacles
    for obstacle in scenario.static_obstacles:
        draw_object_part(obstacle)

    # Generates frames of dynamic obstacles
    frames: List[List[Artist]] = createDynamicObstaclesArtists(scenario.dynamic_obstacles)

    # Add predictions of ego vehicle to the frames
    extendFramesWithPrediction(planning_problem.initial_state, frames, scenario.dt)

    # Add an animation of the frames
    # NOTE The assignment is needed to force execution
    ani = ArtistAnimation(figure, frames, blit=True, interval=int(round(scenario.dt * 1000)), repeat=True)

    plt.gca().set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
