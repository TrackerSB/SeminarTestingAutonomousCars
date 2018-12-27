import os
from typing import Tuple, List, Optional

import matplotlib.patches as pltpat
import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from matplotlib.animation import ArtistAnimation
from matplotlib.artist import Artist

from common import DrawHelp


def createDynamicObstaclesArtists(dynamic_obstacles: List[DynamicObstacle]) -> List[List[Artist]]:
    artists: List[List[Artist]] = []
    for time_step in range(0, 51):
        frame = []
        for obstacle in dynamic_obstacles:
            frame.append(DrawHelp.draw(DrawHelp.convert_to_drawable(obstacle, time_step)))
        artists.append(frame)

    return artists


def extend_frames_with_prediction(initial_state: State, frames: List[List[Artist]], delay_time_step_sec: float) \
        -> List[List[Artist]]:
    DrawHelp.draw(DrawHelp.convert_to_drawable(initial_state))
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
    figure = plt.figure(figsize=(19.20, 10.80), dpi=100)
    plt.xlim([-200, 250])
    plt.ylim([-100, 100])
    file_path: os.path = os.path.join(os.getcwd(), '../scenarios/DEU_B471-1_1_T-1.xml')
    common_road_input: Tuple[Scenario, PlanningProblemSet] = CommonRoadFileReader(file_path).open()
    scenario: Scenario = common_road_input[0]
    planning_problem: Optional[PlanningProblem] = common_road_input[1].planning_problem_dict.get(800)

    # Draw goal region
    DrawHelp.draw(DrawHelp.convert_to_drawable(planning_problem.goal))

    # Draw lanes
    DrawHelp.draw(DrawHelp.convert_to_drawable(scenario.lanelet_network))

    # Draw static obstacles
    for obstacle in scenario.static_obstacles:
        DrawHelp.draw(DrawHelp.convert_to_drawable(obstacle))

    # Generates frames of dynamic obstacles
    frames: List[List[Artist]] = createDynamicObstaclesArtists(scenario.dynamic_obstacles)

    # Add predictions of ego vehicle to the frames
    extend_frames_with_prediction(planning_problem.initial_state, frames, scenario.dt)

    # Add an animation of the frames
    # NOTE The assignment is needed to force execution
    ani: ArtistAnimation = ArtistAnimation(figure, frames, blit=True, interval=int(round(scenario.dt * 1000)), repeat=True)
    ani.save("2018-12-26_BasicUnoptimzedScenario.mp4", writer="ffmpeg")

    plt.gca().set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
