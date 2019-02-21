import os
from typing import Tuple, List, Optional, Dict

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


arrowprops: Dict = dict(facecolor='black', width=3)


def createDynamicObstaclesArtists(dynamic_obstacles: List[DynamicObstacle]) -> List[List[Artist]]:
    artists: List[List[Artist]] = []
    for time_step in range(0, 21):
        frame = []
        for obstacle in dynamic_obstacles:
            position: pltpat.Rectangle = DrawHelp.convert_to_drawable(obstacle, time_step)
            drawable = DrawHelp.draw(position)
            drawable.set_zorder(1000000)
            frame.append(drawable)
            frame.append(plt.annotate("participant", xy=(position.get_x() - 3, position.get_y()),
                                      xytext=(position.get_x(), position.get_y() + 7),
                                      arrowprops=arrowprops, zorder=100000))
        artists.append(frame)

    return artists


def extend_frames_with_prediction(initial_state: State, frames: List[List[Artist]], delay_time_step_sec: float) \
        -> List[List[Artist]]:
    current_state: State = initial_state
    for frame in frames:
        state_point: pltpat.Circle = pltpat.Circle(current_state.position, 2)
        frame.append(plt.gca().add_patch(state_point))
        center = state_point.get_center()
        annotation = plt.annotate("ego_vehicle", xy=center - (0, 2), xytext=center - (-2, 7), arrowprops=arrowprops)
        frame.append(annotation)

        delta_x: float = np.cos(current_state.orientation) * current_state.velocity * delay_time_step_sec
        delta_y: float = np.sin(current_state.orientation) * current_state.velocity * delay_time_step_sec
        translation: np.ndarray = np.array([delta_x, delta_y])
        #current_state = current_state.translate_rotate(translation, 0)

    return frames


def main() -> None:
    figure = plt.figure(figsize=(19.20, 10.80), dpi=100)
    plt.xlim([100, 220])
    plt.ylim([25, 100])
    file_path: os.path = os.path.join(os.getcwd(), '../scenarios/DEU_B471-1_1_T-1_mod_2.xml')
    common_road_input: Tuple[Scenario, PlanningProblemSet] = CommonRoadFileReader(file_path).open()
    scenario: Scenario = common_road_input[0]
    planning_problem: Optional[PlanningProblem] = common_road_input[1].planning_problem_dict.get(800)

    # Draw goal region
    DrawHelp.draw(DrawHelp.convert_to_drawable(planning_problem.goal))
    plt.annotate("goal region", xy=(156, 62), xytext=(160, 55), arrowprops=arrowprops)

    # Draw lanes
    DrawHelp.draw(DrawHelp.convert_to_drawable(scenario.lanelet_network))

    # Draw static obstacles
    for obstacle in scenario.static_obstacles:
        DrawHelp.draw(DrawHelp.convert_to_drawable(obstacle))
        x, y = obstacle.occupancy_at_time(0).shape.center
        plt.annotate("static obstacle", xy=(x + 2, y + 3), xytext=(x, y + 9), arrowprops=arrowprops)

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
