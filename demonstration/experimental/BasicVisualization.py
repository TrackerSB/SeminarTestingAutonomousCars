import os
from typing import Tuple

import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.draw_dispatch_cr import draw_object

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


def main():
    #file_path: os.path = os.path.join(os.getcwd(), '../scenarios/ZAM_Straight-1_1.xml')
    file_path: os.path = os.path.join(os.getcwd(), '../scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')

    common_road: Tuple[Scenario, PlanningProblemSet] = CommonRoadFileReader(file_path).open()
    scenario: Scenario = common_road[0]
    planning_problem_set: PlanningProblemSet = common_road[1]

    plt.figure(figsize=(25, 10))
    draw_object(scenario, draw_params=draw_params)
    draw_object(planning_problem_set, draw_params=draw_params)
    plt.gca().set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
