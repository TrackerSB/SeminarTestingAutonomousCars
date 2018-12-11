import datetime
from datetime import datetime
from typing import List, Dict, Tuple, Optional

import matplotlib.pyplot as plt
from commonroad.common.util import AngleInterval
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.trajectory import State

from common import load_scenario, drawable_types, flatten_dict_values, VehicleInfo
from common.draw import DrawHelp
from common.generation import GenerationHelp
from common.prm import dijkstra_search


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')

    plt.figure(figsize=(25, 10))

    DrawHelp.draw(DrawHelp.convert_to_drawable(scenario))
    DrawHelp.draw(DrawHelp.convert_to_drawable(planning_problem.initial_state))

    start_time: datetime = datetime.now()
    generation_result: Tuple[Dict[int, List[VehicleInfo]], int] \
        = GenerationHelp.generate_states(scenario, planning_problem, 15)
    # NOTE The value 50 is taken from the commonroad file
    num_states_processed: int = generation_result[1]
    valid_converted: Dict[int, List[VehicleInfo]] = generation_result[0]
    valid_states: List[State] = list(map(lambda v: v.state, flatten_dict_values(valid_converted)))
    print("Processed " + str(num_states_processed) + " states in " + str(datetime.now() - start_time))

    # FIXME Properly define goal states
    goal_states: List[State] = []
    for goal in planning_problem.goal.state_list:
        if isinstance(goal.position, Rectangle):
            goal.position = goal.position.center
            goal.orientation = (goal.orientation.end + goal.orientation.start) / 2
        goal.time_step = 16
        goal_states.append(goal)
    search_result: Optional[Tuple[List[State], float]] = dijkstra_search(
        planning_problem.initial_state, goal_states, valid_states)

    if search_result:
        for state in search_result[0]:
            DrawHelp.draw(DrawHelp.convert_to_drawable(state))
    else:
        print("Could not find path to goal")

    plt.gca().set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
