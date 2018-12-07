import datetime
from datetime import datetime
from typing import List, Dict

import matplotlib.pyplot as plt
from shapely.geometry import MultiPolygon

from common import load_scenario, flatten_dict_values, drawable_types
from common.draw import DrawHelp
from common.generation import GenerationHelp
from common.optimizer import generate_area_profile


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')

    plt.figure(figsize=(25, 10))

    DrawHelp.draw(DrawHelp.convert_to_drawable(scenario))
    DrawHelp.draw(DrawHelp.convert_to_drawable(planning_problem.initial_state))

    start_time: datetime = datetime.now()
    valid_converted, num_states_processed = GenerationHelp.generate_states(scenario, planning_problem, 5)
    print("Processed " + str(num_states_processed) + " states in " + str(datetime.now() - start_time))

    drawables: Dict[int, List[drawable_types]] = {}
    for time_step, converted in valid_converted.items():
        drawables[time_step] = list(map(lambda c: c[1], converted))
    print(generate_area_profile(drawables))

    plt.gca().set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
