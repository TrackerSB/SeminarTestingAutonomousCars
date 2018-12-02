import datetime
from datetime import datetime

import matplotlib.pyplot as plt
from shapely.geometry import MultiPolygon

from common import load_scenario, flatten_dict_values
from common.draw import DrawHelp
from common.generation import GenerationHelp


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')

    plt.figure(figsize=(25, 10))

    DrawHelp.draw(DrawHelp.convert_to_drawable(scenario))
    DrawHelp.draw(DrawHelp.convert_to_drawable(planning_problem.initial_state))

    start_time: datetime = datetime.now()
    valid_converted, num_states_processed = GenerationHelp.generate_states(scenario, planning_problem, 5)
    print("Processed " + str(num_states_processed) + " states in " + str(datetime.now() - start_time))

    union: MultiPolygon = DrawHelp.union_to_polygon(list(map(lambda t: t[1], flatten_dict_values(valid_converted))))
    DrawHelp.draw(union)
    print("Drivable area: " + str(union.area))

    plt.gca().set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
