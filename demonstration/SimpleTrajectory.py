import datetime
from datetime import datetime

import matplotlib.pyplot as plt

from common import load_scenario
from common.draw import DrawHelp
from common.generation import GenerationHelp


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')

    plt.figure(figsize=(25, 10))

    DrawHelp.draw(DrawHelp.convert_to_drawable(scenario))
    DrawHelp.draw(DrawHelp.convert_to_drawable(planning_problem.initial_state))

    start_time: datetime = datetime.now()
    prediction, vehicles = GenerationHelp.generate_trajectory(scenario, planning_problem, 70)
    print("Generation took " + str(datetime.now() - start_time))

    for vehicle in vehicles:
        DrawHelp.draw(vehicle.drawable)

    plt.gca().set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
