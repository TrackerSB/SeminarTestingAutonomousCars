from datetime import datetime, timedelta
from typing import Dict

import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from numpy.ma import arange

from common import load_scenario, MyState
from common.generation import GenerationHelp


def main() -> None:
    scenario, planning_problem = load_scenario('../scenarios/DEU_B471-1_1_T-1_mod.xml')
    measurements: Dict[int, float] = {}
    max_steps: int = 21

    plt.figure(figsize=(19.20, 10.80), dpi=100)
    plt.suptitle("Duration for generating states for DEU_B471-1_1_T-1_mod")
    plt.gca().yaxis.set_major_formatter(FuncFormatter(lambda x, pos: str(timedelta(seconds=x))))
    plt.xlim([0, max_steps])
    plt.xticks(arange(0, max_steps, 1.0))

    for num_steps in range(max_steps):
        start_time: datetime = datetime.now()
        GenerationHelp.generate_states(scenario, MyState(planning_problem.initial_state), num_steps)
        duration: timedelta = datetime.now() - start_time
        print("Generated " + str(num_steps) + " in " + str(duration) + ".")
        measurements[num_steps] = duration.total_seconds()
        plt.ylim([0, max(60.0, duration.total_seconds())])
        plt.plot(measurements.keys(), measurements.values(), 'x',
                 measurements.keys(), measurements.values(), '-')
        plt.pause(0.05)

    plt.show()


if __name__ == '__main__':
    main()
