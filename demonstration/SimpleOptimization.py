import datetime
from datetime import datetime

from common import load_scenario
from common.generation import GenerationHelp
from common.optimizer import calculate_area_profile


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')

    start_time: datetime = datetime.now()
    valid_converted, num_states_processed = GenerationHelp.generate_states(scenario, planning_problem, 5)
    print("Processed " + str(num_states_processed) + " states in " + str(datetime.now() - start_time))

    print(calculate_area_profile(valid_converted))


if __name__ == '__main__':
    main()
