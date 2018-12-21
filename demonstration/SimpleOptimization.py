import datetime
from datetime import datetime
from typing import List

from common import load_scenario, VehicleInfo, MyState, flatten_dict_values
from common.generation import GenerationHelp
from common.optimizer import calculate_area_profile, binary_search, update_scenario


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')

    start_time: datetime = datetime.now()
    ego_vehicle: MyState = MyState(planning_problem.initial_state)
    valid_converted, num_states_processed = GenerationHelp.generate_states(scenario, ego_vehicle, 5)
    print("Processed " + str(num_states_processed) + " states in " + str(datetime.now() - start_time))
    print(calculate_area_profile(flatten_dict_values(valid_converted)))

    binary_search(10, vehicles, vehicles, scenario)
    ego_info: VehicleInfo = VehicleInfo(ego_vehicle, -1)
    vehicles: List[VehicleInfo] = [ego_info]
    print("Before update: " + str(planning_problem.initial_state.velocity))
    update_scenario(scenario, planning_problem, ego_info, 0, 42)
    print("After update: " + str(planning_problem.initial_state.velocity))


if __name__ == '__main__':
    main()
