import datetime
from datetime import datetime
from typing import List

from numpy.ma import array

from common import load_scenario, VehicleInfo, MyState
from common.optimizer import update_scenario, optimized_scenario


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1.xml')

    start_time: datetime = datetime.now()
    ego_vehicle: MyState = MyState(planning_problem.initial_state)

    ego_info: VehicleInfo = VehicleInfo(ego_vehicle, -1)
    vehicles: List[VehicleInfo] = [ego_info]
    print("Before update: " + str(planning_problem.initial_state.velocity))
    update_scenario(scenario, planning_problem, ego_info, 0, 42)
    print("After update: " + str(planning_problem.initial_state.velocity))

    optimized_scenario(vehicles, 1000, 10, 10, array([5, 4, 3, 2, 1]), scenario)

    print("Optimization in " + str(datetime.now() - start_time))


if __name__ == '__main__':
    main()
