import datetime
from copy import copy
from datetime import datetime
from typing import List

from numpy.ma import array

from common import load_scenario, VehicleInfo, MyState
from common.optimizer import optimized_scenario


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1_mod.xml')

    start_time: datetime = datetime.now()
    ego_vehicle: MyState = MyState(copy(planning_problem.initial_state))
    vehicles: List[VehicleInfo] = [VehicleInfo(ego_vehicle, -1)]

    optimized_scenario(vehicles, 3000, 10, 10, array([5, 4, 3, 2, 1]), scenario, planning_problem)

    print("Optimization in " + str(datetime.now() - start_time))
    print("Optimized velocity: " + str(planning_problem.initial_state.velocity))


if __name__ == '__main__':
    main()
