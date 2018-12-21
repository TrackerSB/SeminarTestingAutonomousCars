from copy import copy
from datetime import datetime
from typing import List

import matplotlib.pyplot as plt
from shapely.geometry import MultiPolygon

from common import load_scenario, DrawHelp, MyState, flatten_dict_values, VehicleInfo, drawable_types
from common.generation import GenerationHelp


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1_mod.xml')

    num_time_steps: int = 5
    fixed_drawables: List[drawable_types] = []
    for obj in [scenario.lanelet_network, planning_problem.initial_state]:
        fixed_drawables.append(DrawHelp.convert_to_drawable(obj))
    for time_step in range(1, num_time_steps + 1):
        for obs in scenario.static_obstacles:
            fixed_drawables.append(DrawHelp.convert_to_drawable([obs.occupancy_at_time(time_step)]))
        for obs in scenario.dynamic_obstacles:
            fixed_drawables.append(DrawHelp.convert_to_drawable([obs.occupancy_at_time(time_step)]))

    for velocity in range(18, 50):
        planning_problem.initial_state.velocity = velocity
        plt.figure(figsize=(19.20, 10.80), dpi=100)

        for fixed in fixed_drawables:
            DrawHelp.draw(copy(fixed))

        start_time: datetime = datetime.now()
        valid_converted, num_states_processed \
            = GenerationHelp.generate_states(scenario, MyState(planning_problem.initial_state), num_time_steps)
        print("Processed " + str(num_states_processed) + " states in " + str(datetime.now() - start_time))

        all_states: List[VehicleInfo] = flatten_dict_values(valid_converted)

        for vehicle in all_states:
            DrawHelp.draw(vehicle.drawable)

        union: MultiPolygon = DrawHelp.union_to_polygon(list(map(lambda v: v.drawable, all_states)))
        DrawHelp.draw(union)
        print("Drivable area: " + str(union.area))

        plt.gca().set_aspect('equal')
        plt.axis([50, 200, 0, 100])
        plt.savefig("out_velocity_" + str(velocity))
        # plt.close()  # FIXME Causes EOFErrors


if __name__ == '__main__':
    main()
