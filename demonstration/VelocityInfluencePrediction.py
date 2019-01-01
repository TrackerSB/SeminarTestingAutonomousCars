from copy import copy
from datetime import datetime
from typing import List, Dict

import matplotlib.pyplot as plt
from shapely.geometry import MultiPolygon

from common import load_scenario, DrawHelp, MyState, flatten_dict_values, VehicleInfo, drawable_types
from common.generation import GenerationHelp


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1_mod.xml')

    num_time_steps: int = 10
    fixed_drawables: List[drawable_types] = []
    for obj in [scenario.lanelet_network, planning_problem.initial_state]:
        fixed_drawables.append(DrawHelp.convert_to_drawable(obj))
    for time_step in range(1, num_time_steps + 1):
        for obs in scenario.static_obstacles:
            fixed_drawables.append(DrawHelp.convert_to_drawable([obs.occupancy_at_time(time_step)]))
        for obs in scenario.dynamic_obstacles:
            fixed_drawables.append(DrawHelp.convert_to_drawable([obs.occupancy_at_time(time_step)]))

    min_velocity: int = 15
    max_velocity: int = 50
    drivable_areas: Dict[int, float] = {}
    for velocity in range(min_velocity, max_velocity + 1):
        # Prepare figure and subplots
        main_fig, (scenario_fig, area_fig) = plt.subplots(nrows=1, ncols=2, figsize=(19.20, 10.80), dpi=100)
        main_fig.suptitle("Influence of velocity to drivable area", fontsize=32)
        plt.sca(scenario_fig)

        # Draw artists to be shown always
        for fixed in fixed_drawables:
            DrawHelp.draw(copy(fixed))

        # Generate and draw states resulting from the current velocity of the ego vehicle
        planning_problem.initial_state.velocity = velocity
        start_time: datetime = datetime.now()
        valid_converted, num_states_processed \
            = GenerationHelp.generate_states(scenario, MyState(planning_problem.initial_state), num_time_steps)
        print("Processed " + str(num_states_processed) + " states in " + str(datetime.now() - start_time))

        all_states: List[VehicleInfo] = flatten_dict_values(valid_converted)

        for vehicle in all_states:
            DrawHelp.draw(vehicle.drawable)

        union: MultiPolygon = DrawHelp.union_to_polygon(list(map(lambda v: v.drawable, all_states)))
        DrawHelp.draw(union)
        drivable_areas[velocity] = union.area

        scenario_fig.set_aspect('equal')
        scenario_fig.set_xlim([50, 200])
        scenario_fig.set_ylim([0, 100])
        scenario_fig.set_xlabel("position [m]")
        scenario_fig.set_ylabel("position [m]")

        # Draw changes of drivable area
        area_fig.set_xlim([min_velocity, max_velocity])
        area_fig.set_ylim([0, max(90.0, max(drivable_areas.values()))])
        area_fig.plot(drivable_areas.keys(), drivable_areas.values(), 'x',
                      drivable_areas.keys(), drivable_areas.values(), '-')
        area_fig.set_xlabel("velocity [m/s]")
        area_fig.set_ylabel("area [m²]")

        # Save the figure
        plt.savefig("out_velocity_" + str(velocity))
        # plt.close()  # FIXME Causes EOFErrors


if __name__ == '__main__':
    main()
