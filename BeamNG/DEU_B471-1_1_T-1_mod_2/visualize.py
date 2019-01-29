# Visualize DEU_B471-1_1_T-1_mod_2
from beamngpy import BeamNGpy, Scenario, Vehicle, Road, os

user_path = 'G:/gitrepos/SeminarTestingAutonomousCars/BeamNG/BeamNGUserpath'
home_path = 'G:/gitrepos/beamng-research_unlimited/trunk'
scenario_environment = 'smallgrid'
scenario_name = 'commonroad'


def main() -> None:
    # Setup BeamNG
    bng = BeamNGpy('localhost', 64256, home=home_path, user=user_path)
    scenario = Scenario(scenario_environment, scenario_name, authors='Stefan Huber',
                        description='Simple visualization of the modified version of the CommonRoad scenario '
                                    'DEU_B471-1_1_T-1')

    # Add ego vehicle
    ego_vehicle = Vehicle('ego_vehicle', model='etk800', licence='EGO', color='White')
    scenario.add_vehicle(ego_vehicle, pos=(0, 0, 0.212443), rot=(1, 0, 0))

    # Add truck
    truck = Vehicle('truck', model='tanker', licence='TRUCK', color='Red')
    scenario.add_vehicle(truck, pos=(50, 0, 1.21453), rot=(1, 0, 0, 0.137056))

    # Add other traffic participant
    # TODO

    # Create road
    road = Road('track_editor_H_center')  # FIXME overObjects=True is not supported
    nodes = [
        (0, 0, 0, 5),
        (0, 100, 0, 5),
        (100, 0, 0, 5)
    ]
    road.nodes.extend(nodes)
    scenario.add_road(road)

    scenario.make(bng)

    # Manually set the overObjects attribute of all roads to True (Setting this option is currently not supported)
    prefab_path = os.path.join(user_path, 'levels', scenario_environment, 'scenarios', scenario_name + '.prefab')
    lines = open(prefab_path, 'r').readlines()
    for i in range(len(lines)):
        if 'overObjects' in lines[i]:
            lines[i] = lines[i].replace('0', '1')
    open(prefab_path, 'w').writelines(lines)

    # Start simulation
    bng.open(launch=True)
    try:
        bng.load_scenario(scenario)
        bng.start_scenario()
        input("Press enter to end simulation...")
    finally:
        bng.close()


if __name__ == '__main__':
    main()
