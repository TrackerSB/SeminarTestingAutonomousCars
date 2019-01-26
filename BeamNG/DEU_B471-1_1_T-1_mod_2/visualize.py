from beamngpy import BeamNGpy, Scenario, Vehicle, Road


# Visualize DEU_B471-1_1_T-1_mod_2

z_offset: int = 700  # Make sure to position the car outside the buildings
rot_offset: float = 180


def main() -> None:
    # Setup BeamNG
    bng = BeamNGpy('localhost', 64256, home='G:/gitrepos/beamng-research_unlimited/trunk',
          user='G:/gitrepos/SeminarTestingAutonomousCars/BeamNG/BeamNGUserpath')
    scenario = Scenario('smallgrid', 'commonroad', authors='Stefan Huber',
                        description='Simple visualization of the modified version of the CommonRoad scenario '
                                    'DEU_B471-1_1_T-1')

    # Add vehicle
    vehicle = Vehicle('ego_vehicle', model='etk800', licence='EGO', color='White')
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))

    # Create road
    road = Road('AsphaltRoad_variation_02')
    nodes = [
        (0, 0, 0, 5),
        (0, 100, 0, 5),
        (100, 0, 0, 5)
    ]
    road.nodes.extend(nodes)
    scenario.add_road(road)

    scenario.make(bng)

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
