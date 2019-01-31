from beamngpy import BeamNGpy, Scenario, Vehicle, Road, os
from beamngpy.sensors import Camera
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Polygon
from numpy.core._multiarray_umath import rad2deg

# Visualize DEU_B471-1_1_T-1_mod_2

# Configuration
user_path = 'G:/gitrepos/SeminarTestingAutonomousCars/BeamNG/BeamNGUserpath'
home_path = 'G:/gitrepos/beamng-research_unlimited/trunk'
bng_scenario_environment = 'smallgrid'
bng_scenario_name = 'commonroad'
cr_scenario_path = 'G:/gitrepos/SeminarTestingAutonomousCars/demonstration/scenarios/DEU_B471-1_1_T-1_mod_2.xml'
cr_road_material = 'track_editor_C_center'

# Translation offsets from CommonRoad to BeamNG
etk800_z_offset = 0.212443
etk800_ontop_z_offset = 1.10547
tanker_z_offset = 1.21453
semi_z_offset = 0.605691
# FIXME Why is the rot_offset so scary?
rot_offset = -135  # BeamNG = degrees(CommonRoad) + rot_offset


def main() -> None:
    # Read CommonRoad scenario
    cr_scenario, cr_planning_problem_set = CommonRoadFileReader(cr_scenario_path) \
        .open()
    if cr_planning_problem_set:
        for _, pp in cr_planning_problem_set.planning_problem_dict.items():
            cr_planning_problem = pp
            break  # Read only the first one
    else:
        raise Exception("The given CommonRoad scenario does not define a planning problem.")

    # Setup BeamNG
    bng = BeamNGpy('localhost', 64256, home=home_path, user=user_path)
    bng_scenario = Scenario(bng_scenario_environment, bng_scenario_name, authors='Stefan Huber',
                            description='Simple visualization of the modified version of the CommonRoad scenario '
                                        'DEU_B471-1_1_T-1')

    # Add lane network
    lanes = cr_scenario.lanelet_network.lanelets
    for lane in lanes:
        lane_nodes = []
        for center in lane.center_vertices:
            lane_nodes.append((center[0], center[1], 0, 4))  # FIXME Determine appropriate width
        road = Road(cr_road_material)
        road.nodes.extend(lane_nodes)
        bng_scenario.add_road(road)

    # Add ego vehicle
    ego_vehicle = Vehicle('ego_vehicle', model='etk800', licence='EGO', color='White')
    ego_init_state = cr_planning_problem.initial_state
    ego_pos = ego_init_state.position
    ego_rot = rad2deg(ego_init_state.orientation)
    bng_scenario.add_vehicle(ego_vehicle,
                             pos=(ego_pos[0], ego_pos[1], etk800_z_offset),
                             rot=(0, 0, rot_offset + ego_rot))

    # Add truck
    semi = Vehicle('truck', model='semi')
    semi_init_state = cr_scenario.obstacle_by_id(206).initial_state
    semi_pos = semi_init_state.position
    semi_rot = rad2deg(semi_init_state.orientation)
    bng_scenario.add_vehicle(semi,
                             pos=(semi_pos[0], semi_pos[1], semi_z_offset),
                             rot=(0, 0, rot_offset + semi_rot))

    # Add truck trailer
    tanker = Vehicle('truck_trailer', model='tanker', color='Red')
    tanker_pos = semi_init_state.position + [6, 3]
    tanker_rot = rad2deg(semi_init_state.orientation)
    bng_scenario.add_vehicle(tanker,
                             pos=(tanker_pos[0], tanker_pos[1], tanker_z_offset),
                             rot=(0, 0, rot_offset + tanker_rot))

    # Add other traffic participant
    opponent = Vehicle('opponent', model='etk800', licence='VS', color='Red')
    opponent_init_state = cr_scenario.obstacle_by_id(207).initial_state
    opponent_pos = opponent_init_state.position
    opponent_rot = rad2deg(opponent_init_state.orientation)
    bng_scenario.add_vehicle(opponent,
                             pos=(opponent_pos[0], opponent_pos[1], etk800_z_offset),
                             rot=(0, 0, rot_offset + opponent_rot))

    # Add static obstacle
    obstacle_shape: Polygon = cr_scenario.obstacle_by_id(399).obstacle_shape
    obstacle_pos = obstacle_shape.center
    obstacle_rot = rad2deg(opponent_init_state.orientation)
    for w in range(3):
        for h in range(3):
            for d in range(3):
                obstacle = Vehicle('obstacle' + str(w) + str(h) + str(d), model='haybale')  # Alternatives: caravan
                bng_scenario.add_vehicle(obstacle,
                                         pos=(d * 2 + obstacle_pos[0], -w * 1.5 + obstacle_pos[1], h * 1),
                                         rot=(0, 180, rot_offset + obstacle_rot))

    bng_scenario.make(bng)

    # Manually set the overObjects attribute of all roads to True (Setting this option is currently not supported)
    prefab_path = os.path.join(user_path, 'levels', bng_scenario_environment, 'scenarios',
                               bng_scenario_name + '.prefab')
    lines = open(prefab_path, 'r').readlines()
    for i in range(len(lines)):
        if 'overObjects' in lines[i]:
            lines[i] = lines[i].replace('0', '1')
    open(prefab_path, 'w').writelines(lines)

    # Start simulation
    bng.open(launch=True)
    try:
        bng.load_scenario(bng_scenario)
        bng.start_scenario()

        # Make the opponent vehicle move
        opponent.ai_drive_in_lane(False)
        last_pos = cr_scenario.obstacle_by_id(207).occupancy_at_time(21).shape.center
        opponent.ai_set_line([
            {
                'pos': opponent.state['pos'],
                'speed': 60
            }, {
                'pos': (last_pos[0], last_pos[1], etk800_z_offset),
                'speed': 30
            }, {
                'pos': (140, 57, etk800_z_offset),
                'speed': 10
            }
        ])

        # Make the ego vehicle move
        ego_vehicle.ai_drive_in_lane(False)
        ego_vehicle.ai_set_line([
            {
                'pos': ego_vehicle.state['pos'],
                'speed': 10
            }, {
                'pos': (130, 57, etk800_z_offset),
                'speed': 10
            }, {
                'pos': (142, 62, etk800_z_offset),
                'speed': 10
            }
        ])

        input("Press enter to end simulation...")
    finally:
        bng.close()


if __name__ == '__main__':
    main()
