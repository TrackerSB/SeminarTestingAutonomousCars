from cmath import pi
from copy import copy
from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, Road, os
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Polygon
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle
from numpy.core._multiarray_umath import rad2deg
# Configuration
from numpy.core.multiarray import ndarray, cos, sin, array, deg2rad

# Visualize DEU_B471-1_1_T-1_mod_2

user_path = 'G:/gitrepos/SeminarTestingAutonomousCars/BeamNG/BeamNGUserpath'
home_path = 'G:/gitrepos/beamng-research_unlimited/trunk'
bng_scenario_environment = 'smallgrid'
bng_scenario_name = 'commonroad'
cr_scenario_name = 'DEU_B471-1_1_T-1_mod_2.xml'
cr_scenario_path = 'G:/gitrepos/SeminarTestingAutonomousCars/demonstration/scenarios/' + cr_scenario_name
cr_road_material = 'track_editor_C_center'

# Translation offsets from CommonRoad to BeamNG
etk800_z_offset = 0.212443
etk800_ontop_z_offset = 1.10547
tanker_z_offset = 1.21453
semi_z_offset = 0.605691
# FIXME Why is the rot_offset so scary?
rot_offset = -135  # BeamNG = degrees(CommonRoad) + rot_offset


def generate_node_list(obstacle) -> list:
    path = list()
    if isinstance(obstacle, DynamicObstacle):
        prediction = obstacle.prediction
        if isinstance(prediction, TrajectoryPrediction):
            for state in prediction.trajectory.state_list:
                path.append({
                    'pos': (state.position[0], state.position[1], etk800_z_offset),
                    'speed': state.velocity * 3.6 / 6.3  # Manually slow down
                })
        else:
            print(str(type(prediction)) + " not supported, yet.")
    else:
        print(str(type(obstacle)) + " not supported, yet.")
    return path


def add_vehicle_to_bng_scenario(bng_scenario, vehicle, init_state, z_offset) -> None:
    pos = init_state.position
    rot = rad2deg(init_state.orientation)
    bng_scenario.add_vehicle(vehicle,
                             pos=(pos[0], pos[1], z_offset),
                             rot=(0, 0, rot_offset + rot))


def pol2cart(rho: float, phi: float) -> ndarray:
    x = rho * cos(phi)
    y = rho * sin(phi)
    return array([x, y])


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
                            description='Simple visualization of the CommonRoad scenario ' + cr_scenario_name)

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
    ego_vehicle = Vehicle('ego_vehicle', model='etk800', licence='EGO', color='Cornflowerblue')
    ego_init_state = cr_planning_problem.initial_state
    ego_init_state.position[0] = 82.8235
    ego_init_state.position[1] = 31.5786
    add_vehicle_to_bng_scenario(bng_scenario, ego_vehicle, ego_init_state, etk800_z_offset)

    obstacles_to_move = dict()

    # Add truck
    semi = Vehicle('truck', model='semi', color='Red')
    semi_init_state = cr_scenario.obstacle_by_id(206).initial_state
    add_vehicle_to_bng_scenario(bng_scenario, semi, semi_init_state, semi_z_offset)
    obstacles_to_move[206] = semi

    # Add truck trailer
    tanker_init_state = copy(semi_init_state)
    tanker_init_state.position += [6, 3]
    add_vehicle_to_bng_scenario(bng_scenario, Vehicle('truck_trailer', model='tanker', color='Red'),
                                tanker_init_state, tanker_z_offset)

    # Add other traffic participant
    opponent = Vehicle('opponent', model='etk800', licence='VS', color='Cornflowerblue')
    add_vehicle_to_bng_scenario(bng_scenario, opponent, cr_scenario.obstacle_by_id(207).initial_state, etk800_z_offset)
    obstacles_to_move[207] = opponent

    # Add static obstacle
    obstacle_shape: Polygon = cr_scenario.obstacle_by_id(399).obstacle_shape
    obstacle_pos = obstacle_shape.center
    obstacle_rot_deg = rad2deg(semi_init_state.orientation) + rot_offset
    obstacle_rot_rad = semi_init_state.orientation + deg2rad(rot_offset)
    for w in range(3):
        for h in range(3):
            for d in range(2):
                obstacle = Vehicle('obstacle' + str(w) + str(h) + str(d), model='haybale', color='Red')
                haybale_pos_diff = obstacle_pos \
                    + pol2cart(1.3 * d, obstacle_rot_rad + pi / 4) \
                    + pol2cart(2.2 * w, pi / 2 - obstacle_rot_rad)
                bng_scenario.add_vehicle(obstacle,
                                         pos=(haybale_pos_diff[0], haybale_pos_diff[1], h * 1),
                                         rot=(0, 0, obstacle_rot_deg))

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
        bng.pause()

        bng.display_gui_message("The scenario is fully prepared and paused. You may like to position the camera first.")
        delay_to_resume = 15
        input("Press enter to continue the simulation... You have " + str(delay_to_resume)
              + " seconds to switch back to the BeamNG window.")
        sleep(delay_to_resume)
        bng.resume()

        for id, obstacle in obstacles_to_move.items():
            obstacle.ai_drive_in_lane(False)
            obstacle.ai_set_line(generate_node_list(cr_scenario.obstacle_by_id(id)))

        ego_vehicle.ai_drive_in_lane(False)
        # ego_vehicle.ai_set_speed(cr_planning_problem.initial_state.velocity * 3.6, mode='limit')
        speed = 65 / 3.6
        ego_vehicle.ai_set_line([{
            'pos': ego_vehicle.state['pos']
        }, {
            'pos': (129.739, 56.907, etk800_z_offset),
            'speed': speed
        }, {
            'pos': (139.4, 62.3211, etk800_z_offset),
            'speed': speed
        }, {
            'pos': (149.442, 64.3655, etk800_z_offset),
            'speed': speed
        }, {
            'pos': (150.168, 63.3065, etk800_z_offset),
            'speed': speed
        }, {
            'pos': (188.495, 78.8517, etk800_z_offset),
            'speed': speed
        }])
        # FIXME Setting the speed does not work as expected
        # ego_vehicle.ai_set_speed(cr_planning_problem.initial_state.velocity * 3.6, mode='set')

        input("Press enter to end simulation...")
    finally:
        bng.close()


if __name__ == '__main__':
    main()
