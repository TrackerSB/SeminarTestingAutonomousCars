import time
from copy import deepcopy
from multiprocessing import Process, Value, Manager
from typing import Tuple, Union, Optional, Dict, List

import psutil
from commonroad.geometry.shape import Shape, Rectangle
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from numpy import linspace
from numpy.core.multiarray import ndarray
from numpy.core.umath import pi, cos, sin
from numpy.random.mtrand import uniform

from common import is_valid, VehicleInfo, MyState
from common.StatesQueue import StatesQueue
from common.draw import DrawHelp, DrawConfig


class GenerationConfig:
    max_yaw: float = pi / 16  # 11.25°
    yaw_steps: int = 32
    num_threads: int = 8
    position_threshold = 0.5
    angle_threshold = max_yaw * 0.5  # NOTE Needs to be way smaller than max_yaw otherwise the car tends to the right.


class GenerationHelp:
    @staticmethod
    def predict_next_state(scenario: Scenario, current: MyState) -> MyState:
        next: MyState = deepcopy(current)
        delta_x: float = cos(next.state.orientation) * next.state.velocity * scenario.dt
        delta_y: float = sin(next.state.orientation) * next.state.velocity * scenario.dt
        next.state.position[0] += delta_x
        next.state.position[1] += delta_y
        return next

    @staticmethod
    def generate_states(scenario: Scenario, ego_vehicle: MyState, time_steps: int) \
            -> Tuple[Dict[int, List[VehicleInfo]], int]:
        """
        Generates all positions the ego vehicle can have within the next steps in the given scenario and considering the
        given preplanning problem.
        :param scenario: The scenario the ego vehicle is driving in.
        :param ego_vehicle: The initial state of the ego vehicle.
        :param time_steps: The number of steps to simulate.
        :return: A tuple containing a dictionary mapping a time step to all generated valid states and their drawable
        representation of that time step and the number of total states processed.
        """
        num_states_processed: Value = Value('i', 0)
        manager: Manager = Manager()
        valid_converted: Dict[int, List[VehicleInfo]] = manager.dict()
        for step in range(1, time_steps + 1):
            valid_converted[step] = manager.list()
        current_states: StatesQueue = StatesQueue(GenerationConfig.position_threshold, GenerationConfig.angle_threshold)
        current_states.put(ego_vehicle)

        def generate_next_states() -> None:
            while True:
                state: MyState = current_states.get()
                if state.state.time_step < time_steps:
                    yaw_steps: Union[ndarray, Tuple[ndarray, Optional[float]]] \
                        = linspace(-GenerationConfig.max_yaw, GenerationConfig.max_yaw,
                                   num=GenerationConfig.yaw_steps, endpoint=True)
                    for yaw in yaw_steps:
                        transformed: MyState = deepcopy(state)
                        transformed.state.orientation += yaw
                        transformed = GenerationHelp.predict_next_state(scenario, transformed)
                        if transformed not in current_states:
                            vehicle: VehicleInfo = VehicleInfo(transformed, None)
                            if is_valid(vehicle, scenario):
                                transformed.state.time_step += 1
                                valid_converted[transformed.state.time_step].append(vehicle)
                                current_states.put(transformed)
                current_states.task_done()
                num_states_processed.value += 1

        # Start workers
        workers: List[psutil.Process] = []
        for i in range(GenerationConfig.num_threads):
            worker: Process = Process(target=generate_next_states, args=(), daemon=True)
            worker.start()
            workers.append(psutil.Process(worker.pid))

        # Wait for workers to finish
        def wait():
            do_all_sleep = False
            while not do_all_sleep:
                time.sleep(1)
                do_all_sleep = all(map(lambda w: w.status() == "sleeping", workers))
                if do_all_sleep:
                    print("Generation workers fell asleep.")
                    time.sleep(3)
                    # Still sleeping?
                    do_all_sleep = all(map(lambda w: w.status() == "sleeping", workers))
                    if do_all_sleep:
                        print("Generation workers still sleeping --> Assume generation finished.")

        waiter: Process = Process(target=wait, args=(), daemon=True)
        waiter.start()
        waiter.join()
        for worker in workers:
            worker.terminate()

        return valid_converted, num_states_processed.value

    @staticmethod
    def generate_trajectory(scenario: Scenario, planning_problem: PlanningProblem, time_steps: int,
                            max_tries: int = 1000) -> Tuple[TrajectoryPrediction, List[VehicleInfo]]:
        """
        Generates an as straight as possible linear trajectory.
        :param scenario: The scenario the car is driving in.
        :param planning_problem: The preplanning problem to solve.
        :param time_steps: The number of time steps to simulate.
        :param max_tries: The maximum number of tries to find a valid next state. It this number is succeeded some time
        during generation the trajectory may not have as many time steps as specified.
        :return: A tuple containing the generated prediction for a trajectory and a list containing drawable
        representations of all the states of the trajectory.
        """
        shape: Shape = Rectangle(DrawConfig.car_length, DrawConfig.car_width,
                                 planning_problem.initial_state.position, planning_problem.initial_state.orientation)
        states: List[MyState] = [MyState(planning_problem.initial_state)]
        vehicles: List[VehicleInfo] = [VehicleInfo(MyState(planning_problem.initial_state), None,
                                                   DrawHelp.convert_to_drawable(planning_problem.initial_state))]
        for i in range(1, time_steps):
            last_state_copy: MyState = deepcopy(states[i - 1])
            found_valid_next: bool = False
            tries: int = 0
            while not found_valid_next and tries < max_tries:
                next_state: MyState = GenerationHelp.predict_next_state(scenario, last_state_copy)
                next_vehicle: VehicleInfo = VehicleInfo(next_state, None)
                if is_valid(next_vehicle, scenario):
                    states.append(next_state)
                    vehicles.append(next_vehicle)
                    found_valid_next = True
                else:
                    tries += 1
                    last_state_copy.orientation \
                        = states[i - 1].state.orientation + uniform(-GenerationConfig.max_yaw, GenerationConfig.max_yaw)
            if not found_valid_next:
                break
        return TrajectoryPrediction(Trajectory(0, list(map(lambda s: s.state, states))), shape), vehicles
