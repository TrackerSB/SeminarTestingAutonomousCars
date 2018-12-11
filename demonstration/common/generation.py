from copy import deepcopy
from queue import Queue
from threading import Thread
from typing import Tuple, Union, Optional, Dict, List

from commonroad.geometry.shape import Shape, Rectangle
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State, Trajectory
from numpy import linspace, inf
from numpy.core.multiarray import ndarray
from numpy.core.umath import pi, cos, sin
from numpy.random.mtrand import uniform

from common import is_valid, Vehicle
from common.StatesQueue import StatesQueue
from common.draw import DrawHelp, DrawConfig
from common.types import drawable_types


class GenerationConfig:
    max_yaw: float = pi / 16  # 11.25°
    yaw_steps: int = 8
    num_threads: int = 8
    position_threshold = 0.5
    angle_threshold = max_yaw * 0.85  # NOTE Needs to be way smaller than max_yaw otherwise the car tends to the right.


class GenerationHelp:
    @staticmethod
    def predict_next_state(scenario: Scenario, current: State) -> State:
        next: State = deepcopy(current)
        delta_x: float = cos(next.orientation) * next.velocity * scenario.dt
        delta_y: float = sin(next.orientation) * next.velocity * scenario.dt
        next.position[0] += delta_x
        next.position[1] += delta_y
        return next

    @staticmethod
    def generate_states(scenario: Scenario, planning_problem: PlanningProblem, time_steps: int) \
            -> Tuple[Dict[int, List[Vehicle]], int]:
        """
        Generates all positions the ego vehicle can have within the next steps in the given scenario and considering the
        given preplanning problem.
        :param scenario: The scenario the ego vehicle is driving in.
        :param planning_problem: The preplanning problem to solve.
        :param time_steps: The number of steps to simulate.
        :return: A tuple containing a dictionary mapping a time step to all generated valid states and their drawable
        representation of that time step and the number of total states processed.
        """
        num_states_processed: int = 0
        valid_converted: Dict[int, List[Vehicle]] = {}

        def generate_next_states() -> None:
            nonlocal num_states_processed
            while True:
                state: State = current_states.get()
                if state.time_step < time_steps:
                    yaw_steps: Union[ndarray, Tuple[ndarray, Optional[float]]] \
                        = linspace(-GenerationConfig.max_yaw, GenerationConfig.max_yaw,
                                   num=GenerationConfig.yaw_steps, endpoint=True)
                    for yaw in yaw_steps:
                        transformed: State = deepcopy(state)
                        transformed.orientation += yaw
                        transformed = GenerationHelp.predict_next_state(scenario, transformed)
                        if transformed not in current_states:
                            converted: drawable_types = DrawHelp.convert_to_drawable(transformed)
                            if is_valid(converted, scenario):
                                transformed.time_step += 1
                                if transformed.time_step not in valid_converted.keys():
                                    valid_converted[transformed.time_step] = []
                                valid_converted[transformed.time_step].append(Vehicle(transformed, converted))
                                current_states.put(transformed)
                current_states.task_done()
                num_states_processed += 1

        current_states: Queue[State] \
            = StatesQueue(GenerationConfig.position_threshold, GenerationConfig.angle_threshold)
        current_states.put(planning_problem.initial_state)
        for i in range(GenerationConfig.num_threads):
            worker: Thread = Thread(target=generate_next_states, args=(), daemon=True)
            worker.start()

        current_states.join()
        return valid_converted, num_states_processed

    @staticmethod
    def generate_trajectory(scenario: Scenario, planning_problem: PlanningProblem, time_steps: int,
                            max_tries: int = 1000) -> Tuple[TrajectoryPrediction, List[Vehicle]]:
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
        states: List[State] = [planning_problem.initial_state]
        vehicles: List[Vehicle] = [Vehicle(planning_problem.initial_state,
                                           DrawHelp.convert_to_drawable(planning_problem.initial_state))]
        for i in range(1, time_steps):
            last_state_copy: State = deepcopy(states[i - 1])
            found_valid_next: bool = False
            tries: int = 0
            while not found_valid_next and tries < max_tries:
                next_state: State = GenerationHelp.predict_next_state(scenario, last_state_copy)
                next_state_converted: drawable_types = DrawHelp.convert_to_drawable(next_state)
                if is_valid(next_state_converted, scenario):
                    states.append(next_state)
                    vehicles.append(Vehicle(next_state, next_state_converted))
                    found_valid_next = True
                else:
                    tries += 1
                    last_state_copy.orientation \
                        = states[i - 1].orientation + uniform(-GenerationConfig.max_yaw, GenerationConfig.max_yaw)
            if not found_valid_next:
                break
        return TrajectoryPrediction(Trajectory(0, states), shape), vehicles
