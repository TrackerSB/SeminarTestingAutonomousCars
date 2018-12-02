from copy import deepcopy
from queue import Queue
from threading import Thread
from typing import Tuple, Union, Optional, Dict

from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from numpy import linspace
from numpy.core.multiarray import ndarray
from numpy.core.umath import pi, cos, sin

from common import is_valid
from common.StatesQueue import StatesQueue
from common.draw import DrawHelp
from common.types import drawable_types


class GenerationConfig:
    max_yaw: float = pi / 16  # 11.25°
    yaw_steps: int = 8
    num_threads: int = 8
    position_threshold = 0.5
    angle_threshold = max_yaw * 0.85  # NOTE Needs to be way smaller than max_yaw otherwise the car tends to the right.


class GenerationHelp:
    @staticmethod
    def generate_states(scenario: Scenario, planning_problem: PlanningProblem, time_steps: int) \
            -> Tuple[Dict[int, drawable_types], int]:
        """
        Generates all positions the ego vehicle can have within the next steps in the given scenario and considering the
        given preplanning problem.
        :param scenario: The scenario the ego vehicle is driving in.
        :param planning_problem: The preplanning problem to solve.
        :param time_steps: The number of steps to simulate.
        :return: A tuple containing a dictionary mapping a time step to all generated valid states of that time step
        and the number of total states processed.
        """
        num_states_processed: int = 0
        valid_converted: Dict[int, drawable_types] = {}

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
                        delta_x: float = cos(transformed.orientation) * transformed.velocity * scenario.dt
                        delta_y: float = sin(transformed.orientation) * transformed.velocity * scenario.dt
                        transformed.position[0] += delta_x
                        transformed.position[1] += delta_y
                        if transformed not in current_states:
                            converted: drawable_types = DrawHelp.convert_to_drawable(transformed)
                            if is_valid(converted, scenario):
                                transformed.time_step += 1
                                if transformed.time_step not in valid_converted.keys():
                                    valid_converted[transformed.time_step] = []
                                valid_converted[transformed.time_step].append(converted)
                                current_states.put(transformed)
                                DrawHelp.draw(converted)
                current_states.task_done()
                num_states_processed += 1

        current_states: Queue = StatesQueue(GenerationConfig.position_threshold, GenerationConfig.angle_threshold)
        current_states.put(planning_problem.initial_state)
        for i in range(GenerationConfig.num_threads):
            worker: Thread = Thread(target=generate_next_states, args=(), daemon=True)
            worker.start()

        current_states.join()
        return valid_converted, num_states_processed
