from copy import deepcopy
from queue import Queue
from threading import Thread, currentThread
from time import sleep
from typing import Optional, Union, Tuple

import matplotlib.pyplot as plt
import numpy as np
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from numpy.core.multiarray import ndarray
from numpy.core.umath import pi

from StatesQueue import StatesQueue
from common import load_scenario, is_valid_position, convert_and_draw


class Config:
    max_time_step: int = 50
    max_yaw: float = pi / 8  # 22.5°
    yaw_steps: int = 8
    num_threads: int = 8


def generate_next_states(current_states: Queue, scenario: Scenario) -> None:
    while True:
        state: State = current_states.get()
        if state.time_step < Config.max_time_step:
            yaw_steps: Union[ndarray, Tuple[ndarray, Optional[float]]]\
                = np.linspace(-Config.max_yaw, Config.max_yaw, num=Config.yaw_steps, endpoint=True)
            for yaw in yaw_steps:
                delta_x: float = np.cos(state.orientation) * state.velocity * scenario.dt
                delta_y: float = np.sin(state.orientation) * state.velocity * scenario.dt
                transformed: State = deepcopy(state)
                transformed.position[0] += delta_x
                transformed.position[1] += delta_y
                transformed.orientation += yaw
                if is_valid_position(transformed.position, scenario) and transformed not in current_states:
                    transformed.time_step += 1
                    current_states.put(transformed)
                    # print(transformed)
                    convert_and_draw(transformed)
        current_states.task_done()


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')

    plt.figure(figsize=(25, 10))

    convert_and_draw(scenario)
    convert_and_draw(planning_problem.initial_state)

    current_states: Queue = StatesQueue()
    for i in range(Config.num_threads):
        worker: Thread = Thread(target=generate_next_states, args=(current_states, scenario), daemon=True)
        worker.start()

    current_states.put(planning_problem.initial_state)

    def print_state_of_queue(queue: Queue):
        t: Thread = currentThread()
        while getattr(t, "do_run", True):
            print(queue.qsize())
            sleep(1)

    state_worker: Thread = Thread(target=print_state_of_queue, args=(current_states,))
    state_worker.start()

    current_states.join()
    state_worker.do_run = False
    state_worker.join()

    plt.gca().set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
