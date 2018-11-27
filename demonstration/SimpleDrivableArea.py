import datetime
from copy import deepcopy
from datetime import datetime
from queue import Queue
from threading import Thread, currentThread
from time import sleep
from typing import Optional, Union, Tuple

import matplotlib.pyplot as plt
import numpy as np
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from matplotlib.patches import Patch
from numpy.core.multiarray import ndarray
from numpy.core.umath import pi

from StatesQueue import StatesQueue
from common import load_scenario, is_valid, convert_to_drawable, draw


class Config:
    max_time_step: int = 10
    max_yaw: float = pi / 16  # 11.25°
    yaw_steps: int = 8
    num_threads: int = 8
    position_threshold = 0.5
    angle_threshold = max_yaw * 0.85  # NOTE Needs to be way smaller than max_yaw otherwise the car tends to the right.


num_states_processed: int = 0


def generate_next_states(current_states: Queue, scenario: Scenario) -> None:
    global num_states_processed
    while True:
        state: State = current_states.get()
        if state.time_step < Config.max_time_step:
            yaw_steps: Union[ndarray, Tuple[ndarray, Optional[float]]] \
                = np.linspace(-Config.max_yaw, Config.max_yaw, num=Config.yaw_steps, endpoint=True)
            for yaw in yaw_steps:
                transformed: State = deepcopy(state)
                transformed.orientation += yaw
                delta_x: float = np.cos(transformed.orientation) * transformed.velocity * scenario.dt
                delta_y: float = np.sin(transformed.orientation) * transformed.velocity * scenario.dt
                transformed.position[0] += delta_x
                transformed.position[1] += delta_y
                if transformed not in current_states:
                    converted: Union[Patch, Scenario, LaneletNetwork, None] = convert_to_drawable(transformed)
                    if is_valid(converted, scenario):
                        transformed.time_step += 1
                        current_states.put(transformed)
                        draw(converted)
        current_states.task_done()
        num_states_processed += 1


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')

    plt.figure(figsize=(25, 10))

    draw(convert_to_drawable(scenario))
    draw(convert_to_drawable(planning_problem.initial_state))

    current_states: Queue = StatesQueue(Config.position_threshold, Config.angle_threshold)
    current_states.put(planning_problem.initial_state)
    for i in range(Config.num_threads):
        worker: Thread = Thread(target=generate_next_states, args=(current_states, scenario), daemon=True)
        worker.start()

    start_time: datetime = datetime.now()

    def print_state_of_queue(queue: Queue):
        t: Thread = currentThread()
        while getattr(t, "do_run", True):
            print(queue.qsize())
            sleep(1)

    state_worker: Thread = Thread(target=print_state_of_queue, args=(current_states,))
    state_worker.start()

    current_states.join()
    print("Processed " + str(num_states_processed) + " in " + str(datetime.now() - start_time))
    state_worker.do_run = False
    state_worker.join()

    plt.gca().set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    main()
