import datetime
from datetime import datetime
from typing import List, Optional

import matplotlib.pyplot as plt
from matplotlib.animation import ArtistAnimation
from matplotlib.artist import Artist
from shapely.geometry import MultiPolygon

from common import load_scenario, flatten_dict_values, VehicleInfo, MyState
from common.draw import DrawHelp
from common.generation import GenerationHelp

num_steps: int = 10


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1_mod.xml')

    fig = plt.figure(figsize=(25, 10))

    DrawHelp.draw(DrawHelp.convert_to_drawable(scenario))
    DrawHelp.draw(DrawHelp.convert_to_drawable(planning_problem.initial_state))
    # DrawHelp.draw(DrawHelp.convert_to_drawable(planning_problem.goal))

    MyState.set_variable_to(planning_problem.initial_state, 0, 15)

    start_time: datetime = datetime.now()
    valid_converted, num_states_processed \
        = GenerationHelp.generate_states(scenario, MyState(planning_problem.initial_state), num_steps)
    print("Processed " + str(num_states_processed) + " states in " + str(datetime.now() - start_time))

    all_states: List[VehicleInfo] = flatten_dict_values(valid_converted)

    frames: list = []

    for i in range(1, num_steps + 1):
        frame = list(map(lambda v: DrawHelp.draw(v.drawable), filter(lambda v: v.state.state.time_step <= i, all_states)))
        union: MultiPolygon = DrawHelp.union_to_polygon(frame)
        artist: Optional[Artist] = DrawHelp.draw(union)
        if artist is not None:
            if isinstance(artist, list):
                frame.extend(artist)
            else:
                frame.append(artist)
        frames.append(frame)

    print("Drivable area: " + str(union.area))

    plt.gca().set_aspect('equal')
    plt.ylim([35, 60])
    plt.xlim([92, 150])
    ani = ArtistAnimation(fig, frames, interval=500, blit=True, repeat_delay=1000)
    # ani.save("2019-02-16_SimpleDrivableArea_10steps_35ms.mp4", writer="ffmpeg")  # mp4 broken
    plt.show()


if __name__ == '__main__':
    main()
