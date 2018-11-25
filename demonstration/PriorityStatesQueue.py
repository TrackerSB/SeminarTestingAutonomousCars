import heapq
from queue import Queue

from commonroad.scenario.trajectory import State


class PriorityStatesQueue(Queue):
    """
    Implements a priority queue for states. To reduce the memory consumption a high time_step value indicates a high
    priority. NOTE It is likely to be much slower than the StatesQueue.
    """

    def __init__(self, position_threshold: float, angle_threshold: float):
        super().__init__()
        self.queue = []
        self.index = 0
        self.position_threshold = position_threshold
        self.angle_threshold = angle_threshold

    def _put(self, item: State):
        heapq.heappush(self.queue, (-item.time_step, self.index, item))
        self.index += 1

    def _get(self):
        return heapq.heappop(self.queue)[-1]

    def __contains__(self, item: State):
        with self.mutex:
            return any(map(
                lambda s: all(abs(s[2].position - item.position) <= self.position_threshold)
                          and abs(s[2].orientation - item.orientation) <= self.angle_threshold, self.queue))
