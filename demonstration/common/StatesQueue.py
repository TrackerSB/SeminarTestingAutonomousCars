from queue import Queue

from commonroad.scenario.trajectory import State


class StatesQueue(Queue):
    def __init__(self, position_threshold: float, angle_threshold: float):
        super().__init__()
        self.queue = set()
        self.position_threshold = position_threshold
        self.angle_threshold = angle_threshold

    def _put(self, item):
        self.queue.add(item)

    def _get(self):
        return self.queue.pop()

    def __contains__(self, item: State):
        with self.mutex:
            return any(map(
                lambda s: all(abs(s.position - item.position) <= self.position_threshold)
                          and abs(s.orientation - item.orientation) <= self.angle_threshold, self.queue))
