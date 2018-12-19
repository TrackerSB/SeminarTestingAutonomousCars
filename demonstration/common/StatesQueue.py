from queue import Queue

from common import MyState


class StatesQueue(Queue):
    def __init__(self, position_threshold: float, angle_threshold: float):
        super().__init__()
        self.queue = set()
        self.position_threshold = position_threshold
        self.angle_threshold = angle_threshold

    def _put(self, item: MyState):
        self.queue.add(item)

    def _get(self):
        return self.queue.pop()

    def __contains__(self, item: MyState):
        with self.mutex:
            return any(map(
                lambda s: all(abs(s.state.position - item.state.position) <= self.position_threshold)
                          and abs(s.state.orientation - item.state.orientation) <= self.angle_threshold, self.queue))
