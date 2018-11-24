from queue import Queue

from commonroad.scenario.trajectory import State


class StatesQueue(Queue):
    def __init__(self):
        super().__init__()
        self.queue = set()

    def _put(self, item):
        self.queue.add(item)

    def _get(self):
        return self.queue.pop()

    def __contains__(self, item: State):
        acceptance_threshold: float = 0.1  # FIXME This value has no special reason relating its size
        with self.mutex:
            # print("Contains: " + str(item.orientation))
            return any(map(
                lambda s: all(abs(s.position - item.position) <= acceptance_threshold)
                          and abs(s.orientation - item.orientation) <= acceptance_threshold, self.queue))
