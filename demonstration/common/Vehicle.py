from typing import Dict, List

from commonroad.scenario.trajectory import State
from numpy.core.records import ndarray


class Vehicle:
    delta_x: ndarray = []
    states: Dict[int, List[State]] = {}
