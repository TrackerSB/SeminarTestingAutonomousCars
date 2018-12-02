from typing import List, Union

from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from matplotlib.patches import Patch, Rectangle
from shapely.geometry import MultiPolygon

drawable_types = Union[Patch, Scenario, LaneletNetwork, MultiPolygon, Rectangle]
convertible_types = Union[Scenario, LaneletNetwork, List, State]