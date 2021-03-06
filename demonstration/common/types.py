from typing import List, Union

from commonroad.planning.goal import GoalRegion
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from commonroad.visualization.draw_dispatch_cr import plottable_types
from matplotlib.patches import Patch, Rectangle
from shapely.geometry import MultiPolygon, LineString, MultiLineString

drawable_types \
    = Union[Patch, Scenario, LaneletNetwork, MultiPolygon, Rectangle, LineString, MultiLineString, plottable_types]
convertible_types = Union[Scenario, LaneletNetwork, List[plottable_types], State, GoalRegion, plottable_types]  # FIXME Include MyState
