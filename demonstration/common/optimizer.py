# Variables of the paper:
# k                                                     time step
# t_k                                                   time at step k
# delta t                                               time step size
# q                                                     total number of time steps
# A(S, t)                                               drivable area at time t (summing up from t_0 to t)
# a_ref,k = A_ref(t_k)                                  q-dimensional vector reference area over time
# gamma(S) := [a_1,...,a_q], a_k = A(S, t_k)            area profile (development of the drivable area over discrete
#                                                       times t_k)
# arg min_S(gamma(S) - a_ref)^T * W(gamma(S) - a_ref)   discrete-time approximation of the optimization problem
# W = diag(w(t_1),...,w(t_q))                           weights matrix in IR^{p, p}
# w(t)                                                  weight
# V_1,...,V_p                                           vehicles
# p                                                     total number of considered traffic participants
# S_0                                                   initial scenario
# delta x_{0,i}^j                                       variation of i-th component of initial state of vehicle V_j
# S(<i,j>, delta x~_{0,i}^j)                            updated scenario after the i-th initial state of vehicle V_j has
#                                                       changed to x_{0,i}^j += delta x~_{0,i}^j
# b_i^j := (gamma(S(<i,j>, delta x~_{0,i}^j)) - gamma(S_0)) / x~_{0,i}^j) in IR^q
#                                                       change in the area profile
# gamma(S(<i,j>, delta x_{0,i}^j)) ~~ gamma(S_0) + b_i^j * delta x_{0,i}^j)
#                                                       the new area profile
# n(j)                                                  number of states of V_j
# B = [b_1^1,...,b_n(1)^1,b_1^2,...,b_n(2)^2,...,b_n(p)^p] in IR^{p,r}
#                                                       All changes in area profile
# r                                                     Number of states per car considered
# delta x_0 = [delta x_{0,1}^1, delta x_{0,2}^1,...,delta x_{0,n(1)}^1,...,delta x_{0,n(p)}^p]
#                                                       All variations of initial states
# S(delta x_0)                                          new scenario after change delta x_0
# gamma(S(delta x_0)) ~~ gamma(S_0) + sum{j=1:p}(sum{i=1:n(j)}(b_i^j * delta x_{0,i}^j))
#                                                       approximation of delta x_0 by sum of individual changes
# gamma(S(delta x_0)) ~~ B * delta x_0                  Same as line before
# delta a_0 := gamma(S_0) - a_ref
# arg min_{delta x_0}(delta x_0^T * W~ * delta x_0 + c^T * delta x_0
#                                                       quadratic optimization problem
# W~ = B^T * W * B                                      in IR^{r, r}
# c = delta a_0^T * (W * B + W^T * B)
# [0, v_max]                                            restriction of speed of vehicles
# velInd(j)                                             index of state x^j representing velocity of j-th vehicle
# for all j: x_{0,velInd(j)}^j + delta x_{0,velInd(j)}^j in [0, v_max]
#                                                       velocity constraint
# for all i in [0, q]: gamma(S_0)_i + (B * delta x_0)_i >= 0
#                                                       area constraint
# x                                                     state vector in IR^r
# x(t)                                                  state vector at time t
# x_i^j                                                 ith state of jth vehicle
# x_{0,before}                                          initial state before last quadratic update
# x_{0,after}                                           initial state after last quadratic update

# Notes about the approach:
# 2D => invariant to orientation and position => only such as velocity optimized
# delta a_0^T * W * delta a_0 can be removed in quadratic optimization problem
# interest in minimizing delta x_0 => optimizing over the shift of delta x_0
from queue import Queue
from typing import Tuple, Dict, List

import cvxpy
import numpy as np
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
from cvxpy import Variable, Problem, Minimize, quad_form
from dccp import is_dccp  # Make sure there is at least one dccp import. Otherwise dccp is not registered to cvxpy.
from numpy import eye, matrix
from numpy.core.multiarray import ndarray, inner
from numpy.linalg import norm
from shapely.geometry import MultiPolygon, Point, Polygon

from common import drawable_types, VehicleInfo, MyState, flatten_dict_values
from common.draw import DrawHelp
from common.generation import GenerationHelp


def calculate_area_profile(infos: List[VehicleInfo]) -> ndarray:
    """
    Generates a list [a_1,...,a_q] where a_i represents the drivable area at time step i.
    :param infos: All the states at any time step to recognize.
    :return: The area profile (In the paper: gamma(S)).
    """
    partitioned_infos: Dict[int, List[VehicleInfo]] = {}
    for info in infos:
        time_step: int = info.state.state.time_step
        if time_step not in partitioned_infos:
            partitioned_infos[time_step] = []
        partitioned_infos[time_step].append(info)

    area_profile: List[float] = []
    current_area: MultiPolygon = None
    for key in sorted(partitioned_infos.keys()):
        to_union: List[drawable_types] = []
        if current_area:
            to_union.append(current_area)
        for info in partitioned_infos[key]:
            to_union.append(info.drawable)
        current_area = DrawHelp.union_to_polygon(to_union)
        area_profile.append(current_area.area)
    return np.array(area_profile)


def calculate_B(initial_vehicles: List[VehicleInfo], current_vehicles: List[VehicleInfo], scenario: Scenario,
                total_steps: int) -> Dict[Tuple[int, int], float]:
    p: int = len(initial_vehicles)

    initial_area_profiles: Dict[int, ndarray] = {}
    for j in range(len(current_vehicles)):
        initial_area_profiles[j], _ \
            = GenerationHelp.generate_states(scenario, initial_vehicles[j].state, total_steps)

    B: Dict[Tuple[int, int], float] = {}
    for j in range(p):
        new_states, _ = GenerationHelp.generate_states(scenario, current_vehicles[j].state, total_steps)
        new_area_profile: ndarray = calculate_area_profile(flatten_dict_values(new_states))
        state_j: MyState = current_vehicles[j].state
        initial_state_j: MyState = initial_vehicles[j].state
        n_j: int = len(state_j.variables)
        for i in range(n_j):
            variation_ij = state_j.variable(i) - initial_state_j.variable(i)
            if variation_ij == 0:
                B[(i, j)] = -1
            else:
                B[(i, j)] = norm((new_area_profile - initial_area_profiles[j]) / variation_ij)
    return B


def calculate_B_as_matrix(initial_vehicles: List[VehicleInfo], current_vehicles: List[VehicleInfo],
                          scenario: Scenario, total_steps: int) -> matrix:
    B: Dict[Tuple[int, int], float] = calculate_B(initial_vehicles, current_vehicles, scenario, total_steps)
    B_mat: matrix = eye(len(MyState.variables), len(initial_vehicles))
    for (i, j) in B.keys():
        B_mat[i][j] = B[(i, j)]

    return B_mat


def binary_search(my: int, initial_vehicles_before: List[VehicleInfo], initial_vehicles_after: List[VehicleInfo],
                  scenario: Scenario, total_steps: int) -> float:
    # Require
    # x_{0,before,i}^j      initial state before last quadratic update
    # x_{0,after,i}^j       initial state after last quadratic update
    # my                    iteration limit

    # Ensure
    # x_{0,i}^j so that solution space is not empty

    # b_max <- 0
    # for j = 1...p
    #     for i = 1...n(j)
    #         b_{abs,i}^j <- abs((the new area profile - initial profile) / variation of i-th component of initial state of vehicle V_j)
    #     end for
    # end for
    # b_sorted = sort(b_abs)
    # while not empty(b_sorted) do
    #     vI, sI <- pop(b_sorted)
    #     for theta = 1...my do
    #         x_{0,curr,sI}^vI <- 0.5 * (x_{0,before,sI}^vI + x_{0,after,sI}^vI)
    #         a_curr <- S(<sI,vI>, delta x_{0,sI}^vI)
    #         if for all l: a_{curr,l} > 0 then
    #             return x_{0,curr,sI}^vI
    #         else
    #             x_{0,after,sI} <- x_{0,curr,sI}^vI
    #         end if
    #     end for
    # end while
    # return x_{0,before,sI}^vI

    b_max = 0  # FIXME What is this variable for?
    # Absolute values of sensitivities
    b_abs: Dict[Tuple[int, int], float] \
        = calculate_B(initial_vehicles_before, initial_vehicles_after, scenario, total_steps)
    for key in b_abs.keys():
        b_abs[key] = abs(b_abs[key])

    b_sorted: Queue[Tuple[int, int]] = Queue()
    for bij in sorted(b_abs, key=lambda key: b_abs[key], reverse=True):  # Sort based on sensitivity descending
        b_sorted.put(bij)
    current_vehicles: List[VehicleInfo] = initial_vehicles_before  # States to be modified step by step
    while not b_sorted.empty():
        v_i, s_i = b_sorted.get()
        for theta in range(my):
            state_sum: float = initial_vehicles_before[s_i].state.variable(v_i) \
                               + initial_vehicles_after[s_i].state.variable(v_i)
            current_vehicles[s_i].state.set_variable(v_i, 0.5 * state_sum)
            current_area: ndarray = calculate_area_profile(
                flatten_dict_values(
                    GenerationHelp.generate_states(scenario, current_vehicles[s_i].state, total_steps)[0]))
            if all(current_area > 0):
                return current_vehicles[s_i].state.variable(v_i)
            else:
                initial_vehicles_after[s_i].state.set_variable(v_i, current_vehicles[s_i].state.variable(v_i))

    return initial_vehicles_before[s_i].state.variable(v_i)


def kappa(gamma_s: ndarray, a_ref: ndarray, W: np.matrix) -> float:
    """
    Cost function
    """
    # kappa = (gamma(S) - a_ref)^T * W * (gamma(S) - a_ref)

    diff: ndarray = gamma_s - a_ref
    return inner(diff * W, diff)


def update_scenario_vehicle(scenario: Scenario, planning_problem: PlanningProblem, s_i: VehicleInfo, v_i: int):
    """
    Modifies a certain state of a certain vehicle within the scenario.
    :param scenario: The scenario to modify
    :param planning_problem: The preplanning problem containing the initial state of the ego vehicle.
    :param s_i: The VehicleInfo containing the index of the vehicle within the scenario and the new variable value to
    set.
    :param v_i: The index of the variable to modify.
    """
    s_idx: int = s_i.dynamic_obs_index
    if s_idx > -1:
        initial_state: State = scenario.dynamic_obstacles[s_idx].initial_state
    else:
        initial_state: State = planning_problem.initial_state
    MyState.set_variable_to(initial_state, v_i, s_i.state.variable(v_i))


def update_scenario_vehicles(scenario: Scenario, planning_problem: PlanningProblem, vehicle_infos: List[VehicleInfo]):
    """
    Modifies all initial states of all vehicles of the scenario using the given infos.
    :param scenario: The scenario to modify.
    :param planning_problem: The preplanning problem containing the initial state of the ego vehicle.
    :param vehicle_infos: The vehicle infos to set to the scenario
    """
    for info in vehicle_infos:
        for v_i in range(len(MyState.variables)):
            update_scenario_vehicle(scenario, planning_problem, info, v_i)


def optimized_scenario(initial_vehicles: List[VehicleInfo], epsilon: float, it_max: int, my: int, a_ref: ndarray,
                       scenario: Scenario, planning_problem: PlanningProblem, W: np.matrix = None):
    # Require
    # x_0       initial state vector
    # epsilon   threshold
    # it_max    iteration limit
    # my        binary search iteration limit
    # W         weighted matrix
    # a_ref     reference area profile

    # Ensure
    # critical scenario S

    # kappa_new <- 0
    # kappa_old <- -inf
    # it <- 0
    # x_{0,curr} <- x_o
    # while abs(kappa_new - kappa_old) >= epsilon and it < it_max do
    #     success <- true
    #     while abs(kappa_new - kappa_old) >= epsilon and success = true do
    #         kappa_old <- kappa_new
    #         x_{0,old} <- x_{0,curr}
    #         x_{0,curr}, S, success <- quadProg(solve(quadratic optimization problem))
    #         kappa_new <- kappa(S, a_ref, W)
    #     end while
    #     x_{0,s)^v <- binarySearch(x_{0,old}, x_{0,curr}, my)
    #     S <- updateScenario(S, x_{0,s}^v)
    #     kappa_new <- kappa(S, a_ref, W)
    #     it <- it + 1
    # end while

    p: int = len(initial_vehicles)
    r: int = len(MyState.variables)
    total_steps: int = len(a_ref)

    if not W:
        W = eye(p)

    kappa_new: float = 0
    kappa_old: float = -np.inf
    it: int = 0
    current_initial_vehicles = initial_vehicles
    while abs(kappa_new - kappa_old) >= epsilon and it < it_max:
        success: bool = True
        while abs(kappa_new - kappa_old) >= epsilon and success:
            kappa_old = kappa_new
            old_initial_vehicles: List[VehicleInfo] = current_initial_vehicles
            current_generated_vehicles, _ = GenerationHelp.generate_states(
                scenario, current_initial_vehicles[0].state, total_steps)  # FIXME Only ego vehicle considered
            # x_{0,curr}, S, success <- quadProg(solve(quadratic optimization problem))
            B: matrix = calculate_B_as_matrix(old_initial_vehicles, current_initial_vehicles, scenario, total_steps)
            W_tilde: matrix = np.asmatrix(B.transpose() * W * B)
            delta_a0: matrix = np.asmatrix(
                calculate_area_profile(flatten_dict_values(current_generated_vehicles)) - a_ref)
            c: ndarray = delta_a0.transpose() * (W * B + W.transpose() * B)
            delta_x: Variable = Variable((1, r))
            objective: Minimize = Minimize(cvxpy.norm(quad_form(delta_x, W_tilde) + c * delta_x))
            # Instead of the "real" goal region constraint use a minimum velocity needed for getting to the goal region
            # within the given amount of steps
            # FIXME Only ego vehicle considered
            current_initial_position: Point = Point(current_initial_vehicles[0].state.state.position)
            # FIXME Only first goal region entry recognized
            goal_region: Polygon = planning_problem.goal.state_list[0].position.shapely_object
            distance_to_goal_region: float = current_initial_position.distance(goal_region)
            min_velocity: float = distance_to_goal_region / (scenario.dt * total_steps)
            print("min_velocity: " + str(min_velocity))
            # FIXME Really use delta_a0?
            # FIXME If min_velocity is greater than zero the optimization fails
            constraints = [delta_x >= 0, delta_a0 + B * delta_x >= 0, delta_x[0] >= min_velocity]
            problem = Problem(objective, constraints)
            assert is_dccp(problem)
            print("Problem solve result: " + str(problem.solve(method='dccp', solver='ECOS')))
            print("Current optimization result: " + str(delta_x.value))
            # FIXME Change current initial vehicles like this?
            for i in range(len(current_initial_vehicles)):
                for j in range(len(MyState.variables)):
                    # FIXME Only ego vehicle considered
                    current_initial_vehicles[i].state.set_variable(j, delta_x.value[i][j])
            kappa_new = kappa(delta_a0, a_ref, W)  # FIXME Really use delta_a0?
            print("Difference between new and old costs: " + str(abs(kappa_new - kappa_old)))
        binary_search(my, old_initial_vehicles, current_initial_vehicles, scenario, total_steps)  # FIXME What to do with this value?
        # initial_vehicles = ?  # FIXME What to do here?
        update_scenario_vehicles(scenario, planning_problem, initial_vehicles)
        kappa_new = kappa(calculate_area_profile(initial_vehicles), a_ref, W)
        it += 1
