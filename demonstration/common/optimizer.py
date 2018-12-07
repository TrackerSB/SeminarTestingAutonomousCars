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
# W = diag(w(t_1),...,w(t_q))
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
# B = [b_1^1,...,b_n(1)^1,b_1^2,...,b_n(2)^2,...,b_n(p)^p] in IR^{q,r}
#                                                       All changes in area profile
# delta x_0 = [delta x_{0,1}^1, delta x_{0,2}^1,...,delta x_{0,n(1)}^1,...,delta x_{0,n(p)}^p]
#                                                       All variations of initial states
# S(delta x_0)                                          new scenario after change delta x_0
# gamma(S(delta x_0)) ~~ gamma(S_0) + sum{j=1:p}(sum{i=1:n(j)}(b_i^j * delta x_{0,i}^j))
#                                                       approximation of delta x_0 by sum of individual changes
# gamma(S(delta x_0)) ~~ B * delta x_0                  Same as line before
# delta a_0 := gamma(S_0) - a_ref
# arg min_{delta x_0}(delta x_0^T * W~ * delta x_0 + c^T * delta x_0
#                                                       quadratic optimization problem
# W~ = B^T * W * B
# c = delta a_0^T * (W * B + W^T * B)
# [0, v_max]                                            restriction of speed of vehicles
# velInd(j)                                             index of state x^j representing velocity of j-th vehicle
# for all j: x_{0,velInd(j)}^j + delta x_{0,velInd(j)}^j in [0, v_max]
#                                                       velocity constraint
# for all i in [0, q]: gamma(S_0)_i + (B * delta x_0)_i >= 0
#                                                       area constraint
# x_{0,before}                                          initial state before last quadratic update
# x_{0,after}                                           initial state after last quadratic update

# Notes about the approach:
# 2D => invariant to orientation and position => only such as velocity optimized
# delta a_0^T * W * delta a_0 can be removed in quadratic optimization problem
# interest in minimizing delta x_0 => optimizing over the shift of delta x_0
from queue import Queue
from typing import Tuple, Dict, List

import numpy as np
from shapely.geometry import MultiPolygon

from common import drawable_types
from common.draw import DrawHelp


def generate_area_profile(states: Dict[int, List[drawable_types]]) -> np.ndarray:
    area_profile: List[float] = []
    current_area: MultiPolygon = None
    for key in sorted(states.keys()):
        to_union: List[drawable_types] = []
        if current_area:
            to_union.append(current_area)
        for state in states[key]:
            to_union.append(state)
        current_area = DrawHelp.union_to_polygon(to_union)
        area_profile.append(current_area.area)
    return np.array(area_profile)


def binary_search(x_before, x_after, my, vehicles):
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

    b_max = 0
    b_abs: Dict[Tuple[int, int], float] = {}
    for j in range(1, len(vehicles)):
        for i in range(1, len(vehicles[j].states)):
            new_area_profile = None
            initial_area_profile = None
            variation_ij = None
            b_abs = abs((new_area_profile - initial_area_profile) / variation_ij)
    b_sorted: Queue[Tuple[Tuple[int, int], float]] = Queue()
    for bij in sorted(b_abs, key=lambda e: e[1]):
        print(type(bij))
        b_sorted.put(bij)

    raise Exception("Not implemented yet")


def kappa(gamma_s: np.matrix, a_ref: np.matrix, W: np.matrix) -> float:
    """
    Cost function
    """
    # kappa = (gamma(S) - a_ref)^T * W * (gamma(S) - a_ref)

    diff: np.matrix = gamma_s - a_ref
    return diff.transpose() * W * diff


def updateScenario(S, x_0sv):
    raise Exception("Not implemented yet")


def optimized_scenario(x_0, epsilon, it_max, my, W, a_ref):
    # Require
    # x_0       initial state
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
    raise Exception("Not implemented yet")
