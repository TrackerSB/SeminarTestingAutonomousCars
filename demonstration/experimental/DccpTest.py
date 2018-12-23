from cvxpy import Variable, Maximize, square, Problem
from dccp.problem import is_dccp

def main():
    x = Variable(1)
    y = Variable(1)
    constraints = [x >= 0, y >= 0, x+y == 1]
    objective = Maximize(square(x) + square(y))
    problem = Problem(objective, constraints)

    print("problem is DCP:", problem.is_dcp())
    print("problem is DCCP:", is_dccp(problem))

    problem.solve(method='dccp')

    print('solution (x,y): ', x.value, y.value)


if __name__ == '__main__':
    main()
