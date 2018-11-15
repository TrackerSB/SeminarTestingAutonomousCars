from common import load_scenario, is_valid_position


def main() -> None:
    scenario, planning_problem = load_scenario('scenarios/DEU_B471-1_1_T-1/DEU_B471-1_1_T-1.xml')
    print(is_valid_position(planning_problem.initial_state.position, scenario))


if __name__ == '__main__':
    main()
