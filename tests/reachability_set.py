from tests.run_model_optimisation import get_model_path, simulate


class ReachabilitySetsCalculator():
    def __init__(self):
        self.model_path = get_model_path()

    def single_simulation(self):
        h1_init_sim, h2_init_sim, h3_init_sim, init_res = simulate(self.model_path)


def main():
    rsc = ReachabilitySetsCalculator()

if __name__ == '__main__':
    main()
