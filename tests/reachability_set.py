from math import pi

from numpy.ma import arange
from pyfmi.fmi import load_fmu

from pymodelica import compile_fmu
from tests.run_model_optimisation import get_model_path, simulate


class ReachabilitySetsCalculator:
    def __init__(self):
        self.model_path = get_model_path()

    def initialise_costate_equations(self, theta, phi):
        init_fmu = compile_fmu("TanksPkg.ThreeTanksCostateEquationsInitialisation", self.model_path)
        init_model = load_fmu(init_fmu)
        init_model.set('phi', phi)
        init_model.set('theta', theta)
        init_model.initialize()
        return init_model.get(['psi1', 'psi2', 'psi3'])

    def get_reachability_set(self, precision=.01):
        for phi in arange(0, 2*pi, precision):
            for theta in arange(0, 2*pi, precision):
                [psi10, psi20, psi30] = self.initialise_costate_equations(theta, phi)
                print psi10, psi20, psi30

    def single_simulation(self):
        h1_init_sim, h2_init_sim, h3_init_sim, init_res = simulate(self.model_path)


def main():
    rsc = ReachabilitySetsCalculator()
    rsc.get_reachability_set()

if __name__ == '__main__':
    main()
