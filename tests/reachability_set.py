from math import pi

from numpy.ma import arange
from pyfmi.fmi import load_fmu, FMUException

from pymodelica import compile_fmu
from tests.run_model_optimisation import get_model_path, simulate_tanks, U_MAX


class ReachabilitySetsCalculator:
    def __init__(self):
        self.model_path = get_model_path()
        self.init_fmu = compile_fmu("TanksPkg.ThreeTanksCostateEquations",
                                    self.model_path, version=2.0)
        self.init_model = load_fmu(self.init_fmu)

    def initialise_costate_equations(self, theta, phi):
        self.init_model.set('phi', phi)
        self.init_model.set('theta', theta)
        self.init_model.initialize()
        return_param = self.init_model.get(['psi1', 'psi2', 'psi3'])
        self.init_model.reset()
        return return_param

    def get_reachability_set(self, precision=.1):
        for phi in arange(0, 2*pi, precision):
            for theta in arange(0, 2*pi, precision):
                self.init_model.reset()
                try:
                    self.init_model.set('phi', float(phi))
                    self.init_model.set('theta', float(theta))
                except FMUException:
                    self.init_model = load_fmu(self.init_fmu)
                    self.init_model.set('phi', float(phi))
                    self.init_model.set('theta', float(theta))
                init_res = self.single_simulation(self.init_model)
                # print init_model.get_log()
                print init_res

    def single_simulation(self, init_sim_model=None, t_start=0.0, t_final=50.0):
        if not init_sim_model:
            init_sim_fmu = compile_fmu("TanksPkg.ThreeTanksCostateEquations", self.model_path, version=2.0)
            # Load the model
            init_sim_model = load_fmu(init_sim_fmu)
        # Set initial and reference values
        init_sim_model.set('u', U_MAX)  # TODO: develop a way to include real u calculation here
        # Simulate with constant input Tc
        init_res = init_sim_model.simulate(start_time=t_start, final_time=t_final)
        return init_res


def main():
    rsc = ReachabilitySetsCalculator()
    rsc.get_reachability_set()

if __name__ == '__main__':
    main()
