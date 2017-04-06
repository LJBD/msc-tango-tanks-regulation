import os

from pyfmi.fmi import load_fmu

from pymodelica import compile_jmu, compile_fmu
from pyjmi import JMUModel, transfer_optimization_problem
import matplotlib.pyplot as plt


def main():
    # Compiling a JMU- object for optimization based on the double integrator
    project_base = os.path.abspath(__file__)
    project_base = project_base[:project_base.rfind('/')]
    project_base = project_base[:project_base.rfind('/')]
    print "Project base path:", project_base
    os.environ["MODELICAPATH"] += ":%s/optimica_model" % project_base
    model_file = "opt_3_tanks.mop"
    model_path = os.path.join(project_base, 'optimica_model', model_file)
    model_class_name = "TanksPkg.ThreeTanks"
    optimisation_model = "TanksPkg.three_tanks_time_optimal"

    # 1. Solve the initialization problem
    # Compile the stationary initialization model into an FMU
    init_fmu = compile_fmu("TanksPkg.ThreeTanksInit", model_path)
    init_model = load_fmu(init_fmu)

    # Set input for Stationary point A
    u_0_A = 0
    init_model.set('u', u_0_A)
    # Solve the initialization problem using FMI
    init_model.initialize()
    # Store stationary point A
    [h1_0_A, h2_0_A, h3_0_A] = init_model.get(['h1', 'h2', 'h3'])
    # Print some data for stationary point A
    print_stationary_point('A', h1_0_A, h2_0_A, h3_0_A, u_0_A)

    # Set inputs for Stationary point B
    init_model.reset()  # reset the FMU so that we can initialize it again
    u_0_B = 20
    init_model.set('u', u_0_B)
    # Solve the initialization problem using FMI
    init_model.initialize()
    # Store stationary point B
    [h1_0_B, h2_0_B, h3_0_B] = init_model.get(['h1', 'h2', 'h3'])
    # Print some data for stationary point B
    print_stationary_point('B', h1_0_B, h2_0_B, h3_0_B, u_0_B)

    # ### 3. Solve the optimal control problem
    # # Compile and load optimization problem
    # op = transfer_optimization_problem(optimisation_model, model_path)
    #
    # # Set initial values
    # op.set('h10', float(10))
    # op.set('h20', float(8))
    # op.set('h30', float(10))
    #
    # # Set options
    # opt_opts = op.optimize_options()
    # # opt_opts['n_e'] = 19  # Number of elements
    # # opt_opts['init_traj'] = init_res
    # # opt_opts['nominal_traj'] = init_res
    # opt_opts['IPOPT_options']['tol'] = 1e-6
    # opt_opts['verbosity'] = 1
    #
    # # Solve the optimal control problem
    # res = op.optimize(options=opt_opts)
    #
    # # Plotting the results
    # h1 = res['h1']
    # h2 = res['h2']
    # h3 = res['h3']
    # # u is the optimal trajectory
    # u = res['u']
    # time = res['time']
    # print time
    # plt.plot(time, h1, '-r', time, h2, '-g', time, h3, '-b', time, u, '-k')
    # plt.grid(True)
    # # Increasing the plot window to show results
    # # plt.ylim([-1.5, 2])
    # plt.legend(('h1', 'h2', 'h3', 'u'))
    # plt.title('Simple example')
    # plt.show()


def print_stationary_point(identifier, h1_0, h2_0, h3_0, u_0):
    print(' *** Stationary point %s ***' % identifier)
    print('u = %f' % u_0)
    print('h1 = %f' % h1_0)
    print('h2 = %f' % h2_0)
    print('h3 = %f' % h3_0)


if __name__ == '__main__':
    main()
