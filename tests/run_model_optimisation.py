import os

from pymodelica import compile_jmu
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

    # jmu_name = compile_jmu(model_class_name, [model_path],
    #                        compiler_log_level='debug')
    # # Loading the JMU-object
    # model_opt = JMUModel(jmu_name)
    # # Calling the optimisation function with default settings
    # res = model_opt.optimize()

    ### 3. Solve the optimal control problem
    # Compile and load optimization problem
    op = transfer_optimization_problem(optimisation_model, model_path)

    # Set initial values
    op.set('h10', float(15))
    op.set('h20', float(14))

    # Set options
    opt_opts = op.optimize_options()
    opt_opts['n_e'] = 19  # Number of elements
    # opt_opts['init_traj'] = init_res
    # opt_opts['nominal_traj'] = init_res
    opt_opts['IPOPT_options']['tol'] = 1e-10
    opt_opts['verbosity'] = 1

    # Solve the optimal control problem
    res = op.optimize(options=opt_opts)

    # Plotting the results
    h1 = res['h1']
    h2 = res['h2']
    h3 = res['h3']
    # u is the optimal trajectory
    u = res['u']
    time = res['time']
    plt.plot(time, h1, ':k', time, h2, '--k', time, u, '-k')
    plt.grid(True)
    # Increasing the plot window to show results
    plt.ylim([-1.5, 2])
    plt.legend(('h1', 'h2', 'u'))
    plt.title('Simple example')
    plt.show()

if __name__ == '__main__':
    main()
