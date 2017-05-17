import os

from pyfmi.fmi import load_fmu

from pymodelica import compile_jmu, compile_fmu
from pyjmi import JMUModel, transfer_optimization_problem
import matplotlib.pyplot as plt


U_MAX = 30.0


def main():
    project_base = os.path.abspath(__file__)
    project_base = project_base[:project_base.rfind('/')]
    project_base = project_base[:project_base.rfind('/')]
    print "Project base path:", project_base
    os.environ["MODELICAPATH"] += ":%s/optimica_model" % project_base
    model_file = "opt_3_tanks.mop"
    model_path = os.path.join(project_base, 'optimica_model', model_file)

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
    u_0_B = U_MAX
    init_model.set('u', u_0_B)
    # Solve the initialization problem using FMI
    init_model.initialize()
    # Store stationary point B
    [h1_0_B, h2_0_B, h3_0_B] = init_model.get(['h1', 'h2', 'h3'])
    # Print some data for stationary point B
    print_stationary_point('B', h1_0_B, h2_0_B, h3_0_B, u_0_B)

    # 2. Compute initial guess trajectories by means of simulation
    # Compile the optimization initialization model
    init_sim_fmu = compile_fmu("TanksPkg.ThreeTanks", model_path)
    # Load the model
    init_sim_model = load_fmu(init_sim_fmu)
    # Set initial and reference values
    init_sim_model.set('u', U_MAX)

    # Simulate with constant input Tc
    init_res = init_sim_model.simulate(start_time=0.0, final_time=50.0)

    # Extract variable profiles
    t_init_sim = init_res['time']
    h1_init_sim = init_res['h1']
    h2_init_sim = init_res['h2']
    h3_init_sim = init_res['h3']
    u_init_sim = init_res['u']

    # Plot the initial guess trajectories
    plot_results(h1_init_sim, h2_init_sim, h3_init_sim, t_init_sim, u_init_sim,
                 title='Initial guess obtained by simulation')

    h1_sim_final = h1_init_sim[-1]
    h2_sim_final = h2_init_sim[-1]
    h3_sim_final = h3_init_sim[-1]

    print "Final values of the simulation:", h1_sim_final, h2_sim_final, h3_sim_final

    # 3. Solve the optimal control problem
    # Compile and load optimization problem
    op = transfer_optimization_problem("TanksPkg.three_tanks_time_optimal", model_path)

    # Set initial values
    # op.set('h1_final', h1_sim_final)
    # op.set('h2_final', h2_sim_final)
    # op.set('h3_final', h3_sim_final)
    op.set('h1_final', float(h1_0_B))
    op.set('h2_final', float(h2_0_B))
    op.set('h3_final', float(h3_0_B))
    op.set('u_max', U_MAX)

    # Set options
    opt_opts = op.optimize_options()
    # opt_opts['n_e'] = 80  # Number of elements
    opt_opts['variable_scaling'] = False
    opt_opts['init_traj'] = init_res
    # opt_opts['nominal_traj'] = init_res
    opt_opts['IPOPT_options']['tol'] = 1e-5
    opt_opts['verbosity'] = 1
    print "OPTIMISATION OPTIONS:"
    for option, value in opt_opts.items():
        print '\t', option, ':', value

    # Solve the optimal control problem
    res = op.optimize(options=opt_opts)

    # Extract variable profiles
    h1_res = res['h1']
    h2_res = res['h2']
    h3_res = res['h3']
    u_res = res['u']
    time_res = res['time']

    # Plot the results
    plot_results(h1_res, h2_res, h3_res, time_res, u_res,
                 title="Optimised trajectories")


def plot_results(h1, h2, h3, time, u, title):
    plt.close(1)
    plt.figure(1)
    # plt.hold(True)
    plt.subplot(2, 1, 1)
    plt.plot(time, h1, 'r', time, h2, 'g',
             time, h3, 'b')
    plt.grid()
    plt.legend(('h1', 'h2', 'h3'))
    plt.ylabel('Levels in tanks')
    plt.title(title)
    plt.subplot(2, 1, 2)
    plt.plot(time, u)
    plt.grid()
    plt.ylabel('Control')
    plt.xlabel('Time')
    plt.show()


def print_stationary_point(identifier, h1_0, h2_0, h3_0, u_0):
    print(' *** Stationary point %s ***' % identifier)
    print('u = %f' % u_0)
    print('h1 = %f' % h1_0)
    print('h2 = %f' % h2_0)
    print('h3 = %f' % h3_0)


if __name__ == '__main__':
    main()
