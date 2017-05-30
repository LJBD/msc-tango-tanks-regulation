from pyfmi.fmi import load_fmu

from ds_tanks.tanks_utils import plot_results, U_MAX, simulate_tanks, \
    print_stationary_point, get_model_path
from pymodelica import compile_fmu
from pyjmi import transfer_optimization_problem


def main():
    model_path = get_model_path()

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
    u_0_B = 78.0
    init_model.set('u', u_0_B)
    # Solve the initialization problem using FMI
    init_model.initialize()
    # Store stationary point B
    [h1_0_B, h2_0_B, h3_0_B] = init_model.get(['h1', 'h2', 'h3'])
    # Print some data for stationary point B
    print_stationary_point('B', h1_0_B, h2_0_B, h3_0_B, u_0_B)

    h1_sim, h2_sim, h3_sim, init_res = simulate_tanks(model_path, u=u_0_B,
                                                      with_plots=True)

    h1_sim_final = h1_sim[-1]
    h2_sim_final = h2_sim[-1]
    h3_sim_final = h3_sim[-1]

    print "Final values of the simulation:", h1_sim_final, h2_sim_final,\
        h3_sim_final

    # 3. Solve the optimal control problem
    # Compile and load optimization problem
    op = transfer_optimization_problem("TanksPkg.three_tanks_time_optimal",
                                       model_path)

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
    opt_opts['IPOPT_options']['tol'] = 1e-3
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
    print "Final value of time:", time_res[-1]
    print dir(res)
    print res.final("time")

    # Plot the results
    plot_results(h1_res, h2_res, h3_res, time_res, u_res,
                 title="Optimised trajectories")


if __name__ == '__main__':
    main()
