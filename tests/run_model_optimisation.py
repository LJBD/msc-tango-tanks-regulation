from __future__ import print_function
from pyfmi.fmi import load_fmu, FMUException

from ds_tanks.tanks_utils import plot_results, U_MAX, simulate_tanks, \
    print_stationary_point, get_model_path, plot_with_optimal_trajectories
from pymodelica import compile_fmu
from pyjmi import transfer_optimization_problem


def main():
    model_path = get_model_path()

    # 1. Solve the initialization problem
    # Compile the stationary initialization model into an FMU
    init_fmu = compile_fmu("TanksPkg.ThreeTanksInit", model_path)
    init_model = load_fmu(init_fmu)

    # Set input for Stationary point A
    u_0_a = 0
    init_model.set('u', u_0_a)
    # Solve the initialization problem using FMI
    init_model.initialize()
    # Store stationary point A
    [h1_0_a, h2_0_a, h3_0_a] = init_model.get(['h1', 'h2', 'h3'])
    # Print some data for stationary point A
    print_stationary_point('A', h1_0_a, h2_0_a, h3_0_a, u_0_a)

    # Set inputs for Stationary point B
    init_model.reset()  # reset the FMU so that we can initialize it again
    u_0_b = 88.0
    init_model.set('u', u_0_b)
    # Solve the initialization problem using FMI
    init_model.initialize()
    # Store stationary point B
    [h1_0_b, h2_0_b, h3_0_b] = init_model.get(['h1', 'h2', 'h3'])
    # Print some data for stationary point B
    print_stationary_point('B', h1_0_b, h2_0_b, h3_0_b, u_0_b)

    init_res = simulate_tanks(model_path, u=u_0_b, with_plots=True,
                              with_full_traj_obj=True)

    h1_sim = init_res['h1']
    h2_sim = init_res['h2']
    h3_sim = init_res['h3']

    h1_sim_final = h1_sim[-1]
    h2_sim_final = h2_sim[-1]
    h3_sim_final = h3_sim[-1]

    print("Final values of the simulation:", h1_sim_final, h2_sim_final,
          h3_sim_final)

    # 3. Solve the optimal control problem
    # Compile and load optimization problem
    op = transfer_optimization_problem("TanksPkg.three_tanks_time_optimal",
                                       model_path)

    # Set initial values
    # op.set('h1_final', h1_sim_final)
    # op.set('h2_final', h2_sim_final)
    # op.set('h3_final', h3_sim_final)
    op.set('h1_final', float(h1_0_b))
    op.set('h2_final', float(h2_0_b))
    op.set('h3_final', float(h3_0_b))
    op.set('u_max', U_MAX)

    # Set options
    opt_opts = op.optimize_options()
    # opt_opts['n_e'] = 80  # Number of elements
    opt_opts['variable_scaling'] = False
    opt_opts['init_traj'] = init_res
    # opt_opts['nominal_traj'] = init_res
    opt_opts['IPOPT_options']['tol'] = 1e-3
    opt_opts['verbosity'] = 1
    print("OPTIMISATION OPTIONS:")
    for option, value in opt_opts.items():
        print('\t', option, ':', value)

    # Solve the optimal control problem
    res = op.optimize(options=opt_opts)

    # Extract variable profiles
    h1_res = res['h1']
    h2_res = res['h2']
    h3_res = res['h3']
    u_res = res['u']
    time_res = res['time']
    print("Final value of time:", time_res[-1])
    print(dir(res))

    # Plot the results
    plot_results(h1_res, h2_res, h3_res, time_res, u_res,
                 title="Optimised trajectories")

    normalised_u = []
    for val in u_res:
        if val < 50:
            normalised_u.append(0)
        else:
            normalised_u.append(U_MAX)
    for i in range(10):
        try:
            sim_result = simulate_tanks(model_path, u=normalised_u,
                                        t_final=time_res[-1],
                                        with_plots=True)
            plot_with_optimal_trajectories(time_res, sim_result['time'],
                                           sim_result["h1"], sim_result["h2"],
                                           sim_result["h3"], h1_res, h2_res,
                                           h3_res, normalised_u)
            break
        except FMUException:
            pass


if __name__ == '__main__':
    main()
