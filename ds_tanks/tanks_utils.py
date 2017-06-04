import os

import numpy
from matplotlib import pyplot
from pyfmi.fmi import load_fmu
from pymodelica import compile_fmu
from pyjmi import transfer_optimization_problem

U_MAX = 100.0


def simulate_tanks(model_path, u=U_MAX, t_start=0.0, t_final=50.0,
                   tank1_outflow=26, tank2_outflow=26, tank3_outflow=28,
                   with_plots=False):
    """
    Simulate tanks model.
    :param model_path: path to a file containing the model
    :param u: either a constant value or a vector of input
    :param t_start: starting time for the simulation
    :param t_final: final time for the simulation
    :param tank1_outflow: outflow coefficient of the 1st tanks
    :param tank2_outflow: outflow coefficient of the 2nd tanks
    :param tank3_outflow: outflow coefficient of the 3rd tanks
    :param with_plots: should the plots be displayed
    :return: None
    """
    # 2. Compute initial guess trajectories by means of simulation
    # Compile the optimization initialization model
    init_sim_fmu = compile_fmu("TanksPkg.ThreeTanks", model_path)
    # Load the model
    init_sim_model = load_fmu(init_sim_fmu)
    init_sim_model.set("C1", float(tank1_outflow))
    init_sim_model.set("C2", float(tank2_outflow))
    init_sim_model.set("C3", float(tank3_outflow))
    if hasattr(u, "__len__"):
        t = numpy.linspace(0.0, t_final, len(u))
        u_traj = numpy.transpose(numpy.vstack((t, u)))
        init_res = init_sim_model.simulate(start_time=t_start,
                                           final_time=t_final,
                                           input=('u', u_traj))
    else:
        # Set initial and reference values
        init_sim_model.set('u', u)
        # Simulate with constant input Tc
        init_res = init_sim_model.simulate(start_time=t_start,
                                           final_time=t_final)
    # Extract variable profiles
    t_init_sim = init_res['time']
    h1_init_sim = init_res['h1']
    h2_init_sim = init_res['h2']
    h3_init_sim = init_res['h3']
    u_init_sim = init_res['u']
    if with_plots:
        # Plot the initial guess trajectories
        plot_results(h1_init_sim, h2_init_sim, h3_init_sim, t_init_sim,
                     u_init_sim, title='Initial guess obtained by simulation')
    return init_res


def run_optimisation(model_path, tank1_outflow, tank2_outflow, tank3_outflow,
                     h1_final, h2_final, h3_final, max_control, sim_control,
                     ipopt_tolerance=1e-3, t_start=0, t_final=50.0,):
    # 2. Compute initial guess trajectories by means of simulation
    # Compile the optimization initialization model
    init_sim_fmu = compile_fmu("TanksPkg.ThreeTanks", model_path)
    # Load the model
    init_sim_model = load_fmu(init_sim_fmu)
    init_sim_model.set('u', sim_control)
    init_result = init_sim_model.simulate(start_time=t_start,
                                          final_time=t_final)
    # 3. Solve the optimal control problem
    # Compile and load optimization problem
    optimisation_model = "TanksPkg.three_tanks_time_optimal"
    op = transfer_optimization_problem(optimisation_model, model_path)
    print "Created OP definition"
    # Set outflow values from properties
    op.set("C1", float(tank1_outflow))
    op.set("C2", float(tank2_outflow))
    op.set("C3", float(tank3_outflow))
    # Set initial values
    op.set('h1_final', float(h1_final))
    op.set('h2_final', float(h2_final))
    op.set('h3_final', float(h3_final))
    op.set('u_max', max_control)
    print "Optimisation values setup."

    # Set options
    opt_options = op.optimize_options()
    # opt_options['n_e'] = 80  # Number of elements
    opt_options['variable_scaling'] = False
    opt_options['init_traj'] = init_result
    opt_options['IPOPT_options']['tol'] = ipopt_tolerance
    opt_options['verbosity'] = 1
    # Solve the optimal control problem
    res = op.optimize(options=opt_options)
    print "Optimisation complete!"
    opt_result = {"h1": res['h1'], "h2": res['h2'], "h3": res['h3'],
                  "u": res['u'], "time": res['time']}
    return opt_result


def get_initialisation_values(model_path, control_value):
    # 1. Solve the initialization problem
    # Compile the stationary initialization model into an FMU
    init_fmu = compile_fmu("TanksPkg.ThreeTanksInit", model_path)
    init_model = load_fmu(init_fmu)

    init_model.set('u', control_value)
    # Solve the initialization problem using FMI
    init_model.initialize()
    # Return stationary point
    return init_model.get(['h1', 'h2', 'h3'])


def get_model_path(model_file="opt_3_tanks.mop"):
    project_base = os.path.abspath(__file__)
    project_base = project_base[:project_base.rfind('/')]
    project_base = project_base[:project_base.rfind('/')]
    os.environ["MODELICAPATH"] += ":%s/optimica_model" % project_base
    model_path = os.path.join(project_base, 'optimica_model', model_file)
    return model_path


def plot_results(h1, h2, h3, time, u, title):
    pyplot.close(1)
    pyplot.figure(1)
    pyplot.subplot(2, 1, 1)
    pyplot.plot(time, h1, 'r', time, h2, 'g',
                time, h3, 'b')
    pyplot.grid()
    pyplot.legend(('h1', 'h2', 'h3'))
    pyplot.ylabel('Levels in tanks')
    pyplot.title(title)
    pyplot.subplot(2, 1, 2)
    pyplot.plot(time, u)
    pyplot.grid()
    pyplot.ylabel('Control')
    pyplot.xlabel('Time')
    pyplot.show()


def plot_with_optimal_trajectories(time_res, time_sim, h1, h2, h3, h1_opt,
                                   h2_opt, h3_opt, u_opt):
    pyplot.close(3)
    pyplot.figure(4)
    pyplot.subplot(4, 1, 1)
    pyplot.plot(time_res, h1_opt, '--')
    pyplot.plot(time_sim, h1)
    pyplot.legend(('optimized', 'simulated'))
    pyplot.grid(True)
    pyplot.ylabel('1st level')
    pyplot.title('Verification')

    pyplot.subplot(4, 1, 2)
    pyplot.plot(time_res, h2_opt, '--')
    pyplot.plot(time_sim, h2)
    pyplot.grid(True)
    pyplot.ylabel('2nd level')

    pyplot.subplot(4, 1, 3)
    pyplot.plot(time_res, h3_opt, '--')
    pyplot.plot(time_sim, h3)
    pyplot.grid(True)
    pyplot.ylabel('3rd level')

    pyplot.subplot(4, 1, 4)
    pyplot.plot(time_res, u_opt)
    pyplot.grid(True)
    pyplot.ylabel("Optimal control")
    pyplot.xlabel('time')
    pyplot.show()


def print_stationary_point(identifier, h1_0, h2_0, h3_0, u_0):
    print(' *** Stationary point %s ***' % identifier)
    print('u = %f' % u_0)
    print('h1 = %f' % h1_0)
    print('h2 = %f' % h2_0)
    print('h3 = %f' % h3_0)
