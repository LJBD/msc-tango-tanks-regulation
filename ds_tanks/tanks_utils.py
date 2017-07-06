from __future__ import print_function
import os
import logging
import control
import numpy
import sys
from matplotlib import pyplot
from pyfmi.fmi import load_fmu
from pyjmi.linearization import linearize_dae
from pymodelica import compile_fmu, compile_jmu
from pyjmi import transfer_optimization_problem, JMUModel

U_MAX = 100.0


class LinearModel(object):
    def __init__(self, E, A, B, F, g, state_names, input_names, algebraic_names,
                 dx0, x0, u0, w0, t0):
        self.E = E
        self.A = A
        self.B = B
        self.F = F
        self.g = g
        self.state_names = state_names
        self.input_names = input_names
        self.algebraic_names = algebraic_names
        self.dx0 = dx0
        self.x0 = x0
        self.u0 = u0
        self.w0 = w0
        self.t0 = t0

    def __repr__(self):
        representation = "Linear model with states %s," % self.state_names
        representation += "inputs %s," % self.input_names
        representation += " and algebraic values %s" % self.algebraic_names
        representation += "\nE matrix: \n%s" % repr(self.E)
        representation += "\nA matrix: \n%s" % repr(self.A)
        representation += "\nB matrix: \n%s" % repr(self.B)
        representation += "\nF matrix: \n%s" % repr(self.F)
        return representation

    def get_linearisation_point_info(self):
        lin_point = "Point of linearisation:\nx0:\n%s" % self.x0
        lin_point += "\ndx0: \n%s" % self.dx0
        lin_point += "\nu0: \n%s" % self.u0
        return lin_point


def simulate_tanks(model_path, u=U_MAX, t_start=0.0, t_final=50.0,
                   h10=20.0, h20=20.0, h30=20.0, tank1_outflow=26.0,
                   tank2_outflow=26.0, tank3_outflow=28.0,
                   with_full_traj_obj=False, with_plots=False):
    """
    Run simulation of the tanks model.

    :param model_path: path to a file containing the model
    :param u: either a constant value or a vector of input
    :param t_start: starting time for the simulation
    :param t_final: final time for the simulation
    :param h10: initial condition of the level in 1st tank
    :param h20: initial condition of the level in 2nd tank
    :param h30: initial condition of the level in 3rd tank
    :param tank1_outflow: outflow coefficient of the 1st tanks
    :param tank2_outflow: outflow coefficient of the 2nd tanks
    :param tank3_outflow: outflow coefficient of the 3rd tanks
    :param with_full_traj_obj: boolean specifying if the full trajectory should
    be returned
    :param with_plots: boolean specifying if the plots should be displayed
    :return: either a full trajectory object or a dictionary of trajectories
    """
    # 2. Compute initial guess trajectories by means of simulation
    # Compile the optimization initialization model
    simulation_fmu = compile_fmu("TanksPkg.ThreeTanks", model_path)
    # Load the model
    simulation_model = load_fmu(simulation_fmu)
    set_model_parameters(simulation_model,
                         {"h10": h10, "h20": h20, "h30": h30,
                          "C1": tank1_outflow, "C2": tank2_outflow,
                          "C3": tank3_outflow})

    if hasattr(u, "__len__"):
        t = numpy.linspace(0.0, t_final, len(u))
        u_traj = numpy.transpose(numpy.vstack((t, u)))
        print("CONTROL TRAJECTORY TO BE USED:\n", u_traj)
        init_res = simulation_model.simulate(start_time=t_start,
                                             final_time=t_final,
                                             input=('u', u_traj))
    else:
        simulation_model.set('u', u)
        init_res = simulation_model.simulate(start_time=t_start,
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
    if with_full_traj_obj:
        return init_res
    else:
        return_dict = {'h1': init_res['h1'], 'h2': init_res['h2'], 'h3':
            init_res['h3'], 'u': init_res['u'], 'time':
                           init_res['time']}
        return return_dict


def run_optimisation(model_path, tank1_outflow, tank2_outflow, tank3_outflow,
                     h1_final, h2_final, h3_final, max_control, sim_control,
                     h10=20.0, h20=20.0, h30=20.0, ipopt_tolerance=1e-3,
                     t_start=0, t_final=50.0):
    """
    Run optimisation of the tanks model.

    :param model_path: path to the Modelica/Optimica model
    :param tank1_outflow: outflow coefficient of the 1st tanks
    :param tank2_outflow: outflow coefficient of the 2nd tanks
    :param tank3_outflow: outflow coefficient of the 3rd tanks
    :param h1_final: wanted final value of the level in 1st tank
    :param h2_final: wanted final value of the level in 2nd tank
    :param h3_final: wanted final value of the level in 3rd tank
    :param max_control: higher bound of the control
    :param sim_control: control to be used in initialisation simulation
    :param h10: initial condition of the level in 1st tank
    :param h20: initial condition of the level in 2nd tank
    :param h30: initial condition of the level in 3rd tank
    :param ipopt_tolerance: tolerance of the IPOPT solver
    :param t_start: starting time of the initial simulation
    :param t_final: final time of the initial simulation
    :return: a dictionary with optimisation results
    """
    # 2. Compute initial guess trajectories by means of simulation
    # Compile the optimization initialization model
    init_sim_fmu = compile_fmu("TanksPkg.ThreeTanks", model_path)
    # Load the model
    simulation_model = load_fmu(init_sim_fmu)
    set_model_parameters(simulation_model,
                         {'u': sim_control, "h10": h10, "h20": h20, "h30": h30,
                          "C1": tank1_outflow, "C2": tank2_outflow,
                          "C3": tank3_outflow})
    init_result = simulation_model.simulate(start_time=t_start,
                                            final_time=t_final)
    # 3. Solve the optimal control problem
    # Compile and load optimization problem
    optimisation_model = "TanksPkg.three_tanks_time_optimal"
    op = transfer_optimization_problem(optimisation_model, model_path)
    # Set parameters
    set_model_parameters(op, {"h10": h10, "h20": h20, "h30": h30,
                              'h1_final': h1_final, 'h2_final': h2_final,
                              'h3_final': h3_final, "C1": tank1_outflow,
                              "C2": tank2_outflow, "C3": tank3_outflow,
                              'u_max': max_control})

    # Set options
    opt_options = op.optimize_options()
    # opt_options['n_e'] = 80  # Number of elements
    opt_options['variable_scaling'] = False
    opt_options['init_traj'] = init_result
    opt_options['IPOPT_options']['tol'] = ipopt_tolerance
    opt_options['verbosity'] = 1
    # Solve the optimal control problem
    res = op.optimize(options=opt_options)
    opt_result = {"h1": res['h1'], "h2": res['h2'], "h3": res['h3'],
                  "u": res['u'], "time": res['time']}
    return opt_result


def run_linearisation(model_path, parameters=None):
    if parameters is None:
        parameters = {"h1": 20, "h2": 20, "h3": 20}

    nonlinear_jmu = compile_jmu("TanksPkg.ThreeTanks", model_path)
    nonlinear_model = JMUModel(nonlinear_jmu)
    set_model_parameters(nonlinear_model, parameters)

    linear_model = LinearModel(*linearize_dae(nonlinear_model))

    if not numpy.array_equal(linear_model.E, numpy.identity(3)):
        minus_e = numpy.multiply(linear_model.E, -1)
        if numpy.array_equal(minus_e, numpy.identity(3)):
            numpy.multiply(linear_model.A, -1, linear_model.A)
            numpy.multiply(linear_model.B, -1, linear_model.B)
            numpy.multiply(linear_model.E, -1, linear_model.E)
        else:
            raise ValueError("Matrix E not scalable to identity!")
    return linear_model


def get_linear_quadratic_regulator(linear_model, q_matrix=numpy.identity(3),
                                   r_matrix=numpy.identity(1)):
    k_matrix, s_matrix, e_matrix = control.lqr(linear_model.A, linear_model.B,
                                               q_matrix, r_matrix)
    return k_matrix, s_matrix, e_matrix


def set_model_parameters(model, parameters):
    for key, value in parameters.items():
        model.set(key, float(value))


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


def signal_handler(signal, frame, connection=None, kill_event=None, pool=None,
                   process=None, logger=None):
    print('>>> Received %s, closing...!' % signal)
    if connection:
        print(">>> Closing connection...")
        connection.close()
    if kill_event:
        kill_event.set()
    if pool:
        pool.close()
        pool.join(1)
    if process:
        process.join(2)
    if logger:
        logger("Received SIGINT, shut down threads, going down.")
    sys.exit(0)


def setup_logging(log_file_name, formatter, logger_name=__name__):
    print("Setting up logging...")
    logger = logging.getLogger(logger_name)
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    if log_file_name:
        file_handler = setup_logging_to_file(formatter, log_file_name)
        logger.addHandler(file_handler)
    logger.debug("Logging set up.")
    return logger


def setup_logging_to_file(formatter, log_file_name):
    print("Setting up logging to %s..." % log_file_name)
    file_handler = logging.FileHandler(log_file_name)
    file_handler.setFormatter(formatter)
    return file_handler
