import os
from matplotlib import pyplot
from pyfmi.fmi import load_fmu
from pymodelica import compile_fmu

U_MAX = 100.0


def simulate_tanks(model_path, u=U_MAX, t_start=0.0, t_final=50.0,
                   with_plots=False):
    # 2. Compute initial guess trajectories by means of simulation
    # Compile the optimization initialization model
    init_sim_fmu = compile_fmu("TanksPkg.ThreeTanks", model_path)
    # Load the model
    init_sim_model = load_fmu(init_sim_fmu)
    # Set initial and reference values
    init_sim_model.set('u', u)
    # Simulate with constant input Tc
    init_res = init_sim_model.simulate(start_time=t_start, final_time=t_final)
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
    return h1_init_sim, h2_init_sim, h3_init_sim, init_res


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


def get_equilibrium(C1=26.0, C2=26.0, C3=28.0, h10=10, h20=10, h30=10, u=U_MAX,
                    retval="u"):
    pass


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
    # pyplot.hold(True)
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


def print_stationary_point(identifier, h1_0, h2_0, h3_0, u_0):
    print(' *** Stationary point %s ***' % identifier)
    print('u = %f' % u_0)
    print('h1 = %f' % h1_0)
    print('h2 = %f' % h2_0)
    print('h3 = %f' % h3_0)
