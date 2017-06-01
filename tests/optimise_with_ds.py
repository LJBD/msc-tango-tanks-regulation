from time import sleep

from PyTango import DeviceProxy, DevState
from PyTango._PyTango import DevFailed
from numpy import linspace

from ds_tanks.tanks_utils import plot_results, plot_with_optimal_trajectories


def main(with_commands=True):
    opt_dev = DeviceProxy("opt/ctrl/1")
    if with_commands:
        print "Getting initial values..."
        opt_dev.command_inout_asynch("GetEquilibriumFromControl", 73.5)
        sleep(5)
        print "Running simulation..."
        opt_dev.command_inout_asynch("RunSimulation", 0)
        sleep(5)
        print "Optimising..."
        opt_dev.command_inout_asynch("Optimise")
        sleep(5)
    if opt_dev.state() == DevState.ON:
        opt_control = opt_dev.read_attribute("OptimalControl").value
        opt_time = opt_dev.read_attribute("T_opt").value
        time_opt = linspace(0.0, opt_time, len(opt_control))
        opt_h1 = opt_dev.read_attribute("OptimalH1").value
        opt_h2 = opt_dev.read_attribute("OptimalH2").value
        opt_h3 = opt_dev.read_attribute("OptimalH3").value
        # plot_results(opt_h1, opt_h2, opt_h3, time, opt_control,
        #              "Optimised with TanksOptimalControl")
        # opt_dev.command_inout_asynch("RunSimulation", 1)
        h1_sim = opt_dev.read_attribute("H1Simulated").value
        h2_sim = opt_dev.read_attribute("H2Simulated").value
        h3_sim = opt_dev.read_attribute("H3Simulated").value
        time_sim = opt_dev.read_attribute("SimulationTime").value
        plot_with_optimal_trajectories(time_opt, time_sim, h1_sim, h2_sim,
                                       h3_sim, opt_h1, opt_h2, opt_h3,
                                       opt_control)


if __name__ == '__main__':
    main(False)
