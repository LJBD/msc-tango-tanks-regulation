from __future__ import print_function
from time import sleep

from PyTango import DeviceProxy, DevState
from numpy import linspace

from ds_tanks.tanks_utils import plot_with_optimal_trajectories


def main(with_commands=True):
    opt_dev = DeviceProxy("opt/ctrl/1")
    if with_commands:
        print("Getting initial values...")
        opt_dev.command_inout_asynch("GetEquilibriumFromControl", 88.5)
        sleep(5)
        print("Running simulation...")
        opt_dev.command_inout_asynch("RunSimulation")
        sleep(5)
        print("Optimising...")
        opt_dev.command_inout_asynch("Optimise")
        sleep(5)
    if opt_dev.state() == DevState.STANDBY:
        opt_control = opt_dev.read_attribute("OptimalControl").value
        opt_time = opt_dev.read_attribute("OptimalTime").value
        time_opt = linspace(0.0, opt_time, len(opt_control))
        opt_h1 = opt_dev.read_attribute("OptimalH1").value
        opt_h2 = opt_dev.read_attribute("OptimalH2").value
        opt_h3 = opt_dev.read_attribute("OptimalH3").value
        # plot_results(opt_h1, opt_h2, opt_h3, time, opt_control,
        #              "Optimised with TanksOptimalControl")
        # opt_dev.command_inout_asynch("RunVerification")
        h1_sim = opt_dev.read_attribute("H1Simulated").value
        h2_sim = opt_dev.read_attribute("H2Simulated").value
        h3_sim = opt_dev.read_attribute("H3Simulated").value
        time_sim = opt_dev.read_attribute("SimulationTime").value
        print(get_matlab_data(opt_dev))
        plot_with_optimal_trajectories(time_opt, time_sim, h1_sim, h2_sim,
                                       h3_sim, opt_h1, opt_h2, opt_h3,
                                       opt_control)
    else:
        print("Wrong state, exiting!")


def get_matlab_data(device):
    switch_times = device.read_attribute('SwitchTimes').value
    data = [device.read_attribute('H1Final').value,
            device.read_attribute('H2Final').value,
            device.read_attribute('H3Final').value,
            device.read_attribute('OptimalTime').value,
            device.read_attribute('OptimalControl').value[0],
            0,
            switch_times[0],
            switch_times[1]]
    return data


if __name__ == '__main__':
    main(False)
