from __future__ import print_function
from time import sleep

try:
    from tango import DeviceProxy, DevState, ApiUtil
except ImportError:
    from PyTango import DeviceProxy, DevState, ApiUtil
from numpy import linspace

from ds_tanks.tanks_utils import plot_with_optimal_trajectories, plot_results


def main(with_commands=True):
    opt_dev = DeviceProxy("opt/ctrl/1")
    if with_commands:
        print("Setting initial values...")
        opt_dev.write_read_attribute("H1Final", 15)
        opt_dev.write_read_attribute("H2Final", 15)
        opt_dev.write_read_attribute("H3Final", 9)

        print("Optimising...")
        opt_dev.command_inout_asynch("Optimise")
        sleep(20)

        opt_dev.command_inout_asynch("SendControl")

    if opt_dev.state() == DevState.STANDBY:
        opt_control = opt_dev.read_attribute("OptimalControl").value
        opt_time = opt_dev.read_attribute("OptimalTime").value
        time_opt = linspace(0.0, opt_time, len(opt_control))
        opt_h1 = opt_dev.read_attribute("OptimalH1").value
        opt_h2 = opt_dev.read_attribute("OptimalH2").value
        opt_h3 = opt_dev.read_attribute("OptimalH3").value
        plot_results(opt_h1, opt_h2, opt_h3, time_opt, opt_control,
                     "Optimised with TanksOptimalControl")
        # Running verification...
        opt_dev.command_inout_asynch("RunVerification")
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
    k = device.read_attribute("K").value
    data = [device.read_attribute('H1Final').value,
            device.read_attribute('H2Final').value,
            device.read_attribute('H3Final').value,
            device.read_attribute('OptimalTime').value,
            device.read_attribute('OptimalControl').value[0],
            0,
            switch_times[0],
            switch_times[1],
            80.16,
            k[0],
            k[1],
            k[2]]
    return data


if __name__ == '__main__':
    main(True)
