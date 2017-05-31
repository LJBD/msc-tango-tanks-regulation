from time import sleep

from PyTango import DeviceProxy, DevState
from numpy import linspace

from ds_tanks.tanks_utils import plot_results


def main():
    opt_dev = DeviceProxy("opt/ctrl/1")
    print "Getting initial values..."
    opt_dev.command_inout("GetEquilibriumFromControl", 73.5, timeout=15)
    sleep(1)
    print "Running simulation..."
    opt_dev.command_inout("RunSimulation", timeout=15)
    sleep(1)
    print "Optimising..."
    opt_dev.command_inout("Optimise", timeout=15)
    if opt_dev.state() == DevState.ON:
        opt_control = opt_dev.read_attribute("OptimalControl").value
        opt_time = opt_dev.read_attribute("T_opt").value
        time = linspace(0.0, opt_time, len(opt_control))
        plot_results(opt_dev.read_attribute("OptimalH1").value,
                     opt_dev.read_attribute("OptimalH2").value,
                     opt_dev.read_attribute("OptimalH3").value,
                     time, opt_control, "Optimised with TanksOptimalControl")


if __name__ == '__main__':
    main()
