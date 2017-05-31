from time import sleep

from PyTango import DeviceProxy, DevState
from matplotlib import pyplot
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


def plot_with_optimal_trajectories(time_res, time_sim, h1, h2, h3, h1_opt,
                                   h2_opt, h3_opt):
    pyplot.close(3)
    pyplot.figure(3)
    # pyplot.hold(True)
    pyplot.subplot(3, 1, 1)
    pyplot.plot(time_res, h1_opt, '--')
    pyplot.plot(time_sim, h1)
    pyplot.legend(('optimized', 'simulated'))
    pyplot.grid(True)
    pyplot.ylabel('1st level')
    pyplot.title('Verification')

    pyplot.subplot(3, 1, 2)
    pyplot.plot(time_res, h2_opt, '--')
    pyplot.plot(time_sim, h2)
    pyplot.grid(True)
    pyplot.ylabel('Temperature')

    pyplot.subplot(3, 1, 3)
    pyplot.plot(time_res, h3_opt, '--')
    pyplot.plot(time_sim, h3)
    pyplot.grid(True)
    pyplot.ylabel('Cooling temperature')
    pyplot.xlabel('time')
    pyplot.show()


if __name__ == '__main__':
    main()
