from __future__ import print_function
import sys
from tango import DeviceProxy


def print_values(dev):
    dev.NormaliseOptimalControl()
    print("For %d elements:" % dev.read_attribute("FEMElements").value)
    print("Switch times:", dev.read_attribute("SwitchTimes").value)
    print("Number of control elements: %d" % len(dev.read_attribute(
        "OptimalControl").value))
    print("Total optimisation time: %f" % get_time_spent())


def apply_values(dev, h_init=10.0, h_final=20.0, elements=50):
    dev.write_attribute("H1Initial", h_init)
    dev.write_attribute("H2Initial", h_init)
    dev.write_attribute("H3Initial", h_init)
    dev.write_attribute("H1Final", h_final)
    dev.write_attribute("H2Final", h_final)
    dev.write_attribute("H3Final", h_final)
    dev.write_attribute("FEMElements", elements)


def get_time_spent(logs="/var/tmp/toc.log"):
    total_time = 0
    with open(logs) as logfile:
        for line in logfile:
            if "time spent in" in line:
                total_time += [int(s) for s in str.split() if s.isdigit()][0]
    return total_time


if __name__ == '__main__':
    dev = DeviceProxy('opt/ctrl/1')
    apply_values(dev, elements=sys.argv[1])
    dev.Optimise()
    print_values(dev)
