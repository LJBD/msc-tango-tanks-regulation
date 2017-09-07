# coding=utf-8
from __future__ import print_function

import csv
import sys
from time import sleep

import re
from tango import DeviceProxy
from matplotlib import pyplot


def print_values(dev):
    elements = dev.read_attribute("FEMElements").value
    ctrl_elements = len(dev.read_attribute("OptimalControl").value)
    total_time = get_time_spent()
    raw_error = dev.read_attribute("VerificationError").value
    print("For %d elements:" % elements)
    print("Number of control elements: %d" % ctrl_elements)
    print("Total optimisation time: %f" % total_time)
    print("Verification error for raw control: %f" % raw_error)
    dev.NormaliseOptimalControl()
    print("Switch times:", dev.read_attribute("SwitchTimes").value)
    dev.RunVerification()
    sleep(20)
    norm_error = dev.read_attribute("VerificationError").value
    print("Verification error for normalised control: %f" % norm_error)
    put_to_csv(elements, ctrl_elements, total_time, raw_error, norm_error)


def apply_values(dev, h_init=10.0, h_final=20.0, elements=50):
    dev.write_attribute("H1Initial", h_init)
    dev.write_attribute("H2Initial", h_init)
    dev.write_attribute("H3Initial", h_init)
    dev.write_attribute("H1Final", h_final)
    dev.write_attribute("H2Final", h_final)
    dev.write_attribute("H3Final", h_final)
    dev.write_attribute("FEMElements", elements)


def get_time_spent(logs="/var/tmp/toc.log"):
    total_time = 0.0
    with open(logs) as logfile:
        for line in logfile:
            if "time spent in" in line:
                numbers = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                total_time += float(numbers[0])
    return total_time


def put_to_csv(elements, ctrl_elements, opt_time, raw_error, norm_error,
               log_file="elements.csv"):
    with open(log_file, 'ab') as f:
        writer = csv.writer(f)
        writer.writerow((elements, ctrl_elements, opt_time, raw_error,
                         norm_error))


def get_from_csv(log_file="elements.csv"):
    elements = []
    opt_times = []
    raw_errors = []
    norm_errors = []
    last_time = 152.35000000000002
    with open(log_file, 'rb') as f:
        reader = csv.reader(f)
        for row in reader:
            elements.append(row[0])
            opt_times.append(float(row[2]) - last_time)
            last_time = float(row[2])
            raw_errors.append(row[3])
            norm_errors.append(row[4])
    return elements, opt_times, raw_errors, norm_errors


def plot_elements_data(elements, opt_times, raw_errors, norm_errors):
    pyplot.close(1)
    pyplot.figure(1)
    pyplot.subplot(2, 1, 1)
    pyplot.plot(elements, raw_errors, 'ro-', elements, norm_errors, 'go-')
    pyplot.grid()
    pyplot.legend((u'Błędy surowego sterowania', u'Błędy znormalizowanego '
                                                 u'sterowania'))
    pyplot.ylabel(u'Błędy weryfikacji')
    pyplot.title(u"Wpływ liczby elementów na jakość rozwiązania")
    pyplot.subplot(2, 1, 2)
    pyplot.plot(elements, opt_times, 'bo')
    pyplot.grid()
    pyplot.ylabel(u'Czas optymalizacji')
    pyplot.xlabel(u'Liczba elementów')
    pyplot.show()


def main():
    dev = DeviceProxy('opt/ctrl/1')
    for i in range(50, int(sys.argv[1]), 50):
        print("Setting %d elements..." % i)
        apply_values(dev, 1, 15, elements=i)
        dev.Optimise(False)
        sleep(15)
        dev.RunVerification()
        sleep(20)
        print_values(dev)
    plot_elements_data(*get_from_csv())


if __name__ == '__main__':
    main()
