from __future__ import print_function

import control
import numpy
from matplotlib import pyplot
from pyjmi import JMUModel
from pyjmi.linearization import linearize_dae
from pymodelica import compile_jmu

from ds_tanks.tanks_utils import get_model_path, simulate_tanks, \
    set_model_parameters


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


def run_linearisation(parameters=None):
    if parameters is None:
        parameters = {"h1": 20, "h2": 20, "h3": 20, "u": 40}
    model_path = get_model_path()

    nonlinear_jmu = compile_jmu("TanksPkg.ThreeTanks", model_path)
    nonlinear_model = JMUModel(nonlinear_jmu)
    # set_model_parameters(nonlinear_model, parameters)

    linear_model = LinearModel(*linearize_dae(nonlinear_model))

    print(linear_model)
    print(linear_model.get_linearisation_point_info())

    q_matrix = numpy.identity(3)
    r_matrix = numpy.identity(3)

    states = linear_model.B.shape[0]
    inputs = linear_model.B.shape[1]
    print("Got %d states and %d inputs" % (states, inputs))

    # elif (Q.shape[0] != nstates or Q.shape[1] != nstates or
    #               R.shape[0] != ninputs or R.shape[1] != ninputs or
    #               N.shape[0] != nstates or N.shape[1] != ninputs):
    K, S, E = control.lqr(linear_model.A, linear_model.B, q_matrix, r_matrix)
    print("Linear-Quadratic Regulator:\nK matrix:\n%s" % K)
    print("S matrix:\n", S)
    print("E matrix:\n", E)


def get_points_per_time(time):
    model_path = get_model_path()
    init_res = simulate_tanks(model_path, u=90, t_final=time)
    return len(init_res["h1"])


def plot_points_per_simulation():
    times = range(10, 100, 10)
    points = []
    for time in times:
        points.append(get_points_per_time(time))
    pyplot.close(1)
    pyplot.figure(1)
    pyplot.subplot(2, 1, 1)
    pyplot.plot(times, points)
    pyplot.grid()
    pyplot.ylabel('Number of points')
    pyplot.xlabel('Time of simulation')
    pyplot.subplot(2, 1, 2)
    pyplot.plot(times, [times[i] / points[i] for i in range(len(points))])
    pyplot.ylabel('Time step')
    pyplot.xlabel('Time of simulation')
    pyplot.show()


if __name__ == '__main__':
    run_linearisation()
    # plot_points_per_simulation()
