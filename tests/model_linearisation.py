from __future__ import print_function

from matplotlib import pyplot

from ds_tanks.tanks_utils import get_model_path, simulate_tanks, \
    run_linearisation, get_linear_quadratic_regulator


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
    linear_model = run_linearisation(get_model_path(), h10=30, h20=30, h30=22,
                                     u=86.566234)
    print(linear_model)
    print(linear_model.get_linearisation_point_info())

    k_matrix, s_matrix, e_matrix = get_linear_quadratic_regulator(linear_model)
    print("Linear-Quadratic Regulator:\nK matrix:\n%s" % k_matrix)
    print("S matrix:\n", s_matrix)
    print("E matrix:\n", e_matrix)
    # plot_points_per_simulation()
