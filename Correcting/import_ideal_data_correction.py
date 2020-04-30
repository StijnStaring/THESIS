def import_ideal_data_correction():
    import glob
    import pylab as plt
    import pandas as pd

    file = glob.glob("reading_dataset/*.csv")
    print("The name of the file: ", file)

    data = pd.read_csv(file[0])

    data_cl = dict()

    # path = "rading_data\ diff_weights_guess"
    # file = open(path, newline='')
    # reader = csv.reader(file)
    # header = next(reader)
    # data = []

    # "time","x","y","vx","vy","ax","ay","jx","jy","psi","psi_dot","throttle","delta","aty","any"
    data_cl['time_cl'] = plt.array([data.time]).T
    data_cl['x_cl'] = plt.array([data.x]).T
    data_cl['y_cl'] = plt.array([data.y]).T
    data_cl['vx_cl'] = plt.array([data.vx]).T
    data_cl['vy_cl'] = plt.array([data.vy]).T
    data_cl['ax_cl'] = plt.array([data.ax]).T #total acc
    data_cl['jy_cl'] = plt.array([data.jy]).T
    data_cl['psi_cl'] = plt.array([data.psi]).T
    data_cl['psi_dot_cl'] = plt.array([data.psi_dot]).T
    data_cl['throttle_cl'] = plt.array([data.throttle]).T[0:-1,0]
    data_cl['throttle_cl'] = data_cl['throttle_cl'][:,plt.newaxis]
    data_cl['delta_cl'] = plt.array([data.delta]).T[0:-1,0]
    data_cl['delta_cl'] = data_cl['delta_cl'][:,plt.newaxis]
    data_cl['aty_cl'] = plt.array([data.aty]).T
    data_cl['any_cl'] = data_cl['psi_dot_cl'] * data_cl['vx_cl']
    data_cl['dt_cl'] = data_cl['time_cl'][1, 0] - data_cl['time_cl'][0, 0]

    # # Calculation of features
    # # f0: longitudinal acceleration
    # integrand = data_cl['ax_cl'] ** 2
    # f0_cal = 0
    # for i in plt.arange(0, len(integrand) - 1, 1):
    #     f0_cal = f0_cal + 0.5 * (integrand[i] + integrand[i + 1]) * data_cl['dt_cl']
    #
    # # f1: lateral acceleration
    # integrand = data_cl['ay_cl'] ** 2
    # f1_cal = 0
    # for i in plt.arange(0, len(integrand) - 1, 1):
    #     f1_cal = f1_cal + 0.5 * (integrand[i] + integrand[i + 1]) * data_cl['dt_cl']
    # # f1_cal = scipy.integrate.simps(integrand,plt.array(time_list))
    # # print('f1: ',f1_cal)
    #
    # # f2: lateral jerk
    # integrand = plt.array(data_cl['jy_cl']) ** 2
    # f2_cal = 0
    # for i in plt.arange(0, len(integrand) - 1, 1):
    #     f2_cal = f2_cal + 0.5 * (integrand[i] + integrand[i + 1]) * data_cl['dt_cl']
    # # f2_cal = scipy.integrate.simps(integrand,plt.array(time_list))
    #
    # # f3: desired velocity
    # integrand = plt.array(vx_start - data_cl['vx_cl']) ** 2
    # f3_cal = 0
    # for i in plt.arange(0, len(integrand) - 1, 1):
    #     f3_cal = f3_cal + 0.5 * (integrand[i] + integrand[i + 1]) * data_cl['dt_cl']
    # # f4_cal = scipy.integrate.simps(integrand,plt.array(time_list))
    #
    # # f4: desired lane change
    # integrand = plt.array(width_road - data_cl['y_cl']) ** 2
    # f4_cal = 0
    # for i in plt.arange(0, len(integrand) - 1, 1):
    #     f4_cal = f4_cal + 0.5 * (integrand[i] + integrand[i + 1]) * data_cl['dt_cl']

    return data_cl