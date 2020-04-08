def import_ideal_data():
    import glob
    import pylab as plt
    import pandas as pd
    data_guess_list = []
    file_list = glob.glob("reading_dataset/*.csv")
    for file in file_list:

        print("The name of the file: ", file)


        data = pd.read_csv(file)

        data_cl = dict()

        # "time","x","y","vx","vy","ax","ay","jx","jy","psi","psi_dot","throttle","delta","aty","any"
        data_cl['time_cl'] = plt.array([data.time])
        data_cl['time_guess'] = data_cl['time_cl'][-1]
        data_cl['x_cl'] = plt.array([data.x])
        data_cl['y_cl'] = plt.array([data.y])
        data_cl['vx_cl'] = plt.array([data.vx])
        data_cl['vy_cl'] = plt.array([data.vy])
        data_cl['ax_cl'] = plt.array([data.ax]) #total acc
        data_cl['ay_cl'] = plt.array([data.ay]) #total acc
        data_cl['jx_cl'] = plt.array([data.jx])
        data_cl['jy_cl'] = plt.array([data.jy])
        data_cl['psi_cl'] = plt.array([data.psi])
        data_cl['psi_dot_cl'] = plt.array([data.psi_dot])
        data_cl['throttle_cl'] = plt.array([data.throttle])
        data_cl['delta_cl'] = plt.array([data.delta])
        data_cl['aty_cl'] = plt.array([data.aty])
        data_cl['any_cl'] = (data_cl['psi_dot_cl'] * data_cl['vx_cl']).T
        data_cl['dt_cl'] = data_cl['time_cl'][0,1] - data_cl['time_cl'][0, 0]

        data_guess_list.append(data_cl)

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

    return data_guess_list
